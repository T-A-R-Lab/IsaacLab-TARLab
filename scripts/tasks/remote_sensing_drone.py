# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to spawn prims into the scene.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/00_sim/spawn_prims.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse
import os

# Suppress USD warnings at the C++ level
os.environ["OMNI_LOG_LEVEL"] = "error"
os.environ["CARB_LOGGING_LEVEL"] = "error"

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on spawning prims into the scene.")
# append AppLauncher cli args
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import numpy as np
import logging
import warnings

# Suppress warnings and set logging level
warnings.filterwarnings('ignore')
logging.getLogger().setLevel(logging.ERROR)
# Suppress Isaac Sim warnings
logging.getLogger('isaaclab').setLevel(logging.ERROR)
logging.getLogger('omni').setLevel(logging.ERROR)
logging.getLogger('omni.usd').setLevel(logging.CRITICAL)

from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation.articulation_cfg import ArticulationCfg
from isaaclab_assets.robots.quadcopter_gimbal_lidar import CRAZYFLIE_GIMBAL_LIDAR_CFG, LIDAR_RAYCASTER_CFG

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg


class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # Ground-plane

    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())


    # lights

    dome_light = AssetBaseCfg(

        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))

    )

    # Spawn a personalized usd object
    robot: ArticulationCfg = CRAZYFLIE_GIMBAL_LIDAR_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # Spawn an asset
    tree = AssetBaseCfg(
        prim_path="/World/DetectionObjects/TreeGroup",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.join("source/isaaclab_assets/isaaclab_assets/props/TreeGroup_1.usd"),
            scale=(1, 1, 1),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 4.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )


    # Lidar sensor functionality
    lidar = LIDAR_RAYCASTER_CFG


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):

    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # Circular trajectory parameters around the tree
    tree_center = np.array([0.0, 0.0])  # Tree position (x, y)
    radius = 3.0  # Circle radius around tree
    height = 7.0  # Flying height
    angular_speed = 2.5  # radians per second (full lap ~12.5 seconds)
    
    # Get robot asset
    robot = scene["robot"]

    while simulation_app.is_running():
        # Calculate circular position around tree
        angle = sim_time * angular_speed
        
        # Circular trajectory: x = center_x + r*cos(θ), y = center_y + r*sin(θ)
        target_pos = np.array([
            tree_center[0] + radius * np.cos(angle),
            tree_center[1] + radius * np.sin(angle),
            height
        ])
        
        # Calculate drone orientation to face the tree (optional)
        # Direction vector from drone to tree
        to_tree = tree_center - target_pos[:2]
        yaw = np.arctan2(to_tree[1], to_tree[0])
        
        # Convert yaw to quaternion (rotation around Z-axis)
        quat = np.array([
            np.cos(yaw/2),  # w
            0.0,             # x
            0.0,             # y
            np.sin(yaw/2)    # z
        ])
        
        # Set robot position and orientation for each environment
        root_state = robot.data.default_root_state.clone()
        for env_idx in range(args_cli.num_envs):
            root_state[env_idx, 0:3] = torch.tensor(target_pos, device=sim.device) + scene.env_origins[env_idx, :3]
            root_state[env_idx, 3:7] = torch.tensor(quat, device=sim.device)  # Set orientation
        
        robot.write_root_pose_to_sim(root_state[:, :7])
        robot.write_root_velocity_to_sim(root_state[:, 7:])
        
        # Set joint targets
        targets = robot.data.default_joint_pos
        robot.set_joint_position_target(targets)
        
        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)
        
        # Access lidar data
        lidar = scene["lidar"]
        lidar_data = lidar.data
        
        # Print lidar information with clear separator
        if count % 30 == 0:
            print("\n" + "="*60)
            print(f"[INFO] Time: {sim_time:.2f}s | Angle: {np.degrees(angle):.1f}°")
            print(f"[INFO] Position: ({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})")
            print(f"[LIDAR] Point cloud shape: {lidar_data.pos_w.shape}")
            
            # Check for valid hits
            valid_hits = lidar_data.ray_hits_w > 0
            num_valid_hits = valid_hits.sum().item()
            total_rays = lidar_data.ray_hits_w.numel()
            
            print(f"[LIDAR] Valid hits: {num_valid_hits}/{total_rays} ({100*num_valid_hits/total_rays:.1f}%)")
            
            if num_valid_hits > 0:
                distances = lidar_data.ray_hits_w[valid_hits]
                print(f"[LIDAR] Distance range: [{distances.min():.2f}, {distances.max():.2f}] m")
                print(f"[LIDAR] Mean distance: {distances.mean():.2f} m")
                
                # Detect if tree is visible (close hits likely the tree)
                tree_hits = ((distances > 1.0) & (distances < 4.0)).sum().item()
                if tree_hits > 10:
                    print(f"[LIDAR] Tree detected! ({tree_hits} hits)")
            else:
                print("[LIDAR] No objects detected!")
            
            print("="*60 + "\n")


def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([3.0, 3.0, 3.0], [1.0, 1.0, 0.5])
    # Design scene
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=1.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
