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
import sys
import json
from datetime import datetime
import signal
import atexit

import torch
import numpy as np
import logging
import warnings

# Suppress USD warnings at the C++ level
os.environ["OMNI_LOG_LEVEL"] = "error"
os.environ["CARB_LOGGING_LEVEL"] = "error"

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on spawning prims into the scene.")
# append AppLauncher cli args
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--save_dir", type=str, default="lidar_data", help="Directory to save lidar data.")
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

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
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )
    robot: ArticulationCfg = CRAZYFLIE_GIMBAL_LIDAR_CFG.replace(prim_path="/World/envs/env_.*/Robot")
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
    lidar = LIDAR_RAYCASTER_CFG


class LidarDataLogger:
    """Logger for saving lidar data to files."""
    
    def __init__(self, save_dir="lidar_data"):
        # Create directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(save_dir, f"run_{timestamp}")
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Create subdirectories
        self.numpy_dir = os.path.join(self.save_dir, "numpy")
        self.json_dir = os.path.join(self.save_dir, "json")
        self.pcd_dir = os.path.join(self.save_dir, "pointclouds")
        
        os.makedirs(self.numpy_dir, exist_ok=True)
        os.makedirs(self.json_dir, exist_ok=True)
        os.makedirs(self.pcd_dir, exist_ok=True)
        
        # Metadata log
        self.metadata = {
            "start_time": timestamp,
            "frames": []
        }
        
        self.metadata_saved = False  # Track if already saved
        
        print(f">>> [LOGGER] Created save directory: {self.save_dir}", flush=True)
    
    def save_frame(self, frame_id, sim_time, drone_pos, drone_quat, sensor_pos, hit_positions, valid_mask):
        """Save a single frame of lidar data in multiple formats."""
        
        # Convert to numpy for saving
        hit_positions_np = hit_positions.cpu().numpy()
        valid_mask_np = valid_mask.cpu().numpy()
        valid_hits_np = hit_positions_np[valid_mask_np]
        
        # 1. Save raw numpy data (compact binary format)
        np.savez_compressed(
            os.path.join(self.numpy_dir, f"frame_{frame_id:06d}.npz"),
            frame_id=frame_id,
            sim_time=sim_time,
            drone_position=np.array(drone_pos),
            drone_quaternion=np.array(drone_quat),
            sensor_position=sensor_pos.cpu().numpy(),
            hit_positions=hit_positions_np,
            valid_mask=valid_mask_np,
            valid_hits=valid_hits_np
        )
        
        # 2. Save point cloud in simple XYZ format
        if len(valid_hits_np) > 0:
            np.savetxt(
                os.path.join(self.pcd_dir, f"frame_{frame_id:06d}.xyz"),
                valid_hits_np,
                fmt='%.6f',
                header=f'X Y Z\nFrame: {frame_id} | Time: {sim_time:.2f}s | Points: {len(valid_hits_np)}',
                comments='# '
            )
        
        # 3. Save metadata in JSON
        sensor_pos_np = sensor_pos.cpu().numpy()
        if len(valid_hits_np) > 0:
            distances = np.linalg.norm(valid_hits_np - sensor_pos_np, axis=1)
            
            frame_metadata = {
                "frame_id": frame_id,
                "sim_time": float(sim_time),
                "drone_position": drone_pos.tolist(),
                "drone_quaternion": drone_quat.tolist(),
                "sensor_position": sensor_pos_np.tolist(),
                "statistics": {
                    "total_rays": int(len(hit_positions_np)),
                    "valid_hits": int(len(valid_hits_np)),
                    "hit_rate_percent": float(100 * len(valid_hits_np) / len(hit_positions_np)),
                    "distance_min": float(distances.min()),
                    "distance_max": float(distances.max()),
                    "distance_mean": float(distances.mean()),
                    "distance_median": float(np.median(distances))
                }
            }
        else:
            frame_metadata = {
                "frame_id": frame_id,
                "sim_time": float(sim_time),
                "drone_position": drone_pos.tolist(),
                "drone_quaternion": drone_quat.tolist(),
                "sensor_position": sensor_pos_np.tolist(),
                "statistics": {
                    "total_rays": int(len(hit_positions_np)),
                    "valid_hits": 0,
                    "hit_rate_percent": 0.0
                }
            }
        
        # Save individual frame JSON
        with open(os.path.join(self.json_dir, f"frame_{frame_id:06d}.json"), 'w') as f:
            json.dump(frame_metadata, f, indent=2)
        
        # Add to metadata log
        self.metadata["frames"].append(frame_metadata)
    
    def save_metadata(self):
        """Save complete metadata log - IMPORTANT: Call this when stopping!"""
        if self.metadata_saved:
            return  # Already saved, don't save again
        
        metadata_path = os.path.join(self.save_dir, "metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(self.metadata, f, indent=2)
        
        self.metadata_saved = True
        print(f">>> [LOGGER] Saved metadata with {len(self.metadata['frames'])} frames to {metadata_path}", flush=True)
    
    def save_trajectory(self):
        """Save complete trajectory as single numpy file."""
        if len(self.metadata["frames"]) == 0:
            return
        
        positions = np.array([f["drone_position"] for f in self.metadata["frames"]])
        times = np.array([f["sim_time"] for f in self.metadata["frames"]])
        
        np.savez_compressed(
            os.path.join(self.save_dir, "trajectory.npz"),
            positions=positions,
            times=times
        )
        
        print(f">>> [LOGGER] Saved trajectory with {len(positions)} waypoints", flush=True)
    
    def cleanup(self):
        """Force save all data."""
        print(f">>> [INFO] Cleanup called - saving all data...", flush=True)
        self.save_metadata()
        self.save_trajectory()
        print(f">>> [LOGGER] All data saved to: {self.save_dir}", flush=True)


# Global logger reference for signal handler
global_logger = None

def signal_handler(signum, frame):
    """Handle Ctrl+C and other signals to save data before exit."""
    print(f"\n>>> [SIGNAL] Received signal {signum}, saving data...", flush=True)
    if global_logger is not None:
        global_logger.cleanup()
    sys.exit(0)

def cleanup_on_exit():
    """Called automatically on normal exit."""
    if global_logger is not None and not global_logger.metadata_saved:
        print(">>> [ATEXIT] Saving data on exit...", flush=True)
        global_logger.cleanup()


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    global global_logger

    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    save_count = 0

    # Circular trajectory parameters around the tree
    tree_center = np.array([0.0, 0.0])
    radius = 3.0
    height = 7.0
    angular_speed = 2.5
    
    robot = scene["robot"]
    
    # Initialize data logger
    logger = LidarDataLogger(save_dir=args_cli.save_dir)
    global_logger = logger  # Store globally for signal handler
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Termination
    atexit.register(cleanup_on_exit)               # Normal exit
    
    try:
        while simulation_app.is_running():
            angle = sim_time * angular_speed
            
            target_pos = np.array([
                tree_center[0] + radius * np.cos(angle),
                tree_center[1] + radius * np.sin(angle),
                height
            ])
            
            to_tree = tree_center - target_pos[:2]
            yaw = np.arctan2(to_tree[1], to_tree[0])
            
            quat = np.array([
                np.cos(yaw/2),
                0.0,
                0.0,
                np.sin(yaw/2)
            ])
            
            root_state = robot.data.default_root_state.clone()
            for env_idx in range(args_cli.num_envs):
                root_state[env_idx, 0:3] = torch.tensor(target_pos, device=sim.device) + scene.env_origins[env_idx, :3]
                root_state[env_idx, 3:7] = torch.tensor(quat, device=sim.device)
            
            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])
            
            targets = robot.data.default_joint_pos
            robot.set_joint_position_target(targets)
            
            scene.write_data_to_sim()
            sim.step()
            sim_time += sim_dt
            count += 1
            scene.update(sim_dt)
            
            lidar = scene["lidar"]
            lidar_data = lidar.data
            
            # Save every 10 frames
            if count % 10 == 0:
                sensor_pos = lidar_data.pos_w[0]
                hit_positions = lidar_data.ray_hits_w[0]
                valid_mask = torch.all(torch.isfinite(hit_positions), dim=1)
                
                logger.save_frame(
                    frame_id=save_count,
                    sim_time=sim_time,
                    drone_pos=target_pos,
                    drone_quat=quat,
                    sensor_pos=sensor_pos,
                    hit_positions=hit_positions,
                    valid_mask=valid_mask
                )
                save_count += 1
                
                # Save metadata periodically (every 50 frames) as backup
                if save_count % 50 == 0:
                    logger.save_metadata()
                    logger.save_trajectory()
                    print(">>> [INFO] Metadata and trajectory saved as backup", flush=True)
            
            # Print progress
            if count % 100 == 0:
                print(f">>> [INFO] Time: {sim_time:.2f}s | Frames: {save_count} | Angle: {np.degrees(angle):.1f}°", flush=True)
    
    except KeyboardInterrupt:
        print("\n>>> [INFO] KeyboardInterrupt caught in try block", flush=True)
        logger.cleanup()
    except Exception as e:
        print(f"\n>>> [ERROR] Simulation error: {e}", flush=True)
        logger.cleanup()
    finally:
        print(f">>> [INFO] Finally block - ensuring data is saved", flush=True)
        logger.cleanup()


def main():
    """Main function."""
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.0, 3.0, 3.0], [1.0, 1.0, 0.5])
    scene_cfg = NewRobotsSceneCfg(args_cli.num_envs, env_spacing=1.0)
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    print(">>> [SETUP] Complete! Starting simulation...", flush=True)
    print(">>> [INFO] Press Ctrl+C to stop and save data", flush=True)

    run_simulator(sim, scene)


if __name__ == "__main__":
    main()
    simulation_app.close()
