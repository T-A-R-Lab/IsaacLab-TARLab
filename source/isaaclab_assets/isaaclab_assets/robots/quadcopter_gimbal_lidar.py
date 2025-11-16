# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the quadcopters"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.sensors import RayCasterCfg, patterns

##
# Configuration
##

CRAZYFLIE_GIMBAL_LIDAR_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path="${workspaceFolder}/source/isaaclab_assets/isaaclab_assets/robots/models/cf2x_gimbal_lidar_3DOF.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.1),
        joint_pos={
            ".*": 0.0,
        },
        joint_vel={
            "m1_joint": 0.0,
            "m2_joint": 0.0,
            "m3_joint": 0.0,
            "m4_joint": 0.0,
            "Joint_X": 0.0,
            "Joint_Y": 0.0,
            "Joint_Z": 0.0,
        },
    ),
    actuators={
        "motors": ImplicitActuatorCfg(
            joint_names_expr=["m[1-4]_joint"],
            stiffness=0.0,
            damping=0.0,
        ),
        "gimbal_joints": ImplicitActuatorCfg(
            joint_names_expr=["Joint_[XYZ]"],
            stiffness=0.0,
            damping=60.0,
        ),
    },
)
"""Configuration for the modified Crazyflie quadcopter."""

LIDAR_RAYCASTER_CFG = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/cf2x/gimbal_link/OS1",
    update_period=1 / 60,
    offset=RayCasterCfg.OffsetCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(1.0, 0.0, 0.0, 0.0),
    ),
    mesh_prim_paths=["/World/DetectionObjects/TreeGroup"],
    pattern_cfg=patterns.LidarPatternCfg(
        channels=64,
        vertical_fov_range=(-22.5, 22.5),
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=0.35,
    ),
    max_distance=100.0,
    drift_range=(-0.0, 0.0),
    debug_vis=True,
)
"""Configuration for the 3D lidar sensor (Ouster OS1-64 style)."""
