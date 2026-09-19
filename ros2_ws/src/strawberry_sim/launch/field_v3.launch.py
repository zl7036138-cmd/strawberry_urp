"""Launch the opt-in, no-motion strawberry field-v3 qualification scene."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("strawberry_sim")
    sim_launch = os.path.join(package_share, "launch", "sim.launch.py")
    world_file = os.path.join(
        package_share, "worlds", "strawberry_field_v3.sdf"
    )
    scene_config = os.path.join(
        package_share, "config", "scene_field_v3.yaml"
    )
    initial_positions = os.path.join(
        package_share,
        "config",
        "panda_initial_positions_field_v3.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run the field qualification scene without the Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "camera_mount",
                default_value="dual",
                choices=["fixed", "wrist", "dual"],
                description="Field-v3 defaults to the base and wrist RGB-D cameras.",
            ),
            DeclareLaunchArgument(
                "simulation_seed",
                default_value="20260728",
                description="Gazebo RNG seed for repeatable field-v3 inspection.",
            ),
            LogInfo(
                msg=(
                    "Launching field-v3 in no-motion qualification mode; "
                    "the canonical v2 default is unchanged."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "world_file": world_file,
                    "scene_config_file": scene_config,
                    "simulation_seed": LaunchConfiguration("simulation_seed"),
                    "initial_positions_file": initial_positions,
                    "camera_mount": LaunchConfiguration("camera_mount"),
                    "enable_attachment": "false",
                    "enable_pose_control": "false",
                }.items(),
            ),
        ]
    )
