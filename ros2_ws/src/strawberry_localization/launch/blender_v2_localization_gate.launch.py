"""Launch ADR-0039's camera-clear Blender-v2 localization accuracy gate."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    simulation_share = get_package_share_directory("strawberry_sim")
    localization_share = get_package_share_directory("strawberry_localization")
    localization_config = os.path.join(
        localization_share,
        "config",
        "localization_blender_v2.yaml",
    )
    scene_config = os.path.join(
        simulation_share,
        "config",
        "scene.yaml",
    )
    headless = LaunchConfiguration("headless")
    world_file = LaunchConfiguration("world_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("world_file"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(simulation_share, "launch", "sim.launch.py")
                ),
                launch_arguments={
                    "headless": headless,
                    "world_file": world_file,
                    "scene_config_file": scene_config,
                    "enable_attachment": "false",
                    "enable_pose_control": "true",
                    "camera_mount": "fixed",
                }.items(),
            ),
            Node(
                package="strawberry_localization",
                executable="localization_node",
                name="strawberry_localization",
                output="screen",
                parameters=[localization_config],
            ),
        ]
    )
