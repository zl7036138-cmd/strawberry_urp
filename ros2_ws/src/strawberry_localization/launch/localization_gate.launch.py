"""Launch the simulator and production localization node for the T40 gate."""

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
        localization_share, "config", "localization.yaml"
    )
    headless = LaunchConfiguration("headless")
    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(simulation_share, "launch", "sim.launch.py")
                ),
                launch_arguments={
                    "headless": headless,
                    "enable_attachment": "false",
                    "enable_pose_control": "true",
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
