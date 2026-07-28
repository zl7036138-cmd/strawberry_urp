"""Launch the deterministic Strawberry URP Gazebo Harmonic scene."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _world_without_fixed_camera(source: str) -> str:
    """Materialize a temporary world without the historical static camera."""
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError("Gazebo world has no <world> element")
    matches = [
        include
        for include in world.findall("include")
        if (include.findtext("name") or "").strip() == "strawberry_rgbd_camera"
    ]
    if len(matches) != 1:
        raise RuntimeError(
            "wrist camera mode requires exactly one removable "
            "strawberry_rgbd_camera include"
        )
    world.remove(matches[0])
    output_dir = Path(tempfile.gettempdir()) / "strawberry_urp"
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"{Path(source).stem}_wrist_{os.getpid()}.sdf"
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return str(output)


def _launch_nodes(context):
    package_share = get_package_share_directory("strawberry_sim")
    requested_world = LaunchConfiguration("world_file").perform(context).strip()
    world_file = requested_world or os.path.join(
        package_share, "worlds", "strawberry_orchard.sdf"
    )
    world_file = os.path.abspath(os.path.expanduser(world_file))
    if not os.path.isfile(world_file):
        raise RuntimeError(f"Gazebo world file does not exist: {world_file}")
    camera_mount = LaunchConfiguration("camera_mount").perform(context).strip()
    if camera_mount not in {"fixed", "wrist", "dual"}:
        raise RuntimeError("camera_mount must be fixed, wrist, or dual")
    if camera_mount in {"wrist", "dual"}:
        world_file = _world_without_fixed_camera(world_file)
    bridge_file = os.path.join(package_share, "config", "bridge.yaml")
    node_config = os.path.join(package_share, "config", "sim_nodes.yaml")
    requested_scene_config = (
        LaunchConfiguration("scene_config_file").perform(context).strip()
    )
    scene_config = requested_scene_config or os.path.join(
        package_share, "config", "scene.yaml"
    )
    scene_config = os.path.abspath(os.path.expanduser(scene_config))
    if not os.path.isfile(scene_config):
        raise RuntimeError(f"scene config file does not exist: {scene_config}")
    panda_xacro = os.path.join(package_share, "urdf", "panda_gz.urdf.xacro")
    requested_initial_positions = (
        LaunchConfiguration("initial_positions_file").perform(context).strip()
    )
    initial_positions = requested_initial_positions or os.path.join(
        package_share, "config", "panda_initial_positions.yaml"
    )
    initial_positions = os.path.abspath(os.path.expanduser(initial_positions))
    if not os.path.isfile(initial_positions):
        raise RuntimeError(
            f"Panda initial positions file does not exist: {initial_positions}"
        )
    model_path = os.path.join(package_share, "models")
    headless = LaunchConfiguration("headless").perform(context).lower() in {
        "1",
        "true",
        "yes",
    }
    simulation_seed_text = LaunchConfiguration("simulation_seed").perform(context).strip()
    simulation_seed = None
    if simulation_seed_text:
        try:
            simulation_seed = int(simulation_seed_text)
        except ValueError as exc:
            raise RuntimeError("simulation_seed must be an unsigned integer") from exc
        if not 0 <= simulation_seed <= 0xFFFFFFFF:
            raise RuntimeError("simulation_seed must be within [0, 4294967295]")
    enable_attachment = LaunchConfiguration("enable_attachment").perform(context)
    attachment_enabled = enable_attachment.lower() in {"1", "true", "yes"}
    enable_pose_control = LaunchConfiguration("enable_pose_control").perform(context)
    pose_control_enabled = enable_pose_control.lower() in {"1", "true", "yes"}
    # DetachableJoint instances are attached when their model is inserted.
    # Keep physics paused until attachment_manager confirms that every fruit
    # has been detached, otherwise the newly spawned Panda can drag the
    # benchmark fruit away from its declared initial pose.
    gz_command = [FindExecutable(name="gz"), "sim"]
    if not attachment_enabled:
        gz_command.append("-r")
    if headless:
        gz_command.append("-s")
    if simulation_seed is not None:
        gz_command.extend(["--seed", str(simulation_seed)])
    gz_command.extend([world_file, "--force-version", "8"])
    gz_environment = {
        "GZ_SIM_RESOURCE_PATH": model_path
        + os.pathsep
        + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(
            filter(
                None,
                (
                    os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                    os.environ.get("LD_LIBRARY_PATH", ""),
                ),
            )
        ),
    }
    robot_description_xml = xacro.process_file(
        panda_xacro,
        mappings={
            "initial_positions_file": initial_positions,
            "enable_attachment": enable_attachment,
            "camera_mount": camera_mount,
        },
    ).toxml()
    robot_description = ParameterValue(
        robot_description_xml,
        value_type=str,
    )

    nodes = [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            model_path
            + os.pathsep
            + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ),
        ExecuteProcess(
            cmd=gz_command,
            name="gazebo",
            output="screen",
            additional_env=gz_environment,
            shell=False,
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="strawberry_gz_bridge",
            output="screen",
            parameters=[{"config_file": bridge_file, "use_sim_time": True}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="panda_robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": True}
            ],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_panda",
            output="screen",
            arguments=[
                "-world",
                "strawberry_orchard",
                "-name",
                "panda",
                "-topic",
                "robot_description",
                "-allow_renaming",
                "false",
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager-timeout",
                        "30",
                    ],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "panda_arm_controller",
                        "--controller-manager-timeout",
                        "30",
                    ],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "panda_gripper_controller",
                        "--controller-manager-timeout",
                        "30",
                    ],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "panda_gripper_right_controller",
                        "--controller-manager-timeout",
                        "30",
                    ],
                    output="screen",
                ),
            ],
        ),
        Node(
            package="strawberry_sim",
            executable="ground_truth_publisher",
            output="screen",
            parameters=[node_config, {"scene_config_file": scene_config}],
        ),
        Node(
            package="strawberry_sim",
            executable="contact_monitor",
            output="screen",
            parameters=[
                node_config,
                {"scene_config_file": scene_config},
            ],
        ),
        Node(
            package="strawberry_sim",
            executable="attachment_manager",
            output="screen",
            parameters=[
                node_config,
                {
                    "scene_config_file": scene_config,
                    "attachment_backend_enabled": attachment_enabled,
                    "resume_world_after_initialization": attachment_enabled,
                },
            ],
        ),
    ]
    if camera_mount == "fixed":
        nodes[5:5] = [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_mount_static_tf",
                arguments=[
                    "--x", "0.0", "--y", "-1.10", "--z", "1.05",
                    "--roll", "0.0", "--pitch", "0.35",
                    "--yaw", "1.5707963268",
                    "--frame-id", "panda_link0",
                    "--child-frame-id", "strawberry_camera_link",
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_optical_static_tf",
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.0",
                    "--roll", "-1.5707963268", "--pitch", "0.0",
                    "--yaw", "-1.5707963268",
                    "--frame-id", "strawberry_camera_link",
                    "--child-frame-id", "strawberry_camera_optical_frame",
                ],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    if attachment_enabled:
        nodes.insert(
            2,
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="strawberry_world_control_bridge",
                output="screen",
                arguments=[
                    "/world/strawberry_orchard/control@"
                    "ros_gz_interfaces/srv/ControlWorld"
                ],
            ),
        )
    if pose_control_enabled:
        nodes.insert(
            2,
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="strawberry_pose_control_bridge",
                output="screen",
                arguments=[
                    "/world/strawberry_orchard/set_pose@"
                    "ros_gz_interfaces/srv/SetEntityPose"
                ],
            ),
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Run only the Gazebo server for repeatable benchmarks.",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="",
                description=(
                    "Optional absolute materialized SDF path for a frozen "
                    "lighting/occlusion condition. Empty uses the base world."
                ),
            ),
            DeclareLaunchArgument(
                "scene_config_file",
                default_value="",
                description=(
                    "Optional scene manifest matching world_file. Empty uses "
                    "the canonical Blender plant scene."
                ),
            ),
            DeclareLaunchArgument(
                "simulation_seed",
                default_value="",
                description=(
                    "Optional Gazebo RNG seed. Formal benchmark runners must "
                    "set this explicitly; an empty value preserves normal launches."
                ),
            ),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value="",
                description=(
                    "Optional absolute Panda initial-joint YAML. Empty keeps "
                    "the accepted v2 ready configuration."
                ),
            ),
            DeclareLaunchArgument(
                "camera_mount",
                default_value="fixed",
                choices=["fixed", "wrist", "dual"],
                description=(
                    "Use the historical world-fixed RGB-D camera or the "
                    "eye-in-hand camera attached to panda_hand."
                ),
            ),
            DeclareLaunchArgument(
                "enable_attachment",
                default_value="false",
                description="Enable only after the Panda detachable-joint smoke test passes.",
            ),
            DeclareLaunchArgument(
                "enable_pose_control",
                default_value="false",
                description=(
                    "Expose Gazebo set_pose only to deterministic localization "
                    "and benchmark harnesses."
                ),
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
