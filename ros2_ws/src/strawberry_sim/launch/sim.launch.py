"""Launch the deterministic Strawberry URP Gazebo Harmonic scene."""

from __future__ import annotations

import os
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml
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


_BASE_CAMERA_RESOLUTIONS = {
    "320x240": (320, 240),
    "640x480": (640, 480),
}


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


def _bridge_for_fruit_count(
    source: str,
    fruit_count: int,
    *,
    stem_constraints_enabled: bool = False,
) -> str:
    """Extend the frozen three-fruit bridge contract for generalized scenes."""

    if fruit_count <= 3 and not stem_constraints_enabled:
        return source
    with open(source, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, list):
        raise RuntimeError("Gazebo bridge configuration must be a YAML list")
    existing = {str(item.get("ros_topic_name")) for item in document if isinstance(item, dict)}
    for target_id in range(1, fruit_count + 1):
        additions = []
        if target_id >= 4:
            additions.extend([
            {
                "ros_topic_name": f"/strawberry/sim/fruit_{target_id}/pose_tf",
                "gz_topic_name": f"/model/strawberry_{target_id}/pose",
                "ros_type_name": "tf2_msgs/msg/TFMessage",
                "gz_type_name": "gz.msgs.Pose_V",
                "direction": "GZ_TO_ROS",
                "qos_profile": "SENSOR_DATA",
            },
            {
                "ros_topic_name": f"/strawberry/sim/fruit_{target_id}/attach_command",
                "gz_topic_name": f"/strawberry/sim/fruit_{target_id}/attach",
                "ros_type_name": "std_msgs/msg/Empty",
                "gz_type_name": "gz.msgs.Empty",
                "direction": "ROS_TO_GZ",
            },
            {
                "ros_topic_name": f"/strawberry/sim/fruit_{target_id}/detach_command",
                "gz_topic_name": f"/strawberry/sim/fruit_{target_id}/detach",
                "ros_type_name": "std_msgs/msg/Empty",
                "gz_type_name": "gz.msgs.Empty",
                "direction": "ROS_TO_GZ",
            },
            {
                "ros_topic_name": f"/strawberry/sim/fruit_{target_id}/attached_state",
                "gz_topic_name": f"/strawberry/sim/fruit_{target_id}/attached",
                "ros_type_name": "std_msgs/msg/String",
                "gz_type_name": "gz.msgs.StringMsg",
                "direction": "GZ_TO_ROS",
            },
            ])
        if stem_constraints_enabled:
            additions.extend(
                [
                    {
                        "ros_topic_name": f"/strawberry/sim/fruit_{target_id}/stem_attach_command",
                        "gz_topic_name": f"/strawberry/sim/fruit_{target_id}/stem_attach",
                        "ros_type_name": "std_msgs/msg/Empty",
                        "gz_type_name": "gz.msgs.Empty",
                        "direction": "ROS_TO_GZ",
                    },
                    {
                        "ros_topic_name": f"/strawberry/sim/fruit_{target_id}/stem_detach_command",
                        "gz_topic_name": f"/strawberry/sim/fruit_{target_id}/stem_detach",
                        "ros_type_name": "std_msgs/msg/Empty",
                        "gz_type_name": "gz.msgs.Empty",
                        "direction": "ROS_TO_GZ",
                    },
                    {
                        "ros_topic_name": f"/strawberry/sim/fruit_{target_id}/stem_attached_state",
                        "gz_topic_name": f"/strawberry/sim/fruit_{target_id}/stem_attached",
                        "ros_type_name": "std_msgs/msg/String",
                        "gz_type_name": "gz.msgs.StringMsg",
                        "direction": "GZ_TO_ROS",
                    },
                ]
            )
        for item in additions:
            if item["ros_topic_name"] in existing:
                raise RuntimeError("generalized bridge topic is duplicated")
            existing.add(item["ros_topic_name"])
            document.append(item)
    output_dir = Path(tempfile.gettempdir()) / "strawberry_urp"
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"bridge_fruits_{fruit_count}_{os.getpid()}.yaml"
    output.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return str(output)


def _enforce_initial_positions(robot_description_xml: str, source: str) -> str:
    with open(source, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    positions = (document or {}).get("initial_positions")
    if not isinstance(positions, dict) or not positions:
        raise RuntimeError(f"Initial positions YAML is invalid: {source}")
    root = ET.fromstring(robot_description_xml)
    updated: set[str] = set()
    for joint in root.findall(".//ros2_control/joint"):
        name = joint.get("name")
        if name not in positions:
            continue
        initial = next(
            (
                value
                for value in joint.findall(".//param")
                if value.get("name") == "initial_value"
            ),
            None,
        )
        if initial is None:
            raise RuntimeError(
                f"ros2_control joint has no initial_value parameter: {name}"
            )
        initial.text = str(float(positions[name]))
        updated.add(name)
    missing = sorted(set(positions) - updated)
    if missing:
        raise RuntimeError(
            f"Initial positions were not applied to ros2_control joints: {missing}"
        )
    return ET.tostring(root, encoding="unicode")


def _validated_three_vector(value: str, label: str) -> str:
    """Normalize one launch-supplied XYZ/RPY triplet before passing it to xacro."""

    parts = str(value).split()
    if len(parts) != 3:
        raise RuntimeError(f"{label} must contain exactly three numbers")
    try:
        numbers = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise RuntimeError(f"{label} must contain only numbers") from exc
    if not all(math.isfinite(number) for number in numbers):
        raise RuntimeError(f"{label} must contain finite numbers")
    return " ".join(f"{number:.12g}" for number in numbers)


def _validated_base_camera_resolution(value: str) -> tuple[int, int]:
    """Resolve one frozen overview-camera measurement profile."""

    profile = str(value).strip().lower()
    try:
        return _BASE_CAMERA_RESOLUTIONS[profile]
    except KeyError as exc:
        supported = ", ".join(_BASE_CAMERA_RESOLUTIONS)
        raise RuntimeError(
            f"base_camera_resolution must be one of: {supported}"
        ) from exc


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
    base_camera_mast_xyz = _validated_three_vector(
        LaunchConfiguration("base_camera_mast_xyz").perform(context),
        "base_camera_mast_xyz",
    )
    base_camera_xyz = _validated_three_vector(
        LaunchConfiguration("base_camera_xyz").perform(context),
        "base_camera_xyz",
    )
    base_camera_rpy = _validated_three_vector(
        LaunchConfiguration("base_camera_rpy").perform(context),
        "base_camera_rpy",
    )
    base_camera_image_width, base_camera_image_height = (
        _validated_base_camera_resolution(
            LaunchConfiguration("base_camera_resolution").perform(context)
        )
    )
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
    with open(scene_config, encoding="utf-8") as stream:
        scene_document = yaml.safe_load(stream) or {}
    fruits = scene_document.get("fruits")
    if not isinstance(fruits, list) or not 1 <= len(fruits) <= 9:
        raise RuntimeError("scene config must contain between 1 and 9 fruits")
    fruit_attachment_count = len(fruits)
    generalized_scene = isinstance(scene_document.get("generator"), dict)
    bridge_file = _bridge_for_fruit_count(
        bridge_file,
        fruit_attachment_count,
        stem_constraints_enabled=generalized_scene,
    )
    attachment_initialization_attempts = (
        150 if generalized_scene or fruit_attachment_count > 3 else 10
    )
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
    print(
        "[strawberry_sim] resolved Panda initial positions file: "
        f"{initial_positions}",
        flush=True,
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
    # The gravity-compensating gz_ros2_control build (upstream branch
    # add/gravity_compensation) shadows the system plugin when its lib dir is
    # first on the plugin path (same soname). Build it once with
    # scripts/build_gc_plugin.sh and point STRAWBERRY_GC_PLUGIN_LIB_DIR at
    # its install lib dir.
    gc_plugin_lib_dir = os.environ.get("STRAWBERRY_GC_PLUGIN_LIB_DIR", "")
    # Prepend to LD_LIBRARY_PATH itself: the loader resolves
    # libgz_ros2_control-system.so by soname and must find the
    # gravity-compensating build before the system one.
    if gc_plugin_lib_dir:
        os.environ["LD_LIBRARY_PATH"] = (
            gc_plugin_lib_dir
            + os.pathsep
            + os.environ.get("LD_LIBRARY_PATH", "")
        )
    gz_environment = {
        "GZ_SIM_RESOURCE_PATH": model_path
        + os.pathsep
        + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        "LD_LIBRARY_PATH": os.environ.get("LD_LIBRARY_PATH", ""),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(
            filter(
                None,
                (
                    gc_plugin_lib_dir,
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
            "enable_stem_attachment": "true" if generalized_scene else "false",
            "fruit_attachment_count": str(fruit_attachment_count),
            "camera_mount": camera_mount,
            "base_camera_mast_xyz": base_camera_mast_xyz,
            "base_camera_xyz": base_camera_xyz,
            "base_camera_rpy": base_camera_rpy,
            "base_camera_image_width": str(base_camera_image_width),
            "base_camera_image_height": str(base_camera_image_height),
        },
    ).toxml()
    robot_description_xml = _enforce_initial_positions(
        robot_description_xml,
        initial_positions,
    )
    description_root = ET.fromstring(robot_description_xml)
    applied_joint1 = description_root.find(
        ".//ros2_control/joint[@name='panda_joint1']/"
        "state_interface/param[@name='initial_value']"
    )
    print(
        "[strawberry_sim] enforced panda_joint1 initial value: "
        f"{None if applied_joint1 is None else applied_joint1.text}",
        flush=True,
    )
    robot_description = ParameterValue(
        robot_description_xml,
        value_type=str,
    )
    spawn_description_topic = "/strawberry/sim/robot_description"

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
            remappings=[
                ("robot_description", spawn_description_topic),
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
                spawn_description_topic,
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
                    "backend_initialization_attempts": attachment_initialization_attempts,
                    "resume_world_after_initialization": attachment_enabled,
                    "stem_constraints_enabled": generalized_scene,
                    "contact_resolved_only": generalized_scene,
                    # Generalized GPU perception runs below real time.  Keep
                    # the physical 1 s simulated-contact requirement intact,
                    # but allow enough wall time for the released fruit to
                    # fall and then accumulate that interval.
                    "verification_timeout_wall_sec": (
                        12.0 if generalized_scene else 3.0
                    ),
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
                "base_camera_mast_xyz",
                default_value="-0.35 0.45 0.05",
                description=(
                    "Dual-mode eye-to-hand mast origin in panda_link0. This is "
                    "a hardware-layout parameter, not a per-target control input."
                ),
            ),
            DeclareLaunchArgument(
                "base_camera_xyz",
                default_value="0 0 1.00",
                description="Base RGB-D camera origin relative to its fixed mast.",
            ),
            DeclareLaunchArgument(
                "base_camera_rpy",
                default_value="0 0.543 -0.480",
                description="Base RGB-D camera RPY relative to its fixed mast.",
            ),
            DeclareLaunchArgument(
                "base_camera_resolution",
                default_value="320x240",
                choices=list(_BASE_CAMERA_RESOLUTIONS),
                description=(
                    "Global base RGB-D measurement profile. Use one frozen "
                    "value for an entire development or qualification batch."
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
