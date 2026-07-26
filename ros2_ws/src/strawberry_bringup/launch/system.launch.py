"""Compose the Strawberry URP simulation and optional pipeline stages."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from strawberry_sim.core import load_scene_config


def _configure_scene_geometry(context, *, sim_share: str):
    """Resolve one scene manifest as the geometry source for all subsystems."""

    requested_scene = LaunchConfiguration("scene_config_file").perform(
        context
    ).strip()
    scene_path = requested_scene or os.path.join(
        sim_share, "config", "scene.yaml"
    )
    scene_path = os.path.abspath(os.path.expanduser(scene_path))
    if not os.path.isfile(scene_path):
        raise RuntimeError(f"scene config file does not exist: {scene_path}")
    scene = load_scene_config(scene_path)

    requested_offset = LaunchConfiguration(
        "surface_to_center_offset_m"
    ).perform(context).strip()
    if requested_offset:
        try:
            offset = float(requested_offset)
        except ValueError as exc:
            raise RuntimeError(
                "surface_to_center_offset_m must be a number"
            ) from exc
        if abs(offset - scene.fruit_collision_radius_m) > 1.0e-9:
            raise RuntimeError(
                "surface_to_center_offset_m must match the selected scene "
                f"fruit radius ({scene.fruit_collision_radius_m:.6f} m)"
            )
    else:
        offset = scene.fruit_collision_radius_m
    return [
        SetLaunchConfiguration("scene_config_file", scene_path),
        SetLaunchConfiguration(
            "surface_to_center_offset_m", f"{offset:.9f}"
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("strawberry_bringup")
    sim_share = get_package_share_directory("strawberry_sim")
    perception_share = get_package_share_directory("strawberry_perception")
    localization_share = get_package_share_directory("strawberry_localization")
    system_config = os.path.join(bringup_share, "config", "system.yaml")
    perception_config = os.path.join(
        perception_share, "config", "perception.yaml"
    )
    localization_config = os.path.join(
        localization_share, "config", "localization.yaml"
    )

    headless = LaunchConfiguration("headless")
    world_file = LaunchConfiguration("world_file")
    scene_config_file = LaunchConfiguration("scene_config_file")
    camera_mount = LaunchConfiguration("camera_mount")
    simulation_seed = LaunchConfiguration("simulation_seed")
    start_perception = LaunchConfiguration("start_perception")
    start_oracle_provider = LaunchConfiguration("start_oracle_provider")
    start_manipulation = LaunchConfiguration("start_manipulation")
    start_orchestrator = LaunchConfiguration("start_orchestrator")
    enable_attachment = LaunchConfiguration("enable_attachment")
    enable_pose_control = LaunchConfiguration("enable_pose_control")
    model_path = LaunchConfiguration("model_path")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    perception_detections_topic = LaunchConfiguration(
        "perception_detections_topic"
    )
    perception_target_topic = LaunchConfiguration("perception_target_topic")
    perception_image_topic = LaunchConfiguration("perception_image_topic")
    localization_depth_topic = LaunchConfiguration("localization_depth_topic")
    localization_camera_info_topic = LaunchConfiguration(
        "localization_camera_info_topic"
    )
    surface_to_center_offset_m = LaunchConfiguration(
        "surface_to_center_offset_m"
    )
    localization_selection_roi_min_x_px = LaunchConfiguration(
        "localization_selection_roi_min_x_px"
    )
    localization_selection_roi_min_y_px = LaunchConfiguration(
        "localization_selection_roi_min_y_px"
    )
    localization_selection_roi_max_x_px = LaunchConfiguration(
        "localization_selection_roi_max_x_px"
    )
    localization_selection_roi_max_y_px = LaunchConfiguration(
        "localization_selection_roi_max_y_px"
    )
    allow_stationary_latest_tf_fallback = LaunchConfiguration(
        "allow_stationary_latest_tf_fallback"
    )
    oracle_target_topic = LaunchConfiguration("oracle_target_topic")
    oracle_target_id = LaunchConfiguration("oracle_target_id")
    target_source = LaunchConfiguration("target_source")
    control_target_topic = LaunchConfiguration("control_target_topic")
    shadow_enabled = LaunchConfiguration("shadow_enabled")
    shadow_target_topic = LaunchConfiguration("shadow_target_topic")
    shadow_detections_topic = LaunchConfiguration("shadow_detections_topic")
    confidence_threshold_parameter = ParameterValue(
        confidence_threshold,
        value_type=float,
    )
    oracle_target_id_parameter = ParameterValue(oracle_target_id, value_type=int)
    shadow_enabled_parameter = ParameterValue(shadow_enabled, value_type=bool)
    stationary_tf_fallback_parameter = ParameterValue(
        allow_stationary_latest_tf_fallback, value_type=bool
    )
    surface_to_center_offset_parameter = ParameterValue(
        surface_to_center_offset_m, value_type=float
    )
    selection_roi_parameters = {
        "selection_roi_min_x_px": ParameterValue(
            localization_selection_roi_min_x_px, value_type=int
        ),
        "selection_roi_min_y_px": ParameterValue(
            localization_selection_roi_min_y_px, value_type=int
        ),
        "selection_roi_max_x_px": ParameterValue(
            localization_selection_roi_max_x_px, value_type=int
        ),
        "selection_roi_max_y_px": ParameterValue(
            localization_selection_roi_max_y_px, value_type=int
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("world_file", default_value=""),
            DeclareLaunchArgument(
                "scene_config_file",
                default_value="",
                description=(
                    "Scene geometry manifest. Empty selects Blender plant v2."
                ),
            ),
            DeclareLaunchArgument(
                "surface_to_center_offset_m",
                default_value="",
                description=(
                    "Optional audited override; it must equal the selected "
                    "scene fruit radius. Empty derives it from the manifest."
                ),
            ),
            DeclareLaunchArgument(
                "camera_mount",
                default_value="fixed",
                choices=["fixed", "wrist", "dual"],
            ),
            DeclareLaunchArgument(
                "simulation_seed",
                default_value="",
                description="Optional Gazebo RNG seed forwarded unchanged to gz sim.",
            ),
            DeclareLaunchArgument(
                "start_perception",
                default_value="false",
                description=(
                    "Run YOLO and RGB-D localization. During the T60 oracle "
                    "gate their output must remain on the shadow topics."
                ),
            ),
            DeclareLaunchArgument("start_oracle_provider", default_value="false"),
            DeclareLaunchArgument(
                "start_manipulation",
                default_value="false",
                description=(
                    "Enable after Panda MoveIt/Gazebo integration passes its gate."
                ),
            ),
            DeclareLaunchArgument(
                "start_orchestrator",
                default_value="false",
                description="Start trials only when manipulation is enabled.",
            ),
            DeclareLaunchArgument(
                "enable_attachment",
                default_value="false",
                description="Enable only with the verified detachable-joint backend.",
            ),
            DeclareLaunchArgument(
                "enable_pose_control",
                default_value="false",
                description="Expose Gazebo set_pose only to deterministic gates.",
            ),
            DeclareLaunchArgument(
                "model_path", default_value="weights/yolo11s_640_best.pt"
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.60",
                description=(
                    "Minimum detection confidence shared by perception and "
                    "localization."
                ),
            ),
            DeclareLaunchArgument(
                "perception_image_topic",
                default_value="/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "localization_depth_topic",
                default_value="/camera/depth/image_raw",
            ),
            DeclareLaunchArgument(
                "localization_camera_info_topic",
                default_value="/camera/camera_info",
            ),
            DeclareLaunchArgument(
                "localization_selection_roi_min_x_px", default_value="-1"
            ),
            DeclareLaunchArgument(
                "localization_selection_roi_min_y_px", default_value="-1"
            ),
            DeclareLaunchArgument(
                "localization_selection_roi_max_x_px", default_value="-1"
            ),
            DeclareLaunchArgument(
                "localization_selection_roi_max_y_px", default_value="-1"
            ),
            DeclareLaunchArgument(
                "allow_stationary_latest_tf_fallback",
                default_value="false",
                description=(
                    "Allow latest dynamic TF only when the localization node "
                    "proves the arm is stationary and the transform is bounded."
                ),
            ),
            DeclareLaunchArgument(
                "perception_detections_topic",
                default_value="/strawberry/shadow/detections",
            ),
            DeclareLaunchArgument(
                "perception_target_topic",
                default_value="/strawberry/shadow/target_pose",
            ),
            DeclareLaunchArgument(
                "oracle_target_topic",
                default_value="/strawberry/oracle/target_pose",
            ),
            DeclareLaunchArgument("oracle_target_id", default_value="1"),
            DeclareLaunchArgument(
                "target_source",
                default_value="oracle",
                choices=["oracle", "perception"],
            ),
            DeclareLaunchArgument(
                "control_target_topic",
                default_value="/strawberry/oracle/target_pose",
            ),
            DeclareLaunchArgument("shadow_enabled", default_value="false"),
            DeclareLaunchArgument(
                "shadow_target_topic",
                default_value="/strawberry/shadow/target_pose",
            ),
            DeclareLaunchArgument(
                "shadow_detections_topic",
                default_value="/strawberry/shadow/detections",
            ),
            OpaqueFunction(
                function=_configure_scene_geometry,
                kwargs={"sim_share": sim_share},
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_share, "launch", "sim.launch.py")
                ),
                launch_arguments={
                    "headless": headless,
                    "world_file": world_file,
                    "scene_config_file": scene_config_file,
                    "camera_mount": camera_mount,
                    "simulation_seed": simulation_seed,
                    "enable_attachment": enable_attachment,
                    "enable_pose_control": enable_pose_control,
                }.items(),
            ),
            Node(
                package="strawberry_bringup",
                executable="oracle_target_provider",
                name="strawberry_oracle_target_provider",
                output="screen",
                condition=IfCondition(start_oracle_provider),
                parameters=[
                    {
                        "use_sim_time": True,
                        "output_topic": oracle_target_topic,
                        "oracle_target_id": oracle_target_id_parameter,
                    }
                ],
            ),
            Node(
                package="strawberry_perception",
                executable="perception_node",
                name="strawberry_perception",
                output="screen",
                condition=IfCondition(start_perception),
                parameters=[
                    perception_config,
                    {
                        "model_path": model_path,
                        "confidence_threshold": confidence_threshold_parameter,
                        "detections_topic": perception_detections_topic,
                        "image_topic": perception_image_topic,
                    },
                ],
            ),
            Node(
                package="strawberry_localization",
                executable="localization_node",
                name="strawberry_localization",
                output="screen",
                condition=IfCondition(start_perception),
                parameters=[
                    localization_config,
                    {
                        "confidence_threshold": confidence_threshold_parameter,
                        "detections_topic": perception_detections_topic,
                        "target_pose_topic": perception_target_topic,
                        "depth_topic": localization_depth_topic,
                        "camera_info_topic": localization_camera_info_topic,
                        "surface_to_center_offset_m":
                            surface_to_center_offset_parameter,
                        **selection_roi_parameters,
                        "allow_stationary_latest_tf_fallback":
                            stationary_tf_fallback_parameter,
                    },
                ],
            ),
            Node(
                package="strawberry_manipulation",
                executable="pick_and_place_server",
                name="strawberry_pick_and_place",
                output="screen",
                condition=IfCondition(start_manipulation),
                parameters=[
                    {
                        "use_sim_time": True,
                        "camera_mount": camera_mount,
                        "scene_config_file": scene_config_file,
                    }
                ],
            ),
            Node(
                package="strawberry_bringup",
                executable="trial_orchestrator",
                name="strawberry_trial_orchestrator",
                output="screen",
                condition=IfCondition(start_orchestrator),
                parameters=[
                    system_config,
                    {
                        "target_source": target_source,
                        "control_target_topic": control_target_topic,
                        "shadow_enabled": shadow_enabled_parameter,
                        "shadow_target_topic": shadow_target_topic,
                        "shadow_detections_topic": shadow_detections_topic,
                    },
                ],
            ),
            LogInfo(
                condition=UnlessCondition(start_perception),
                msg=(
                    "Perception/localization are gated off until trained weights "
                    "are supplied with model_path:=..."
                ),
            ),
            LogInfo(
                condition=UnlessCondition(start_manipulation),
                msg=(
                    "Manipulation/attachment remain fail-closed until the Panda "
                    "integration smoke gate passes."
                ),
            ),
        ]
    )
