"""Launch the fixed-base, dual-camera, multi-target harvesting pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = FindPackageShare("strawberry_sim")
    localization_share = FindPackageShare("strawberry_localization")
    perception_share = FindPackageShare("strawberry_perception")
    scene_config = LaunchConfiguration("scene_config_file")
    world_file = LaunchConfiguration("world_file")
    model_path = LaunchConfiguration("model_path")
    device = LaunchConfiguration("device")
    device_parameter = ParameterValue(device, value_type=str)
    confidence_threshold = ParameterValue(
        LaunchConfiguration("confidence_threshold"), value_type=float
    )
    image_size = ParameterValue(LaunchConfiguration("image_size"), value_type=int)
    nms_iou_threshold = ParameterValue(
        LaunchConfiguration("nms_iou_threshold"), value_type=float
    )
    harvest_control_enabled = LaunchConfiguration("harvest_control_enabled")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_share, "launch", "sim.launch.py"])
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "world_file": world_file,
            "scene_config_file": scene_config,
            "simulation_seed": LaunchConfiguration("simulation_seed"),
            "base_camera_mast_xyz": LaunchConfiguration("base_camera_mast_xyz"),
            "base_camera_xyz": LaunchConfiguration("base_camera_xyz"),
            "base_camera_rpy": LaunchConfiguration("base_camera_rpy"),
            "base_camera_resolution": LaunchConfiguration(
                "base_camera_resolution"
            ),
            "camera_mount": "dual",
            "enable_attachment": "true",
            "enable_pose_control": "true",
        }.items(),
    )

    common_perception = PathJoinSubstitution(
        [perception_share, "config", "perception.yaml"]
    )
    generalized_localization = PathJoinSubstitution(
        [localization_share, "config", "localization_generalized.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("simulation_seed", default_value="17036"),
            DeclareLaunchArgument(
                "base_camera_mast_xyz", default_value="-0.35 0.45 0.05"
            ),
            DeclareLaunchArgument("base_camera_xyz", default_value="0 0 1.00"),
            DeclareLaunchArgument(
                "base_camera_rpy", default_value="0 0.543 -0.480"
            ),
            DeclareLaunchArgument(
                "base_camera_resolution",
                default_value="320x240",
                choices=["320x240", "640x480"],
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution(
                    [sim_share, "worlds", "strawberry_orchard.sdf"]
                ),
            ),
            DeclareLaunchArgument(
                "scene_config_file",
                default_value=PathJoinSubstitution(
                    [sim_share, "config", "scene.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "model_path", default_value="weights/yolo11s_640_best.pt"
            ),
            # Frozen from generalized validation, then used once without
            # retuning on the independent development qualification split.
            DeclareLaunchArgument("confidence_threshold", default_value="0.20524564385414124"),
            DeclareLaunchArgument("image_size", default_value="800"),
            DeclareLaunchArgument("nms_iou_threshold", default_value="0.50"),
            DeclareLaunchArgument("device", default_value="0"),
            DeclareLaunchArgument(
                "harvest_control_enabled", default_value="true"
            ),
            simulation,
            Node(
                package="strawberry_perception",
                executable="perception_node",
                name="strawberry_base_perception",
                output="screen",
                parameters=[
                    common_perception,
                    {
                        "use_sim_time": True,
                        "model_path": model_path,
                        "device": device_parameter,
                        "confidence_threshold": confidence_threshold,
                        "image_size": image_size,
                        "nms_iou_threshold": nms_iou_threshold,
                        "image_topic": "/camera/base/color/image_raw",
                        "detections_topic": "/strawberry/base/detections",
                    },
                ],
            ),
            Node(
                package="strawberry_localization",
                executable="generalized_localization_node",
                name="strawberry_base_localization",
                output="screen",
                parameters=[
                    generalized_localization,
                    {
                        "use_sim_time": True,
                        "detections_topic": "/strawberry/base/detections",
                        "depth_topic": "/camera/base/depth/image_raw",
                        "camera_info_topic": "/camera/base/camera_info",
                        "tracked_targets_topic": "/strawberry/tracked_targets",
                        "target_pose_topic": "/strawberry/localization_best_pose",
                        "ground_truth_association_enabled": False,
                        "confidence_threshold": confidence_threshold,
                    },
                ],
            ),
            Node(
                package="strawberry_perception",
                executable="perception_node",
                name="strawberry_wrist_perception",
                output="screen",
                parameters=[
                    common_perception,
                    {
                        "use_sim_time": True,
                        "model_path": model_path,
                        "device": device_parameter,
                        "confidence_threshold": confidence_threshold,
                        "image_size": image_size,
                        "nms_iou_threshold": nms_iou_threshold,
                        "image_topic": "/camera/wrist/color/image_raw",
                        "detections_topic": "/strawberry/wrist/detections",
                    },
                ],
            ),
            Node(
                package="strawberry_localization",
                executable="generalized_localization_node",
                name="strawberry_wrist_localization",
                output="screen",
                parameters=[
                    generalized_localization,
                    {
                        "use_sim_time": True,
                        "detections_topic": "/strawberry/wrist/detections",
                        "depth_topic": "/camera/wrist/depth/image_raw",
                        "camera_info_topic": "/camera/wrist/camera_info",
                        "tracked_targets_topic": "/strawberry/wrist/tracked_targets",
                        "target_pose_topic": "/strawberry/wrist/target_pose",
                        "completed_track_topic": "",
                        "ground_truth_association_enabled": False,
                        "confidence_threshold": confidence_threshold,
                        "allow_stationary_latest_tf_fallback": True,
                        "allow_stationary_sensor_sync_fallback": True,
                        "geometry_target_radius_m": 0.020,
                        "geometry_size_residual_sigma_weight": 0.5,
                        "use_bbox_center_bearing": True,
                        "target_hint_topic": "/strawberry/wrist/target_hint",
                        "target_hint_max_distance_m": 0.05,
                        "target_hint_wall_timeout_sec": 60.0,
                        "require_target_hint": True,
                    },
                ],
            ),
            Node(
                package="strawberry_bringup",
                executable="target_selector",
                name="strawberry_target_selector",
                output="screen",
                condition=IfCondition(harvest_control_enabled),
                parameters=[
                    {
                        "use_sim_time": True,
                        "scene_config_file": scene_config,
                        "confidence_threshold": confidence_threshold,
                    }
                ],
            ),
            Node(
                package="strawberry_manipulation",
                executable="pick_and_place_server",
                name="strawberry_pick_and_place",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scene_config_file": scene_config,
                        "camera_mount": "dual",
                        "fruit_pose_source": "tracked",
                        "tracked_targets_topic": "/strawberry/tracked_targets",
                        "target_refinement_topic": "",
                    }
                ],
            ),
            Node(
                package="strawberry_bringup",
                executable="harvest_orchestrator",
                name="strawberry_harvest_orchestrator",
                output="screen",
                condition=IfCondition(harvest_control_enabled),
                parameters=[
                    {
                        "use_sim_time": True,
                        "require_wrist_confirmation": True,
                        "wrist_confirmation_timeout_sec": 3.0,
                        "wrist_min_confidence": confidence_threshold,
                        "minimum_reobservation_baseline_m": 0.04,
                    }
                ],
            ),
        ]
    )
