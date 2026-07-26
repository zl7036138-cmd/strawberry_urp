"""Build the MoveItPy configuration from tracked package assets."""

from __future__ import annotations

from pathlib import Path


def build_moveit_config(
    camera_mount: str = "fixed",
    *,
    enable_trajectory_execution: bool = True,
) -> dict:
    """Return a complete in-process MoveItPy configuration dictionary."""
    if camera_mount not in {"fixed", "wrist", "dual"}:
        raise ValueError("camera_mount must be fixed, wrist, or dual")

    try:
        from ament_index_python.packages import get_package_share_directory
        from moveit_configs_utils import MoveItConfigsBuilder
        import yaml
    except ImportError as exc:  # pragma: no cover - ROS integration only
        raise RuntimeError("MoveIt configuration dependencies are unavailable") from exc

    manipulation_share = Path(
        get_package_share_directory("strawberry_manipulation")
    )
    sim_share = Path(get_package_share_directory("strawberry_sim"))
    robot_xacro = sim_share / "urdf" / "panda_gz.urdf.xacro"
    initial_positions = sim_share / "config" / "panda_initial_positions.yaml"

    builder = (
        MoveItConfigsBuilder(
            "panda",
            package_name="strawberry_manipulation",
        )
        .robot_description(
            file_path=str(robot_xacro),
            mappings={
                "initial_positions_file": str(initial_positions),
                "enable_attachment": "false",
                "camera_mount": camera_mount,
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
    )
    if enable_trajectory_execution:
        builder = builder.trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
    moveit_config = (
        builder
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
            load_all=False,
        )
        .to_moveit_configs()
    )
    config = moveit_config.to_dict()
    moveit_py_path = manipulation_share / "config" / "moveit_py.yaml"
    moveit_py_options = yaml.safe_load(moveit_py_path.read_text(encoding="utf-8"))
    if not isinstance(moveit_py_options, dict):
        raise RuntimeError("moveit_py.yaml must contain a mapping")
    config.update(moveit_py_options)
    if not enable_trajectory_execution:
        # MoveItConfigsBuilder auto-loads the package's conventional
        # moveit_controllers.yaml even when trajectory_execution() was not
        # called.  A read-only planning-scene audit must not create controller
        # plugins or action clients, so remove the injected execution surface.
        for key in (
            "moveit_controller_manager",
            "moveit_manage_controllers",
            "moveit_simple_controller_manager",
            "trajectory_execution",
        ):
            config.pop(key, None)
    return config
