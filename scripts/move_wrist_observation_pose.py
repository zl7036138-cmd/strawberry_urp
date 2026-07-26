#!/usr/bin/env python3
"""Move Panda to one bounded eye-in-hand observation pose."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import threading

def main(args=None) -> int:  # pragma: no cover - ROS / MoveIt integration
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from strawberry_manipulation.core import Pose
    from strawberry_manipulation.moveit_backend import MoveItBackend
    from strawberry_manipulation.moveit_config import build_moveit_config
    from strawberry_sim.core import load_scene_config

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--x", type=float, default=0.28)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.72)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.9063077870)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=0.4226182617)
    parser.add_argument(
        "--camera-mount",
        choices=("wrist", "dual"),
        default="wrist",
        help="Robot description variant used by MoveIt.",
    )
    parser.add_argument(
        "--scene-config",
        type=Path,
        help="Scene manifest used to load all fruit collision obstacles.",
    )
    parser.add_argument(
        "--planning-attempts",
        type=int,
        default=3,
        help=(
            "Bounded collision-checked replans allowed only before any "
            "trajectory execution starts."
        ),
    )
    options, ros_args = parser.parse_known_args(args)
    if options.output.exists():
        raise ValueError(f"refusing to overwrite {options.output}")
    if options.planning_attempts <= 0:
        raise ValueError("planning_attempts must be positive")

    requested = Pose(
        options.x, options.y, options.z,
        options.qx, options.qy, options.qz, options.qw,
    ).normalized()
    scene_config = options.scene_config
    if scene_config is None:
        scene_config = (
            Path(get_package_share_directory("strawberry_sim"))
            / "config"
            / "scene.yaml"
        )
    scene = load_scene_config(scene_config)
    if scene.base_frame != "panda_link0":
        raise ValueError("observation motion requires scene base frame panda_link0")
    fruit_obstacles = {
        fruit.target_id: Pose(
            fruit.initial_pose.x,
            fruit.initial_pose.y,
            fruit.initial_pose.z,
        )
        for fruit in scene.ordered_fruits
    }
    rclpy.init(args=ros_args)
    node = Node("strawberry_wrist_observation_pose")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    backend = None
    status = 1
    try:
        backend = MoveItBackend(
            node,
            fruit_obstacles=fruit_obstacles,
            config_dict=build_moveit_config(camera_mount=options.camera_mount),
        )
        attempt_receipts = []
        outcome = None
        for attempt_index in range(1, options.planning_attempts + 1):
            outcome = backend.move_to(requested, "WRIST_OBSERVATION")
            attempt_receipts.append(
                {
                    "attempt": attempt_index,
                    "success": bool(outcome.success),
                    "planning_time_sec": float(outcome.planning_time_sec),
                    "execution_time_sec": float(outcome.execution_time_sec),
                    "collision": bool(outcome.collision),
                }
            )
            if outcome.success:
                break
            if outcome.execution_time_sec > 0.0:
                node.get_logger().error(
                    "observation motion failed after trajectory execution "
                    "started; refusing to replan"
                )
                break
            if attempt_index < options.planning_attempts:
                node.get_logger().warning(
                    "collision-checked observation plan was rejected before "
                    f"execution; bounded replan {attempt_index + 1}/"
                    f"{options.planning_attempts}"
                )
        if outcome is None:
            raise RuntimeError("observation planning produced no outcome")
        payload = {
            "schema_version": 1,
            "scope": "NON_ACCEPTANCE_WRIST_CAMERA_OBSERVATION_POSE",
            "camera_mount": options.camera_mount,
            "scene_config": str(scene_config),
            "fruit_collision_obstacle_ids": sorted(fruit_obstacles),
            "robot_motion_started": True,
            "fruit_manipulation_started": False,
            "planning_attempt_limit": options.planning_attempts,
            "planning_attempt_count": len(attempt_receipts),
            "planning_attempts": attempt_receipts,
            "requested_hand_pose": {
                "frame_id": "panda_link0",
                "position_m": [requested.x, requested.y, requested.z],
                "orientation_xyzw": [
                    requested.qx, requested.qy, requested.qz, requested.qw
                ],
            },
            "success": bool(outcome.success),
            "planning_time_sec": sum(
                receipt["planning_time_sec"]
                for receipt in attempt_receipts
            ),
            "execution_time_sec": sum(
                receipt["execution_time_sec"]
                for receipt in attempt_receipts
            ),
        }
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, sort_keys=True))
        status = 0 if outcome.success else 1
    finally:
        if backend is not None:
            backend.shutdown()
        executor.shutdown(timeout_sec=10)
        spin_thread.join(timeout=10)
        node.destroy_node()
        rclpy.try_shutdown()
    # Avoid the known Jazzy MoveItPy destructor crash after explicit shutdown.
    os._exit(status)

if __name__ == "__main__":
    main()
