#!/usr/bin/env python3
"""Plan and execute a small Panda named-state round trip with MoveItPy."""

from __future__ import annotations

import json
import os
import sys
import time

from moveit.planning import MoveItPy
from strawberry_manipulation.moveit_config import build_moveit_config
from strawberry_manipulation.moveit_scene import apply_static_collision_scene


def main() -> int:
    moveit = MoveItPy(
        node_name="strawberry_moveit_round_trip",
        config_dict=build_moveit_config(),
    )
    arm = moveit.get_planning_component("panda_arm")
    planning_scene_monitor = moveit.get_planning_scene_monitor()
    report: dict[str, object] = {}
    # MoveItCpp creates its state-monitor thread asynchronously.  Wait for a
    # real Panda sample instead of planning from MoveIt's all-zero default.
    state_deadline = time.monotonic() + 20.0
    while True:
        with planning_scene_monitor.read_only() as scene:
            initial_positions = list(
                scene.current_state.get_joint_group_positions("panda_arm")
            )
        if initial_positions and max(abs(value) for value in initial_positions) > 0.1:
            break
        if time.monotonic() >= state_deadline:
            raise RuntimeError("MoveIt did not receive a live Panda joint state")
        time.sleep(0.1)
    # Allow the independently-created trajectory action client to finish DDS
    # discovery after the state subscription becomes live.
    time.sleep(0.5)
    expected_collision_ids = apply_static_collision_scene(
        planning_scene_monitor, "panda_link0"
    )
    with planning_scene_monitor.read_only() as scene:
        observed_collision_ids = tuple(
            sorted(
                collision_object.id
                for collision_object in (
                    scene.planning_scene_message.world.collision_objects
                )
            )
        )
    missing_collision_ids = sorted(
        set(expected_collision_ids) - set(observed_collision_ids)
    )
    if missing_collision_ids:
        raise RuntimeError(
            "MoveIt planning scene is missing static objects: "
            + ", ".join(missing_collision_ids)
        )
    report["static_collision_objects"] = list(observed_collision_ids)

    def run_stage(configuration: str) -> None:
        with planning_scene_monitor.read_only() as scene:
            current_before_plan = list(
                scene.current_state.get_joint_group_positions("panda_arm")
            )
        arm.set_start_state_to_current_state()
        arm.set_goal_state(configuration_name=configuration)
        planning_started = time.perf_counter()
        plan = arm.plan()
        planning_time = time.perf_counter() - planning_started
        if not plan:
            raise RuntimeError(f"MoveIt failed to plan named state {configuration}")
        trajectory_message = plan.trajectory.get_robot_trajectory_msg()
        trajectory_start = list(
            trajectory_message.joint_trajectory.points[0].positions
        )
        with planning_scene_monitor.read_only() as scene:
            current_before_execute = list(
                scene.current_state.get_joint_group_positions("panda_arm")
            )
        execution_started = time.perf_counter()
        execution_result = moveit.execute(plan.trajectory, controllers=[])
        execution_time = time.perf_counter() - execution_started
        execution_ok = (
            execution_result is None
            or execution_result is True
            or getattr(execution_result, "val", None) == 1
            or (
                hasattr(execution_result, "status")
                and bool(execution_result)
            )
        )
        if not execution_ok:
            print(
                json.dumps(
                    {
                        "failed_stage": configuration,
                        "current_before_plan": current_before_plan,
                        "trajectory_start": trajectory_start,
                        "current_before_execute": current_before_execute,
                        "execution_result_type": type(execution_result).__name__,
                        "execution_result_value": getattr(execution_result, "val", None),
                        "execution_status": getattr(execution_result, "status", None),
                    },
                    indent=2,
                    sort_keys=True,
                ),
                flush=True,
            )
            raise RuntimeError(f"MoveIt failed to execute named state {configuration}")
        report[configuration] = {
            "planning_time_sec": round(planning_time, 4),
            "execution_time_sec": round(execution_time, 4),
            "execution_result_type": type(execution_result).__name__,
            "trajectory_start": trajectory_start,
        }

    try:
        run_stage("smoke")
        run_stage("ready")
        report["success"] = True
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        # ros-jazzy-moveit-py 2.12.4 has an upstream teardown defect that can
        # crash while destroying MoveItPy even after explicit shutdown calls:
        # https://github.com/moveit/moveit2/issues/3721
        # Terminate only after every stage and the flushed success report have
        # completed.  Exceptions still take the normal non-zero failure path.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
    finally:
        del arm
        del planning_scene_monitor
        del moveit
        # This standalone smoke test deliberately uses only MoveItPy's rclcpp
        # context.  Creating an otherwise-unused rclpy context makes Jazzy
        # report a non-zero interpreter shutdown after successful execution.


if __name__ == "__main__":
    raise SystemExit(main())
