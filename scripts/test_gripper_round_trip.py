#!/usr/bin/env python3
"""Verify that both Panda fingers follow one ros2_control gripper command."""

from __future__ import annotations

import argparse
import json
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import ParallelGripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def wait_future(node: Node, future, timeout_sec: float):
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError("gripper action future timed out")
    result = future.result()
    if result is None:
        raise RuntimeError("gripper action future returned no result")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--open-position", type=float, default=0.04)
    parser.add_argument("--closed-position", type=float, default=0.0)
    parser.add_argument("--position-tolerance", type=float, default=0.004)
    parser.add_argument("--symmetry-tolerance", type=float, default=0.002)
    args = parser.parse_args()
    if args.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be positive")
    if not 0.0 <= args.closed_position < args.open_position:
        parser.error("positions must satisfy 0 <= closed < open")

    rclpy.init()
    node = Node("panda_gripper_round_trip_test")
    latest: dict[str, float] = {}

    def on_joint_state(message: JointState) -> None:
        latest.update(zip(message.name, message.position, strict=False))

    subscription = node.create_subscription(
        JointState,
        "/joint_states",
        on_joint_state,
        qos_profile_sensor_data,
    )
    del subscription  # rclpy node retains the subscription
    client = ActionClient(
        node,
        ParallelGripperCommand,
        "/panda_gripper_controller/gripper_cmd",
    )
    report: dict[str, object] = {}

    def current_pair() -> tuple[float, float]:
        try:
            return latest["panda_finger_joint1"], latest["panda_finger_joint2"]
        except KeyError as exc:
            raise RuntimeError("joint_states does not contain both Panda fingers") from exc

    def wait_for_position(target: float) -> tuple[float, float]:
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if {
                "panda_finger_joint1",
                "panda_finger_joint2",
            }.issubset(latest):
                pair = current_pair()
                if all(
                    abs(position - target) <= args.position_tolerance
                    for position in pair
                ):
                    if abs(pair[0] - pair[1]) > args.symmetry_tolerance:
                        raise RuntimeError(
                            f"finger positions are asymmetric at target {target}: {pair}"
                        )
                    return pair
        pair = current_pair()
        raise TimeoutError(f"fingers did not reach {target}: {pair}")

    def command(position: float) -> tuple[float, float]:
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ["panda_finger_joint1"]
        goal.command.position = [position]
        goal.command.effort = [40.0]
        goal_handle = wait_future(
            node,
            client.send_goal_async(goal),
            args.timeout_sec,
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"gripper rejected position {position}")
        wrapped = wait_future(
            node,
            goal_handle.get_result_async(),
            args.timeout_sec,
        )
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"gripper action ended with status {wrapped.status} at {position}"
            )
        return wait_for_position(position)

    try:
        if not client.wait_for_server(timeout_sec=args.timeout_sec):
            raise RuntimeError("Panda gripper action server is unavailable")

        # Start from the known open state, close fully, and reopen.  Reaching
        # both endpoints proves that gz_ros2_control is applying the URDF mimic
        # relation even when DART reports no native physics mimic constraint.
        report["initial_open"] = command(args.open_position)
        report["closed"] = command(args.closed_position)
        report["reopened"] = command(args.open_position)
        report["success"] = True
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
