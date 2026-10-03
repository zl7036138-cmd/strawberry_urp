#!/usr/bin/env python3
"""Capture a read-only complete Panda state before the v2 execution action."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from typing import Mapping


READY_JOINTS = {
    "panda_joint1": 0.0,
    "panda_joint2": -0.785,
    "panda_joint3": 0.0,
    "panda_joint4": -2.356,
    "panda_joint5": 0.0,
    "panda_joint6": 1.571,
    "panda_joint7": 0.785,
}
FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")


def evaluate_snapshot(
    arm_positions: Mapping[str, object],
    finger_positions: Mapping[str, object],
    target_attached: object,
    complete_sample_count: int,
    *,
    required_samples: int,
    maximum_ready_error_rad: float,
) -> tuple[list[str], float]:
    violations = []
    if set(arm_positions) != set(READY_JOINTS):
        violations.append("initial arm state incomplete")
        ready_error = math.inf
    else:
        try:
            ready_error = max(
                abs(float(arm_positions[name]) - expected)
                for name, expected in READY_JOINTS.items()
            )
        except (TypeError, ValueError):
            ready_error = math.inf
        if not math.isfinite(ready_error):
            violations.append("initial arm state is non-finite")
        elif ready_error > maximum_ready_error_rad:
            violations.append("initial arm state is not ready")
    if not all(
        isinstance(finger_positions.get(name), (int, float))
        and math.isfinite(float(finger_positions[name]))
        for name in FINGER_JOINTS
    ):
        violations.append("initial gripper state incomplete")
    if target_attached is not False:
        violations.append("target is not confirmed detached")
    if complete_sample_count < required_samples:
        violations.append("insufficient complete joint samples")
    return violations, ready_error


def main() -> int:  # pragma: no cover - exercised in ROS integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--target-id", type=int, default=1)
    parser.add_argument("--required-samples", type=int, default=10)
    parser.add_argument("--maximum-ready-error-rad", type=float, default=0.02)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    if (
        args.target_id <= 0
        or args.required_samples <= 0
        or args.maximum_ready_error_rad <= 0.0
        or args.timeout_sec <= 0.0
    ):
        raise SystemExit("preflight parameters must be positive")

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init()
    node = Node(
        "blender_v2_execution_initial_state",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    arm_positions = {}
    finger_positions = {}
    complete_sample_count = 0
    target_attached = None
    attachment_events = []

    def on_joint_state(message: JointState) -> None:
        nonlocal complete_sample_count
        positions = dict(zip(message.name, message.position, strict=False))
        if all(name in positions for name in READY_JOINTS):
            complete_sample_count += 1
            arm_positions.update(
                {
                    name: float(positions[name])
                    for name in READY_JOINTS
                }
            )
        for name in FINGER_JOINTS:
            if name in positions:
                finger_positions[name] = float(positions[name])

    def on_attachment(message: String) -> None:
        nonlocal target_attached
        state = str(message.data).strip().lower()
        if state not in {"attached", "detached"}:
            attachment_events.append({"state": state, "attached": None})
            return
        attached = state == "attached"
        if attached is not target_attached:
            target_attached = attached
            attachment_events.append(
                {"state": state, "attached": attached}
            )

    subscriptions = [
        node.create_subscription(
            JointState,
            "/joint_states",
            on_joint_state,
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            String,
            f"/strawberry/sim/fruit_{args.target_id}/attached_state",
            on_attachment,
            qos_profile_sensor_data,
        ),
    ]
    deadline = time.monotonic() + args.timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        violations, ready_error = evaluate_snapshot(
            arm_positions,
            finger_positions,
            target_attached,
            complete_sample_count,
            required_samples=args.required_samples,
            maximum_ready_error_rad=args.maximum_ready_error_rad,
        )
        if not violations:
            break
    else:
        violations, ready_error = evaluate_snapshot(
            arm_positions,
            finger_positions,
            target_attached,
            complete_sample_count,
            required_samples=args.required_samples,
            maximum_ready_error_rad=args.maximum_ready_error_rad,
        )

    result = {
        "schema_version": 1,
        "passed": not violations,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "target_id": args.target_id,
        "required_samples": args.required_samples,
        "complete_joint_sample_count": complete_sample_count,
        "arm_positions_rad": arm_positions,
        "finger_positions_m": finger_positions,
        "target_attached": target_attached,
        "attachment_events": attachment_events,
        "ready_error_rad": ready_error,
        "control_command_count": 0,
        "violations": violations,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True), flush=True)
    subscriptions.clear()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
