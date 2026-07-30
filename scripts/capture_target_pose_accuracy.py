#!/usr/bin/env python3
"""Compare a bounded live TargetPose window with one simulator truth stream."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time


def percentile(values: list[float], fraction: float) -> float:
    """Return a deterministic nearest-rank percentile."""

    if not values:
        raise ValueError("percentile requires at least one value")
    if not 0.0 < fraction <= 1.0:
        raise ValueError("percentile fraction must be in (0, 1]")
    ordered = sorted(float(value) for value in values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def summarize_errors(errors_m: list[float]) -> dict[str, float | int]:
    if not errors_m:
        raise ValueError("error summary requires at least one sample")
    errors_mm = [1000.0 * float(value) for value in errors_m]
    return {
        "sample_count": len(errors_mm),
        "minimum_mm": min(errors_mm),
        "median_mm": statistics.median(errors_mm),
        "mean_mm": statistics.fmean(errors_mm),
        "p95_mm": percentile(errors_mm, 0.95),
        "maximum_mm": max(errors_mm),
    }


def main() -> int:  # pragma: no cover - exercised in ROS integration
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from strawberry_interfaces.msg import TargetPose

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--target-topic", default="/strawberry/shadow/target_pose"
    )
    parser.add_argument(
        "--truth-topic", default="/strawberry/ground_truth/fruit_1/pose"
    )
    parser.add_argument("--expected-target-id", type=int, default=1)
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--median-error-mm-max", type=float, default=15.0)
    parser.add_argument("--p95-error-mm-max", type=float, default=30.0)
    options = parser.parse_args()
    if options.output.exists():
        raise SystemExit(f"refusing to overwrite {options.output}")
    if options.expected_target_id <= 0 or options.samples <= 0:
        raise SystemExit("target ID and sample count must be positive")
    if options.timeout_sec <= 0.0:
        raise SystemExit("timeout must be positive")
    if options.median_error_mm_max <= 0.0 or options.p95_error_mm_max <= 0.0:
        raise SystemExit("accuracy bounds must be positive")

    rclpy.init()
    node = Node("strawberry_target_pose_accuracy")
    latest_truth = None
    errors_m: list[float] = []
    records = []
    observed_target_ids: set[int] = set()

    def on_truth(message) -> None:
        nonlocal latest_truth
        if message.header.frame_id != "panda_link0":
            return
        position = message.pose.position
        latest_truth = (
            float(position.x),
            float(position.y),
            float(position.z),
        )

    def on_target(message) -> None:
        observed_target_ids.add(int(message.target_id))
        if (
            int(message.target_id) != options.expected_target_id
            or latest_truth is None
            or message.header.frame_id != "panda_link0"
            or len(errors_m) >= options.samples
        ):
            return
        position = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        )
        error = math.dist(position, latest_truth)
        errors_m.append(error)
        records.append(
            {
                "target_xyz_m": list(position),
                "truth_xyz_m": list(latest_truth),
                "error_mm": 1000.0 * error,
                "detection_confidence": float(message.detection_confidence),
                "position_sigma_m": float(message.position_sigma_m),
            }
        )

    node.create_subscription(
        PoseStamped,
        options.truth_topic,
        on_truth,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        TargetPose,
        options.target_topic,
        on_target,
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + options.timeout_sec
    try:
        while len(errors_m) < options.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(errors_m) < options.samples:
            raise RuntimeError(
                f"captured {len(errors_m)}/{options.samples} target samples"
            )
        summary = summarize_errors(errors_m)
        violations = []
        if float(summary["median_mm"]) > options.median_error_mm_max:
            violations.append("median localization error exceeds bound")
        if float(summary["p95_mm"]) > options.p95_error_mm_max:
            violations.append("p95 localization error exceeds bound")
        payload = {
            "schema_version": 1,
            "scope": "NON_FORMAL_NO_MOTION_TARGET_POSE_ACCURACY",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "expected_target_id": options.expected_target_id,
            "observed_target_ids": sorted(observed_target_ids),
            "target_topic": options.target_topic,
            "truth_topic": options.truth_topic,
            "accuracy_bounds_mm": {
                "median_max": options.median_error_mm_max,
                "p95_max": options.p95_error_mm_max,
            },
            "error": summary,
            "control_commands_sent": 0,
            "pick_action_started": False,
            "passed": not violations,
            "violations": violations,
            "samples": records,
        }
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, sort_keys=True))
        return 0 if not violations else 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
