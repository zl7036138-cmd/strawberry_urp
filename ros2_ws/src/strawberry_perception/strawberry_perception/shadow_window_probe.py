"""Collect an exact post-setup window from the YOLO Shadow topics."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Mapping, Sequence


def find_consecutive_true_run(
    values: Sequence[bool], required_count: int
) -> tuple[int, int] | None:
    """Return the first half-open run containing the required true values."""

    if required_count <= 0:
        raise ValueError("required consecutive count must be positive")
    run_start = 0
    run_count = 0
    for index, value in enumerate(values):
        if bool(value):
            if run_count == 0:
                run_start = index
            run_count += 1
            if run_count >= required_count:
                return run_start, index + 1
        else:
            run_count = 0
    return None


def summarize_window_frames(
    frames: Sequence[Mapping[str, object]], required_frames: int
) -> dict[str, object]:
    if required_frames <= 0 or len(frames) != required_frames:
        raise ValueError("fixed Shadow window has the wrong frame count")
    detection_confidences = [
        float(value)
        for frame in frames
        for value in frame.get("detection_confidences", [])
    ]
    ripe_confidences = [
        float(value)
        for frame in frames
        for value in frame.get("ripe_confidences", [])
    ]
    unripe_confidences = [
        float(value)
        for frame in frames
        for value in frame.get("unripe_confidences", [])
    ]
    roi_areas = [
        int(roi["width"]) * int(roi["height"])
        for frame in frames
        for roi in frame.get("detection_rois", [])
    ]

    def confidence_summary(values: list[float]) -> dict[str, float | int | None]:
        return {
            "count": len(values),
            "mean": statistics.fmean(values) if values else None,
            "median": statistics.median(values) if values else None,
            "minimum": min(values) if values else None,
            "maximum": max(values) if values else None,
        }

    return {
        "frame_count": len(frames),
        "frames_with_any_detection": sum(
            int(frame["detection_count"]) > 0 for frame in frames
        ),
        "frames_with_ripe_detection": sum(
            int(frame["ripe_detection_count"]) > 0 for frame in frames
        ),
        "frames_with_unripe_detection": sum(
            int(frame["unripe_detection_count"]) > 0 for frame in frames
        ),
        "frames_with_target_pose": sum(
            bool(frame["target_pose_received"]) for frame in frames
        ),
        "detection_count": sum(int(frame["detection_count"]) for frame in frames),
        "ripe_detection_count": sum(
            int(frame["ripe_detection_count"]) for frame in frames
        ),
        "unripe_detection_count": sum(
            int(frame["unripe_detection_count"]) for frame in frames
        ),
        "confidence": {
            "all": confidence_summary(detection_confidences),
            "ripe": confidence_summary(ripe_confidences),
            "unripe": confidence_summary(unripe_confidences),
        },
        "roi": {
            "count": len(roi_areas),
            "mean_area_px": statistics.fmean(roi_areas) if roi_areas else None,
            "median_area_px": statistics.median(roi_areas) if roi_areas else None,
            "minimum_area_px": min(roi_areas) if roi_areas else None,
            "maximum_area_px": max(roi_areas) if roi_areas else None,
        },
    }


def _stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def main(args=None) -> int:  # pragma: no cover - exercised by ROS integration
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.utilities import remove_ros_args
        from strawberry_interfaces.msg import StrawberryDetectionArray, TargetPose
    except ImportError as error:
        raise RuntimeError(
            "shadow_window_probe requires the built ROS workspace"
        ) from error

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--lighting", required=True)
    parser.add_argument("--occlusion", required=True)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--warmup-frames", type=int, default=0)
    parser.add_argument(
        "--ready-consecutive-target-pose-frames",
        type=int,
        default=0,
        help=(
            "Before warmup and measurement, require this many consecutive "
            "detection stamps to receive TargetPose; zero disables the gate."
        ),
    )
    parser.add_argument(
        "--window-boundary",
        default="after_condition_probe_before_robot_motion",
    )
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--post-window-wait-sec", type=float, default=1.0)
    application_args = remove_ros_args(args=args if args is not None else sys.argv)
    options = parser.parse_args(application_args[1:])
    if options.frames <= 0 or options.timeout_sec <= 0.0:
        raise ValueError("frames and timeout must be positive")
    if options.warmup_frames < 0:
        raise ValueError("warmup frames cannot be negative")
    if options.ready_consecutive_target_pose_frames < 0:
        raise ValueError("readiness frame count cannot be negative")
    if options.post_window_wait_sec < 0.0:
        raise ValueError("post-window wait cannot be negative")
    if not options.window_boundary.strip():
        raise ValueError("window boundary cannot be empty")
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite Shadow window: {options.output_json}")

    class WindowNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_shadow_window_probe")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.detections = []
            self.target_ids_by_stamp: dict[tuple[int, int], set[int]] = {}
            self.create_subscription(
                StrawberryDetectionArray,
                "/strawberry/shadow/detections",
                self.detections.append,
                10,
            )
            self.create_subscription(
                TargetPose,
                "/strawberry/shadow/target_pose",
                self._on_target,
                10,
            )

        def _on_target(self, message) -> None:
            stamp = _stamp_key(message.header.stamp)
            self.target_ids_by_stamp.setdefault(stamp, set()).add(
                int(message.target_id)
            )

        def spin_until(self, predicate, timeout_sec: float, description: str) -> None:
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and time.monotonic() < deadline:
                if predicate():
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            if not predicate():
                raise RuntimeError(f"timed out waiting for {description}")

        def spin_for(self, duration_sec: float) -> None:
            deadline = time.monotonic() + duration_sec
            while rclpy.ok() and time.monotonic() < deadline:
                timeout = min(0.05, deadline - time.monotonic())
                rclpy.spin_once(self, timeout_sec=timeout)

        def target_pose_readiness_run(
            self, required_count: int
        ) -> tuple[int, int] | None:
            statuses = [
                bool(self.target_ids_by_stamp.get(_stamp_key(message.header.stamp)))
                for message in self.detections
            ]
            return find_consecutive_true_run(statuses, required_count)

    rclpy.init(args=args)
    node = WindowNode()
    try:
        readiness_run = None
        measurement_start = 0
        required_ready = options.ready_consecutive_target_pose_frames
        if required_ready:
            node.spin_until(
                lambda: node.target_pose_readiness_run(required_ready) is not None,
                options.timeout_sec,
                (
                    f"{required_ready} consecutive detection frames with "
                    "matching TargetPose"
                ),
            )
            readiness_run = node.target_pose_readiness_run(required_ready)
            measurement_start = len(node.detections)

        total_frames = (
            measurement_start + options.warmup_frames + options.frames
        )
        node.spin_until(
            lambda: len(node.detections) >= total_frames,
            options.timeout_sec,
            (
                f"{options.warmup_frames} post-readiness warmup and "
                f"{options.frames} measurement Shadow frames"
            ),
        )
        selected_start = measurement_start + options.warmup_frames
        selected_end = selected_start + options.frames
        selected = list(node.detections[selected_start:selected_end])
        node.spin_for(options.post_window_wait_sec)
        frame_records = []
        for index, message in enumerate(selected, start=1):
            stamp = _stamp_key(message.header.stamp)
            ripe = sum(
                int(item.maturity) == int(item.RIPE)
                for item in message.detections
            )
            unripe = sum(
                int(item.maturity) == int(item.UNRIPE) for item in message.detections
            )
            detection_confidences = [
                float(item.confidence) for item in message.detections
            ]
            ripe_confidences = [
                float(item.confidence)
                for item in message.detections
                if int(item.maturity) == int(item.RIPE)
            ]
            unripe_confidences = [
                float(item.confidence)
                for item in message.detections
                if int(item.maturity) == int(item.UNRIPE)
            ]
            frame_records.append(
                {
                    "frame_index": index,
                    "stamp_sec": stamp[0],
                    "stamp_nanosec": stamp[1],
                    "detection_count": len(message.detections),
                    "ripe_detection_count": ripe,
                    "unripe_detection_count": unripe,
                    "detection_confidences": detection_confidences,
                    "ripe_confidences": ripe_confidences,
                    "unripe_confidences": unripe_confidences,
                    "detection_rois": [
                        {
                            "target_id": int(item.target_id),
                            "maturity": int(item.maturity),
                            "x_offset": int(item.bbox.x_offset),
                            "y_offset": int(item.bbox.y_offset),
                            "width": int(item.bbox.width),
                            "height": int(item.bbox.height),
                            "area_px": int(item.bbox.width) * int(item.bbox.height),
                        }
                        for item in message.detections
                    ],
                    "target_pose_received": bool(node.target_ids_by_stamp.get(stamp)),
                    "target_pose_ids": sorted(node.target_ids_by_stamp.get(stamp, set())),
                }
            )
        summary = summarize_window_frames(frame_records, options.frames)
        result = {
            "schema_version": 1,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "scope": "NON_ACCEPTANCE_POST_SETUP_SHADOW_WINDOW",
            "formal_acceptance": False,
            "held_out_test_consumed": False,
            "scenario_id": options.scenario_id,
            "lighting": options.lighting,
            "occlusion": options.occlusion,
            "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
            "window_boundary": options.window_boundary,
            "warmup_frames_discarded": options.warmup_frames,
            "readiness_gate": {
                "enabled": bool(required_ready),
                "required_consecutive_target_pose_frames": required_ready,
                "satisfied": readiness_run is not None if required_ready else True,
                "qualifying_run_start_detection_index": (
                    readiness_run[0] + 1 if readiness_run is not None else None
                ),
                "qualifying_run_end_detection_index": (
                    readiness_run[1] if readiness_run is not None else None
                ),
                "measurement_started_after_detection_index": measurement_start,
            },
            "required_frames": options.frames,
            "summary": summary,
            "frames": frame_records,
        }
        options.output_json.parent.mkdir(parents=True, exist_ok=True)
        options.output_json.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(summary, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
