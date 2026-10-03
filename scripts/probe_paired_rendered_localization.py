#!/usr/bin/env python3
"""Probe two localization nodes on identical rendered RGB-D frames."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "strawberry_localization"
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import (  # noqa: E402
    CameraIntrinsics,
)
from strawberry_localization.gate_core import (  # noqa: E402
    sphere_projection_bbox,
)


def _stamp_key(message: Any) -> tuple[int, int]:
    stamp = message.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _fingerprint(path: Path) -> dict[str, Any]:
    path = path.resolve(strict=True)
    try:
        label = path.relative_to(ROOT).as_posix()
    except ValueError:
        label = path.as_posix()
    return {
        "path": label,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _nearest_rank(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def summarize_mode(records: list[dict[str, Any]], mode: str) -> dict[str, Any]:
    """Summarize one estimator while preserving missing-pose observations."""

    values = [record[mode] for record in records]
    estimated = [value for value in values if value["status"] == "ESTIMATED"]
    errors = [float(value["error_mm"]) for value in estimated]
    sigmas = [float(value["position_sigma_m"]) for value in estimated]
    return {
        "requested_samples": len(values),
        "estimated_samples": len(estimated),
        "no_pose_samples": len(values) - len(estimated),
        "acceptance_rate": len(estimated) / len(values) if values else 0.0,
        "median_error_mm": statistics.median(errors) if errors else None,
        "p95_error_mm": _nearest_rank(errors, 0.95),
        "maximum_error_mm": max(errors) if errors else None,
        "median_sigma_m": statistics.median(sigmas) if sigmas else None,
        "p95_sigma_m": _nearest_rank(sigmas, 0.95),
        "sigma_under_15mm_rate": (
            sum(value <= 0.015 for value in sigmas) / len(sigmas)
            if sigmas
            else 0.0
        ),
    }


def _write_json_lf(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n"
        )


def main() -> int:  # pragma: no cover - exercised in ROS/Gazebo integration
    import rclpy
    import tf2_geometry_msgs  # noqa: F401 - register stamped-pose conversion
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from strawberry_interfaces.msg import (
        StrawberryDetection,
        StrawberryDetectionArray,
        TargetPose,
    )
    from tf2_ros import Buffer, TransformListener

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--condition-receipt", type=Path, required=True)
    parser.add_argument("--condition-probe", type=Path, required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--position-label", required=True)
    parser.add_argument("--occlusion", choices=("none", "partial", "heavy"), required=True)
    parser.add_argument("--detections-topic", default="/strawberry/rendered/detections")
    parser.add_argument("--center-topic", default="/strawberry/rendered/center_target_pose")
    parser.add_argument("--geometry-topic", default="/strawberry/rendered/geometry_target_pose")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--truth-topic", default="/strawberry/ground_truth/fruit_1/pose")
    parser.add_argument("--target-frame", default="panda_link0")
    parser.add_argument("--target-id", type=int, default=1)
    parser.add_argument("--target-radius-m", type=float, default=0.026)
    parser.add_argument("--bbox-padding", type=float, default=1.05)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--pair-wait-sec", type=float, default=0.50)
    options = parser.parse_args()

    output = options.output.resolve()
    if output.exists():
        raise SystemExit(f"refusing to overwrite {output}")
    if options.target_id <= 0 or options.samples <= 0:
        raise SystemExit("target ID and sample count must be positive")
    if options.target_radius_m <= 0.0:
        raise SystemExit("target radius must be positive")
    if not math.isfinite(options.bbox_padding) or options.bbox_padding < 1.0:
        raise SystemExit("bounding-box padding must be finite and at least one")
    if options.timeout_sec <= 0.0 or options.pair_wait_sec <= 0.0:
        raise SystemExit("timeouts must be positive")

    condition_receipt_path = options.condition_receipt.resolve(strict=True)
    condition_probe_path = options.condition_probe.resolve(strict=True)
    condition_receipt = json.loads(
        condition_receipt_path.read_text(encoding="utf-8")
    )
    condition_probe = json.loads(condition_probe_path.read_text(encoding="utf-8"))
    if condition_receipt.get("occlusion_level") != options.occlusion:
        raise SystemExit("condition receipt occlusion differs from the scenario")
    if condition_probe.get("occlusion_level") != options.occlusion:
        raise SystemExit("condition probe occlusion differs from the scenario")
    if condition_probe.get("formal_acceptance") is not False:
        raise SystemExit("condition probe violates the non-formal boundary")

    rclpy.init()
    node = Node("paired_rendered_localization_probe")
    node.set_parameters([Parameter("use_sim_time", value=True)])
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    del tf_listener
    camera_info = None
    truth = None
    depth_sequence = 0
    latest_depth = None
    targets: dict[str, dict[tuple[int, int], Any]] = {
        "center_median": {},
        "geometry_layer": {},
    }

    def on_camera_info(message: CameraInfo) -> None:
        nonlocal camera_info
        camera_info = message

    def on_truth(message: PoseStamped) -> None:
        nonlocal truth
        if str(message.header.frame_id) == options.target_frame:
            truth = message

    def on_depth(message: Image) -> None:
        nonlocal depth_sequence, latest_depth
        depth_sequence += 1
        latest_depth = message

    def on_target(mode: str, message: TargetPose) -> None:
        cache = targets[mode]
        cache[_stamp_key(message)] = message
        if len(cache) > 200:
            for key in sorted(cache)[:100]:
                del cache[key]

    node.create_subscription(
        CameraInfo,
        options.camera_info_topic,
        on_camera_info,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Image,
        options.depth_topic,
        on_depth,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        PoseStamped,
        options.truth_topic,
        on_truth,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        TargetPose,
        options.center_topic,
        lambda message: on_target("center_median", message),
        30,
    )
    node.create_subscription(
        TargetPose,
        options.geometry_topic,
        lambda message: on_target("geometry_layer", message),
        30,
    )
    publisher = node.create_publisher(
        StrawberryDetectionArray,
        options.detections_topic,
        30,
    )

    def wait_for(predicate, timeout_sec: float, description: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=0.05)
        if not predicate():
            raise RuntimeError(f"timed out waiting for {description}")

    def transform_is_ready() -> bool:
        if latest_depth is None:
            return False
        return tf_buffer.can_transform(
            str(latest_depth.header.frame_id),
            options.target_frame,
            Time(),
            timeout=Duration(seconds=0.0),
        )

    records = []
    deadline = time.monotonic() + options.timeout_sec
    previous_depth_sequence = 0
    try:
        wait_for(
            lambda: (
                camera_info is not None
                and truth is not None
                and latest_depth is not None
                and publisher.get_subscription_count() >= 2
                and node.count_subscribers(options.depth_topic) >= 3
                and node.count_subscribers(options.camera_info_topic) >= 3
                and transform_is_ready()
            ),
            min(30.0, options.timeout_sec),
            (
                "camera, truth, depth, two localization subscribers, and "
                "both localizers' depth/CameraInfo subscriptions, and TF"
            ),
        )
        for sample_index in range(1, options.samples + 1):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    f"captured {len(records)}/{options.samples} paired frames"
                )
            wait_for(
                lambda: depth_sequence > previous_depth_sequence,
                min(remaining, 5.0),
                "a fresh depth frame",
            )
            depth_message = latest_depth
            selected_sequence = depth_sequence
            key = _stamp_key(depth_message)
            info = camera_info
            truth_message = truth

            stamped_truth = PoseStamped()
            stamped_truth.header = truth_message.header
            stamped_truth.header.stamp = depth_message.header.stamp
            stamped_truth.pose = truth_message.pose
            camera_truth = tf_buffer.transform(
                stamped_truth,
                str(depth_message.header.frame_id),
                timeout=Duration(seconds=0.5),
            )
            camera_position = camera_truth.pose.position
            intrinsics = CameraIntrinsics(
                float(info.k[0]),
                float(info.k[4]),
                float(info.k[2]),
                float(info.k[5]),
            )
            box = sphere_projection_bbox(
                (
                    float(camera_position.x),
                    float(camera_position.y),
                    float(camera_position.z),
                ),
                intrinsics,
                sphere_radius_m=options.target_radius_m,
                image_width=int(info.width),
                image_height=int(info.height),
                padding=options.bbox_padding,
            )
            detections = StrawberryDetectionArray()
            detections.header = depth_message.header
            detection = StrawberryDetection()
            detection.target_id = options.target_id
            detection.maturity = StrawberryDetection.RIPE
            detection.confidence = 1.0
            detection.bbox.x_offset = box.x
            detection.bbox.y_offset = box.y
            detection.bbox.width = box.width
            detection.bbox.height = box.height
            detections.detections.append(detection)
            publisher.publish(detections)

            pair_deadline = min(
                deadline,
                time.monotonic() + options.pair_wait_sec,
            )
            while rclpy.ok() and time.monotonic() < pair_deadline:
                if all(key in cache for cache in targets.values()):
                    break
                rclpy.spin_once(node, timeout_sec=0.02)

            truth_xyz = (
                float(truth_message.pose.position.x),
                float(truth_message.pose.position.y),
                float(truth_message.pose.position.z),
            )
            record: dict[str, Any] = {
                "sample_index": sample_index,
                "stamp_sec_nanosec": list(key),
                "bbox_xywh": [box.x, box.y, box.width, box.height],
                "truth_xyz_m": list(truth_xyz),
            }
            for mode in ("center_median", "geometry_layer"):
                message = targets[mode].pop(key, None)
                if message is None:
                    record[mode] = {"status": "NO_POSE"}
                    continue
                point = message.pose.position
                estimated_xyz = (
                    float(point.x),
                    float(point.y),
                    float(point.z),
                )
                record[mode] = {
                    "status": "ESTIMATED",
                    "target_id": int(message.target_id),
                    "estimated_xyz_m": list(estimated_xyz),
                    "error_mm": 1000.0 * math.dist(estimated_xyz, truth_xyz),
                    "position_sigma_m": float(message.position_sigma_m),
                }
            records.append(record)
            previous_depth_sequence = selected_sequence

        coverage = [
            float(frame["blue_coverage_ratio"])
            for frame in condition_probe.get("frames", [])
        ]
        payload = {
            "schema_version": 1,
            "kind": "paired_rendered_localization_trial",
            "scenario_id": options.scenario_id,
            "position_label": options.position_label,
            "occlusion": options.occlusion,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "completed": len(records) == options.samples,
            "sample_count": len(records),
            "target_id": options.target_id,
            "target_radius_m": options.target_radius_m,
            "oracle_bbox_padding": options.bbox_padding,
            "oracle_bbox_from_truth": True,
            "ground_truth_association_in_localizers": False,
            "summaries": {
                mode: summarize_mode(records, mode)
                for mode in ("center_median", "geometry_layer")
            },
            "rendered_occlusion": {
                "level": options.occlusion,
                "probe_frame_count": len(coverage),
                "mean_blue_coverage_ratio": (
                    statistics.fmean(coverage) if coverage else None
                ),
            },
            "records": records,
            "bindings": {
                "condition_receipt": _fingerprint(condition_receipt_path),
                "condition_probe": _fingerprint(condition_probe_path),
                "probe_script": _fingerprint(Path(__file__)),
            },
            "safety": {
                "robot_motion_started": False,
                "manipulation_started": False,
                "orchestrator_started": False,
                "attachment_enabled": False,
                "perception_model_started": False,
                "control_commands_sent": 0,
                "simulated_fruit_pose_motion_precedes_measurement": True,
            },
            "scope": {
                "simulator_only": True,
                "rendered_visual_occlusion": True,
                "localization_isolation": True,
                "formal_p3_or_p4_evidence": False,
                "runtime_promotion_authorized": False,
                "held_out_real_test_consumed": False,
            },
        }
        _write_json_lf(output, payload)
        print(json.dumps(payload["summaries"], sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
