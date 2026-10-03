#!/usr/bin/env python3
"""Capture synchronized RGB-D, detection, TF, and truth localization samples."""

from __future__ import annotations

import argparse
from collections import OrderedDict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np


def _stamp_key(message) -> tuple[int, int]:
    stamp = message.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


def _trim(values: OrderedDict, capacity: int = 90) -> None:
    while len(values) > capacity:
        values.popitem(last=False)


def _select_detection(
    message,
    *,
    confidence_threshold: float,
    selection_roi_xyxy: tuple[int, int, int, int],
):
    x_min, y_min, x_max, y_max = selection_roi_xyxy
    candidates = []
    for item in message.detections:
        center_x = float(item.bbox.x_offset) + 0.5 * float(item.bbox.width)
        center_y = float(item.bbox.y_offset) + 0.5 * float(item.bbox.height)
        if (
            int(item.maturity) == int(item.RIPE)
            and math.isfinite(float(item.confidence))
            and float(item.confidence) >= confidence_threshold
            and x_min <= center_x < x_max
            and y_min <= center_y < y_max
        ):
            candidates.append(item)
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda item: (-float(item.confidence), int(item.target_id)),
    )


def _fingerprint(path: Path) -> dict[str, object]:
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def main() -> int:  # pragma: no cover - exercised in ROS integration
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseArray
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from strawberry_interfaces.msg import StrawberryDetectionArray
    from tf2_ros import Buffer, TransformListener

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-npz", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--rgb-topic", default="/camera/wrist/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/wrist/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/wrist/camera_info")
    parser.add_argument(
        "--detections-topic", default="/strawberry/shadow/detections"
    )
    parser.add_argument("--target-frame", default="panda_link0")
    parser.add_argument("--expected-target-id", type=int, required=True)
    parser.add_argument("--roi", type=int, nargs=4, required=True)
    parser.add_argument("--confidence-threshold", type=float, default=0.58)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    options = parser.parse_args()

    if options.output_npz.exists() or options.output_json.exists():
        raise SystemExit("refusing to overwrite an RGB-D diagnostic output")
    if options.samples <= 0 or options.timeout_sec <= 0.0:
        raise SystemExit("sample count and timeout must be positive")
    if options.expected_target_id <= 0:
        raise SystemExit("expected target ID must be positive")
    if not 0.0 <= options.confidence_threshold <= 1.0:
        raise SystemExit("confidence threshold must be in [0, 1]")
    roi = tuple(int(value) for value in options.roi)
    if min(roi) < 0 or roi[2] <= roi[0] or roi[3] <= roi[1]:
        raise SystemExit("ROI must be a positive-area x_min y_min x_max y_max box")

    rclpy.init()
    node = Node("strawberry_rgbd_localization_capture")
    bridge = CvBridge()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    del tf_listener

    rgb_messages: OrderedDict[tuple[int, int], object] = OrderedDict()
    depth_messages: OrderedDict[tuple[int, int], object] = OrderedDict()
    detection_messages: OrderedDict[tuple[int, int], object] = OrderedDict()
    camera_infos: dict[str, object] = {}
    truth_order: list[int] = []
    truth_points: dict[int, tuple[float, float, float]] = {}

    def on_rgb(message) -> None:
        rgb_messages[_stamp_key(message)] = message
        _trim(rgb_messages)

    def on_depth(message) -> None:
        depth_messages[_stamp_key(message)] = message
        _trim(depth_messages)

    def on_detections(message) -> None:
        detection_messages[_stamp_key(message)] = message
        _trim(detection_messages)

    def on_camera_info(message) -> None:
        camera_infos[str(message.header.frame_id)] = message

    def on_catalog(message) -> None:
        nonlocal truth_order
        try:
            payload = json.loads(message.data)
            order = [int(value) for value in payload["pose_array_order"]]
            if int(payload["schema_version"]) != 1 or not order:
                raise ValueError("invalid catalog")
            truth_order = order
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            truth_order = []

    def on_truth(message) -> None:
        nonlocal truth_points
        if (
            str(message.header.frame_id) != options.target_frame
            or len(message.poses) != len(truth_order)
        ):
            return
        truth_points = {
            target_id: (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            )
            for target_id, pose in zip(truth_order, message.poses)
        }

    sensor_qos = QoSProfile(depth=30)
    sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    sensor_qos.durability = DurabilityPolicy.VOLATILE
    node.create_subscription(Image, options.rgb_topic, on_rgb, sensor_qos)
    node.create_subscription(Image, options.depth_topic, on_depth, sensor_qos)
    node.create_subscription(
        CameraInfo, options.camera_info_topic, on_camera_info, sensor_qos
    )
    node.create_subscription(
        StrawberryDetectionArray,
        options.detections_topic,
        on_detections,
        30,
    )
    catalog_qos = QoSProfile(depth=1)
    catalog_qos.reliability = ReliabilityPolicy.RELIABLE
    catalog_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(
        String,
        "/strawberry/ground_truth/catalog",
        on_catalog,
        catalog_qos,
    )
    node.create_subscription(
        PoseArray,
        "/strawberry/ground_truth/poses",
        on_truth,
        qos_profile_sensor_data,
    )

    captured_keys: set[tuple[int, int]] = set()
    colors = []
    depths = []
    boxes = []
    confidences = []
    intrinsics = []
    translations = []
    rotations = []
    truths = []
    stamps = []
    deadline = time.monotonic() + options.timeout_sec
    try:
        while len(colors) < options.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            common = (
                rgb_messages.keys()
                & depth_messages.keys()
                & detection_messages.keys()
            )
            pending = sorted(key for key in common if key not in captured_keys)
            for key in pending:
                detection_message = detection_messages[key]
                detection = _select_detection(
                    detection_message,
                    confidence_threshold=options.confidence_threshold,
                    selection_roi_xyxy=roi,
                )
                if detection is None:
                    captured_keys.add(key)
                    continue
                frame_id = str(detection_message.header.frame_id)
                info = camera_infos.get(frame_id)
                truth = truth_points.get(options.expected_target_id)
                if info is None or truth is None:
                    continue
                try:
                    transform = tf_buffer.lookup_transform(
                        options.target_frame,
                        frame_id,
                        Time.from_msg(detection_message.header.stamp),
                        timeout=Duration(seconds=0.2),
                    )
                except Exception:
                    try:
                        transform = tf_buffer.lookup_transform(
                            options.target_frame,
                            frame_id,
                            Time(),
                            timeout=Duration(seconds=0.2),
                        )
                    except Exception:
                        continue
                rgb = np.asarray(
                    bridge.imgmsg_to_cv2(
                        rgb_messages[key], desired_encoding="bgr8"
                    ),
                    dtype=np.uint8,
                )
                depth = np.asarray(
                    bridge.imgmsg_to_cv2(
                        depth_messages[key], desired_encoding="32FC1"
                    ),
                    dtype=np.float32,
                )
                if rgb.shape[:2] != depth.shape or depth.shape != (
                    int(info.height),
                    int(info.width),
                ):
                    captured_keys.add(key)
                    continue
                value = transform.transform
                colors.append(rgb.copy())
                depths.append(depth.copy())
                boxes.append(
                    [
                        int(detection.bbox.x_offset),
                        int(detection.bbox.y_offset),
                        int(detection.bbox.width),
                        int(detection.bbox.height),
                    ]
                )
                confidences.append(float(detection.confidence))
                intrinsics.append(
                    [
                        float(info.k[0]),
                        float(info.k[4]),
                        float(info.k[2]),
                        float(info.k[5]),
                    ]
                )
                translations.append(
                    [
                        float(value.translation.x),
                        float(value.translation.y),
                        float(value.translation.z),
                    ]
                )
                rotations.append(
                    [
                        float(value.rotation.x),
                        float(value.rotation.y),
                        float(value.rotation.z),
                        float(value.rotation.w),
                    ]
                )
                truths.append(list(truth))
                stamps.append(list(key))
                captured_keys.add(key)
                if len(colors) >= options.samples:
                    break
        if len(colors) < options.samples:
            raise RuntimeError(
                f"captured {len(colors)}/{options.samples} synchronized samples"
            )

        options.output_npz.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            options.output_npz,
            color_bgr=np.stack(colors),
            depth_m=np.stack(depths),
            roi_xywh=np.asarray(boxes, dtype=np.int32),
            confidence=np.asarray(confidences, dtype=np.float32),
            intrinsics_fx_fy_cx_cy=np.asarray(intrinsics, dtype=np.float64),
            base_from_camera_translation_m=np.asarray(
                translations, dtype=np.float64
            ),
            base_from_camera_rotation_xyzw=np.asarray(rotations, dtype=np.float64),
            truth_target_xyz_m=np.asarray(truths, dtype=np.float64),
            stamp_sec_nanosec=np.asarray(stamps, dtype=np.int64),
        )
        payload = {
            "schema_version": 1,
            "scope": "NATURAL_PLANT_WRIST_RGBD_LOCALIZATION_DIAGNOSTIC",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "sample_count": len(colors),
            "expected_target_id": options.expected_target_id,
            "selection_roi_xyxy_px": list(roi),
            "confidence_threshold": options.confidence_threshold,
            "topics": {
                "rgb": options.rgb_topic,
                "depth": options.depth_topic,
                "camera_info": options.camera_info_topic,
                "detections": options.detections_topic,
            },
            "control_commands_sent": 0,
            "pick_action_started": False,
            "truth_use": "OFFLINE_LOCALIZATION_DIAGNOSTIC_ONLY",
            "dataset": _fingerprint(options.output_npz),
        }
        options.output_json.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
