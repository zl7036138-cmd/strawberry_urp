#!/usr/bin/env python3
"""Record one read-only generalized RGB-D localization diagnostic frame.

The runtime localizer deliberately has no access to simulator truth.  This
development-only tool reproduces the same depth gate for every detector box,
then uses the truth topics only to score accepted estimates and to report which
truth centres were covered by detector boxes.  It never publishes commands or
poses.
"""

from __future__ import annotations

import argparse
from collections import OrderedDict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
LOCALIZATION_SOURCE = REPO_ROOT / "ros2_ws" / "src" / "strawberry_localization"
if str(LOCALIZATION_SOURCE) not in sys.path:
    sys.path.insert(0, str(LOCALIZATION_SOURCE))

from strawberry_localization.core import (  # noqa: E402
    BoundingBox,
    CameraIntrinsics,
    LocalizationError,
    localize_bbox,
)
from strawberry_localization.generalized_depth import (  # noqa: E402
    adjust_point_along_optical_ray,
    calibrate_runtime_geometry_uncertainty,
    expand_bounding_box,
    point_on_pixel_bearing,
    retain_foreground_depth_band,
    retain_support_ranked_geometry_layer,
)


def _stamp_s(message: Any) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _trim(values: OrderedDict[float, Any], capacity: int = 90) -> None:
    while len(values) > capacity:
        values.popitem(last=False)


def _rotation_matrix_xyzw(quaternion: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("transform quaternion must have finite non-zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _camera_to_base(point: np.ndarray, transform: Any) -> np.ndarray:
    value = transform.transform
    rotation = _rotation_matrix_xyzw(
        (
            float(value.rotation.x),
            float(value.rotation.y),
            float(value.rotation.z),
            float(value.rotation.w),
        )
    )
    translation = np.asarray(
        [value.translation.x, value.translation.y, value.translation.z],
        dtype=np.float64,
    )
    return rotation @ np.asarray(point, dtype=np.float64) + translation


def _base_to_camera(point: np.ndarray, transform: Any) -> np.ndarray:
    value = transform.transform
    rotation = _rotation_matrix_xyzw(
        (
            float(value.rotation.x),
            float(value.rotation.y),
            float(value.rotation.z),
            float(value.rotation.w),
        )
    )
    translation = np.asarray(
        [value.translation.x, value.translation.y, value.translation.z],
        dtype=np.float64,
    )
    return rotation.T @ (np.asarray(point, dtype=np.float64) - translation)


def _layer_summary(
    depth: np.ndarray,
    box: BoundingBox,
    intrinsics: CameraIntrinsics,
    parameters: dict[str, Any],
) -> dict[str, Any]:
    crop = np.asarray(
        depth[box.y : box.y + box.height, box.x : box.x + box.width],
        dtype=np.float64,
    )
    valid_mask = (
        np.isfinite(crop)
        & (crop >= parameters["min_depth_m"])
        & (crop <= parameters["max_depth_m"])
    )
    valid_y, valid_x = np.nonzero(valid_mask)
    values = crop[valid_mask]
    required = max(
        parameters["min_valid_pixels"],
        int(math.ceil(crop.size * parameters["geometry_min_layer_fraction"])),
    )
    radius_max_px = 0.5 * math.sqrt(float(box.width * box.height))
    quantization_margin = parameters["geometry_bbox_quantization_margin_px"]
    radius_min_px = 0.5 * math.sqrt(
        max(float(box.width) - quantization_margin, 1e-6)
        * max(float(box.height) - quantization_margin, 1e-6)
    )
    focal = math.sqrt(intrinsics.fx * intrinsics.fy)

    def expected_surface(radius_px: float) -> float:
        centre_depth = parameters["geometry_target_radius_m"] * math.sqrt(
            1.0 + (focal / radius_px) ** 2
        )
        return centre_depth - parameters["geometry_target_radius_m"]

    expected_min = expected_surface(radius_max_px)
    expected_max = expected_surface(radius_min_px)
    result: dict[str, Any] = {
        "valid_pixel_count": int(values.size),
        "required_layer_pixels": int(required),
        "expected_surface_interval_m": [expected_min, expected_max],
        "layers": [],
    }
    if values.size == 0:
        return result
    order = np.argsort(values, kind="stable")
    boundaries = (
        np.flatnonzero(np.diff(values[order]) > parameters["geometry_layer_gap_m"]) + 1
    )
    for indices in np.split(order, boundaries):
        if indices.size < required:
            continue
        median = float(np.median(values[indices]))
        error = max(expected_min - median, median - expected_max, 0.0)
        result["layers"].append(
            {
                "median_depth_m": median,
                "pixel_count": int(indices.size),
                "expected_depth_error_m": error,
                "centroid_uv_px": [
                    float(box.x + np.mean(valid_x[indices])),
                    float(box.y + np.mean(valid_y[indices])),
                ],
            }
        )
    result["layers"].sort(
        key=lambda item: (
            item["expected_depth_error_m"],
            -item["pixel_count"],
            item["median_depth_m"],
        )
    )
    return result


def _load_parameters(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    source = payload["/**"]["ros__parameters"]
    names = (
        "min_depth_m",
        "max_depth_m",
        "min_valid_pixels",
        "surface_to_center_offset_m",
        "geometry_search_fraction",
        "geometry_layer_gap_m",
        "geometry_min_layer_fraction",
        "geometry_expected_depth_tolerance_m",
        "geometry_ambiguity_margin_m",
        "geometry_ambiguity_min_support_ratio",
        "geometry_bbox_quantization_margin_px",
        "geometry_size_residual_sigma_weight",
        "geometry_foreground_band_m",
        "geometry_foreground_min_band_fraction",
        "geometry_bbox_padding_px",
        "geometry_target_radius_m",
        "use_bbox_center_bearing",
    )
    result = {name: source[name] for name in names if name in source}
    # The base runtime inherits this node default; the wrist launch overrides
    # it explicitly.  Preserve the base behavior when the YAML omits it.
    result["use_bbox_center_bearing"] = bool(
        source.get("use_bbox_center_bearing", False)
    )
    result["geometry_size_residual_sigma_weight"] = float(
        source.get("geometry_size_residual_sigma_weight", 1.0)
    )
    return result


def _nearest_depth(
    depths: OrderedDict[float, Any], stamp_s: float, tolerance_s: float
) -> tuple[Any, float] | None:
    candidates = [
        (message, sample_stamp - stamp_s)
        for sample_stamp, message in depths.items()
        if abs(sample_stamp - stamp_s) <= tolerance_s
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda item: (abs(item[1]), item[1]))


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
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT
        / "ros2_ws"
        / "src"
        / "strawberry_localization"
        / "config"
        / "localization_generalized.yaml",
    )
    parser.add_argument("--detections-topic", default="/strawberry/base/detections")
    parser.add_argument("--depth-topic", default="/camera/base/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/base/camera_info")
    parser.add_argument("--target-frame", default="panda_link0")
    parser.add_argument("--sync-tolerance-sec", type=float, default=0.05)
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--minimum-detections", type=int, default=1)
    options = parser.parse_args()
    if options.output.exists():
        raise SystemExit("refusing to overwrite an RGB-D diagnostic output")
    if options.timeout_sec <= 0.0 or options.sync_tolerance_sec <= 0.0:
        raise SystemExit("timeouts must be positive")
    if options.minimum_detections <= 0:
        raise SystemExit("minimum detections must be positive")

    parameters = _load_parameters(options.config)
    rclpy.init()
    node = Node("generalized_rgbd_frame_diagnostic")
    bridge = CvBridge()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    del tf_listener
    depths: OrderedDict[float, Any] = OrderedDict()
    detections: OrderedDict[float, Any] = OrderedDict()
    camera_infos: dict[str, Any] = {}
    truth_order: list[int] = []
    truth_points: dict[int, np.ndarray] = {}

    def on_depth(message: Any) -> None:
        depths[_stamp_s(message)] = message
        _trim(depths)

    def on_detections(message: Any) -> None:
        if len(message.detections) >= options.minimum_detections:
            detections[_stamp_s(message)] = message
            _trim(detections)

    def on_camera_info(message: Any) -> None:
        camera_infos[str(message.header.frame_id)] = message

    def on_catalog(message: Any) -> None:
        nonlocal truth_order
        try:
            payload = json.loads(message.data)
            order = [int(value) for value in payload["pose_array_order"]]
            truth_order = order if int(payload["schema_version"]) == 1 else []
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            truth_order = []

    def on_truth(message: Any) -> None:
        nonlocal truth_points
        if str(message.header.frame_id) != options.target_frame or len(
            message.poses
        ) != len(truth_order):
            return
        truth_points = {
            target_id: np.asarray(
                [pose.position.x, pose.position.y, pose.position.z],
                dtype=np.float64,
            )
            for target_id, pose in zip(truth_order, message.poses)
        }

    sensor_qos = QoSProfile(depth=30)
    sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    sensor_qos.durability = DurabilityPolicy.VOLATILE
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
        String, "/strawberry/ground_truth/catalog", on_catalog, catalog_qos
    )
    node.create_subscription(
        PoseArray,
        "/strawberry/ground_truth/poses",
        on_truth,
        qos_profile_sensor_data,
    )

    deadline = time.monotonic() + options.timeout_sec
    selected = None
    transform = None
    try:
        while time.monotonic() < deadline and selected is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not truth_points:
                continue
            for stamp_s, detection_message in reversed(detections.items()):
                matched = _nearest_depth(depths, stamp_s, options.sync_tolerance_sec)
                if matched is None:
                    continue
                frame_id = str(detection_message.header.frame_id)
                camera_info = camera_infos.get(frame_id)
                if camera_info is None:
                    continue
                try:
                    transform = tf_buffer.lookup_transform(
                        options.target_frame,
                        frame_id,
                        Time.from_msg(detection_message.header.stamp),
                        timeout=Duration(seconds=0.2),
                    )
                except Exception:
                    continue
                selected = (
                    stamp_s,
                    detection_message,
                    matched[0],
                    matched[1],
                    camera_info,
                )
                break
        if selected is None or transform is None:
            raise RuntimeError(
                "no synchronized detection/depth/calibration/truth/TF frame arrived"
            )

        stamp_s, detection_message, depth_message, depth_delta_s, camera_info = selected
        depth = np.asarray(
            bridge.imgmsg_to_cv2(depth_message, desired_encoding="32FC1"),
            dtype=np.float32,
        )
        intrinsics = CameraIntrinsics(
            fx=float(camera_info.k[0]),
            fy=float(camera_info.k[4]),
            cx=float(camera_info.k[2]),
            cy=float(camera_info.k[5]),
        )
        detection_reports = []
        for index, item in enumerate(detection_message.detections):
            roi = item.bbox
            raw_box = BoundingBox(
                int(roi.x_offset),
                int(roi.y_offset),
                int(roi.width),
                int(roi.height),
            )
            box = expand_bounding_box(
                raw_box,
                padding_px=int(parameters["geometry_bbox_padding_px"]),
                image_width=depth.shape[1],
                image_height=depth.shape[0],
            )
            report: dict[str, Any] = {
                "detection_index": index,
                "detector_target_id": int(item.target_id),
                "maturity": int(item.maturity),
                "confidence": float(item.confidence),
                "raw_bbox_xywh": [raw_box.x, raw_box.y, raw_box.width, raw_box.height],
                "expanded_bbox_xywh": [box.x, box.y, box.width, box.height],
                "raw_depth_layers": _layer_summary(depth, box, intrinsics, parameters),
            }
            try:
                filtered = retain_foreground_depth_band(
                    depth,
                    box,
                    search_fraction=float(parameters["geometry_search_fraction"]),
                    min_depth_m=float(parameters["min_depth_m"]),
                    max_depth_m=float(parameters["max_depth_m"]),
                    min_layer_pixels=int(parameters["min_valid_pixels"]),
                    min_layer_fraction=float(parameters["geometry_min_layer_fraction"]),
                    layer_gap_m=float(parameters["geometry_layer_gap_m"]),
                    maximum_band_width_m=float(
                        parameters["geometry_foreground_band_m"]
                    ),
                    minimum_band_fraction=float(
                        parameters["geometry_foreground_min_band_fraction"]
                    ),
                )
                filtered = retain_support_ranked_geometry_layer(
                    filtered,
                    box,
                    fx=intrinsics.fx,
                    fy=intrinsics.fy,
                    target_radius_m=float(parameters["geometry_target_radius_m"]),
                    search_fraction=float(parameters["geometry_search_fraction"]),
                    min_depth_m=float(parameters["min_depth_m"]),
                    max_depth_m=float(parameters["max_depth_m"]),
                    min_layer_pixels=int(parameters["min_valid_pixels"]),
                    min_layer_fraction=float(
                        parameters["geometry_min_layer_fraction"]
                    ),
                    layer_gap_m=float(parameters["geometry_layer_gap_m"]),
                    expected_depth_tolerance_m=float(
                        parameters["geometry_expected_depth_tolerance_m"]
                    ),
                    ambiguity_margin_m=float(
                        parameters["geometry_ambiguity_margin_m"]
                    ),
                    ambiguity_min_support_ratio=float(
                        parameters["geometry_ambiguity_min_support_ratio"]
                    ),
                    bbox_quantization_margin_px=float(
                        parameters["geometry_bbox_quantization_margin_px"]
                    ),
                )
                report["filtered_depth_layers"] = _layer_summary(
                    filtered, box, intrinsics, parameters
                )
                point, estimate = localize_bbox(
                    filtered,
                    box,
                    intrinsics,
                    min_depth_m=float(parameters["min_depth_m"]),
                    max_depth_m=float(parameters["max_depth_m"]),
                    min_valid_pixels=int(parameters["min_valid_pixels"]),
                    surface_to_center_offset_m=float(
                        parameters["geometry_target_radius_m"]
                    ),
                    depth_estimator_mode="geometry_layer",
                    geometry_search_fraction=float(
                        parameters["geometry_search_fraction"]
                    ),
                    geometry_layer_gap_m=float(parameters["geometry_layer_gap_m"]),
                    geometry_min_layer_fraction=float(
                        parameters["geometry_min_layer_fraction"]
                    ),
                    geometry_expected_depth_tolerance_m=float(
                        parameters["geometry_expected_depth_tolerance_m"]
                    ),
                    geometry_ambiguity_margin_m=float(
                        parameters["geometry_ambiguity_margin_m"]
                    ),
                    geometry_ambiguity_min_support_ratio=float(
                        parameters["geometry_ambiguity_min_support_ratio"]
                    ),
                    geometry_bbox_quantization_margin_px=float(
                        parameters["geometry_bbox_quantization_margin_px"]
                    ),
                )
                estimate = calibrate_runtime_geometry_uncertainty(
                    estimate,
                    weight=float(
                        parameters["geometry_size_residual_sigma_weight"]
                    ),
                )
                point = adjust_point_along_optical_ray(
                    point,
                    float(parameters["surface_to_center_offset_m"])
                    - float(parameters["geometry_target_radius_m"]),
                )
                if bool(parameters["use_bbox_center_bearing"]):
                    point = point_on_pixel_bearing(
                        point,
                        u=float(raw_box.x) + 0.5 * float(raw_box.width),
                        v=float(raw_box.y) + 0.5 * float(raw_box.height),
                        fx=intrinsics.fx,
                        fy=intrinsics.fy,
                        cx=intrinsics.cx,
                        cy=intrinsics.cy,
                    )
                base_point = _camera_to_base(point, transform)
                target_id, error_m = min(
                    (
                        (target_id, float(np.linalg.norm(base_point - truth)))
                        for target_id, truth in truth_points.items()
                    ),
                    key=lambda pair: (pair[1], pair[0]),
                )
                report.update(
                    {
                        "localization_status": "ACCEPTED",
                        "depth_m": float(estimate.depth_m),
                        "sigma_m": float(estimate.sigma_m),
                        "camera_point_m": point.tolist(),
                        "base_point_m": base_point.tolist(),
                        "nearest_truth_target_id": int(target_id),
                        "nearest_truth_error_m": error_m,
                    }
                )
            except (LocalizationError, TypeError, ValueError) as exc:
                report.update(
                    {
                        "localization_status": "REJECTED",
                        "rejection_reason": str(exc),
                    }
                )
            detection_reports.append(report)

        truth_reports = []
        for target_id, truth in sorted(truth_points.items()):
            camera_point = _base_to_camera(truth, transform)
            if camera_point[2] <= 0.0:
                u = v = None
            else:
                u = intrinsics.fx * camera_point[0] / camera_point[2] + intrinsics.cx
                v = intrinsics.fy * camera_point[1] / camera_point[2] + intrinsics.cy
            containing = []
            if u is not None and v is not None:
                for index, item in enumerate(detection_message.detections):
                    roi = item.bbox
                    if float(roi.x_offset) <= u < float(
                        roi.x_offset + roi.width
                    ) and float(roi.y_offset) <= v < float(roi.y_offset + roi.height):
                        containing.append(index)
            truth_reports.append(
                {
                    "target_id": int(target_id),
                    "base_point_m": truth.tolist(),
                    "camera_point_m": camera_point.tolist(),
                    "projected_center_uv_px": [u, v],
                    "projected_center_in_image": bool(
                        u is not None
                        and 0.0 <= u < float(depth.shape[1])
                        and 0.0 <= v < float(depth.shape[0])
                    ),
                    "containing_detection_indices": containing,
                }
            )

        payload = {
            "schema_version": 1,
            "kind": "generalized_rgbd_frame_diagnostic",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "scope": "DEVELOPMENT_ONLY_READ_ONLY_TRUTH_SCORED_DIAGNOSTIC",
            "runtime_truth_use": False,
            "commands_published": 0,
            "formal_seed_consumed": False,
            "topics": {
                "detections": options.detections_topic,
                "depth": options.depth_topic,
                "camera_info": options.camera_info_topic,
            },
            "detection_stamp_s": stamp_s,
            "depth_delta_s": depth_delta_s,
            "camera_frame": str(detection_message.header.frame_id),
            "image_shape_hw": [int(depth.shape[0]), int(depth.shape[1])],
            "intrinsics_fx_fy_cx_cy": [
                intrinsics.fx,
                intrinsics.fy,
                intrinsics.cx,
                intrinsics.cy,
            ],
            "parameters": parameters,
            "detections": detection_reports,
            "truth_projection_scoring": truth_reports,
        }
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(
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
