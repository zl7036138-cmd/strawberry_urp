"""Measure one materialized lighting/occlusion world through the ROS camera."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Callable, Sequence

from .scene_conditions import fingerprint


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--sensor-timeout-sec", type=float, default=15.0)
    return parser.parse_args(argv)


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def main(args=None) -> int:  # pragma: no cover - exercised by ROS integration
    try:
        import numpy as np
        import rclpy
        import tf2_geometry_msgs  # noqa: F401 - register PoseStamped conversion
        from geometry_msgs.msg import PoseStamped
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.utilities import remove_ros_args
        from ros_gz_interfaces.msg import Entity
        from ros_gz_interfaces.srv import SetEntityPose
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        raise RuntimeError(
            "scene_condition_probe requires ROS 2, NumPy, ros_gz_interfaces, and tf2"
        ) from error

    options = _parse_args(remove_ros_args(args=args)[1:])
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite probe artifact: {options.output_json}")
    if options.sensor_timeout_sec <= 0.0:
        raise ValueError("sensor timeout must be positive")
    receipt_path = options.receipt.resolve(strict=True)
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    if receipt.get("formal_acceptance") or receipt.get("held_out_test_consumed"):
        raise ValueError("scene-condition receipt violates the non-acceptance boundary")
    world_record = receipt["materialized_world"]
    world_path = Path(world_record["path"])
    if fingerprint(world_path)["sha256"] != world_record["sha256"]:
        raise ValueError("materialized world differs from its receipt")
    validation = receipt["validation_scene"]
    frames_required = int(validation["frames_per_condition"])
    target_id = int(validation["target_id"])
    target_model = str(validation["target_model_name"])
    target_position = tuple(float(value) for value in validation["target_position_m"])
    radius_m = float(validation["fruit_radius_m"])
    configured_models = [
        (target_model, target_id, target_position),
        *[
            (
                str(item["model_name"]),
                int(item["target_id"]),
                tuple(float(value) for value in item["position_m"]),
            )
            for item in validation["parked_models"]
        ],
    ]

    class ProbeNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_scene_condition_probe")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.camera_info = None
            self.truth: dict[int, object] = {}
            self.image_sequence = 0
            self.images: list[tuple[int, object]] = []
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pose_client = self.create_client(
                SetEntityPose, "/world/strawberry_orchard/set_pose"
            )
            self.create_subscription(
                CameraInfo,
                "/camera/camera_info",
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/camera/color/image_raw",
                self._on_image,
                qos_profile_sensor_data,
            )
            for identity in (1, 2, 3):
                self.create_subscription(
                    PoseStamped,
                    f"/strawberry/ground_truth/fruit_{identity}/pose",
                    lambda message, target=identity: self._on_truth(target, message),
                    qos_profile_sensor_data,
                )

        def _on_camera_info(self, message) -> None:
            self.camera_info = message

        def _on_truth(self, identity: int, message) -> None:
            self.truth[identity] = message

        def _on_image(self, message) -> None:
            self.image_sequence += 1
            self.images.append((self.image_sequence, message))
            if len(self.images) > 100:
                del self.images[:50]

        def wait_for(
            self, predicate: Callable[[], bool], timeout_sec: float, description: str
        ) -> None:
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and time.monotonic() < deadline:
                if predicate():
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            if not predicate():
                raise RuntimeError(f"timed out waiting for {description}")

        def set_model_pose(
            self, model_name: str, position: tuple[float, float, float]
        ) -> int:
            if not self.pose_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("Gazebo set_pose service is unavailable")
            for attempt in (1, 2):
                request = SetEntityPose.Request()
                request.entity.name = model_name
                request.entity.type = Entity.MODEL
                request.pose.position.x = position[0]
                request.pose.position.y = position[1]
                request.pose.position.z = position[2]
                request.pose.orientation.w = 1.0
                future = self.pose_client.call_async(request)
                try:
                    self.wait_for(future.done, 5.0, f"set_pose for {model_name}")
                except RuntimeError:
                    remove_pending = getattr(
                        self.pose_client, "remove_pending_request", None
                    )
                    if callable(remove_pending):
                        remove_pending(future)
                    if attempt == 2:
                        raise
                    continue
                if future.exception() is not None or not future.result().success:
                    raise RuntimeError(f"Gazebo rejected set_pose for {model_name}")
                return attempt
            raise AssertionError("unreachable pose retry state")

        def truth_xyz(self, identity: int) -> tuple[float, float, float]:
            position = self.truth[identity].pose.position
            return float(position.x), float(position.y), float(position.z)

        def target_roi(self, stamp) -> tuple[int, int, int, int]:
            info = self.camera_info
            truth = self.truth[target_id]
            pose = PoseStamped()
            pose.header = truth.header
            pose.header.stamp = stamp
            pose.pose = truth.pose
            camera_pose = self.tf_buffer.transform(
                pose,
                str(info.header.frame_id),
                timeout=Duration(seconds=0.5),
            )
            point = camera_pose.pose.position
            if point.z <= radius_m:
                raise RuntimeError("target is not in front of the RGB-D camera")
            fx, fy, cx, cy = (
                float(info.k[0]),
                float(info.k[4]),
                float(info.k[2]),
                float(info.k[5]),
            )
            u = cx + fx * float(point.x) / float(point.z)
            v = cy + fy * float(point.y) / float(point.z)
            rx = fx * radius_m / float(point.z)
            ry = fy * radius_m / float(point.z)
            x1 = max(0, int(math.floor(u - rx)))
            y1 = max(0, int(math.floor(v - ry)))
            x2 = min(int(info.width), int(math.ceil(u + rx)))
            y2 = min(int(info.height), int(math.ceil(v + ry)))
            if x2 <= x1 or y2 <= y1:
                raise RuntimeError("projected target ROI is empty")
            return x1, y1, x2, y2

        @staticmethod
        def image_rgb(message):
            encoding = str(message.encoding).lower()
            channels = 4 if encoding in {"rgba8", "bgra8"} else 3
            if encoding not in {"rgb8", "bgr8", "rgba8", "bgra8"}:
                raise RuntimeError(f"unsupported color encoding {encoding!r}")
            rows = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
                int(message.height), int(message.step)
            )
            packed = rows[:, : int(message.width) * channels].reshape(
                int(message.height), int(message.width), channels
            )
            rgb = packed[:, :, :3]
            if encoding in {"bgr8", "bgra8"}:
                rgb = rgb[:, :, ::-1]
            return rgb

        def measure(self, message, frame_index: int) -> dict[str, object]:
            rgb = self.image_rgb(message)
            roi_xyxy = self.target_roi(message.header.stamp)
            x1, y1, x2, y2 = roi_xyxy
            roi = rgb[y1:y2, x1:x2]
            red = roi[:, :, 0].astype(np.float32)
            green = roi[:, :, 1].astype(np.float32)
            blue = roi[:, :, 2].astype(np.float32)
            blue_mask = (blue >= 1.5 * np.maximum(red, green) + 5.0)
            full = rgb.astype(np.float32)
            luminance = (
                0.2126 * full[:, :, 0]
                + 0.7152 * full[:, :, 1]
                + 0.0722 * full[:, :, 2]
            )
            roi_luminance = (
                0.2126 * red + 0.7152 * green + 0.0722 * blue
            )
            return {
                "frame_index": frame_index,
                "stamp_sec": int(message.header.stamp.sec),
                "stamp_nanosec": int(message.header.stamp.nanosec),
                "encoding": str(message.encoding),
                "width": int(message.width),
                "height": int(message.height),
                "target_roi_xyxy": list(roi_xyxy),
                "target_roi_pixel_count": int(roi.shape[0] * roi.shape[1]),
                "luminance_mean_8bit": float(luminance.mean()),
                "target_roi_luminance_mean_8bit": float(roi_luminance.mean()),
                "blue_coverage_ratio": float(blue_mask.mean()),
                "image_sha256": hashlib.sha256(bytes(message.data)).hexdigest(),
            }

    rclpy.init(args=args)
    node = ProbeNode()
    pose_attempts = []
    try:
        node.wait_for(
            lambda: (
                node.camera_info is not None
                and len(node.truth) == 3
                and node.image_sequence > 0
                and node.pose_client.service_is_ready()
            ),
            options.sensor_timeout_sec,
            "camera, truth, and pose-control interfaces",
        )
        start_sequence = node.image_sequence
        for model_name, identity, position in configured_models:
            attempts = node.set_model_pose(model_name, position)
            pose_attempts.append({"model_name": model_name, "attempts": attempts})
        node.wait_for(
            lambda: all(
                identity in node.truth
                and _distance(node.truth_xyz(identity), position) <= 0.001
                for _, identity, position in configured_models
            ),
            options.sensor_timeout_sec,
            "configured fruit ground truth",
        )
        node.wait_for(
            lambda: node.image_sequence >= start_sequence + 10,
            options.sensor_timeout_sec,
            "ten settled camera frames",
        )
        collection_start = node.image_sequence
        node.wait_for(
            lambda: node.image_sequence >= collection_start + frames_required,
            options.sensor_timeout_sec,
            f"{frames_required} measured camera frames",
        )
        selected = [
            message
            for sequence, message in node.images
            if collection_start < sequence <= collection_start + frames_required
        ]
        if len(selected) != frames_required:
            raise RuntimeError("camera cache did not preserve the measured frame window")
        frames = [
            node.measure(message, index)
            for index, message in enumerate(selected, start=1)
        ]
        result = {
            "schema_version": 1,
            "gate": "SCENE_CONDITION_PROBE",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "scope": receipt["scope"],
            "formal_acceptance": False,
            "held_out_test_consumed": False,
            "lighting_level": receipt["lighting_level"],
            "occlusion_level": receipt["occlusion_level"],
            "seed": receipt["seed"],
            "receipt": fingerprint(receipt_path),
            "materialized_world_sha256": world_record["sha256"],
            "pose_configuration_attempts": pose_attempts,
            "configured_models": [
                {
                    "model_name": name,
                    "target_id": identity,
                    "position_m": list(position),
                }
                for name, identity, position in configured_models
            ],
            "frames": frames,
        }
        options.output_json.parent.mkdir(parents=True, exist_ok=True)
        options.output_json.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(
            json.dumps(
                {
                    "lighting": result["lighting_level"],
                    "occlusion": result["occlusion_level"],
                    "frames": len(frames),
                    "luminance_mean_8bit": sum(
                        frame["luminance_mean_8bit"] for frame in frames
                    )
                    / len(frames),
                    "blue_coverage_ratio": sum(
                        frame["blue_coverage_ratio"] for frame in frames
                    )
                    / len(frames),
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
