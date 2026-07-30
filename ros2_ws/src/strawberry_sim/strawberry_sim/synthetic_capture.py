"""Capture one hash-bound synthetic image/label group from Gazebo truth."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Callable, Sequence

from .scene_conditions import (
    OCCLUDER_MODEL_NAME,
    fingerprint,
    load_scene_condition_config,
    resolve_occluder_parameters,
)
from .synthetic_capture_core import project_sphere_in_camera, xyxy_to_yolo


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--group-id", required=True)
    parser.add_argument("--split", choices=("train", "heldout"), required=True)
    parser.add_argument("--maturity", choices=("RIPE", "UNRIPE"), required=True)
    parser.add_argument("--condition-id", required=True)
    parser.add_argument("--sensor-timeout-sec", type=float, default=30.0)
    return parser.parse_args(argv)


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(
        sum((float(a) - float(b)) ** 2 for a, b in zip(left, right))
    )


def normalized_orientation_xyzw(
    value: object,
) -> tuple[float, float, float, float]:
    """Validate and normalize one manifest quaternion."""

    if value is None:
        return (0.0, 0.0, 0.0, 1.0)
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 4
    ):
        raise ValueError("orientation_xyzw must contain four values")
    orientation = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in orientation):
        raise ValueError("orientation_xyzw must be finite")
    norm = math.sqrt(sum(component * component for component in orientation))
    if norm <= 1.0e-12:
        raise ValueError("orientation_xyzw must be non-zero")
    return tuple(component / norm for component in orientation)


def main(args=None) -> int:  # pragma: no cover - exercised by ROS integration
    try:
        import cv2
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
            "synthetic_capture requires ROS 2, OpenCV, NumPy, ros_gz_interfaces, and tf2"
        ) from error

    options = _parse_args(remove_ros_args(args=args if args is not None else sys.argv)[1:])
    if options.sensor_timeout_sec <= 0.0:
        raise ValueError("sensor timeout must be positive")
    manifest_path = options.manifest.resolve(strict=True)
    scene_config_path = options.scene_config.resolve(strict=True)
    receipt_path = options.receipt.resolve(strict=True)
    output_root = options.output_root.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scene_config = load_scene_condition_config(scene_config_path)
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    if (
        manifest.get("formal_acceptance")
        or manifest.get("held_out_test_consumed")
        or manifest.get("training_started")
    ):
        raise ValueError("synthetic capture violates the frozen non-acceptance boundary")
    if int(scene_config["schema_version"]) != 3:
        raise ValueError("synthetic capture requires scene-condition schema v3")
    if receipt.get("condition_config", {}).get("sha256") != fingerprint(
        scene_config_path
    )["sha256"]:
        raise ValueError("condition receipt is not bound to the supplied scene config")

    class_entry = next(
        (
            item
            for item in manifest["classes"]
            if item["maturity"] == options.maturity
        ),
        None,
    )
    condition = next(
        (
            item
            for item in manifest["conditions"]
            if item["condition_id"] == options.condition_id
        ),
        None,
    )
    if class_entry is None or condition is None:
        raise ValueError("group class or condition is outside the capture manifest")
    expected_group = (
        f"syn__{options.split}__{options.maturity.lower()}__{options.condition_id}"
    )
    if options.group_id != expected_group:
        raise ValueError("group ID differs from the frozen naming contract")
    split = manifest["splits"][options.split]
    if int(receipt.get("seed", 0)) != int(split["materialization_id"]):
        raise ValueError("condition receipt uses the wrong split materialization ID")
    if receipt.get("lighting_level") != condition["lighting"]:
        raise ValueError("condition receipt lighting mismatch")
    if receipt.get("occlusion_level") != condition["occlusion"]:
        raise ValueError("condition receipt occlusion mismatch")

    group_path = output_root / "groups" / f"{options.group_id}.json"
    if group_path.exists():
        raise ValueError(f"refusing to overwrite capture group: {group_path}")
    image_dir = output_root / "images" / options.split
    label_dir = output_root / "labels" / options.split
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    group_path.parent.mkdir(parents=True, exist_ok=True)

    capture = manifest["capture"]
    expected_size = (int(capture["image_width"]), int(capture["image_height"]))
    settled_frames = int(capture["settled_frames_after_pose"])
    radius_m = float(capture["fruit_radius_m"])
    compression = int(capture["png_compression"])
    target_id = int(class_entry["target_id"])
    target_model = str(class_entry["target_model_name"])
    class_id = int(class_entry["class_id"])
    parked = {
        str(item["model_name"]): tuple(float(value) for value in item["position_m"])
        for item in manifest["parked_models"]
    }
    fruit_id_by_model = {
        str(item["model_name"]): int(item["target_id"])
        for item in manifest["parked_models"]
    }

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_synthetic_capture")
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
                    lambda message, selected=identity: self._on_truth(
                        selected, message
                    ),
                    qos_profile_sensor_data,
                )

        def _on_camera_info(self, message) -> None:
            self.camera_info = message

        def _on_image(self, message) -> None:
            self.image_sequence += 1
            self.images.append((self.image_sequence, message))
            if len(self.images) > 200:
                del self.images[:100]

        def _on_truth(self, identity: int, message) -> None:
            self.truth[identity] = message

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
            self,
            model_name: str,
            position: Sequence[float],
            orientation_xyzw: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        ) -> int:
            if not self.pose_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("Gazebo set_pose service is unavailable")
            for attempt in (1, 2):
                request = SetEntityPose.Request()
                request.entity.name = model_name
                request.entity.type = Entity.MODEL
                request.pose.position.x = float(position[0])
                request.pose.position.y = float(position[1])
                request.pose.position.z = float(position[2])
                request.pose.orientation.x = float(orientation_xyzw[0])
                request.pose.orientation.y = float(orientation_xyzw[1])
                request.pose.orientation.z = float(orientation_xyzw[2])
                request.pose.orientation.w = float(orientation_xyzw[3])
                future = self.pose_client.call_async(request)
                try:
                    self.wait_for(
                        future.done, 5.0, f"set_pose response for {model_name}"
                    )
                except RuntimeError:
                    remove_pending = getattr(
                        self.pose_client, "remove_pending_request", None
                    )
                    if callable(remove_pending):
                        remove_pending(future)
                    if attempt == 2:
                        raise
                    continue
                if future.exception() is None and future.result().success:
                    return attempt
                if attempt == 2:
                    raise RuntimeError(f"Gazebo rejected set_pose for {model_name}")
            raise AssertionError("unreachable pose retry state")

        def truth_xyz(self, identity: int) -> tuple[float, float, float]:
            position = self.truth[identity].pose.position
            return float(position.x), float(position.y), float(position.z)

        def image_rgb(self, message):
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
            return np.ascontiguousarray(rgb)

        def truth_bbox(self, message) -> tuple[float, float, float, float]:
            info = self.camera_info
            truth = self.truth[target_id]
            pose = PoseStamped()
            pose.header = truth.header
            pose.header.stamp = message.header.stamp
            pose.pose = truth.pose
            camera_pose = self.tf_buffer.transform(
                pose,
                str(info.header.frame_id),
                timeout=Duration(seconds=0.5),
            )
            point = camera_pose.pose.position
            return project_sphere_in_camera(
                (float(point.x), float(point.y), float(point.z)),
                (
                    float(info.k[0]),
                    float(info.k[4]),
                    float(info.k[2]),
                    float(info.k[5]),
                ),
                (int(info.width), int(info.height)),
                radius_m,
            )

    rclpy.init(args=args)
    node = CaptureNode()
    sample_records = []
    try:
        node.wait_for(
            lambda: (
                node.camera_info is not None
                and len(node.truth) == 3
                and node.image_sequence > 0
                and node.pose_client.service_is_ready()
            ),
            options.sensor_timeout_sec,
            "camera, truth, image, and pose-control interfaces",
        )
        if (
            int(node.camera_info.width),
            int(node.camera_info.height),
        ) != expected_size:
            raise RuntimeError("camera dimensions differ from the capture manifest")
        for position_entry in split["positions"]:
            position_id = str(position_entry["position_id"])
            target_position = tuple(
                float(value) for value in position_entry["position_m"]
            )
            target_orientation = normalized_orientation_xyzw(
                position_entry.get("orientation_xyzw")
            )
            desired_positions = dict(parked)
            desired_positions[target_model] = target_position
            desired_orientations = {
                model_name: (0.0, 0.0, 0.0, 1.0)
                for model_name in parked
            }
            desired_orientations[target_model] = target_orientation
            start_sequence = node.image_sequence
            pose_attempts = []
            for model_name in ("strawberry_1", "strawberry_2", "strawberry_3"):
                attempts = node.set_model_pose(
                    model_name,
                    desired_positions[model_name],
                    desired_orientations[model_name],
                )
                pose_attempts.append(
                    {"model_name": model_name, "attempts": attempts}
                )
            occluder = resolve_occluder_parameters(
                scene_config, condition["occlusion"], target_position
            )
            if occluder["enabled"]:
                attempts = node.set_model_pose(
                    OCCLUDER_MODEL_NAME, occluder["pose_xyz_rpy"][:3]
                )
                pose_attempts.append(
                    {"model_name": OCCLUDER_MODEL_NAME, "attempts": attempts}
                )
            node.wait_for(
                lambda: all(
                    identity in node.truth
                    and _distance(
                        node.truth_xyz(identity),
                        desired_positions[model_name],
                    )
                    <= 0.001
                    for model_name, identity in fruit_id_by_model.items()
                ),
                options.sensor_timeout_sec,
                f"settled fruit truth for {position_id}",
            )
            capture_sequence = start_sequence + settled_frames
            node.wait_for(
                lambda: node.image_sequence >= capture_sequence,
                options.sensor_timeout_sec,
                f"{settled_frames} settled images for {position_id}",
            )
            matching = [
                message for sequence, message in node.images if sequence == capture_sequence
            ]
            if len(matching) != 1:
                raise RuntimeError("image cache did not retain the selected frame")
            message = matching[0]
            rgb = node.image_rgb(message)
            bbox = node.truth_bbox(message)
            yolo = xyxy_to_yolo(bbox, expected_size)
            sample_id = (
                f"syn__{options.split}__{options.maturity.lower()}__"
                f"{options.condition_id}__{position_id}"
            )
            image_path = image_dir / f"{sample_id}.png"
            label_path = label_dir / f"{sample_id}.txt"
            if image_path.exists() or label_path.exists():
                raise RuntimeError(f"refusing to overwrite sample {sample_id}")
            bgr = np.ascontiguousarray(rgb[:, :, ::-1])
            encoded_ok, encoded = cv2.imencode(
                ".png",
                bgr,
                [int(cv2.IMWRITE_PNG_COMPRESSION), compression],
            )
            if not encoded_ok:
                raise RuntimeError("OpenCV failed to encode the synthetic PNG")
            with image_path.open("xb") as stream:
                stream.write(encoded.tobytes())
            label_text = (
                f"{class_id} "
                + " ".join(format(value, ".10f") for value in yolo)
                + "\n"
            )
            with label_path.open("x", encoding="utf-8", newline="\n") as stream:
                stream.write(label_text)
            sample_records.append(
                {
                    "sample_id": sample_id,
                    "split": options.split,
                    "maturity": options.maturity,
                    "class_id": class_id,
                    "target_id": target_id,
                    "target_model_name": target_model,
                    "position_id": position_id,
                    "target_position_m": list(target_position),
                    "target_orientation_xyzw": list(target_orientation),
                    "lighting": condition["lighting"],
                    "occlusion": condition["occlusion"],
                    "condition_id": options.condition_id,
                    "materialization_id": int(split["materialization_id"]),
                    "image_stamp_sec": int(message.header.stamp.sec),
                    "image_stamp_nanosec": int(message.header.stamp.nanosec),
                    "source_encoding": str(message.encoding),
                    "source_image_sha256": hashlib.sha256(
                        bytes(message.data)
                    ).hexdigest(),
                    "bbox_xyxy": list(bbox),
                    "yolo_xywh": list(yolo),
                    "pose_configuration_attempts": pose_attempts,
                    "resolved_occluder": occluder,
                    "image": fingerprint(image_path),
                    "label": fingerprint(label_path),
                }
            )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    group_result = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "capture_id": manifest["capture_id"],
        "group_id": options.group_id,
        "scope": "NON_ACCEPTANCE_SYNTHETIC_CAPTURE_PREFLIGHT",
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "training_started": False,
        "robot_motion_started": False,
        "ros_domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
        "split": options.split,
        "maturity": options.maturity,
        "class_id": class_id,
        "target_id": target_id,
        "target_model_name": target_model,
        "condition_id": options.condition_id,
        "lighting": condition["lighting"],
        "occlusion": condition["occlusion"],
        "materialization_id": int(split["materialization_id"]),
        "manifest": fingerprint(manifest_path),
        "scene_condition_config": fingerprint(scene_config_path),
        "condition_receipt": fingerprint(receipt_path),
        "materialized_world_sha256": receipt["materialized_world"]["sha256"],
        "sample_count": len(sample_records),
        "samples": sample_records,
    }
    group_path.write_text(
        json.dumps(group_result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "group_id": options.group_id,
                "samples": len(sample_records),
                "split": options.split,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
