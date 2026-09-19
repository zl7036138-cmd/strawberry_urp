"""Capture one development-only multi-object YOLO sample from Gazebo."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import signal
import sys
import time
from typing import Callable, Sequence

import yaml

from .generalized_capture_core import (
    depth_visible_yolo_labels,
    find_capture_spec,
    load_capture_plan,
    sha256_file,
    validate_visibility_partition,
)
from .generalized_scene import validate_generated_scene


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--formal-matrix", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--scene-receipt", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--split", choices=("train", "validation", "qualification"), required=True
    )
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--sensor-timeout-sec", type=float, default=45.0)
    return parser.parse_args(argv)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _verify_scene(scene: dict, receipt: dict, spec: dict, scene_path: Path) -> None:
    validate_generated_scene(scene)
    generator = scene["generator"]
    expected = {
        "seed": int(spec["seed"]),
        "profile": str(spec["profile"]),
        "position_band": str(spec["position_band"]),
    }
    actual = {key: generator.get(key) for key in expected}
    if actual != expected:
        raise ValueError(f"scene generator differs from capture plan: {actual}")
    if len(scene["plants"]) != int(spec["plant_count"]):
        raise ValueError("scene plant count differs from capture plan")
    if scene["condition"]["occlusion"] != spec["occlusion"]:
        raise ValueError("scene occlusion differs from capture plan")
    if int(receipt.get("seed", -1)) != int(spec["seed"]):
        raise ValueError("scene receipt seed differs from capture plan")
    if receipt.get("truth_for_runtime_control") is not False:
        raise ValueError("scene receipt crosses the runtime truth boundary")
    if receipt.get("scene", {}).get("sha256") != sha256_file(scene_path):
        raise ValueError("scene receipt does not bind the supplied scene")


def main(args=None) -> int:  # pragma: no cover - exercised by ROS integration
    try:
        import cv2
        import numpy as np
        import rclpy
        import tf2_geometry_msgs  # noqa: F401 - register PoseStamped conversion
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseStamped
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.utilities import remove_ros_args
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        raise RuntimeError(
            "generalized capture requires ROS 2, OpenCV, NumPy, cv_bridge, and tf2"
        ) from error

    options = _parse_args(remove_ros_args(args=args if args is not None else sys.argv)[1:])
    if options.sensor_timeout_sec <= 0.0:
        raise ValueError("sensor timeout must be positive")
    plan_path = options.plan.resolve(strict=True)
    formal_path = options.formal_matrix.resolve(strict=True)
    scene_path = options.scene_config.resolve(strict=True)
    scene_receipt_path = options.scene_receipt.resolve(strict=True)
    output_root = options.output_root.resolve()
    plan = load_capture_plan(plan_path, formal_path)
    spec = find_capture_spec(plan, split=options.split, seed=options.seed)
    scene = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    scene_receipt = json.loads(scene_receipt_path.read_text(encoding="utf-8"))
    _verify_scene(scene, scene_receipt, spec, scene_path)
    capture = plan["capture"]
    expected_size = (int(capture["image_width"]), int(capture["image_height"]))
    sync_tolerance = float(capture["sync_tolerance_sec"])
    sample_id = str(spec["sample_id"])
    image_path = output_root / "images" / options.split / f"{sample_id}.png"
    label_path = output_root / "labels" / options.split / f"{sample_id}.txt"
    receipt_path = output_root / "receipts" / options.split / f"{sample_id}.json"
    for path in (image_path, label_path, receipt_path):
        if path.exists():
            raise ValueError(f"refusing to overwrite capture artifact: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_generalized_development_capture")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.bridge = CvBridge()
            self.camera_info = None
            self.color = None
            self.depth = None
            self.color_sequence = 0
            self.truth: dict[int, object] = {}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(
                CameraInfo,
                "/camera/camera_info",
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/camera/color/image_raw",
                self._on_color,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/camera/depth/image_raw",
                self._on_depth,
                qos_profile_sensor_data,
            )
            self._truth_subscriptions = []
            for fruit in scene["fruits"]:
                identity = int(fruit["target_id"])
                subscription = self.create_subscription(
                    PoseStamped,
                    f"/strawberry/ground_truth/fruit_{identity}/pose",
                    lambda message, selected=identity: self._on_truth(selected, message),
                    qos_profile_sensor_data,
                )
                self._truth_subscriptions.append(subscription)

        def _on_camera_info(self, message) -> None:
            self.camera_info = message

        def _on_color(self, message) -> None:
            self.color = message
            self.color_sequence += 1

        def _on_depth(self, message) -> None:
            self.depth = message

        def _on_truth(self, identity: int, message) -> None:
            self.truth[identity] = message

        def synchronized(self) -> bool:
            return bool(
                self.color is not None
                and self.depth is not None
                and abs(
                    _stamp_seconds(self.color.header.stamp)
                    - _stamp_seconds(self.depth.header.stamp)
                )
                <= sync_tolerance
            )

        def ready(self) -> bool:
            return bool(
                self.camera_info is not None
                and self.synchronized()
                and len(self.truth) == len(scene["fruits"])
            )

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

        def fruit_rows_in_camera(self, image_stamp) -> list[dict]:
            camera_frame = str(self.camera_info.header.frame_id)
            rows = []
            scene_radius = float(scene["fruit_collision_radius_m"])
            for fruit in scene["fruits"]:
                identity = int(fruit["target_id"])
                truth = self.truth[identity]
                pose = PoseStamped()
                pose.header = truth.header
                pose.header.stamp = image_stamp
                pose.pose = truth.pose
                camera_pose = self.tf_buffer.transform(
                    pose, camera_frame, timeout=Duration(seconds=0.5)
                )
                point = camera_pose.pose.position
                rows.append(
                    {
                        "target_id": identity,
                        "maturity": str(fruit["maturity"]),
                        "center_camera_m": (
                            float(point.x),
                            float(point.y),
                            float(point.z),
                        ),
                        "radius_m": scene_radius * float(fruit.get("scale", 1.0)),
                    }
                )
            return rows

        def color_bgr(self):
            return np.ascontiguousarray(
                self.bridge.imgmsg_to_cv2(self.color, desired_encoding="bgr8")
            )

        def depth_meters(self):
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
            )
            encoding = str(self.depth.encoding).upper()
            if encoding == "16UC1":
                return depth.astype(np.float32) * 0.001
            if encoding == "32FC1":
                return depth.astype(np.float32)
            raise RuntimeError(f"unsupported depth encoding {self.depth.encoding!r}")

    rclpy.init(args=args)
    node = CaptureNode()
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        node.wait_for(node.ready, options.sensor_timeout_sec, "camera, depth, and all truth poses")
        if (int(node.camera_info.width), int(node.camera_info.height)) != expected_size:
            raise RuntimeError("camera dimensions differ from the capture plan")
        first_sequence = node.color_sequence
        settled_frames = int(capture["settled_frames"])
        node.wait_for(
            lambda: node.ready() and node.color_sequence >= first_sequence + settled_frames,
            options.sensor_timeout_sec,
            f"{settled_frames} settled synchronized frames",
        )
        color_message = node.color
        depth_message = node.depth
        info = node.camera_info
        fruit_rows = node.fruit_rows_in_camera(color_message.header.stamp)
        depth = node.depth_meters()
        labels, excluded = depth_visible_yolo_labels(
            fruit_rows,
            depth_image_m=depth,
            intrinsics=(float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])),
            image_size=expected_size,
            minimum_visible_pixels=int(capture["minimum_visible_pixels"]),
            minimum_visible_fraction=float(capture["minimum_visible_fraction"]),
            depth_surface_padding_m=float(capture["depth_surface_padding_m"]),
        )
        negative_image = validate_visibility_partition(fruit_rows, labels, excluded)

        encoded_ok, encoded = cv2.imencode(
            ".png",
            node.color_bgr(),
            [int(cv2.IMWRITE_PNG_COMPRESSION), int(capture["png_compression"])],
        )
        if not encoded_ok:
            raise RuntimeError("OpenCV failed to encode the capture image")
        with image_path.open("xb") as stream:
            stream.write(encoded.tobytes())
        label_text = "".join(
            f"{int(row['class_id'])} "
            + " ".join(format(float(value), ".10f") for value in row["yolo_xywh"])
            + "\n"
            for row in labels
        )
        with label_path.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(label_text)
        result = {
            "schema_version": 1,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "capture_id": plan["capture_id"],
            "scope": plan["scope"],
            "sample": spec,
            "formal_acceptance": False,
            "formal_results_consumed": False,
            "runtime_control_authorized": False,
            "robot_motion_started": False,
            "labels_source": "gazebo_truth_projection_with_depth_visibility_gate",
            "negative_image": negative_image,
            "image_stamp_sec": int(color_message.header.stamp.sec),
            "image_stamp_nanosec": int(color_message.header.stamp.nanosec),
            "depth_stamp_sec": int(depth_message.header.stamp.sec),
            "depth_stamp_nanosec": int(depth_message.header.stamp.nanosec),
            "source_color_sha256": hashlib.sha256(bytes(color_message.data)).hexdigest(),
            "visible_labels": labels,
            "excluded_truth_targets": excluded,
            "plan": {"path": str(plan_path), "sha256": sha256_file(plan_path)},
            "formal_matrix": {"path": str(formal_path), "sha256": sha256_file(formal_path)},
            "scene": {"path": str(scene_path), "sha256": sha256_file(scene_path)},
            "scene_receipt": {
                "path": str(scene_receipt_path),
                "sha256": sha256_file(scene_receipt_path),
            },
            "image": {"path": str(image_path), "sha256": sha256_file(image_path)},
            "label": {"path": str(label_path), "sha256": sha256_file(label_path)},
        }
        with receipt_path.open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(result, separators=(",", ":"), sort_keys=True))
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    raise SystemExit(main())
