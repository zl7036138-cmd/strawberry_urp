"""Capture reproducible RGB/BGR and truth-association shadow evidence."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import signal
import time

import numpy as np

from .core import validate_model_class_contract, validate_runtime_parameters
from .perception_node import _prediction_arguments, _result_rows
from .shadow_diagnostic_core import (
    associate_rows_to_truth,
    bgr_from_rgb,
    project_sphere,
    summarize_modes,
)


def _sha256_bytes(array: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import cv2
        import rclpy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseArray
        from rclpy.executors import ExternalShutdownException
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
        from tf2_ros import Buffer, TransformListener
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError(
            "shadow_diagnostic requires ROS 2, OpenCV, cv_bridge, tf2_ros, and ultralytics"
        ) from error

    class ShadowDiagnostic(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_shadow_diagnostic")
            self.declare_parameter("model_path", "weights/yolo11s_640_best.pt")
            self.declare_parameter("output_dir", "results/t60/shadow_forensics")
            self.declare_parameter("frame_count", 5)
            self.declare_parameter("sample_period_sec", 0.5)
            self.declare_parameter("confidence_threshold", 0.31)
            self.declare_parameter("image_size", 640)
            self.declare_parameter("nms_iou_threshold", 0.70)
            self.declare_parameter("device", "0")
            self.declare_parameter("fruit_radius_m", 0.035)

            parameters = validate_runtime_parameters(
                model_path=self.get_parameter("model_path").value,
                confidence_threshold=self.get_parameter("confidence_threshold").value,
                image_size=self.get_parameter("image_size").value,
                nms_iou_threshold=self.get_parameter("nms_iou_threshold").value,
            )
            self._output_dir = Path(
                str(self.get_parameter("output_dir").value)
            ).expanduser().resolve()
            if self._output_dir.exists() and any(self._output_dir.iterdir()):
                raise RuntimeError("output_dir must be absent or empty")
            self._output_dir.mkdir(parents=True, exist_ok=True)
            self._frame_limit = int(self.get_parameter("frame_count").value)
            if self._frame_limit <= 0:
                raise RuntimeError("frame_count must be positive")
            self._sample_period_sec = float(
                self.get_parameter("sample_period_sec").value
            )
            if self._sample_period_sec < 0.0:
                raise RuntimeError("sample_period_sec must be non-negative")

            # Do not use ``_parameters``: rclpy.Node owns that private mapping.
            self._runtime_parameters = parameters
            self._device = str(self.get_parameter("device").value)
            self._fruit_radius_m = float(
                self.get_parameter("fruit_radius_m").value
            )
            self._model = YOLO(parameters.model_path)
            self._class_names = validate_model_class_contract(self._model.names)
            self._bridge = CvBridge()
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._camera_info = None
            self._catalog = None
            self._truth_poses = None
            self._records = []
            self._last_capture_wall = 0.0

            self.create_subscription(
                CameraInfo,
                "/camera/camera_info",
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PoseArray,
                "/strawberry/ground_truth/poses",
                self._on_truth_poses,
                qos_profile_sensor_data,
            )
            catalog_qos = QoSProfile(depth=1)
            catalog_qos.reliability = ReliabilityPolicy.RELIABLE
            catalog_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                String,
                "/strawberry/ground_truth/catalog",
                self._on_catalog,
                catalog_qos,
            )
            self.create_subscription(
                Image,
                "/camera/color/image_raw",
                self._on_image,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "Waiting for RGB, CameraInfo, ground truth, and camera TF"
            )

        def _on_camera_info(self, message) -> None:
            self._camera_info = message

        def _on_catalog(self, message) -> None:
            try:
                catalog = json.loads(message.data)
                if int(catalog["schema_version"]) != 1:
                    raise ValueError("unsupported catalog schema")
                by_id = {
                    int(item["target_id"]): str(item["maturity"]).upper()
                    for item in catalog["fruits"]
                }
                order = tuple(int(value) for value in catalog["pose_array_order"])
                if set(order) != set(by_id):
                    raise ValueError("catalog order and fruit IDs differ")
                self._catalog = {
                    "frame_id": str(catalog["frame_id"]),
                    "order": order,
                    "maturity": by_id,
                }
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                self.get_logger().error(f"Rejecting ground-truth catalog: {error}")

        def _on_truth_poses(self, message) -> None:
            self._truth_poses = message

        def _project_truth(self, image_message):
            if self._camera_info is None or self._catalog is None or self._truth_poses is None:
                return None
            if tuple(self._catalog["order"]) and len(self._truth_poses.poses) != len(
                self._catalog["order"]
            ):
                return None
            camera_frame = str(image_message.header.frame_id)
            try:
                transform = self._tf_buffer.lookup_transform(
                    camera_frame,
                    str(self._catalog["frame_id"]),
                    Time(),
                )
            except Exception as error:  # noqa: BLE001 - wait for live TF
                self.get_logger().warning(f"Waiting for camera TF: {error}")
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            intrinsics = (
                float(self._camera_info.k[0]),
                float(self._camera_info.k[4]),
                float(self._camera_info.k[2]),
                float(self._camera_info.k[5]),
            )
            projections = []
            for target_id, pose in zip(
                self._catalog["order"], self._truth_poses.poses
            ):
                projected = project_sphere(
                    target_id=target_id,
                    maturity=self._catalog["maturity"][target_id],
                    center_in_source_m=(
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                    ),
                    source_to_camera_translation_m=(
                        translation.x,
                        translation.y,
                        translation.z,
                    ),
                    source_to_camera_quaternion_xyzw=(
                        rotation.x,
                        rotation.y,
                        rotation.z,
                        rotation.w,
                    ),
                    intrinsics=intrinsics,
                    image_size=(int(image_message.width), int(image_message.height)),
                    radius_m=self._fruit_radius_m,
                )
                if projected is not None:
                    projections.append(projected)
            return projections

        def _predict(self, source):
            results = self._model.predict(
                **_prediction_arguments(
                    source=source,
                    image_size=self._runtime_parameters.image_size,
                    confidence_threshold=self._runtime_parameters.confidence_threshold,
                    nms_iou_threshold=self._runtime_parameters.nms_iou_threshold,
                    device=self._device,
                )
            )
            return _result_rows(results[0])

        @staticmethod
        def _draw_overlay(image_bgr, projections, associations, title):
            canvas = image_bgr.copy()
            cv2.putText(
                canvas,
                title,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            for truth in projections:
                x1, y1, x2, y2 = (int(round(value)) for value in truth.bbox_xyxy)
                truth_color = (0, 0, 255) if truth.maturity == "RIPE" else (0, 255, 0)
                cv2.rectangle(canvas, (x1, y1), (x2, y2), truth_color, 2)
                cv2.putText(
                    canvas,
                    f"GT {truth.target_id} {truth.maturity}",
                    (x1, max(45, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    truth_color,
                    1,
                    cv2.LINE_AA,
                )
            for detection in associations:
                x1, y1, x2, y2 = (
                    int(round(value)) for value in detection["bbox_xyxy"]
                )
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.putText(
                    canvas,
                    f"P {detection['class_name']} {detection['confidence']:.2f} "
                    f"gt={detection['associated_target_id']} iou={detection['association_iou']:.2f}",
                    (x1, min(canvas.shape[0] - 5, y2 + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (255, 0, 255),
                    1,
                    cv2.LINE_AA,
                )
            return canvas

        def _on_image(self, message) -> None:
            if len(self._records) >= self._frame_limit:
                return
            wall_now = time.monotonic()
            if wall_now - self._last_capture_wall < self._sample_period_sec:
                return
            projections = self._project_truth(message)
            if projections is None:
                return
            try:
                rgb = np.asarray(
                    self._bridge.imgmsg_to_cv2(message, desired_encoding="rgb8"),
                    dtype=np.uint8,
                )
                bgr = np.asarray(
                    self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"),
                    dtype=np.uint8,
                )
                channel_swap_exact = np.array_equal(bgr, bgr_from_rgb(rgb))
                if not channel_swap_exact:
                    raise RuntimeError("CvBridge RGB/BGR arrays are not exact channel swaps")
                legacy_rows = self._predict(rgb)
                correct_rows = self._predict(bgr)
                legacy = associate_rows_to_truth(
                    legacy_rows, self._class_names, projections
                )
                correct = associate_rows_to_truth(
                    correct_rows, self._class_names, projections
                )
            except Exception as error:  # noqa: BLE001 - diagnostic must preserve reason
                self.get_logger().error(f"Frame diagnostic failed: {error}")
                return

            frame_number = len(self._records) + 1
            prefix = f"frame_{frame_number:03d}"
            cv2.imwrite(str(self._output_dir / f"{prefix}_source_rgb.png"), bgr)
            cv2.imwrite(
                str(self._output_dir / f"{prefix}_legacy_rgb_overlay.png"),
                self._draw_overlay(bgr, projections, legacy, "LEGACY: RGB ndarray passed to YOLO"),
            )
            cv2.imwrite(
                str(self._output_dir / f"{prefix}_correct_bgr_overlay.png"),
                self._draw_overlay(bgr, projections, correct, "CORRECT: BGR ndarray passed to YOLO"),
            )
            record = {
                "frame_number": frame_number,
                "stamp": {
                    "sec": int(message.header.stamp.sec),
                    "nanosec": int(message.header.stamp.nanosec),
                },
                "frame_id": str(message.header.frame_id),
                "source_encoding": str(message.encoding),
                "rgb_sha256": _sha256_bytes(rgb),
                "bgr_sha256": _sha256_bytes(bgr),
                "channel_swap_exact": channel_swap_exact,
                "truth": [
                    {
                        "target_id": truth.target_id,
                        "maturity": truth.maturity,
                        "bbox_xyxy": list(truth.bbox_xyxy),
                        "center_uv": list(truth.center_uv),
                        "depth_m": truth.depth_m,
                    }
                    for truth in projections
                ],
                "legacy_rgb": legacy,
                "correct_bgr": correct,
            }
            self._records.append(record)
            self._last_capture_wall = wall_now
            self.get_logger().info(
                f"Captured frame {frame_number}/{self._frame_limit}: "
                f"legacy={len(legacy)}, correct={len(correct)}"
            )
            if len(self._records) >= self._frame_limit:
                self._write_summary()
                rclpy.shutdown()

        def _write_summary(self) -> None:
            model_path = Path(self._runtime_parameters.model_path)
            summary = {
                "schema_version": 1,
                "diagnostic": "shadow_rgb_bgr_truth_association",
                "model": {
                    "path": str(model_path),
                    "sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
                    "class_names": list(self._class_names),
                },
                "inference": {
                    "confidence_threshold": self._runtime_parameters.confidence_threshold,
                    "image_size": self._runtime_parameters.image_size,
                    "nms_iou_threshold": self._runtime_parameters.nms_iou_threshold,
                    "device": self._device,
                    "ultralytics_numpy_contract": "BGR ndarray; predictor converts BGR to RGB",
                },
                "frame_count": len(self._records),
                "all_channel_swaps_exact": all(
                    bool(record["channel_swap_exact"]) for record in self._records
                ),
                "modes": summarize_modes(self._records),
                "frames": self._records,
            }
            summary_path = self._output_dir / "summary.json"
            summary_path.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            self.get_logger().info(f"Wrote {summary_path}")

    rclpy.init(args=args)
    node = None
    try:
        node = ShadowDiagnostic()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
