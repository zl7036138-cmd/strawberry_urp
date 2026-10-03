"""ROS 2 node that localizes the best ripe detection from RGB-D data."""

from __future__ import annotations

import inspect
import json
import math
import signal
import threading
import time
from collections import deque
from typing import Sequence

import numpy as np

from .core import (
    BoundingBox,
    CameraIntrinsics,
    LocalizationError,
    MatchedSensorFrames,
    SensorFrameCache,
    associate_nearest_target,
    localize_bbox,
    validate_bbox_within_image,
    validate_sensor_metadata,
)


def shutdown_executor_and_wait(
    executor, timeout_sec: float | None = None
) -> bool:
    """Join Jazzy executor workers and consume completed task exceptions."""

    try:
        supports_thread_wait = (
            "wait_for_threads"
            in inspect.signature(executor.shutdown).parameters
        )
    except (TypeError, ValueError):
        supports_thread_wait = False
    if supports_thread_wait:
        callbacks_stopped = bool(
            executor.shutdown(
                timeout_sec=timeout_sec,
                wait_for_threads=True,
            )
        )
    else:
        callbacks_stopped = bool(executor.shutdown(timeout_sec=timeout_sec))
        worker_pool = getattr(executor, "_executor", None)
        if worker_pool is not None and hasattr(worker_pool, "shutdown"):
            worker_pool.shutdown(wait=True)
    tasks = getattr(executor, "_futures", None)
    if tasks is not None:
        for task in tuple(tasks):
            if not task.done():
                continue
            try:
                task.result()
            except BaseException:
                pass
        tasks.clear()
    return callbacks_stopped


def select_sensor_frames(
    cache: SensorFrameCache,
    *,
    detection_stamp_s: float,
    detection_frame: str,
    now_stamp_s: float,
    sync_tolerance_sec: float,
    stale_after_sec: float,
) -> MatchedSensorFrames:
    """Select and validate the cached RGB-D inputs for one detection."""

    cache.prune(now_stamp_s)
    matched = cache.match(
        detection_stamp_s=detection_stamp_s,
        detection_frame=detection_frame,
        sync_tolerance_sec=sync_tolerance_sec,
    )
    validate_sensor_metadata(
        detection_stamp_s=detection_stamp_s,
        detection_frame=detection_frame,
        depth_stamp_s=matched.depth.stamp_s,
        depth_frame=matched.depth.frame_id,
        camera_info_stamp_s=matched.camera_info.stamp_s,
        camera_info_frame=matched.camera_info.frame_id,
        now_stamp_s=now_stamp_s,
        sync_tolerance_sec=sync_tolerance_sec,
        stale_after_sec=stale_after_sec,
    )
    return matched


def joint_samples_are_stationary(
    samples: Sequence[Sequence[float]],
    *,
    minimum_samples: int,
    maximum_delta_rad: float,
) -> bool:
    """Return true only when every observed arm joint stayed within a bound."""
    if minimum_samples < 2 or maximum_delta_rad <= 0.0:
        raise ValueError("stationarity limits must be positive")
    if len(samples) < minimum_samples:
        return False
    selected = samples[-minimum_samples:]
    width = len(selected[0])
    if width == 0 or any(len(sample) != width for sample in selected):
        raise ValueError("joint samples must have one consistent non-zero width")
    return all(
        max(float(sample[index]) for sample in selected)
        - min(float(sample[index]) for sample in selected)
        <= maximum_delta_rad
        for index in range(width)
    )


def localization_retry_is_fresh(
    *,
    detection_stamp_s: float,
    now_stamp_s: float,
    stale_after_sec: float,
    sync_tolerance_sec: float,
) -> bool:
    """Allow a deferred observation only inside the existing freshness gate."""

    values = (
        detection_stamp_s,
        now_stamp_s,
        stale_after_sec,
        sync_tolerance_sec,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("retry timing values must be finite")
    if detection_stamp_s < 0.0 or now_stamp_s < 0.0:
        raise ValueError("retry timestamps must be non-negative")
    if stale_after_sec <= 0.0 or sync_tolerance_sec < 0.0:
        raise ValueError("retry timing bounds are invalid")
    age = now_stamp_s - detection_stamp_s
    return -sync_tolerance_sec <= age <= stale_after_sec


def validate_sensor_qos_depth(value: int) -> int:
    """Validate a bounded sensor subscription history depth."""

    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("sensor QoS depth must be a positive integer")
    if value > 120:
        raise ValueError("sensor QoS depth cannot exceed 120")
    return value


def validate_selection_roi(
    roi_xyxy_px: Sequence[int] | None,
) -> tuple[int, int, int, int] | None:
    """Validate an optional pixel-space attention region."""

    if roi_xyxy_px is None:
        return None
    if len(roi_xyxy_px) != 4:
        raise ValueError("selection ROI must contain x_min, y_min, x_max, y_max")
    values = tuple(int(value) for value in roi_xyxy_px)
    if values == (-1, -1, -1, -1):
        return None
    x_min, y_min, x_max, y_max = values
    if min(values) < 0 or x_max <= x_min or y_max <= y_min:
        raise ValueError("selection ROI must be a positive-area pixel rectangle")
    return values


def select_ripe_detection(
    detections,
    *,
    confidence_threshold: float,
    selection_roi_xyxy_px: Sequence[int] | None = None,
):
    """Choose the best ripe detection, optionally inside a coarse attention ROI."""

    if not math.isfinite(confidence_threshold) or not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence threshold must be in [0, 1]")
    selection_roi = validate_selection_roi(selection_roi_xyxy_px)
    candidates = []
    for item in detections:
        if (
            item.maturity != item.RIPE
            or not math.isfinite(float(item.confidence))
            or float(item.confidence) < confidence_threshold
        ):
            continue
        if selection_roi is not None:
            x_min, y_min, x_max, y_max = selection_roi
            center_x = float(item.bbox.x_offset) + 0.5 * float(item.bbox.width)
            center_y = float(item.bbox.y_offset) + 0.5 * float(item.bbox.height)
            if not (x_min <= center_x < x_max and y_min <= center_y < y_max):
                continue
        candidates.append(item)
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda item: (-float(item.confidence), int(item.target_id)),
    )


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped with tf2
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseArray, PoseStamped
        from rclpy.callback_groups import (
            MutuallyExclusiveCallbackGroup,
            ReentrantCallbackGroup,
        )
        from rclpy.duration import Duration
        from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.time import Time
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from std_msgs.msg import String
        from strawberry_interfaces.msg import (
            StrawberryDetectionArray,
            TargetPose,
        )
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class LocalizationNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_localization")
            self.declare_parameter("target_frame", "panda_link0")
            self.declare_parameter("confidence_threshold", 0.60)
            self.declare_parameter("stale_after_sec", 0.5)
            self.declare_parameter("sync_tolerance_sec", 0.05)
            self.declare_parameter("sensor_cache_capacity", 60)
            self.declare_parameter("sensor_cache_retention_sec", 2.0)
            self.declare_parameter("sensor_qos_depth", 5)
            self.declare_parameter("center_fraction", 0.30)
            self.declare_parameter("min_depth_m", 0.05)
            self.declare_parameter("max_depth_m", 5.0)
            self.declare_parameter("min_valid_pixels", 9)
            self.declare_parameter("surface_to_center_offset_m", 0.0)
            self.declare_parameter("depth_estimator_mode", "center_median")
            self.declare_parameter("geometry_search_fraction", 1.0)
            self.declare_parameter("geometry_layer_gap_m", 0.015)
            self.declare_parameter("geometry_min_layer_fraction", 0.03)
            self.declare_parameter(
                "geometry_expected_depth_tolerance_m", 0.08
            )
            self.declare_parameter("geometry_ambiguity_margin_m", 0.01)
            self.declare_parameter(
                "geometry_ambiguity_min_support_ratio", 0.50
            )
            self.declare_parameter(
                "geometry_bbox_quantization_margin_px", 0.0
            )
            self.declare_parameter("ground_truth_association_enabled", True)
            self.declare_parameter("association_max_distance_m", 0.08)
            self.declare_parameter("camera_info_topic", "/camera/camera_info")
            self.declare_parameter("depth_topic", "/camera/depth/image_raw")
            self.declare_parameter("selection_roi_min_x_px", -1)
            self.declare_parameter("selection_roi_min_y_px", -1)
            self.declare_parameter("selection_roi_max_x_px", -1)
            self.declare_parameter("selection_roi_max_y_px", -1)
            self.declare_parameter("allow_stationary_latest_tf_fallback", False)
            self.declare_parameter(
                "allow_stationary_sensor_sync_fallback", False
            )
            self.declare_parameter("stationary_sync_tolerance_sec", 0.105)
            self.declare_parameter("stationary_tf_minimum_samples", 5)
            self.declare_parameter("stationary_tf_maximum_joint_delta_rad", 0.002)
            self.declare_parameter("stationary_tf_joint_state_wall_timeout_sec", 0.25)
            self.declare_parameter("stationary_tf_maximum_transform_age_sec", 5.0)
            self.declare_parameter("detections_topic", "/strawberry/detections")
            self.declare_parameter("target_pose_topic", "/strawberry/target_pose")
            self._sensor_callback_group = ReentrantCallbackGroup()
            self._localization_callback_group = MutuallyExclusiveCallbackGroup()
            self._bridge = CvBridge()
            self._sensor_lock = threading.RLock()
            self._joint_lock = threading.RLock()
            self._truth_lock = threading.RLock()
            self._pending_lock = threading.RLock()
            self._pending_detections = {}
            self._sensor_cache = SensorFrameCache(
                capacity=int(self.get_parameter("sensor_cache_capacity").value),
                retention_sec=float(
                    self.get_parameter("sensor_cache_retention_sec").value
                ),
            )
            self._ground_truth_order = None
            self._ground_truth_points = None
            self._ground_truth_stamp = None
            self._arm_joint_names = tuple(
                f"panda_joint{index}" for index in range(1, 8)
            )
            self._joint_samples = deque(maxlen=30)
            self._last_joint_state_wall_sec = None
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._publisher = self.create_publisher(
                TargetPose,
                str(self.get_parameter("target_pose_topic").value),
                10,
            )
            sensor_qos = QoSProfile(
                depth=validate_sensor_qos_depth(
                    self.get_parameter("sensor_qos_depth").value
                )
            )
            sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            sensor_qos.durability = DurabilityPolicy.VOLATILE
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._on_camera_info,
                sensor_qos,
                callback_group=self._sensor_callback_group,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("depth_topic").value),
                self._on_depth,
                sensor_qos,
                callback_group=self._sensor_callback_group,
            )
            self.create_subscription(
                JointState,
                "/joint_states",
                self._on_joint_state,
                qos_profile_sensor_data,
                callback_group=self._sensor_callback_group,
            )
            self.create_subscription(
                StrawberryDetectionArray,
                str(self.get_parameter("detections_topic").value),
                self._on_detections,
                10,
                callback_group=self._localization_callback_group,
            )
            catalog_qos = QoSProfile(depth=1)
            catalog_qos.reliability = ReliabilityPolicy.RELIABLE
            catalog_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                String,
                "/strawberry/ground_truth/catalog",
                self._on_ground_truth_catalog,
                catalog_qos,
                callback_group=self._sensor_callback_group,
            )
            self.create_subscription(
                PoseArray,
                "/strawberry/ground_truth/poses",
                self._on_ground_truth_poses,
                qos_profile_sensor_data,
                callback_group=self._sensor_callback_group,
            )
            self._retry_timer = self.create_timer(
                0.01,
                self._retry_pending_detections,
                callback_group=self._localization_callback_group,
            )

        def prepare_shutdown(self) -> None:
            """Stop background retry work before ROS entities are destroyed."""

            self._retry_timer.cancel()
            with self._pending_lock:
                self._pending_detections.clear()

        def _on_camera_info(self, message) -> None:
            try:
                intrinsics = CameraIntrinsics(
                    fx=float(message.k[0]),
                    fy=float(message.k[4]),
                    cx=float(message.k[2]),
                    cy=float(message.k[5]),
                )
                with self._sensor_lock:
                    self._sensor_cache.add_camera_info(
                        stamp_s=self._seconds(message.header.stamp),
                        frame_id=message.header.frame_id,
                        width=int(message.width),
                        height=int(message.height),
                        intrinsics=intrinsics,
                        payload=message,
                    )
            except (IndexError, LocalizationError, TypeError, ValueError) as exc:
                self.get_logger().warning(f"rejecting CameraInfo: {exc}")

        def _on_depth(self, message) -> None:
            if message.encoding != "32FC1":
                self.get_logger().warning(
                    f"rejecting depth encoding {message.encoding!r}; expected '32FC1'"
                )
                return
            try:
                converted = self._bridge.imgmsg_to_cv2(
                    message, desired_encoding="32FC1"
                )
            except Exception as exc:
                self.get_logger().warning(f"depth conversion failed: {exc}")
                return
            try:
                with self._sensor_lock:
                    self._sensor_cache.add_depth(
                        stamp_s=self._seconds(message.header.stamp),
                        frame_id=message.header.frame_id,
                        image=np.asarray(converted, dtype=np.float32),
                    )
            except (LocalizationError, TypeError, ValueError) as exc:
                self.get_logger().warning(f"rejecting depth frame: {exc}")

        def _on_joint_state(self, message) -> None:
            values = dict(zip(message.name, message.position, strict=False))
            if not all(name in values for name in self._arm_joint_names):
                return
            with self._joint_lock:
                self._joint_samples.append(
                    tuple(float(values[name]) for name in self._arm_joint_names)
                )
                self._last_joint_state_wall_sec = time.monotonic()

        def _arm_is_stationary(self) -> bool:
            with self._joint_lock:
                if self._last_joint_state_wall_sec is None:
                    return False
                if time.monotonic() - self._last_joint_state_wall_sec > float(
                    self.get_parameter(
                        "stationary_tf_joint_state_wall_timeout_sec"
                    ).value
                ):
                    return False
                return joint_samples_are_stationary(
                    tuple(self._joint_samples),
                    minimum_samples=int(
                        self.get_parameter("stationary_tf_minimum_samples").value
                    ),
                    maximum_delta_rad=float(
                        self.get_parameter(
                            "stationary_tf_maximum_joint_delta_rad"
                        ).value
                    ),
                )

        def _joint_state_wall_age_sec(self) -> float | None:
            with self._joint_lock:
                if self._last_joint_state_wall_sec is None:
                    return None
                return time.monotonic() - self._last_joint_state_wall_sec

        def _on_ground_truth_catalog(self, message) -> None:
            try:
                catalog = json.loads(message.data)
                order = tuple(int(value) for value in catalog["pose_array_order"])
                if int(catalog["schema_version"]) != 1:
                    raise ValueError("unsupported schema version")
                if not order or any(value <= 0 for value in order):
                    raise ValueError("target IDs must be positive")
                if len(order) != len(set(order)):
                    raise ValueError("target IDs must be unique")
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.get_logger().error(f"invalid ground-truth catalog: {exc}")
                return
            with self._truth_lock:
                self._ground_truth_order = order

        def _on_ground_truth_poses(self, message) -> None:
            target_frame = str(self.get_parameter("target_frame").value)
            if message.header.frame_id != target_frame:
                self.get_logger().error(
                    "ground-truth PoseArray is not in the target frame"
                )
                return
            with self._truth_lock:
                if self._ground_truth_order is None:
                    return
                if len(message.poses) != len(self._ground_truth_order):
                    self.get_logger().error(
                        "ground-truth PoseArray length does not match its catalog"
                    )
                    return
                self._ground_truth_points = {
                    target_id: (
                        float(pose.position.x),
                        float(pose.position.y),
                        float(pose.position.z),
                    )
                    for target_id, pose in zip(
                        self._ground_truth_order, message.poses
                    )
                }
                self._ground_truth_stamp = message.header.stamp

        @staticmethod
        def _seconds(stamp) -> float:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9

        @staticmethod
        def _stamp_key(message) -> tuple[int, int]:
            stamp = message.header.stamp
            return int(stamp.sec), int(stamp.nanosec)

        def _defer_if_fresh(
            self,
            message,
            *,
            reason: str,
            is_retry: bool,
        ) -> bool:
            detection_stamp_s = self._seconds(message.header.stamp)
            now_stamp_s = float(self.get_clock().now().nanoseconds) * 1e-9
            if not localization_retry_is_fresh(
                detection_stamp_s=detection_stamp_s,
                now_stamp_s=now_stamp_s,
                stale_after_sec=float(
                    self.get_parameter("stale_after_sec").value
                ),
                sync_tolerance_sec=float(
                    self.get_parameter("sync_tolerance_sec").value
                ),
            ):
                if is_retry:
                    self.get_logger().warning(
                        f"deferred localization expired; {reason}"
                    )
                return False
            key = self._stamp_key(message)
            with self._pending_lock:
                previous = self._pending_detections.get(key)
                retry_count = int(previous[1]) + 1 if previous else 0
                self._pending_detections[key] = (message, retry_count)
            if not is_retry:
                self.get_logger().warning(
                    f"deferred localization until coherent inputs arrive; {reason}"
                )
            return True

        def _retry_pending_detections(self) -> None:
            with self._pending_lock:
                pending = list(self._pending_detections.values())
                self._pending_detections.clear()
            for message, _ in pending:
                self._process_detections(message, is_retry=True)

        def _on_detections(self, message) -> None:
            self._process_detections(message, is_retry=False)

        def _process_detections(self, message, *, is_retry: bool) -> None:
            threshold = float(self.get_parameter("confidence_threshold").value)
            try:
                selection_roi = validate_selection_roi(
                    (
                        int(self.get_parameter("selection_roi_min_x_px").value),
                        int(self.get_parameter("selection_roi_min_y_px").value),
                        int(self.get_parameter("selection_roi_max_x_px").value),
                        int(self.get_parameter("selection_roi_max_y_px").value),
                    )
                )
                detection = select_ripe_detection(
                    message.detections,
                    confidence_threshold=threshold,
                    selection_roi_xyxy_px=selection_roi,
                )
            except (TypeError, ValueError) as exc:
                self.get_logger().error(str(exc))
                return
            if detection is None:
                return
            roi = detection.bbox
            detection_stamp_s = self._seconds(message.header.stamp)
            now_stamp_s = float(self.get_clock().now().nanoseconds) * 1e-9
            sync_tolerance_sec = float(
                self.get_parameter("sync_tolerance_sec").value
            )
            try:
                with self._sensor_lock:
                    matched = select_sensor_frames(
                        self._sensor_cache,
                        detection_stamp_s=detection_stamp_s,
                        detection_frame=message.header.frame_id,
                        now_stamp_s=now_stamp_s,
                        sync_tolerance_sec=sync_tolerance_sec,
                        stale_after_sec=float(
                            self.get_parameter("stale_after_sec").value
                        ),
                    )
            except (LocalizationError, TypeError, ValueError) as exc:
                matched = None
                if (
                    str(exc)
                    == "no synchronized depth/CameraInfo pair matches the detection"
                    and bool(
                        self.get_parameter(
                            "allow_stationary_sensor_sync_fallback"
                        ).value
                    )
                    and self._arm_is_stationary()
                ):
                    stationary_tolerance_sec = float(
                        self.get_parameter(
                            "stationary_sync_tolerance_sec"
                        ).value
                    )
                    if stationary_tolerance_sec <= sync_tolerance_sec:
                        self.get_logger().error(
                            "stationary sync tolerance must exceed the nominal "
                            "sync tolerance"
                        )
                        return
                    if stationary_tolerance_sec > float(
                        self.get_parameter("stale_after_sec").value
                    ):
                        self.get_logger().error(
                            "stationary sync tolerance cannot exceed the "
                            "sensor freshness bound"
                        )
                        return
                    try:
                        with self._sensor_lock:
                            matched = select_sensor_frames(
                                self._sensor_cache,
                                detection_stamp_s=detection_stamp_s,
                                detection_frame=message.header.frame_id,
                                now_stamp_s=now_stamp_s,
                                sync_tolerance_sec=stationary_tolerance_sec,
                                stale_after_sec=float(
                                    self.get_parameter("stale_after_sec").value
                                ),
                            )
                        self.get_logger().warning(
                            "used bounded stationary sensor-sync fallback; "
                            f"detection_stamp_sec={detection_stamp_s:.6f}; "
                            "depth_delta_sec="
                            f"{matched.depth.stamp_s - detection_stamp_s:.6f}; "
                            "camera_info_delta_sec="
                            f"{matched.camera_info.stamp_s - detection_stamp_s:.6f}"
                        )
                    except (LocalizationError, TypeError, ValueError):
                        matched = None
                if matched is None:
                    with self._sensor_lock:
                        diagnostics = self._sensor_cache.diagnostics(
                            detection_stamp_s=detection_stamp_s,
                            detection_frame=message.header.frame_id,
                        )
                    reason = (
                        f"{exc}; detection_stamp_sec={detection_stamp_s:.6f}; "
                        f"cache={json.dumps(diagnostics, sort_keys=True)}"
                    )
                    if (
                        str(exc)
                        == (
                            "no synchronized depth/CameraInfo pair matches "
                            "the detection"
                        )
                        and self._defer_if_fresh(
                            message,
                            reason=reason,
                            is_retry=is_retry,
                        )
                    ):
                        return
                    if not is_retry:
                        self.get_logger().warning(reason)
                    return

            try:
                depth = matched.depth.image
                camera_info = matched.camera_info
                image_height, image_width = depth.shape
                box = BoundingBox(
                    x=int(roi.x_offset),
                    y=int(roi.y_offset),
                    width=int(roi.width),
                    height=int(roi.height),
                )
                validate_bbox_within_image(box, image_width, image_height)
                point, estimate = localize_bbox(
                    depth,
                    box,
                    camera_info.intrinsics,
                    center_fraction=float(
                        self.get_parameter("center_fraction").value
                    ),
                    min_depth_m=float(self.get_parameter("min_depth_m").value),
                    max_depth_m=float(self.get_parameter("max_depth_m").value),
                    min_valid_pixels=int(
                        self.get_parameter("min_valid_pixels").value
                    ),
                    surface_to_center_offset_m=float(
                        self.get_parameter("surface_to_center_offset_m").value
                    ),
                    depth_estimator_mode=str(
                        self.get_parameter("depth_estimator_mode").value
                    ),
                    geometry_search_fraction=float(
                        self.get_parameter("geometry_search_fraction").value
                    ),
                    geometry_layer_gap_m=float(
                        self.get_parameter("geometry_layer_gap_m").value
                    ),
                    geometry_min_layer_fraction=float(
                        self.get_parameter("geometry_min_layer_fraction").value
                    ),
                    geometry_expected_depth_tolerance_m=float(
                        self.get_parameter(
                            "geometry_expected_depth_tolerance_m"
                        ).value
                    ),
                    geometry_ambiguity_margin_m=float(
                        self.get_parameter("geometry_ambiguity_margin_m").value
                    ),
                    geometry_ambiguity_min_support_ratio=float(
                        self.get_parameter(
                            "geometry_ambiguity_min_support_ratio"
                        ).value
                    ),
                    geometry_bbox_quantization_margin_px=float(
                        self.get_parameter(
                            "geometry_bbox_quantization_margin_px"
                        ).value
                    ),
                )
            except (LocalizationError, TypeError, ValueError) as exc:
                self.get_logger().warning(str(exc))
                return

            camera_pose = PoseStamped()
            camera_pose.header = message.header
            camera_pose.pose.position.x = float(point[0])
            camera_pose.pose.position.y = float(point[1])
            camera_pose.pose.position.z = float(point[2])
            camera_pose.pose.orientation.w = 1.0
            target_frame = str(self.get_parameter("target_frame").value)
            try:
                base_pose = self._tf_buffer.transform(
                    camera_pose, target_frame, timeout=Duration(seconds=0.2)
                )
            except Exception as exc:
                fallback_allowed = bool(
                    self.get_parameter(
                        "allow_stationary_latest_tf_fallback"
                    ).value
                )
                arm_stationary = (
                    self._arm_is_stationary() if fallback_allowed else False
                )
                if not fallback_allowed or not arm_stationary:
                    reason = (
                        f"TF transform failed: {exc}; "
                        f"stationary_fallback_allowed={fallback_allowed}; "
                        f"arm_stationary={arm_stationary}; "
                        "joint_state_wall_age_sec="
                        f"{self._joint_state_wall_age_sec()}"
                    )
                    if self._defer_if_fresh(
                        message,
                        reason=reason,
                        is_retry=is_retry,
                    ):
                        return
                    if not is_retry:
                        self.get_logger().warning(reason)
                    return
                try:
                    latest = self._tf_buffer.lookup_transform(
                        target_frame,
                        camera_pose.header.frame_id,
                        Time(),
                        timeout=Duration(seconds=0.2),
                    )
                    transform_age = (
                        detection_stamp_s - self._seconds(latest.header.stamp)
                    )
                    maximum_age = float(
                        self.get_parameter(
                            "stationary_tf_maximum_transform_age_sec"
                        ).value
                    )
                    if (
                        transform_age < -sync_tolerance_sec
                        or transform_age > maximum_age
                    ):
                        raise RuntimeError(
                            "latest dynamic TF age is outside the stationary "
                            f"fallback bound: {transform_age:.3f} s"
                        )
                    base_pose = tf2_geometry_msgs.do_transform_pose_stamped(
                        camera_pose, latest
                    )
                    # The spatial transform is sampled at the latest stationary
                    # joint state, but the localized observation retains the
                    # acquisition stamp.
                    base_pose.header.stamp = message.header.stamp
                    self.get_logger().warning(
                        "used bounded latest-TF fallback while Panda was stationary"
                    )
                except Exception as fallback_exc:
                    reason = (
                        f"TF transform failed: {exc}; latest fallback failed: "
                        f"{fallback_exc}"
                    )
                    if self._defer_if_fresh(
                        message,
                        reason=reason,
                        is_retry=is_retry,
                    ):
                        return
                    if not is_retry:
                        self.get_logger().warning(reason)
                    return

            target_id = int(detection.target_id)
            if bool(
                self.get_parameter("ground_truth_association_enabled").value
            ):
                with self._truth_lock:
                    ground_truth_points = (
                        dict(self._ground_truth_points)
                        if self._ground_truth_points is not None
                        else None
                    )
                    ground_truth_stamp = self._ground_truth_stamp
                if ground_truth_points is None or ground_truth_stamp is None:
                    reason = "waiting for simulation ground-truth identity catalog"
                    if self._defer_if_fresh(
                        message,
                        reason=reason,
                        is_retry=is_retry,
                    ):
                        return
                    if not is_retry:
                        self.get_logger().warning(reason)
                    return
                truth_age = abs(
                    self._seconds(message.header.stamp)
                    - self._seconds(ground_truth_stamp)
                )
                if truth_age > float(self.get_parameter("stale_after_sec").value):
                    self.get_logger().warning(
                        "rejecting stale ground-truth identity association"
                    )
                    return
                try:
                    target_id = associate_nearest_target(
                        (
                            base_pose.pose.position.x,
                            base_pose.pose.position.y,
                            base_pose.pose.position.z,
                        ),
                        ground_truth_points,
                        max_distance_m=float(
                            self.get_parameter("association_max_distance_m").value
                        ),
                    )
                except (LocalizationError, ValueError) as exc:
                    self.get_logger().warning(str(exc))
                    return

            result = TargetPose()
            result.header = base_pose.header
            result.target_id = target_id
            result.pose = base_pose.pose
            result.detection_confidence = detection.confidence
            result.position_sigma_m = max(float(estimate.sigma_m), 1e-6)
            self._publisher.publish(result)
            if is_retry:
                self.get_logger().info(
                    "recovered deferred localization; "
                    f"detection_stamp_sec={detection_stamp_s:.6f}"
                )

    # Keep the context valid while callbacks and TF subscriptions drain.
    # The default rclpy SIGTERM handler shuts the context first, which can
    # leave executor tasks raising InvalidHandle during process-group cleanup.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = LocalizationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    stop_requested = threading.Event()
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def _request_stop(signum, frame) -> None:
        del signum, frame
        stop_requested.set()

    signal.signal(signal.SIGTERM, _request_stop)
    try:
        while rclpy.ok() and not stop_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok() and not stop_requested.is_set():
            raise
    finally:
        node.prepare_shutdown()
        try:
            callbacks_stopped = shutdown_executor_and_wait(
                executor, timeout_sec=10.0
            )
            if not callbacks_stopped:
                node.get_logger().error(
                    "localization callbacks did not stop before shutdown"
                )
        except RuntimeError:
            if rclpy.ok():
                raise
        try:
            node.destroy_node()
        except RuntimeError:
            if rclpy.ok():
                raise
        rclpy.try_shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
