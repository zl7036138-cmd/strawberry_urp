"""ROS 2 adapter around Ultralytics inference.

Heavy runtime dependencies are confined to this module so ``core`` remains
importable on Windows and in unit-test environments without ROS 2.
"""

from __future__ import annotations

import signal
import threading
from typing import Any, Callable, Dict, List

from .core import (
    parse_yolo_detections,
    validate_model_class_contract,
    validate_runtime_parameters,
)


# Ultralytics 8.4.92 interprets NumPy HWC sources as BGR and reverses their
# channels during preprocessing.  CvBridge must therefore supply BGR here even
# though the ROS camera contract itself is rgb8.
ULTRALYTICS_NUMPY_ENCODING = "bgr8"


def _result_rows(result: Any) -> List[List[float]]:
    """Convert one Ultralytics result to dependency-free rows."""

    if result.boxes is None:
        return []
    xyxy = result.boxes.xyxy.detach().cpu().tolist()
    confidences = result.boxes.conf.detach().cpu().tolist()
    class_ids = result.boxes.cls.detach().cpu().tolist()
    return [
        list(box) + [confidence, class_id]
        for box, confidence, class_id in zip(xyxy, confidences, class_ids)
    ]


def _prediction_arguments(
    source: Any,
    image_size: int,
    confidence_threshold: float,
    nms_iou_threshold: float,
    device: str,
) -> Dict[str, Any]:
    """Build the explicit Ultralytics prediction contract."""

    return {
        "source": source,
        "imgsz": image_size,
        "conf": confidence_threshold,
        "iou": nms_iou_threshold,
        "device": device,
        "verbose": False,
    }


def _run_guarded_frame(
    process_frame: Callable[[], Any],
    publish: Callable[[Any], None],
    log_error: Callable[[str], None],
) -> bool:
    """Process and publish one frame without leaking adapter exceptions."""

    try:
        output = process_frame()
        publish(output)
    except Exception as error:  # noqa: BLE001 - a ROS callback must survive bad frames
        log_error(
            "Dropping image frame after perception failure "
            f"({type(error).__name__}): {error}"
        )
        return False
    return True


def main(args: Any = None) -> None:
    try:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.executors import (
            ExternalShutdownException,
            SingleThreadedExecutor,
        )
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image
        from strawberry_interfaces.msg import StrawberryDetection, StrawberryDetectionArray
        from ultralytics import YOLO
    except ImportError as error:  # pragma: no cover - exercised in ROS environment
        raise RuntimeError(
            "perception_node requires ROS 2, cv_bridge, strawberry_interfaces, and ultralytics"
        ) from error

    class PerceptionNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_perception")
            self.declare_parameter("model_path", "weights/yolo11s_640_best.pt")
            self.declare_parameter("image_topic", "/camera/color/image_raw")
            self.declare_parameter("detections_topic", "/strawberry/detections")
            self.declare_parameter("confidence_threshold", 0.60)
            self.declare_parameter("image_size", 640)
            self.declare_parameter("nms_iou_threshold", 0.70)
            self.declare_parameter("device", "0")

            runtime_parameters = validate_runtime_parameters(
                model_path=self.get_parameter("model_path").value,
                confidence_threshold=self.get_parameter("confidence_threshold").value,
                image_size=self.get_parameter("image_size").value,
                nms_iou_threshold=self.get_parameter("nms_iou_threshold").value,
            )
            self._threshold = runtime_parameters.confidence_threshold
            self._image_size = runtime_parameters.image_size
            self._nms_iou_threshold = runtime_parameters.nms_iou_threshold
            self._device = str(self.get_parameter("device").value)
            self._bridge = CvBridge()
            self._model = YOLO(runtime_parameters.model_path)
            self._class_names = validate_model_class_contract(self._model.names)
            output_topic = str(self.get_parameter("detections_topic").value)
            image_topic = str(self.get_parameter("image_topic").value)
            self._publisher = self.create_publisher(StrawberryDetectionArray, output_topic, 10)
            self._subscription = self.create_subscription(
                Image, image_topic, self._on_image, qos_profile_sensor_data
            )

        def _process_image(self, image_message: Any) -> Any:
            image = self._bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding=ULTRALYTICS_NUMPY_ENCODING,
            )
            results = self._model.predict(
                **_prediction_arguments(
                    source=image,
                    image_size=self._image_size,
                    confidence_threshold=self._threshold,
                    nms_iou_threshold=self._nms_iou_threshold,
                    device=self._device,
                )
            )
            result = results[0]
            detections = parse_yolo_detections(
                _result_rows(result),
                self._class_names,
                confidence_threshold=self._threshold,
                image_size=(int(image_message.width), int(image_message.height)),
                # Zero is reserved as the invalid/no-target action identifier.
                target_id_offset=1,
                strict=False,
            )

            output = StrawberryDetectionArray()
            output.header = image_message.header
            for detection in detections:
                item = StrawberryDetection()
                item.target_id = detection.target_id
                item.maturity = int(detection.maturity)
                item.confidence = float(detection.confidence)
                item.bbox.x_offset = detection.roi.x_offset
                item.bbox.y_offset = detection.roi.y_offset
                item.bbox.width = detection.roi.width
                item.bbox.height = detection.roi.height
                item.bbox.do_rectify = False
                output.detections.append(item)
            return output

        def _on_image(self, image_message: Any) -> None:
            _run_guarded_frame(
                process_frame=lambda: self._process_image(image_message),
                publish=self._publisher.publish,
                log_error=self.get_logger().error,
            )

    # Preserve a valid context until the inference callback and ROS entities
    # have been drained during process-group SIGTERM cleanup.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = None
    executor = None
    stop_requested = threading.Event()
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def _request_stop(signum, frame) -> None:
        del signum, frame
        stop_requested.set()

    signal.signal(signal.SIGTERM, _request_stop)
    try:
        node = PerceptionNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not stop_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=10.0)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm_handler)
