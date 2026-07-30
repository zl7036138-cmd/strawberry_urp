#!/usr/bin/env python3
"""Show the live field-v3 wrist image with detections and 3D target data."""

from __future__ import annotations

import argparse
from typing import Any


def main(args=None) -> int:  # pragma: no cover - interactive ROS/Qt display
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--image-topic",
        default="/camera/wrist/color/image_raw",
    )
    parser.add_argument(
        "--detections-topic",
        default="/strawberry/demo/detections",
    )
    parser.add_argument(
        "--target-topic",
        default="/strawberry/demo/target_pose",
    )
    parser.add_argument(
        "--window-title",
        default="Field-v3 | Wrist YOLO + 3D localization",
    )
    parser.add_argument(
        "--banner",
        default="FIELD-v3 WRIST PERCEPTION | DEMO ONLY | NO ROBOT MOTION",
    )
    parser.add_argument(
        "--roi",
        nargs=4,
        type=int,
        metavar=("X1", "Y1", "X2", "Y2"),
        default=(320, 240, 640, 480),
    )
    options, ros_args = parser.parse_known_args(args)

    try:
        import cv2
        from cv_bridge import CvBridge
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from strawberry_interfaces.msg import (
            StrawberryDetectionArray,
            TargetPose,
        )
    except ImportError as exc:
        raise RuntimeError(
            "live perception display requires ROS 2, OpenCV, cv_bridge, "
            "and strawberry_interfaces"
        ) from exc

    maturity_names = {
        0: "UNKNOWN",
        1: "RIPE",
        2: "UNRIPE",
    }
    maturity_colors = {
        0: (80, 180, 255),
        1: (45, 55, 235),
        2: (40, 205, 75),
    }

    class Display(Node):
        def __init__(self) -> None:
            super().__init__("field_v3_perception_display")
            self.bridge = CvBridge()
            self.image = None
            self.detections = []
            self.target = None
            self.image_count = 0
            self.detection_count = 0
            self.target_count = 0
            self.create_subscription(
                Image,
                options.image_topic,
                self._on_image,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                StrawberryDetectionArray,
                options.detections_topic,
                self._on_detections,
                10,
            )
            self.create_subscription(
                TargetPose,
                options.target_topic,
                self._on_target,
                10,
            )

        def _on_image(self, message: Any) -> None:
            self.image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            ).copy()
            self.image_count += 1

        def _on_detections(self, message: Any) -> None:
            self.detections = list(message.detections)
            self.detection_count += 1

        def _on_target(self, message: Any) -> None:
            self.target = message
            self.target_count += 1

    def draw_text(
        canvas,
        value: str,
        origin: tuple[int, int],
        *,
        scale: float = 0.52,
        color: tuple[int, int, int] = (255, 255, 255),
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            canvas,
            value,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def render(node: Display):
        if node.image is None:
            canvas = cv2.UMat(480, 640, cv2.CV_8UC3).get()
            canvas[:] = (24, 28, 34)
            draw_text(
                canvas,
                f"Waiting for {options.image_topic} ...",
                (55, 240),
                scale=0.7,
                color=(170, 205, 245),
                thickness=2,
            )
            return canvas

        canvas = node.image.copy()
        height, width = canvas.shape[:2]
        roi_x1, roi_y1, roi_x2, roi_y2 = options.roi
        roi_x1 = max(0, min(width - 1, roi_x1))
        roi_y1 = max(0, min(height - 1, roi_y1))
        roi_x2 = max(roi_x1 + 1, min(width, roi_x2))
        roi_y2 = max(roi_y1 + 1, min(height, roi_y2))

        overlay = canvas.copy()
        cv2.rectangle(
            overlay,
            (0, 0),
            (width, 58),
            (15, 20, 27),
            -1,
        )
        cv2.addWeighted(overlay, 0.74, canvas, 0.26, 0.0, canvas)
        draw_text(
            canvas,
            options.banner,
            (12, 23),
            scale=0.54,
            color=(245, 248, 252),
            thickness=2,
        )
        draw_text(
            canvas,
            (
                f"image={node.image_count}  detection-msg={node.detection_count}"
                f"  target-msg={node.target_count}"
            ),
            (12, 47),
            scale=0.44,
            color=(170, 205, 240),
        )

        cv2.rectangle(
            canvas,
            (roi_x1, roi_y1),
            (roi_x2 - 1, roi_y2 - 1),
            (255, 210, 50),
            2,
        )
        draw_text(
            canvas,
            "TARGET-1 ROI",
            (roi_x1 + 7, min(height - 8, roi_y1 + 22)),
            scale=0.48,
            color=(255, 220, 80),
            thickness=2,
        )

        for detection in node.detections:
            x1 = max(0, int(detection.bbox.x_offset))
            y1 = max(0, int(detection.bbox.y_offset))
            x2 = min(width - 1, x1 + int(detection.bbox.width))
            y2 = min(height - 1, y1 + int(detection.bbox.height))
            maturity = int(detection.maturity)
            color = maturity_colors.get(maturity, maturity_colors[0])
            label = (
                f"{maturity_names.get(maturity, 'UNKNOWN')} "
                f"{float(detection.confidence):.2f} "
                f"det-id={int(detection.target_id)}"
            )
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 3)
            label_y = max(70, y1 - 8)
            draw_text(
                canvas,
                label,
                (x1 + 3, label_y),
                scale=0.47,
                color=color,
                thickness=2,
            )

        if node.target is not None:
            position = node.target.pose.position
            target_lines = (
                (
                    f"3D target={int(node.target.target_id)} "
                    f"confidence={float(node.target.detection_confidence):.2f}"
                ),
                (
                    f"panda_link0 xyz = "
                    f"[{float(position.x):+.3f}, "
                    f"{float(position.y):+.3f}, "
                    f"{float(position.z):+.3f}] m"
                ),
                (
                    f"sigma={1000.0 * float(node.target.position_sigma_m):.2f} mm"
                ),
            )
            panel_y = height - 84
            cv2.rectangle(
                canvas,
                (8, panel_y - 24),
                (width - 8, height - 8),
                (18, 25, 32),
                -1,
            )
            for index, line in enumerate(target_lines):
                draw_text(
                    canvas,
                    line,
                    (18, panel_y + 24 * index),
                    scale=0.50,
                    color=(235, 242, 248),
                    thickness=2 if index == 0 else 1,
                )
        return canvas

    rclpy.init(args=ros_args)
    node = Display()
    cv2.namedWindow(options.window_title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(options.window_title, 960, 720)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            cv2.imshow(options.window_title, render(node))
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
