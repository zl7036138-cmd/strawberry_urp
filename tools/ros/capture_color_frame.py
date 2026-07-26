"""Capture one ROS 2 RGB image as a PNG for visual scene verification."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    return parser.parse_args()


def main() -> None:
    options = parse_args()
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_scene_frame_capture")
            self.message = None
            self.subscription = self.create_subscription(
                Image,
                options.topic,
                self._on_image,
                qos_profile_sensor_data,
            )

        def _on_image(self, message: Image) -> None:
            if self.message is None:
                self.message = message

    rclpy.init()
    node = CaptureNode()
    deadline = time.monotonic() + options.timeout_sec
    try:
        while node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.message is None:
            raise TimeoutError(f"no image received from {options.topic}")
        message = node.message
        if message.encoding not in {"rgb8", "bgr8"}:
            raise RuntimeError(f"unsupported image encoding {message.encoding}")
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(
            int(message.height), int(message.step)
        )
        image = row[:, : int(message.width) * 3].reshape(
            int(message.height), int(message.width), 3
        )
        if message.encoding == "rgb8":
            image = image[:, :, ::-1]
        options.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(options.output), np.ascontiguousarray(image)):
            raise RuntimeError(f"failed to encode {options.output}")
        print(
            json.dumps(
                {
                    "topic": options.topic,
                    "output": str(options.output),
                    "width": int(message.width),
                    "height": int(message.height),
                    "encoding": str(message.encoding),
                    "stamp_sec": int(message.header.stamp.sec),
                    "stamp_nanosec": int(message.header.stamp.nanosec),
                },
                sort_keys=True,
            )
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
