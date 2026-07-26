#!/usr/bin/env python3
"""Capture one ROS RGB image for visual verification of a materialized asset."""

from __future__ import annotations

import argparse
from pathlib import Path
import time


def main() -> int:  # pragma: no cover - exercised in ROS integration
    import cv2
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    options = parser.parse_args()
    if options.output.exists():
        raise SystemExit(f"refusing to overwrite {options.output}")
    if options.timeout_sec <= 0.0:
        raise SystemExit("timeout must be positive")

    rclpy.init()
    node = Node("strawberry_single_rgb_capture")
    bridge = CvBridge()
    received = []
    subscription = node.create_subscription(
        Image, options.topic, received.append, qos_profile_sensor_data
    )
    try:
        deadline = time.monotonic() + options.timeout_sec
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not received:
            raise RuntimeError(f"timed out waiting for {options.topic}")
        image = bridge.imgmsg_to_cv2(received[-1], desired_encoding="bgr8")
        options.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(options.output), image):
            raise RuntimeError(f"failed to write {options.output}")
        print(options.output)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
