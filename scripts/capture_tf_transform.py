#!/usr/bin/env python3
"""Capture one live TF transform as non-overwriting JSON evidence."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time

def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-frame", required=True)
    parser.add_argument("--target-frame", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    options = parser.parse_args()
    if options.output.exists():
        raise ValueError(f"refusing to overwrite {options.output}")
    rclpy.init()
    node = Node("strawberry_tf_capture")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    del listener
    transform = None
    deadline = time.monotonic() + options.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform(
                    options.source_frame, options.target_frame, Time()
                )
                break
            except TransformException:
                pass
        if transform is None:
            raise RuntimeError(
                f"timed out waiting for {options.source_frame} <- "
                f"{options.target_frame}"
            )
        value = transform.transform
        payload = {
            "source_frame": options.source_frame,
            "target_frame": options.target_frame,
            "translation_m": [
                float(value.translation.x),
                float(value.translation.y),
                float(value.translation.z),
            ],
            "rotation_xyzw": [
                float(value.rotation.x),
                float(value.rotation.y),
                float(value.rotation.z),
                float(value.rotation.w),
            ],
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
