#!/usr/bin/env python3
"""Record a split-screen field-v3 submission demonstration with live status."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import signal
import time
from typing import Any


WIDTH = 1280
HEIGHT = 720
VISUAL_WIDTH = 900
PANEL_X = 920


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:  # pragma: no cover - exercised in ROS integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--maximum-duration-sec", type=float, default=420.0)
    parser.add_argument(
        "--base-image-topic",
        default="/camera/base/color/image_raw",
    )
    parser.add_argument(
        "--wrist-image-topic",
        default="/camera/wrist/color/image_raw",
    )
    parser.add_argument(
        "--detections-topic",
        default="/strawberry/field_demo/detections",
    )
    parser.add_argument(
        "--target-topic",
        default="/strawberry/field_demo/target_pose",
    )
    parser.add_argument(
        "--status-topic",
        default="/strawberry/field_demo/trial_status",
    )
    arguments = parser.parse_args()
    if arguments.fps <= 0.0 or arguments.maximum_duration_sec <= 0.0:
        raise SystemExit("fps and maximum duration must be positive")

    try:
        import cv2
        from cv_bridge import CvBridge
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from strawberry_interfaces.msg import (
            StrawberryDetectionArray,
            TargetPose,
        )
    except ImportError as exc:
        raise SystemExit(
            f"field-v3 recorder dependencies are unavailable: {exc}"
        ) from exc

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("field_v3_submission_recorder")
            self.bridge = CvBridge()
            self.base_image = None
            self.wrist_image = None
            self.detections: list[dict[str, Any]] = []
            self.target: dict[str, Any] | None = None
            self.status: dict[str, Any] = {}
            self.counts = {
                "base_images": 0,
                "wrist_images": 0,
                "detection_messages": 0,
                "target_messages": 0,
                "status_messages": 0,
            }
            self.stages: list[str] = []
            self.outcomes: list[str] = []
            self.create_subscription(
                Image,
                arguments.base_image_topic,
                self._on_base_image,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                arguments.wrist_image_topic,
                self._on_wrist_image,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                StrawberryDetectionArray,
                arguments.detections_topic,
                self._on_detections,
                10,
            )
            self.create_subscription(
                TargetPose,
                arguments.target_topic,
                self._on_target,
                10,
            )
            self.create_subscription(
                String,
                arguments.status_topic,
                self._on_status,
                10,
            )

        def _image(self, message: Any) -> Any:
            return self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            ).copy()

        def _on_base_image(self, message: Any) -> None:
            self.base_image = self._image(message)
            self.counts["base_images"] += 1

        def _on_wrist_image(self, message: Any) -> None:
            self.wrist_image = self._image(message)
            self.counts["wrist_images"] += 1

        def _on_detections(self, message: Any) -> None:
            self.counts["detection_messages"] += 1
            self.detections = [
                {
                    "target_id": int(item.target_id),
                    "maturity": int(item.maturity),
                    "confidence": float(item.confidence),
                    "x": int(item.bbox.x_offset),
                    "y": int(item.bbox.y_offset),
                    "width": int(item.bbox.width),
                    "height": int(item.bbox.height),
                }
                for item in message.detections
            ]

        def _on_target(self, message: Any) -> None:
            self.counts["target_messages"] += 1
            self.target = {
                "target_id": int(message.target_id),
                "confidence": float(message.detection_confidence),
                "sigma_m": float(message.position_sigma_m),
                "xyz_m": [
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                    float(message.pose.position.z),
                ],
            }

        def _on_status(self, message: Any) -> None:
            self.counts["status_messages"] += 1
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if not isinstance(value, dict):
                return
            self.status = value
            stage = str(value.get("state") or "")
            outcome = str(value.get("outcome") or "")
            if stage and (not self.stages or self.stages[-1] != stage):
                self.stages.append(stage)
            if outcome and outcome not in self.outcomes:
                self.outcomes.append(outcome)

    def text(
        image: Any,
        value: str,
        origin: tuple[int, int],
        *,
        scale: float = 0.52,
        color: tuple[int, int, int] = (236, 242, 248),
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            image,
            value,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def fit(source: Any, width: int, height: int) -> Any:
        if source is None:
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            canvas[:] = (22, 28, 35)
            text(
                canvas,
                "Waiting for camera stream ...",
                (max(15, width // 2 - 150), height // 2),
                scale=0.65,
                color=(145, 184, 224),
                thickness=2,
            )
            return canvas
        source_height, source_width = source.shape[:2]
        ratio = min(width / source_width, height / source_height)
        resized_width = max(1, int(source_width * ratio))
        resized_height = max(1, int(source_height * ratio))
        resized = cv2.resize(source, (resized_width, resized_height))
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:] = (18, 23, 29)
        x = (width - resized_width) // 2
        y = (height - resized_height) // 2
        canvas[y : y + resized_height, x : x + resized_width] = resized
        return canvas

    def annotate_wrist(node: Recorder) -> Any:
        if node.wrist_image is None:
            return None
        image = node.wrist_image.copy()
        height, width = image.shape[:2]
        cv2.rectangle(
            image,
            (320, 240),
            (min(639, width - 1), min(479, height - 1)),
            (255, 210, 50),
            2,
        )
        for item in node.detections:
            x1 = max(0, item["x"])
            y1 = max(0, item["y"])
            x2 = min(width - 1, x1 + item["width"])
            y2 = min(height - 1, y1 + item["height"])
            ripe = item["maturity"] == 1
            color = (45, 55, 235) if ripe else (40, 205, 75)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
            text(
                image,
                (
                    f"{'RIPE' if ripe else 'UNRIPE'} "
                    f"{item['confidence']:.2f} id={item['target_id']}"
                ),
                (x1 + 3, max(20, y1 - 7)),
                scale=0.50,
                color=color,
                thickness=2,
            )
        return image

    def render(node: Recorder, elapsed: float) -> Any:
        canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        canvas[:] = (15, 20, 27)
        base = fit(node.base_image, VISUAL_WIDTH, 398)
        wrist = fit(annotate_wrist(node), VISUAL_WIDTH, 290)
        canvas[0:398, 0:VISUAL_WIDTH] = base
        canvas[414:704, 0:VISUAL_WIDTH] = wrist
        cv2.rectangle(canvas, (0, 0), (VISUAL_WIDTH - 1, 36), (10, 15, 21), -1)
        text(
            canvas,
            "BASE OVERVIEW | field-v3 workcell",
            (14, 25),
            scale=0.62,
            thickness=2,
        )
        cv2.rectangle(
            canvas,
            (0, 414),
            (VISUAL_WIDTH - 1, 450),
            (10, 15, 21),
            -1,
        )
        text(
            canvas,
            "WRIST RGB-D | YOLO + target-1 ROI",
            (14, 439),
            scale=0.62,
            thickness=2,
        )

        x = PANEL_X
        text(
            canvas,
            "FIELD-v3 SUBMISSION DEMO",
            (x, 35),
            scale=0.64,
            color=(110, 190, 245),
            thickness=2,
        )
        text(
            canvas,
            "SIMULATION | FIXED SCENE/TARGET",
            (x, 62),
            scale=0.43,
            color=(140, 165, 188),
        )
        cv2.line(canvas, (x, 80), (WIDTH - 18, 80), (70, 82, 96), 1)
        stage = str(node.status.get("state") or "WAITING")
        outcome = str(node.status.get("outcome") or "-")
        progress = float(node.status.get("progress") or 0.0)
        text(canvas, "ACTION STAGE", (x, 115), color=(140, 190, 240))
        text(canvas, stage, (x, 151), scale=0.78, thickness=2)
        text(canvas, f"Outcome: {outcome}", (x, 183), scale=0.50)
        text(canvas, f"Progress: {100.0 * progress:5.1f}%", (x, 211), scale=0.50)
        text(canvas, f"Elapsed: {elapsed:6.1f} s", (x, 239), scale=0.50)
        cv2.line(canvas, (x, 258), (WIDTH - 18, 258), (70, 82, 96), 1)

        text(canvas, "PERCEPTION", (x, 291), color=(140, 190, 240))
        text(
            canvas,
            f"Detections: {len(node.detections)}",
            (x, 320),
            scale=0.50,
        )
        if node.target is None:
            text(canvas, "3D target: waiting", (x, 349), scale=0.50)
        else:
            target = node.target
            xyz = target["xyz_m"]
            text(
                canvas,
                (
                    f"Target {target['target_id']} "
                    f"conf={target['confidence']:.2f}"
                ),
                (x, 349),
                scale=0.49,
            )
            text(
                canvas,
                f"x={xyz[0]:+.3f} y={xyz[1]:+.3f}",
                (x, 378),
                scale=0.47,
            )
            text(
                canvas,
                f"z={xyz[2]:+.3f} sigma={1000.0 * target['sigma_m']:.1f}mm",
                (x, 407),
                scale=0.44,
            )
        cv2.line(canvas, (x, 430), (WIDTH - 18, 430), (70, 82, 96), 1)
        text(canvas, "OBSERVED STAGES", (x, 462), color=(140, 190, 240))
        for index, value in enumerate(node.stages[-7:]):
            text(
                canvas,
                f"{index + 1}. {value}",
                (x, 491 + index * 27),
                scale=0.47,
            )
        text(
            canvas,
            "No hardware / damage / sim-to-real claim",
            (x, 699),
            scale=0.37,
            color=(120, 150, 182),
        )
        return canvas

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.receipt.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(arguments.output),
        cv2.VideoWriter_fourcc(*"MJPG"),
        arguments.fps,
        (WIDTH, HEIGHT),
    )
    if not writer.isOpened():
        raise SystemExit("cannot open field-v3 MJPG video writer")

    rclpy.init()
    node = Recorder()
    stopping = False

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    start = time.monotonic()
    next_frame = start
    frames = 0
    try:
        while not stopping:
            now = time.monotonic()
            if now - start >= arguments.maximum_duration_sec:
                break
            rclpy.spin_once(node, timeout_sec=min(0.02, 1.0 / arguments.fps))
            now = time.monotonic()
            if now >= next_frame:
                writer.write(render(node, now - start))
                frames += 1
                next_frame += 1.0 / arguments.fps
                if now - next_frame > 1.0:
                    next_frame = now
    finally:
        duration = time.monotonic() - start
        writer.release()
        node.destroy_node()
        rclpy.try_shutdown()

    receipt = {
        "schema_version": 1,
        "kind": "field_v3_submission_demo_recording",
        "scope": "NON_FORMAL_FIXED_SCENE_PERCEPTION_PICK",
        "topics": {
            "base_image": arguments.base_image_topic,
            "wrist_image": arguments.wrist_image_topic,
            "detections": arguments.detections_topic,
            "target_pose": arguments.target_topic,
            "status": arguments.status_topic,
        },
        "video": {
            "path": arguments.output.name,
            "size_bytes": arguments.output.stat().st_size,
            "sha256": _sha256(arguments.output),
            "codec": "MJPG",
            "width": WIDTH,
            "height": HEIGHT,
            "fps": arguments.fps,
            "frames": frames,
            "duration_sec": duration,
        },
        "observations": {
            **node.counts,
            "stages": node.stages,
            "outcomes": node.outcomes,
            "target_id": None if node.target is None else node.target["target_id"],
        },
        "safety": {
            "formal_evidence": False,
            "fixed_scene": True,
            "fixed_target": True,
            "held_out_real_test_consumed": False,
            "physical_robot_evidence": False,
            "fruit_damage_evidence": False,
            "sim_to_real_claim": False,
        },
    }
    arguments.receipt.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
