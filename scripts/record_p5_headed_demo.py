#!/usr/bin/env python3
"""Record the headed demonstration camera with detections and trial state."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import signal
import time
from typing import Any


CANVAS_WIDTH = 1280
CANVAS_HEIGHT = 720
CAMERA_WIDTH = 960


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
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--maximum-duration-sec", type=float, default=240.0)
    parser.add_argument(
        "--detections-topic", default="/strawberry/shadow/detections"
    )
    parser.add_argument("--control-source", default="ORACLE ONLY")
    parser.add_argument(
        "--model-role", default="REJECTED MODEL IS SHADOW-ONLY"
    )
    arguments = parser.parse_args()
    if arguments.fps <= 0.0:
        raise SystemExit("--fps must be positive")

    try:
        import cv2
        from cv_bridge import CvBridge
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from strawberry_interfaces.msg import StrawberryDetectionArray
    except ImportError as exc:
        raise SystemExit(f"headed recorder dependencies are unavailable: {exc}") from exc

    class Recorder(Node):
        def __init__(self) -> None:
            super().__init__("p5_headed_demo_recorder")
            self.bridge = CvBridge()
            self.image = None
            self.detections: list[dict[str, Any]] = []
            self.status: dict[str, Any] = {}
            self.image_messages = 0
            self.detection_messages = 0
            self.status_messages = 0
            self.ripe_detection_seen = False
            self.maximum_detection_count = 0
            self.states: list[str] = []
            self.outcomes: list[str] = []
            self.create_subscription(
                Image,
                "/camera/color/image_raw",
                self._on_image,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                StrawberryDetectionArray,
                arguments.detections_topic,
                self._on_detections,
                10,
            )
            self.create_subscription(
                String,
                "/strawberry/trial_status",
                self._on_status,
                10,
            )

        def _on_image(self, message: Any) -> None:
            self.image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            ).copy()
            self.image_messages += 1

        def _on_detections(self, message: Any) -> None:
            self.detection_messages += 1
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
            self.maximum_detection_count = max(
                self.maximum_detection_count, len(self.detections)
            )
            self.ripe_detection_seen = self.ripe_detection_seen or any(
                item["maturity"] == 1 for item in self.detections
            )

        def _on_status(self, message: Any) -> None:
            self.status_messages += 1
            try:
                value = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if not isinstance(value, dict):
                return
            self.status = value
            state = str(value.get("state") or "")
            outcome = str(value.get("outcome") or "")
            if state and (not self.states or self.states[-1] != state):
                self.states.append(state)
            if outcome and outcome not in self.outcomes:
                self.outcomes.append(outcome)

    def text(
        image: Any,
        value: str,
        origin: tuple[int, int],
        *,
        scale: float = 0.62,
        color: tuple[int, int, int] = (235, 240, 245),
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

    def render(node: Recorder, elapsed: float) -> Any:
        canvas = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH, 3), dtype=np.uint8)
        canvas[:, :CAMERA_WIDTH] = (18, 22, 27)
        canvas[:, CAMERA_WIDTH:] = (28, 34, 42)
        if node.image is None:
            text(
                canvas,
                "Waiting for /camera/color/image_raw ...",
                (190, 350),
                scale=0.9,
                color=(160, 190, 230),
                thickness=2,
            )
        else:
            source = node.image.copy()
            height, width = source.shape[:2]
            for item in node.detections:
                x1 = max(0, item["x"])
                y1 = max(0, item["y"])
                x2 = min(width - 1, x1 + item["width"])
                y2 = min(height - 1, y1 + item["height"])
                ripe = item["maturity"] == 1
                color = (45, 55, 235) if ripe else (40, 205, 75)
                label = (
                    f"{'RIPE' if ripe else 'UNRIPE'} "
                    f"{item['confidence']:.2f} id={item['target_id']}"
                )
                cv2.rectangle(source, (x1, y1), (x2, y2), color, 3)
                cv2.rectangle(
                    source,
                    (x1, max(0, y1 - 26)),
                    (min(width - 1, x1 + 230), y1),
                    color,
                    -1,
                )
                text(
                    source,
                    label,
                    (x1 + 4, max(18, y1 - 7)),
                    scale=0.55,
                    color=(255, 255, 255),
                    thickness=2,
                )
            canvas[:, :CAMERA_WIDTH] = cv2.resize(
                source, (CAMERA_WIDTH, CANVAS_HEIGHT)
            )

        cv2.rectangle(
            canvas, (0, 0), (CAMERA_WIDTH - 1, 48), (12, 18, 25), -1
        )
        text(
            canvas,
            "NON-FORMAL HEADED DEMO | camera + Shadow perception",
            (18, 32),
            scale=0.68,
            color=(255, 255, 255),
            thickness=2,
        )
        x = CAMERA_WIDTH + 18
        text(canvas, "CONTROL", (x, 42), scale=0.55, color=(130, 180, 240))
        text(canvas, arguments.control_source, (x, 72), scale=0.70, thickness=2)
        text(
            canvas,
            arguments.model_role,
            (x, 104),
            scale=0.43,
            color=(80, 190, 255),
        )
        cv2.line(
            canvas,
            (x, 124),
            (CANVAS_WIDTH - 18, 124),
            (65, 76, 88),
            1,
        )
        state = str(node.status.get("state") or "WAITING")
        outcome = str(node.status.get("outcome") or "-")
        text(canvas, "STATE", (x, 160), scale=0.55, color=(130, 180, 240))
        text(canvas, state, (x, 197), scale=0.88, thickness=2)
        text(canvas, f"Outcome: {outcome}", (x, 232), scale=0.55)
        text(canvas, f"Elapsed: {elapsed:6.1f} s", (x, 270), scale=0.55)
        text(
            canvas,
            f"Detections: {len(node.detections)}",
            (x, 306),
            scale=0.55,
        )
        text(
            canvas,
            f"Shadow frames: {node.status.get('shadow_detection_frames', 0)}",
            (x, 340),
            scale=0.51,
        )
        text(
            canvas,
            f"Shadow ripe: {node.status.get('shadow_ripe_detection_count', 0)}",
            (x, 374),
            scale=0.51,
        )
        text(
            canvas,
            f"Planning: {node.status.get('planning_time_sec') or '-'}",
            (x, 408),
            scale=0.48,
        )
        cv2.line(
            canvas,
            (x, 430),
            (CANVAS_WIDTH - 18, 430),
            (65, 76, 88),
            1,
        )
        text(canvas, "Observed states", (x, 460), scale=0.55, color=(130, 180, 240))
        for index, value in enumerate(node.states[-7:]):
            text(canvas, f"{index + 1}. {value}", (x, 490 + 29 * index), scale=0.49)
        text(
            canvas,
            "SIMULATION ONLY | no real-robot claim",
            (x, 700),
            scale=0.43,
            color=(120, 160, 205),
        )
        return canvas

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.receipt.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(arguments.output),
        cv2.VideoWriter_fourcc(*"MJPG"),
        arguments.fps,
        (CANVAS_WIDTH, CANVAS_HEIGHT),
    )
    if not writer.isOpened():
        raise SystemExit("cannot open MJPG video writer")

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
        "kind": "p5_headed_demo_camera_recording",
        "scope": "NON_FORMAL_HEADED_DEVELOPMENT_DEMO",
        "control_source": arguments.control_source,
        "detections_topic": arguments.detections_topic,
        "model_role": arguments.model_role,
        "video": {
            "path": arguments.output.name,
            "size_bytes": arguments.output.stat().st_size,
            "sha256": _sha256(arguments.output),
            "codec": "MJPG",
            "width": CANVAS_WIDTH,
            "height": CANVAS_HEIGHT,
            "fps": arguments.fps,
            "frames": frames,
            "duration_sec": duration,
        },
        "observations": {
            "image_messages": node.image_messages,
            "detection_messages": node.detection_messages,
            "status_messages": node.status_messages,
            "ripe_detection_seen": node.ripe_detection_seen,
            "maximum_detection_count": node.maximum_detection_count,
            "states": node.states,
            "outcomes": node.outcomes,
        },
        "safety": {
            "formal_evidence": False,
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_used_for_control": False,
            "physical_robot_evidence": False,
            "sim_to_real_claim": False,
        },
    }
    arguments.receipt.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
