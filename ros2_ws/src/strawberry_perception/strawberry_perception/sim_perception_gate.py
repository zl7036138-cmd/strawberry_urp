"""Run the leakage-safe simple-scene simulator perception pre-gate."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Callable, Sequence

from .shadow_diagnostic_core import box_iou, project_sphere
from .sim_gate_core import (
    RIPE,
    UNRIPE,
    default_sim_perception_scenarios,
    summarize_sim_perception_gate,
)


ORIGINAL_POSITIONS = {
    "strawberry_1": (0.42, -0.12, 0.52),
    "strawberry_2": (0.44, 0.04, 0.54),
    "strawberry_3": (0.38, 0.17, 0.50),
}
PARKED_POSITIONS = {
    "strawberry_1": (0.0, -2.0, 1.0),
    "strawberry_2": (0.2, -2.0, 1.0),
    "strawberry_3": (0.4, -2.0, 1.0),
}
FRUIT_ID_BY_MODEL = {
    "strawberry_1": 1,
    "strawberry_2": 2,
    "strawberry_3": 3,
}


class GateRuntimeError(RuntimeError):
    """Raised when one required live simulator observation is unavailable."""


def _stamp_key(stamp: object) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(
        sum((float(first) - float(second)) ** 2 for first, second in zip(left, right))
    )


def _fingerprint(path: Path) -> dict[str, object]:
    content = path.read_bytes()
    return {
        "path": str(path),
        "size_bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--model-path", type=Path, required=True)
    parser.add_argument("--runner-script", type=Path, required=True)
    parser.add_argument("--frames-per-scenario", type=int, default=10)
    parser.add_argument("--sensor-timeout-sec", type=float, default=10.0)
    parser.add_argument("--post-detection-wait-sec", type=float, default=1.0)
    parser.add_argument("--association-iou", type=float, default=0.25)
    return parser.parse_args(argv)


def main(args=None) -> int:  # pragma: no cover - exercised in ROS integration
    try:
        from ament_index_python.packages import get_package_share_directory
        import rclpy
        import strawberry_perception
        import strawberry_sim
        import tf2_geometry_msgs  # noqa: F401 - register PoseStamped conversion
        from geometry_msgs.msg import PoseStamped
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.utilities import remove_ros_args
        from ros_gz_interfaces.msg import Entity
        from ros_gz_interfaces.srv import SetEntityPose
        from sensor_msgs.msg import CameraInfo
        from strawberry_interfaces.msg import StrawberryDetectionArray, TargetPose
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        raise RuntimeError(
            "sim_perception_gate requires ROS 2, ros_gz_interfaces, tf2, and project packages"
        ) from error

    application_args = remove_ros_args(args=args if args is not None else sys.argv)
    options = _parse_args(application_args[1:])
    if options.frames_per_scenario <= 0:
        raise ValueError("frames-per-scenario must be positive")
    if options.sensor_timeout_sec <= 0.0 or options.post_detection_wait_sec < 0.0:
        raise ValueError("timeouts must be positive or zero where allowed")
    if not 0.0 < options.association_iou <= 1.0:
        raise ValueError("association-iou must be in (0, 1]")
    model_path = options.model_path.expanduser().resolve(strict=True)
    runner_script = options.runner_script.expanduser().resolve(strict=True)
    for output in (options.output_json, options.output_csv):
        if output.exists():
            raise ValueError(f"refusing to overwrite gate artifact: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
    ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not 0 <= ros_domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID must be between 0 and 232")

    scenarios = default_sim_perception_scenarios()

    class SimPerceptionGateNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_sim_perception_gate")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.camera_info = None
            self.truth: dict[int, object] = {}
            self.detection_sequence = 0
            self.detections: list[tuple[int, object]] = []
            self.target_pose_ids_by_stamp: dict[tuple[int, int], set[int]] = {}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pose_client = self.create_client(
                SetEntityPose, "/world/strawberry_orchard/set_pose"
            )
            self.create_subscription(
                CameraInfo,
                "/camera/camera_info",
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            for target_id in (1, 2, 3):
                self.create_subscription(
                    PoseStamped,
                    f"/strawberry/ground_truth/fruit_{target_id}/pose",
                    lambda message, identity=target_id: self._on_truth(identity, message),
                    qos_profile_sensor_data,
                )
            self.create_subscription(
                StrawberryDetectionArray,
                "/strawberry/shadow/detections",
                self._on_detections,
                10,
            )
            self.create_subscription(
                TargetPose,
                "/strawberry/shadow/target_pose",
                self._on_target_pose,
                10,
            )

        def _on_camera_info(self, message) -> None:
            self.camera_info = message

        def _on_truth(self, target_id: int, message) -> None:
            self.truth[target_id] = message

        def _on_detections(self, message) -> None:
            self.detection_sequence += 1
            self.detections.append((self.detection_sequence, message))
            if len(self.detections) > 1000:
                del self.detections[:500]

        def _on_target_pose(self, message) -> None:
            key = _stamp_key(message.header.stamp)
            self.target_pose_ids_by_stamp.setdefault(key, set()).add(
                int(message.target_id)
            )

        def wait_for(
            self,
            predicate: Callable[[], bool],
            timeout_sec: float,
            description: str,
        ) -> None:
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and time.monotonic() < deadline:
                if predicate():
                    return
                rclpy.spin_once(self, timeout_sec=0.05)
            if not predicate():
                raise GateRuntimeError(f"timed out waiting for {description}")

        def spin_for_wall_time(self, duration_sec: float) -> None:
            deadline = time.monotonic() + duration_sec
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

        def set_model_pose(self, model_name: str, position: Sequence[float]) -> None:
            if not self.pose_client.wait_for_service(timeout_sec=5.0):
                raise GateRuntimeError("Gazebo set_pose service is unavailable")
            request = SetEntityPose.Request()
            request.entity.name = model_name
            request.entity.type = Entity.MODEL
            request.pose.position.x = float(position[0])
            request.pose.position.y = float(position[1])
            request.pose.position.z = float(position[2])
            request.pose.orientation.w = 1.0
            future = self.pose_client.call_async(request)
            self.wait_for(future.done, 5.0, f"set_pose response for {model_name}")
            if future.exception() is not None or not future.result().success:
                raise GateRuntimeError(f"Gazebo rejected set_pose for {model_name}")

        def current_truth(self, target_id: int) -> tuple[float, float, float]:
            message = self.truth[target_id]
            position = message.pose.position
            return float(position.x), float(position.y), float(position.z)

        def configure_scenario(self, scenario) -> None:
            desired = dict(PARKED_POSITIONS)
            desired[scenario.model_name] = scenario.position_m
            previous_detection_sequence = self.detection_sequence
            for model_name, position in desired.items():
                self.set_model_pose(model_name, position)
            self.wait_for(
                lambda: (
                    len(self.truth) == 3
                    and all(
                        _distance(
                            self.current_truth(FRUIT_ID_BY_MODEL[model_name]), position
                        )
                        <= 0.001
                        for model_name, position in desired.items()
                    )
                ),
                options.sensor_timeout_sec,
                f"settled truth for {scenario.scenario_id}",
            )
            self.wait_for(
                lambda: self.detection_sequence >= previous_detection_sequence + 5,
                options.sensor_timeout_sec,
                f"five settled detector frames for {scenario.scenario_id}",
            )

        def collect_frames(self, scenario):
            start_sequence = self.detection_sequence
            required_sequence = start_sequence + options.frames_per_scenario
            self.wait_for(
                lambda: self.detection_sequence >= required_sequence,
                options.sensor_timeout_sec,
                f"{options.frames_per_scenario} detector frames for {scenario.scenario_id}",
            )
            selected = [
                message
                for sequence, message in self.detections
                if start_sequence < sequence <= required_sequence
            ]
            if len(selected) != options.frames_per_scenario:
                raise GateRuntimeError("detector frame cache did not preserve the requested window")
            self.spin_for_wall_time(options.post_detection_wait_sec)
            return selected

        def projected_truth_box(self, scenario, stamp) -> tuple[float, float, float, float]:
            if self.camera_info is None:
                raise GateRuntimeError("CameraInfo is unavailable")
            truth_pose = PoseStamped()
            truth_pose.header = self.truth[scenario.target_id].header
            truth_pose.header.stamp = stamp
            truth_pose.pose = self.truth[scenario.target_id].pose
            optical_frame = str(self.camera_info.header.frame_id)
            try:
                camera_pose = self.tf_buffer.transform(
                    truth_pose,
                    optical_frame,
                    timeout=Duration(seconds=0.5),
                )
            except Exception as error:
                raise GateRuntimeError(f"truth-to-camera TF failed: {error}") from error
            position = camera_pose.pose.position
            projection = project_sphere(
                target_id=scenario.target_id,
                maturity=scenario.expected_maturity,
                center_in_source_m=(position.x, position.y, position.z),
                source_to_camera_translation_m=(0.0, 0.0, 0.0),
                source_to_camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                intrinsics=(
                    float(self.camera_info.k[0]),
                    float(self.camera_info.k[4]),
                    float(self.camera_info.k[2]),
                    float(self.camera_info.k[5]),
                ),
                image_size=(int(self.camera_info.width), int(self.camera_info.height)),
                radius_m=0.035,
            )
            if projection is None:
                raise GateRuntimeError("visible fruit did not project into the image")
            return projection.bbox_xyxy

        def evaluate_frame(self, scenario, message, frame_index: int) -> dict:
            truth_box = self.projected_truth_box(scenario, message.header.stamp)
            detections = []
            for item in message.detections:
                roi = item.bbox
                predicted_box = (
                    float(roi.x_offset),
                    float(roi.y_offset),
                    float(roi.x_offset + roi.width),
                    float(roi.y_offset + roi.height),
                )
                overlap = box_iou(predicted_box, truth_box)
                maturity = (
                    RIPE
                    if int(item.maturity) == int(item.RIPE)
                    else UNRIPE
                    if int(item.maturity) == int(item.UNRIPE)
                    else "UNKNOWN"
                )
                detections.append(
                    {
                        "maturity": maturity,
                        "confidence": float(item.confidence),
                        "bbox_xyxy": list(predicted_box),
                        "truth_iou": overlap,
                        "truth_match": overlap >= options.association_iou,
                    }
                )
            stamp = _stamp_key(message.header.stamp)
            ripe_truth_match = any(
                item["maturity"] == RIPE and item["truth_match"] for item in detections
            )
            unripe_truth_match = any(
                item["maturity"] == UNRIPE and item["truth_match"] for item in detections
            )
            target_pose_received = scenario.target_id in self.target_pose_ids_by_stamp.get(
                stamp, set()
            )
            return {
                "scenario_id": scenario.scenario_id,
                "frame_index": frame_index,
                "stamp_sec": stamp[0],
                "stamp_nanosec": stamp[1],
                "expected_target_id": scenario.target_id,
                "expected_maturity": scenario.expected_maturity,
                "detection_count": len(detections),
                "ripe_detection_count": sum(item["maturity"] == RIPE for item in detections),
                "unripe_detection_count": sum(
                    item["maturity"] == UNRIPE for item in detections
                ),
                "ripe_truth_match": ripe_truth_match,
                "unripe_truth_match": unripe_truth_match,
                "false_ripe": bool(
                    scenario.expected_maturity == UNRIPE
                    and any(item["maturity"] == RIPE for item in detections)
                ),
                "target_pose_received": target_pose_received,
                "detections": detections,
            }

        def restore_scene_best_effort(self) -> list[str]:
            errors = []
            for model_name, position in ORIGINAL_POSITIONS.items():
                try:
                    self.set_model_pose(model_name, position)
                except Exception as error:  # noqa: BLE001 - preserve all cleanup errors
                    errors.append(f"{model_name}: {error}")
            return errors

    package_source = Path(strawberry_perception.__file__).resolve().parent
    simulation_source = Path(strawberry_sim.__file__).resolve().parent
    bringup_share = Path(get_package_share_directory("strawberry_bringup")).resolve()
    sim_share = Path(get_package_share_directory("strawberry_sim")).resolve()
    provenance = {
        "perception/source/perception_node.py": package_source / "perception_node.py",
        "perception/source/sim_gate_core.py": package_source / "sim_gate_core.py",
        "perception/source/sim_perception_gate.py": package_source / "sim_perception_gate.py",
        "simulation/source/ground_truth_publisher.py": simulation_source / "ground_truth_publisher.py",
        "bringup/launch/system.launch.py": bringup_share / "launch" / "system.launch.py",
        "simulation/config/scene.yaml": sim_share / "config" / "scene.yaml",
        "simulation/worlds/strawberry_orchard.sdf": sim_share / "worlds" / "strawberry_orchard.sdf",
        "runner/run_sim_perception_gate.sh": runner_script,
        "model/weights.pt": model_path,
    }
    missing = [label for label, path in provenance.items() if not path.is_file()]
    if missing:
        raise RuntimeError("missing pre-gate provenance files: " + ", ".join(missing))

    rclpy.init(args=args)
    node = SimPerceptionGateNode()
    frame_records = []
    cleanup_errors = []
    try:
        node.wait_for(
            lambda: node.camera_info is not None and len(node.truth) == 3,
            options.sensor_timeout_sec,
            "CameraInfo and all fruit truth topics",
        )
        for scenario in scenarios:
            node.get_logger().info(f"Running {scenario.scenario_id}")
            node.configure_scenario(scenario)
            messages = node.collect_frames(scenario)
            frame_records.extend(
                node.evaluate_frame(scenario, message, index)
                for index, message in enumerate(messages, start=1)
            )
    finally:
        cleanup_errors = node.restore_scene_best_effort()
        node.destroy_node()
        rclpy.try_shutdown()

    metrics = summarize_sim_perception_gate(
        frame_records,
        expected_scenarios=scenarios,
        frames_per_scenario=options.frames_per_scenario,
    )
    summary = {
        "schema_version": 1,
        "gate": "SIM_PERCEPTION_PRE_GATE",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "ros_domain_id": ros_domain_id,
        "control_source": "none",
        "perception_topic": "/strawberry/shadow/detections",
        "target_pose_topic": "/strawberry/shadow/target_pose",
        "association_iou_minimum": options.association_iou,
        "model": _fingerprint(model_path),
        "scenarios": [
            {
                "scenario_id": scenario.scenario_id,
                "target_id": scenario.target_id,
                "model_name": scenario.model_name,
                "expected_maturity": scenario.expected_maturity,
                "position_m": list(scenario.position_m),
            }
            for scenario in scenarios
        ],
        "metrics": metrics,
        "cleanup_errors": cleanup_errors,
        "provenance": {
            label: _fingerprint(path) for label, path in sorted(provenance.items())
        },
        "frames": frame_records,
        "passed": bool(metrics["passed"] and not cleanup_errors),
        "acceptance_scope": (
            "simple camera-clear simulator pre-gate only; not T30, P3, or real-image evidence"
        ),
    }
    options.output_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_fields = (
        "scenario_id",
        "frame_index",
        "stamp_sec",
        "stamp_nanosec",
        "expected_target_id",
        "expected_maturity",
        "detection_count",
        "ripe_detection_count",
        "unripe_detection_count",
        "ripe_truth_match",
        "unripe_truth_match",
        "false_ripe",
        "target_pose_received",
    )
    with options.output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(frame_records)
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 1
