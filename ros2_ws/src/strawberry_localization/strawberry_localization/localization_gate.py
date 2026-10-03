"""Run the T40 depth/TF gate over 100 distinct Gazebo truth positions."""

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
from typing import Callable, Optional, Sequence

from .core import CameraIntrinsics, LocalizationError
from .gate_core import benchmark_positions, sphere_projection_bbox, summarize_gate
from .v2_gate_contract import (
    load_contract as load_v2_gate_contract,
    position_grid as v2_position_grid,
    validate_world_receipt,
)


FRUIT_RADIUS_M = 0.035
TARGET_MODEL = "strawberry_1"
ORIGINAL_POSITIONS = {
    "strawberry_1": (0.42, -0.12, 0.52),
    "strawberry_2": (0.44, 0.04, 0.54),
    "strawberry_3": (0.38, 0.17, 0.50),
}
PARKED_POSITIONS = {
    "strawberry_2": (0.0, -2.0, 1.0),
    "strawberry_3": (0.2, -2.0, 1.0),
}

# These installed strawberry_sim assets determine the T40 camera stream,
# transforms, fruit geometry, ground truth, and set-pose test fixture.  The
# scene uses separate ripe/unripe models; there is no models/strawberry path.
SIMULATION_SHARE_PROVENANCE_PATHS = (
    "launch/sim.launch.py",
    "worlds/strawberry_orchard.sdf",
    "models/rgbd_camera/model.sdf",
    "models/strawberry_ripe/model.sdf",
    "models/strawberry_unripe/model.sdf",
    "config/bridge.yaml",
    "config/scene.yaml",
    "config/sim_nodes.yaml",
    "config/panda_initial_positions.yaml",
    "urdf/panda_gz.urdf.xacro",
)
SIMULATION_SOURCE_PROVENANCE_PATHS = (
    "core.py",
    "ground_truth_publisher.py",
)


class GateRuntimeError(RuntimeError):
    """Raised when the simulator cannot produce one required measurement."""


def _stamp_key(stamp: object) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def _distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(first, second)))


def _file_fingerprint(path: Path) -> dict[str, object]:
    content = path.read_bytes()
    return {
        "name": path.name,
        "size_bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--attempts-per-position", type=int, default=3)
    parser.add_argument("--sensor-timeout-sec", type=float, default=5.0)
    parser.add_argument("--runner-timeout-sec", type=float, default=360.0)
    parser.add_argument("--launch-headless", action="store_true")
    parser.add_argument("--runner-script", type=Path, required=True)
    parser.add_argument(
        "--domain-selection-mode", choices=("automatic", "explicit"), required=True
    )
    parser.add_argument("--domain-probe-spin-sec", type=float, required=True)
    parser.add_argument("--gate-contract", type=Path)
    parser.add_argument("--repository-root", type=Path)
    parser.add_argument("--materialized-world", type=Path)
    parser.add_argument("--world-receipt", type=Path)
    return parser.parse_args(argv)


def main(args: Optional[Sequence[str]] = None) -> int:  # pragma: no cover - ROS gate
    try:
        from ament_index_python.packages import get_package_share_directory
        import rclpy
        import strawberry_sim
        from geometry_msgs.msg import PoseStamped
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.utilities import remove_ros_args
        from ros_gz_interfaces.msg import Entity
        from ros_gz_interfaces.srv import SetEntityPose
        from sensor_msgs.msg import CameraInfo, Image
        from strawberry_interfaces.msg import (
            StrawberryDetection,
            StrawberryDetectionArray,
            TargetPose,
        )
        import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped conversion
        from tf2_ros import Buffer, TransformListener
    except ImportError as error:
        raise RuntimeError(
            "localization_gate requires ROS 2, ros_gz_interfaces, and tf2_geometry_msgs"
        ) from error

    application_args = remove_ros_args(args=args if args is not None else sys.argv)
    options = _parse_args(application_args[1:])
    if options.attempts_per_position <= 0:
        raise ValueError("attempts-per-position must be positive")
    if options.sensor_timeout_sec <= 0.0:
        raise ValueError("sensor-timeout-sec must be positive")
    if options.runner_timeout_sec <= 0.0:
        raise ValueError("runner-timeout-sec must be positive")
    if options.domain_probe_spin_sec <= 0.0:
        raise ValueError("domain-probe-spin-sec must be positive")
    try:
        ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError as error:
        raise ValueError("ROS_DOMAIN_ID must be an integer") from error
    if not 0 <= ros_domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID must be between 0 and 232")

    v2_contract = None
    if options.gate_contract is not None:
        if (
            options.repository_root is None
            or options.materialized_world is None
            or options.world_receipt is None
        ):
            raise ValueError(
                "v2 gate requires repository root, materialized world, "
                "and world receipt"
            )
        v2_contract = load_v2_gate_contract(
            options.gate_contract,
            options.repository_root,
        )
        validate_world_receipt(
            options.world_receipt,
            options.materialized_world,
            v2_contract,
        )
        runtime = v2_contract["runtime"]
        if (
            options.attempts_per_position
            != int(runtime["attempts_per_position"])
            or options.sensor_timeout_sec
            != float(runtime["sensor_timeout_sec"])
            or options.runner_timeout_sec
            != float(runtime["runner_timeout_sec"])
            or not options.launch_headless
        ):
            raise ValueError("runtime arguments differ from the frozen v2 contract")
        target_model = str(v2_contract["target"]["model_name"])
        fruit_radius_m = float(v2_contract["target"]["fruit_radius_m"])
        original_positions = {
            str(model_name): tuple(float(value) for value in position)
            for model_name, position in v2_contract[
                "scene_materialization"
            ]["canonical_restore_positions_m"].items()
        }
        parked_positions = {
            str(model_name): tuple(float(value) for value in pose[:3])
            for model_name, pose in v2_contract[
                "scene_materialization"
            ]["parked_fruit_poses_xyz_rpy"].items()
        }
        positions = v2_position_grid(v2_contract)
        gate_name = str(v2_contract["gate_id"])
        gate_scope = str(v2_contract["scope"])
        localization_config_name = "localization_blender_v2.yaml"
        launch_name = "blender_v2_localization_gate.launch.py"
        thresholds = v2_contract["thresholds"]
        minimum_positions = int(thresholds["minimum_positions"])
        median_limit_mm = float(thresholds["median_error_mm_max"])
        p95_limit_mm = float(thresholds["p95_error_mm_max"])
    else:
        if any(
            value is not None
            for value in (
                options.repository_root,
                options.materialized_world,
                options.world_receipt,
            )
        ):
            raise ValueError("v2-only arguments require --gate-contract")
        target_model = TARGET_MODEL
        fruit_radius_m = FRUIT_RADIUS_M
        original_positions = ORIGINAL_POSITIONS
        parked_positions = PARKED_POSITIONS
        positions = benchmark_positions()
        gate_name = "T40_localization"
        gate_scope = "FORMAL_T40_LOCALIZATION"
        localization_config_name = "localization.yaml"
        launch_name = "localization_gate.launch.py"
        minimum_positions = 100
        median_limit_mm = 15.0
        p95_limit_mm = 30.0

    package_source = Path(__file__).resolve().parent
    package_share = Path(
        get_package_share_directory("strawberry_localization")
    ).resolve()
    simulation_source = Path(strawberry_sim.__file__).resolve().parent
    simulation_share = Path(get_package_share_directory("strawberry_sim")).resolve()
    runner_script = options.runner_script.resolve()
    provenance_files = {
        "source/core.py": package_source / "core.py",
        "source/gate_core.py": package_source / "gate_core.py",
        "source/localization_gate.py": package_source / "localization_gate.py",
        "source/node.py": package_source / "node.py",
        f"config/{localization_config_name}": (
            package_share / "config" / localization_config_name
        ),
        f"launch/{launch_name}": (
            package_share / "launch" / launch_name
        ),
        f"runner/{runner_script.name}": runner_script,
    }
    if v2_contract is not None:
        provenance_files.update(
            {
                "source/v2_gate_contract.py": (
                    package_source / "v2_gate_contract.py"
                ),
                "contract/gate.json": options.gate_contract.resolve(
                    strict=True
                ),
                "contract/materialized_world.sdf": (
                    options.materialized_world.resolve(strict=True)
                ),
                "contract/materialized_world_receipt.json": (
                    options.world_receipt.resolve(strict=True)
                ),
            }
        )
        for label, value in v2_contract["_resolved_paths"].items():
            provenance_files[f"contract/binding/{label}"] = Path(value)
    provenance_files.update(
        {
            f"strawberry_sim/share/{relative_path}": (
                simulation_share / relative_path
            )
            for relative_path in SIMULATION_SHARE_PROVENANCE_PATHS
        }
    )
    provenance_files.update(
        {
            f"strawberry_sim/source/{relative_path}": (
                simulation_source / relative_path
            )
            for relative_path in SIMULATION_SOURCE_PROVENANCE_PATHS
        }
    )
    missing_provenance_files = [
        label for label, path in provenance_files.items() if not path.is_file()
    ]
    if missing_provenance_files:
        raise RuntimeError(
            f"cannot fingerprint installed {gate_name} files: "
            + ", ".join(missing_provenance_files)
        )

    class LocalizationGateNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_localization_gate")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.camera_info = None
            self.depth = None
            self.depth_sequence = 0
            self.truth = None
            self.truth_sequence = 0
            self.target = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.detection_publisher = self.create_publisher(
                StrawberryDetectionArray, "/strawberry/detections", 10
            )
            self.pose_client = self.create_client(
                SetEntityPose, "/world/strawberry_orchard/set_pose"
            )
            self.create_subscription(
                CameraInfo,
                "/camera/camera_info",
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/camera/depth/image_raw",
                self._on_depth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PoseStamped,
                "/strawberry/ground_truth/fruit_1/pose",
                self._on_truth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                TargetPose, "/strawberry/target_pose", self._on_target, 10
            )

        def _on_camera_info(self, message: object) -> None:
            self.camera_info = message

        def _on_depth(self, message: object) -> None:
            self.depth = message
            self.depth_sequence += 1

        def _on_truth(self, message: object) -> None:
            self.truth = message
            self.truth_sequence += 1

        def _on_target(self, message: object) -> None:
            self.target = message

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
            if predicate():
                return
            raise GateRuntimeError(f"timed out waiting for {description}")

        def set_model_pose(self, model_name: str, xyz: Sequence[float]) -> None:
            if not self.pose_client.wait_for_service(timeout_sec=5.0):
                raise GateRuntimeError("Gazebo set_pose service is unavailable")
            request = SetEntityPose.Request()
            request.entity.name = model_name
            request.entity.type = Entity.MODEL
            request.pose.position.x = float(xyz[0])
            request.pose.position.y = float(xyz[1])
            request.pose.position.z = float(xyz[2])
            request.pose.orientation.w = 1.0
            future = self.pose_client.call_async(request)
            self.wait_for(future.done, 5.0, f"set_pose response for {model_name}")
            if future.exception() is not None:
                raise GateRuntimeError(
                    f"set_pose failed for {model_name}: {future.exception()}"
                )
            if not future.result().success:
                raise GateRuntimeError(f"Gazebo rejected set_pose for {model_name}")

        def current_truth_xyz(self) -> tuple[float, float, float]:
            if self.truth is None:
                raise GateRuntimeError("fruit truth pose is unavailable")
            position = self.truth.pose.position
            return float(position.x), float(position.y), float(position.z)

        def wait_for_position(self, expected: Sequence[float]) -> None:
            previous_truth_sequence = self.truth_sequence
            self.wait_for(
                lambda: (
                    self.truth_sequence > previous_truth_sequence
                    and self.truth is not None
                    and _distance(self.current_truth_xyz(), expected) <= 0.001
                ),
                options.sensor_timeout_sec,
                "updated fruit ground truth",
            )
            previous_depth_sequence = self.depth_sequence
            self.wait_for(
                lambda: self.depth_sequence >= previous_depth_sequence + 3,
                options.sensor_timeout_sec,
                "three settled RGB-D frames",
            )

        def intrinsics(self) -> CameraIntrinsics:
            if self.camera_info is None:
                raise GateRuntimeError("CameraInfo is unavailable")
            return CameraIntrinsics(
                fx=float(self.camera_info.k[0]),
                fy=float(self.camera_info.k[4]),
                cx=float(self.camera_info.k[2]),
                cy=float(self.camera_info.k[5]),
            )

        def oracle_box(self):
            if self.camera_info is None or self.depth is None or self.truth is None:
                raise GateRuntimeError("camera, depth, and truth inputs are required")
            optical_frame = str(self.camera_info.header.frame_id)
            if optical_frame != "strawberry_camera_optical_frame":
                raise GateRuntimeError(
                    f"unexpected CameraInfo frame: {optical_frame!r}"
                )
            truth_pose = PoseStamped()
            truth_pose.header = self.truth.header
            truth_pose.header.stamp = self.depth.header.stamp
            truth_pose.pose = self.truth.pose
            try:
                camera_pose = self.tf_buffer.transform(
                    truth_pose,
                    optical_frame,
                    timeout=Duration(seconds=0.5),
                )
            except Exception as error:
                raise GateRuntimeError(f"truth-to-camera TF failed: {error}") from error
            point = camera_pose.pose.position
            camera_xyz = (float(point.x), float(point.y), float(point.z))
            box = sphere_projection_bbox(
                camera_xyz,
                self.intrinsics(),
                sphere_radius_m=fruit_radius_m,
                image_width=int(self.camera_info.width),
                image_height=int(self.camera_info.height),
            )
            return box, camera_xyz

        def request_estimate(self):
            if self.depth is None:
                raise GateRuntimeError("depth input is unavailable")
            box, camera_xyz = self.oracle_box()
            message = StrawberryDetectionArray()
            message.header = self.depth.header
            detection = StrawberryDetection()
            detection.target_id = 1
            detection.maturity = StrawberryDetection.RIPE
            detection.confidence = 1.0
            detection.bbox.x_offset = box.x
            detection.bbox.y_offset = box.y
            detection.bbox.width = box.width
            detection.bbox.height = box.height
            detection.bbox.do_rectify = False
            message.detections.append(detection)

            expected_stamp = _stamp_key(message.header.stamp)
            self.target = None
            self.detection_publisher.publish(message)
            self.wait_for(
                lambda: (
                    self.target is not None
                    and _stamp_key(self.target.header.stamp) == expected_stamp
                ),
                options.sensor_timeout_sec,
                "localized TargetPose",
            )
            if int(self.target.target_id) != 1:
                raise GateRuntimeError(
                    f"localized target identity is {self.target.target_id}, expected 1"
                )
            return self.target, box, camera_xyz, expected_stamp

        def measure_position(self, index: int, desired: Sequence[float]):
            self.set_model_pose(target_model, desired)
            self.wait_for_position(desired)
            attempt_errors = []
            for attempt in range(1, options.attempts_per_position + 1):
                if attempt > 1:
                    previous_depth_sequence = self.depth_sequence
                    self.wait_for(
                        lambda: self.depth_sequence > previous_depth_sequence,
                        options.sensor_timeout_sec,
                        "retry depth frame",
                    )
                try:
                    target, box, camera_xyz, stamp = self.request_estimate()
                except (GateRuntimeError, LocalizationError) as error:
                    attempt_errors.append(str(error))
                    continue
                truth_xyz = self.current_truth_xyz()
                estimated_xyz = (
                    float(target.pose.position.x),
                    float(target.pose.position.y),
                    float(target.pose.position.z),
                )
                error_mm = 1000.0 * _distance(estimated_xyz, truth_xyz)
                return {
                    "index": index,
                    "desired_xyz_m": list(map(float, desired)),
                    "truth_xyz_m": list(truth_xyz),
                    "estimated_xyz_m": list(estimated_xyz),
                    "camera_truth_xyz_m": list(camera_xyz),
                    "bbox": {
                        "x": box.x,
                        "y": box.y,
                        "width": box.width,
                        "height": box.height,
                    },
                    "stamp": {"sec": stamp[0], "nanosec": stamp[1]},
                    "position_sigma_m": float(target.position_sigma_m),
                    "error_mm": error_mm,
                    "attempt": attempt,
                }
            raise GateRuntimeError("; ".join(attempt_errors) or "all attempts failed")

        def run(self) -> tuple[dict[str, object], bool]:
            self.wait_for(
                lambda: self.camera_info is not None
                and self.depth is not None
                and self.truth is not None,
                30.0,
                "initial RGB-D and truth inputs",
            )
            for model_name, xyz in parked_positions.items():
                self.set_model_pose(model_name, xyz)

            samples = []
            failures = []
            try:
                for index, desired in enumerate(positions, start=1):
                    try:
                        sample = self.measure_position(index, desired)
                    except (GateRuntimeError, LocalizationError) as error:
                        failures.append(
                            {
                                "index": index,
                                "desired_xyz_m": list(desired),
                                "error": str(error),
                            }
                        )
                        self.get_logger().error(
                            f"position {index}/{len(positions)} failed: {error}"
                        )
                    else:
                        samples.append(sample)
                        self.get_logger().info(
                            f"position {index}/{len(positions)}: "
                            f"{sample['error_mm']:.2f} mm"
                        )
            finally:
                for model_name, xyz in original_positions.items():
                    try:
                        self.set_model_pose(model_name, xyz)
                    except GateRuntimeError as error:
                        self.get_logger().warning(
                            f"failed to restore {model_name}: {error}"
                        )

            summary = summarize_gate(
                [float(sample["error_mm"]) for sample in samples],
                requested_positions=len(positions),
                minimum_positions=minimum_positions,
                median_limit_mm=median_limit_mm,
                p95_limit_mm=p95_limit_mm,
            )
            result = {
                "schema_version": 2,
                "gate": gate_name,
                "scope": gate_scope,
                "formal_acceptance": False if v2_contract else True,
                "formal_real_test_accessed": False,
                "robot_motion_started": False,
                "gripper_command_started": False,
                "attachment_started": False,
                "perception_model_started": False,
                "simulated_fruit_pose_motion_started": True,
                "target_frame": "panda_link0",
                "camera_frame": "strawberry_camera_optical_frame",
                "target_model": target_model,
                "fruit_radius_m": fruit_radius_m,
                "attempts_per_position": options.attempts_per_position,
                "provenance": {
                    "generated_at_utc": datetime.now(timezone.utc).isoformat(),
                    "ros_domain_id": ros_domain_id,
                    "run_parameters": {
                        "attempts_per_position": options.attempts_per_position,
                        "sensor_timeout_sec": options.sensor_timeout_sec,
                        "runner_timeout_sec": options.runner_timeout_sec,
                        "launch_headless": bool(options.launch_headless),
                        "benchmark_position_count": len(positions),
                        "domain_selection_mode": options.domain_selection_mode,
                        "domain_probe": {
                            "command": "ros2 node list --no-daemon --all",
                            "spin_time_sec": options.domain_probe_spin_sec,
                            "result_before_launch": "no external nodes observed",
                        },
                    },
                    "files": {
                        label: _file_fingerprint(path)
                        for label, path in sorted(provenance_files.items())
                    },
                },
                "summary": summary,
                "failures": failures,
                "samples": samples,
            }
            return result, bool(summary["passed"])

    def write_outputs(result: dict[str, object]) -> None:
        options.output_json.parent.mkdir(parents=True, exist_ok=True)
        options.output_csv.parent.mkdir(parents=True, exist_ok=True)
        options.output_json.write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        with options.output_csv.open("w", encoding="utf-8", newline="") as stream:
            fieldnames = (
                "index",
                "desired_x_m",
                "desired_y_m",
                "desired_z_m",
                "truth_x_m",
                "truth_y_m",
                "truth_z_m",
                "estimated_x_m",
                "estimated_y_m",
                "estimated_z_m",
                "error_mm",
                "attempt",
            )
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for sample in result["samples"]:
                desired = sample["desired_xyz_m"]
                truth = sample["truth_xyz_m"]
                estimated = sample["estimated_xyz_m"]
                writer.writerow(
                    {
                        "index": sample["index"],
                        "desired_x_m": desired[0],
                        "desired_y_m": desired[1],
                        "desired_z_m": desired[2],
                        "truth_x_m": truth[0],
                        "truth_y_m": truth[1],
                        "truth_z_m": truth[2],
                        "estimated_x_m": estimated[0],
                        "estimated_y_m": estimated[1],
                        "estimated_z_m": estimated[2],
                        "error_mm": sample["error_mm"],
                        "attempt": sample["attempt"],
                    }
                )

    rclpy.init(args=args)
    node = LocalizationGateNode()
    try:
        result, passed = node.run()
        write_outputs(result)
        print(json.dumps(result["summary"], indent=2), flush=True)
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
