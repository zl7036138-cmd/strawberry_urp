"""Measure the live Gazebo / ROS data plane for the P0 stability gate."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import time


EXPECTED_CONTROLLERS = frozenset(
    {
        "joint_state_broadcaster",
        "panda_arm_controller",
        "panda_gripper_controller",
    }
)

MINIMUM_TOPIC_RATES_HZ = {
    "clock": 20.0,
    "color": 2.0,
    "depth": 2.0,
    "camera_info": 2.0,
    "truth_1": 5.0,
    "truth_2": 5.0,
    "truth_3": 5.0,
    "joint_states": 5.0,
}


@dataclass
class TopicObservation:
    """Wall-clock receipt state for one required topic."""

    count: int = 0
    first_wall_sec: float | None = None
    last_wall_sec: float | None = None

    def record(self, now_sec: float) -> None:
        if self.first_wall_sec is None:
            self.first_wall_sec = now_sec
        self.last_wall_sec = now_sec
        self.count += 1


def measured_rates(
    observations: dict[str, TopicObservation],
    baseline_counts: dict[str, int],
    elapsed_sec: float,
) -> dict[str, float]:
    """Return post-readiness message rates without counting startup bursts."""

    if elapsed_sec <= 0.0:
        raise ValueError("elapsed_sec must be positive")
    return {
        name: max(0, item.count - baseline_counts.get(name, 0)) / elapsed_sec
        for name, item in observations.items()
    }


def main(args=None) -> None:  # pragma: no cover - exercised by runtime gate
    parser = argparse.ArgumentParser(
        description="Validate continuous Gazebo RGB-D, truth, and control output."
    )
    parser.add_argument("--duration-sec", type=float, default=1800.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=60.0)
    parser.add_argument("--max-silence-sec", type=float, default=5.0)
    parser.add_argument("--report-interval-sec", type=float, default=60.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--camera-prefix", default="/camera")
    parser.add_argument(
        "--camera-frame", default="strawberry_camera_optical_frame"
    )
    parser.add_argument("--expected-width", type=int, default=640)
    parser.add_argument("--expected-height", type=int, default=480)
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.duration_sec <= 0.0:
        parser.error("--duration-sec must be positive")
    if parsed.startup_timeout_sec <= 0.0:
        parser.error("--startup-timeout-sec must be positive")
    if parsed.max_silence_sec <= 0.0:
        parser.error("--max-silence-sec must be positive")
    camera_prefix = parsed.camera_prefix.rstrip("/")
    if not camera_prefix.startswith("/") or camera_prefix == "":
        parser.error("--camera-prefix must be an absolute ROS topic prefix")
    if not parsed.camera_frame.strip():
        parser.error("--camera-frame cannot be empty")
    if parsed.expected_width <= 0 or parsed.expected_height <= 0:
        parser.error("expected camera dimensions must be positive")
    expected_dimensions = (parsed.expected_width, parsed.expected_height)

    try:
        import rclpy
        from controller_manager_msgs.srv import ListControllers
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, Image, JointState
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class RuntimeHealthNode(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_sim_runtime_health")
            self.observations = {
                name: TopicObservation() for name in MINIMUM_TOPIC_RATES_HZ
            }
            self.violations: set[str] = set()
            self.controller_states: dict[str, str] = {}
            self.last_controller_response_sec: float | None = None
            self._controller_future = None
            self._last_controller_request_sec = 0.0
            self.first_sim_sec: float | None = None
            self.last_sim_sec: float | None = None

            self._managed_subscriptions = [
                self.create_subscription(
                    Clock,
                    "/clock",
                    self._on_clock,
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    Image,
                    f"{camera_prefix}/color/image_raw",
                    self._on_color,
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    Image,
                    f"{camera_prefix}/depth/image_raw",
                    self._on_depth,
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    CameraInfo,
                    f"{camera_prefix}/camera_info",
                    self._on_camera_info,
                    qos_profile_sensor_data,
                ),
                self.create_subscription(
                    JointState,
                    "/joint_states",
                    self._on_joint_states,
                    qos_profile_sensor_data,
                ),
            ]
            for target_id in (1, 2, 3):
                self._managed_subscriptions.append(
                    self.create_subscription(
                        PoseStamped,
                        f"/strawberry/ground_truth/fruit_{target_id}/pose",
                        lambda message, target_id=target_id: self._on_truth(
                            target_id, message
                        ),
                        qos_profile_sensor_data,
                    )
                )
            self._controller_client = self.create_client(
                ListControllers, "/controller_manager/list_controllers"
            )

        def _record(self, name: str) -> None:
            self.observations[name].record(time.monotonic())

        def _on_clock(self, message) -> None:
            self._record("clock")
            current = float(message.clock.sec) + float(message.clock.nanosec) * 1e-9
            if self.last_sim_sec is not None and current < self.last_sim_sec:
                self.violations.add("simulation clock moved backwards")
            if self.first_sim_sec is None:
                self.first_sim_sec = current
            self.last_sim_sec = current

        def _on_color(self, message) -> None:
            self._record("color")
            if (message.width, message.height) != expected_dimensions:
                self.violations.add(
                    "color image is not "
                    f"{parsed.expected_width}x{parsed.expected_height}"
                )
            if message.encoding.lower() != "rgb8":
                self.violations.add(
                    f"color encoding is {message.encoding!r}, expected 'rgb8'"
                )

        def _on_depth(self, message) -> None:
            self._record("depth")
            if (message.width, message.height) != expected_dimensions:
                self.violations.add(
                    "depth image is not "
                    f"{parsed.expected_width}x{parsed.expected_height}"
                )
            if message.encoding.upper() != "32FC1":
                self.violations.add(
                    f"depth encoding is {message.encoding!r}, expected '32FC1'"
                )

        def _on_camera_info(self, message) -> None:
            self._record("camera_info")
            if (message.width, message.height) != expected_dimensions:
                self.violations.add(
                    "CameraInfo is not "
                    f"{parsed.expected_width}x{parsed.expected_height}"
                )
            if message.header.frame_id != parsed.camera_frame:
                self.violations.add(
                    f"CameraInfo frame is not {parsed.camera_frame}"
                )
            if len(message.k) != 9 or message.k[0] <= 0.0 or message.k[4] <= 0.0:
                self.violations.add("CameraInfo has invalid focal lengths")

        def _on_truth(self, target_id: int, message) -> None:
            self._record(f"truth_{target_id}")
            if message.header.frame_id != "panda_link0":
                self.violations.add(
                    f"fruit {target_id} truth frame is not panda_link0"
                )

        def _on_joint_states(self, message) -> None:
            self._record("joint_states")
            required = {f"panda_joint{index}" for index in range(1, 8)}
            if not required.issubset(message.name):
                self.violations.add("joint_states is missing Panda arm joints")

        def update_controller_state(self, now_sec: float) -> None:
            if self._controller_future is not None:
                if not self._controller_future.done():
                    return
                try:
                    response = self._controller_future.result()
                except Exception as exc:  # service transport failure
                    self.violations.add(f"list_controllers failed: {exc}")
                else:
                    self.controller_states = {
                        controller.name: controller.state
                        for controller in response.controller
                    }
                    self.last_controller_response_sec = now_sec
                self._controller_future = None

            if (
                self._controller_future is None
                and now_sec - self._last_controller_request_sec >= 2.0
                and self._controller_client.service_is_ready()
            ):
                self._last_controller_request_sec = now_sec
                self._controller_future = self._controller_client.call_async(
                    ListControllers.Request()
                )

        @property
        def controllers_active(self) -> bool:
            return all(
                self.controller_states.get(name) == "active"
                for name in EXPECTED_CONTROLLERS
            )

        @property
        def data_ready(self) -> bool:
            return all(item.count > 0 for item in self.observations.values())

    def write_result(result: dict) -> None:
        payload = json.dumps(result, indent=2, sort_keys=True)
        print(payload, flush=True)
        if parsed.output is not None:
            parsed.output.parent.mkdir(parents=True, exist_ok=True)
            parsed.output.write_text(payload + "\n", encoding="utf-8")

    rclpy.init(args=ros_args)
    node = RuntimeHealthNode()
    process_start = time.monotonic()
    ready_wall: float | None = None
    baseline_counts: dict[str, int] = {}
    baseline_sim: float | None = None
    last_report = process_start
    early_failure: str | None = None

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            node.update_controller_state(now)

            if ready_wall is None:
                if node.data_ready and node.controllers_active:
                    ready_wall = now
                    baseline_counts = {
                        name: item.count for name, item in node.observations.items()
                    }
                    baseline_sim = node.last_sim_sec
                    print(
                        json.dumps(
                            {
                                "event": "ready",
                                "startup_sec": round(now - process_start, 3),
                                "controllers": node.controller_states,
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                    last_report = now
                elif now - process_start >= parsed.startup_timeout_sec:
                    early_failure = "startup readiness timeout"
                    break
                continue

            elapsed = now - ready_wall
            silent = [
                name
                for name, item in node.observations.items()
                if item.last_wall_sec is None
                or now - item.last_wall_sec > parsed.max_silence_sec
            ]
            if silent:
                early_failure = "required topics became silent: " + ", ".join(silent)
                break
            if (
                node.last_controller_response_sec is None
                or now - node.last_controller_response_sec
                > parsed.max_silence_sec + 2.0
            ):
                early_failure = "controller manager stopped responding"
                break
            if not node.controllers_active:
                early_failure = "one or more required controllers left active state"
                break

            if now - last_report >= parsed.report_interval_sec:
                rates = measured_rates(node.observations, baseline_counts, elapsed)
                print(
                    json.dumps(
                        {
                            "event": "heartbeat",
                            "elapsed_sec": round(elapsed, 3),
                            "rates_hz": {
                                key: round(value, 3) for key, value in rates.items()
                            },
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                last_report = now

            if elapsed >= parsed.duration_sec:
                break
    except KeyboardInterrupt:
        early_failure = "health checker interrupted"
    finally:
        finish_wall = time.monotonic()
        node.destroy_node()
        rclpy.try_shutdown()

    measured_elapsed = (
        max(0.0, finish_wall - ready_wall) if ready_wall is not None else 0.0
    )
    rates = (
        measured_rates(node.observations, baseline_counts, measured_elapsed)
        if measured_elapsed > 0.0 and baseline_counts
        else {name: 0.0 for name in node.observations}
    )
    violations = set(node.violations)
    if early_failure:
        violations.add(early_failure)
    for name, minimum_rate in MINIMUM_TOPIC_RATES_HZ.items():
        if rates[name] < minimum_rate:
            violations.add(
                f"{name} rate {rates[name]:.3f} Hz is below {minimum_rate:.3f} Hz"
            )
    if ready_wall is None:
        violations.add("runtime never reached ready state")
    if measured_elapsed + 0.05 < parsed.duration_sec:
        violations.add(
            f"measured only {measured_elapsed:.3f} of {parsed.duration_sec:.3f} seconds"
        )

    sim_elapsed = None
    if baseline_sim is not None and node.last_sim_sec is not None:
        sim_elapsed = max(0.0, node.last_sim_sec - baseline_sim)
    result = {
        "schema_version": 1,
        "camera_prefix": camera_prefix,
        "camera_frame": parsed.camera_frame,
        "expected_resolution": [
            parsed.expected_width,
            parsed.expected_height,
        ],
        "passed": not violations,
        "requested_wall_duration_sec": parsed.duration_sec,
        "measured_wall_duration_sec": round(measured_elapsed, 3),
        "measured_sim_duration_sec": (
            round(sim_elapsed, 3) if sim_elapsed is not None else None
        ),
        "topic_counts": {
            name: item.count for name, item in node.observations.items()
        },
        "topic_rates_hz": {name: round(rate, 3) for name, rate in rates.items()},
        "controllers": node.controller_states,
        "violations": sorted(violations),
    }
    write_result(result)
    raise SystemExit(0 if result["passed"] else 1)
