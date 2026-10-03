"""Passively record controller/joint/clock evidence without sending commands.

Outputs are diagnostic, not a release of G0/G1 or evidence of safe harvesting.
Use a dedicated ROS_DOMAIN_ID for transport tests. A running simulator must be
started separately under its own authorization and cleanup procedure.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_manipulation"))
from strawberry_manipulation.arm_telemetry import TelemetryRecorder  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--home-positions-file", type=Path)
    parser.add_argument("--use-wall-time", action="store_true",
                        help="Use ROS wall clock instead of /clock; never mix clock domains")
    options = parser.parse_args(argv)
    if not math.isfinite(options.duration) or not 0 < options.duration <= 3600:
        parser.error("duration must be finite and in (0, 3600]")
    home = None
    if options.home_positions_file:
        import yaml
        data = yaml.safe_load(options.home_positions_file.read_text(encoding="utf-8"))
        home = [float(data["initial_positions"][f"panda_joint{i}"]) for i in range(1, 8)]
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    recorder = TelemetryRecorder(options.output, run_id=options.run_id,
                                 scenario_id=options.scenario_id, home_positions=home)
    rclpy.init(args=[])
    node = Node("strawberry_passive_arm_telemetry",
                parameter_overrides=[Parameter("use_sim_time", value=not options.use_wall_time)])
    stopping = False

    def stop_signal(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)

    def times():
        return dict(received_ns=time.monotonic_ns(), ros_now_ns=node.get_clock().now().nanoseconds)

    node.create_subscription(JointTrajectoryControllerState, "/panda_arm_controller/controller_state",
                             lambda msg: recorder.controller(msg, **times()), qos_profile_sensor_data)
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: recorder.joints(msg, **times()), qos_profile_sensor_data)
    node.create_subscription(Clock, "/clock",
                             lambda msg: recorder.clock(msg, **times()), qos_profile_sensor_data)
    node.create_subscription(String, "/strawberry/motion_evidence",
                             lambda msg: recorder.command(msg.data, **times()), 100)
    print(json.dumps({"state": "RECORDER_READY", "output": str(options.output.resolve()),
                      "run_id": options.run_id, "command_publishers": 0}), flush=True)
    deadline = time.monotonic() + options.duration
    error = None
    try:
        while rclpy.ok() and not stopping and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        recorder._record("RECORDER_EXCEPTION", {"error": error}, **{
            "received_ns": time.monotonic_ns(), "ros_now_ns": node.get_clock().now().nanoseconds},
            issues=["RECORDER_EXCEPTION"])
    finally:
        result = recorder.close(now_ns=time.monotonic_ns(), ros_now_ns=node.get_clock().now().nanoseconds)
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({"state": "RECORDER_CLOSED", "joint_samples": result.get("joint_sample_count", 0),
                      "controller_samples": result.get("controller_sample_count", 0),
                      "independent_stop": result["independent_stop"]["stop_status"],
                      "error": error}), flush=True)
    return 1 if error else 0 if (result.get("joint_sample_count", 0) > 0
                                and result.get("controller_sample_count", 0) > 0) else 2


if __name__ == "__main__":
    raise SystemExit(main())
