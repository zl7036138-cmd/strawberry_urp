"""Development-only, bounded cleared-world replay of a recorded arm command.

Default: no payload, perception, truth subscriptions or batch harvesting.
Optional welded/released 30 g no-contact fixture is scenario assembly only,
never proof of contact grasp, bin placement or harvesting qualification.
Initial posture and trajectory come from a recorded COMMAND_PREPARED, not truth.
Does not count as harvesting qualification or formal acceptance.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import copy
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_manipulation"))
from strawberry_manipulation.moveit_backend import joint_limit_margin_violation


def bounded_prefix(trajectory, duration_sec):
    if not math.isfinite(duration_sec) or not 0 < duration_sec <= 1:
        raise ValueError("Diagnostic prefix must be in (0, 1] seconds")
    result = copy.deepcopy(trajectory)
    result["points"] = [p for p in result["points"]
                        if p["time_from_start_ns"] <= int(duration_sec * 1e9)]
    if len(result["points"]) < 2:
        raise ValueError("Prefix contains fewer than two recorded points")
    # Do not ask the controller to continue past the recorded prefix.
    last = result["points"][-1]
    last["velocities"] = [0.0] * 7
    last["accelerations"] = [0.0] * 7
    validate_trajectory(result)
    return result


def extract_command(samples, minimum_sim_sec, prefix_duration_sec=None):
    with samples.open(encoding="utf-8") as stream:
        for line in stream:
            outer = json.loads(line)
            if outer.get("event_type") != "BACKEND_COMMAND_EVENT":
                continue
            event = outer["payload"]["event"]
            if (event["event_type"] == "COMMAND_PREPARED"
                    and event["ros_time_ns"] / 1e9 >= minimum_sim_sec):
                trajectory = event["payload"]["trajectory"]
                if prefix_duration_sec is not None:
                    event = copy.deepcopy(event)
                    event["diagnostic_derivation"] = {
                        "kind": "RECORDED_PREFIX_WITH_ZERO_TERMINAL_DERIVATIVES",
                        "requested_prefix_duration_sec": prefix_duration_sec,
                        "original_trajectory_sha256": hashlib.sha256(
                            json.dumps(trajectory, sort_keys=True).encode()).hexdigest(),
                        "full_original_command_replayed": False,
                    }
                    trajectory = bounded_prefix(trajectory, prefix_duration_sec)
                    event["payload"]["trajectory"] = trajectory
                validate_trajectory(trajectory)
                return event
    raise ValueError("No matching recorded command")


def validate_trajectory(trajectory):
    names = [f"panda_joint{i}" for i in range(1, 8)]
    if trajectory["joint_names"] != names:
        raise ValueError("Replay requires ordered seven-joint Panda command")
    points = trajectory["points"]
    if not 2 <= len(points) <= 512:
        raise ValueError("Invalid point count")
    previous = -1
    travel = 0.0
    for index, point in enumerate(points):
        q = point["positions"]
        if len(q) != 7 or not all(math.isfinite(x) for x in q):
            raise ValueError("Invalid positions")
        for field in ("velocities", "accelerations", "effort"):
            values = point.get(field, [])
            if len(values) not in (0, 7) or not all(math.isfinite(x) for x in values):
                raise ValueError(f"Invalid {field}")
        if joint_limit_margin_violation(tuple(q), 0.02):
            raise ValueError("Replay violates existing joint margin")
        stamp = point["time_from_start_ns"]
        if not isinstance(stamp, int) or not previous < stamp <= 10_000_000_000:
            raise ValueError("Invalid or unbounded command timing")
        if index:
            travel += sum(abs(a-b) for a, b in zip(q, points[index-1]["positions"]))
        previous = stamp
    if previous <= 0 or travel > 2.0:
        raise ValueError("Replay exceeds empty-arm diagnostic bound")


def final_feedback_metrics(feedback, endpoint, now):
    valid = [frame for frame in feedback if
             len(frame.get("reference", [])) == len(frame.get("actual", [])) == 7
             and all(math.isfinite(x) for x in frame["reference"] + frame["actual"])]
    if not valid or valid[-1] is not feedback[-1]:
        return {"final_stop_observed": False}
    recent = [frame for frame in feedback if
              frame["receipt_monotonic_sec"] >= feedback[-1]["receipt_monotonic_sec"] - 0.5]
    valid_recent = all(
        len(frame.get("actual", [])) == len(frame.get("velocities", [])) == 7
        and all(math.isfinite(x) for x in frame["actual"] + frame["velocities"])
        for frame in recent)
    stopped = (
        len(recent) >= 3 and valid_recent
        and all(b["sim_sec"] > a["sim_sec"] for a, b in zip(recent, recent[1:]))
        and all(b["receipt_monotonic_sec"] > a["receipt_monotonic_sec"]
                for a, b in zip(recent, recent[1:]))
        and recent[-1]["receipt_monotonic_sec"] - recent[0]["receipt_monotonic_sec"] >= 0.4
        and 0 <= now - recent[-1]["receipt_monotonic_sec"] <= 1
        and all(abs(v) <= 0.02 for frame in recent for v in frame["velocities"])
        and all(max(frame["actual"][i] for frame in recent) -
                min(frame["actual"][i] for frame in recent) <= 0.002 for i in range(7)))
    return {
        "joint5_peak_error_rad": max(abs(frame["reference"][4] - frame["actual"][4])
                                     for frame in valid),
        "final_endpoint_error_rad": max(abs(a-b) for a,b in zip(valid[-1]["actual"], endpoint)),
        "final_stop_observed": stopped,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=Path, required=True)
    parser.add_argument("--minimum-sim-sec", type=float, required=True)
    parser.add_argument("--audit-plugin", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prefix-duration-sec", type=float,
                        help="Development short segment, not full-command replay; bounds unchanged")
    parser.add_argument("--diagnostic-load-state", choices=("empty", "welded", "released"), default="empty")
    parser.add_argument("--attachment-plugin", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.minimum_sim_sec) or not args.audit_plugin.is_file():
        parser.error("Invalid command selector or audit plugin")
    if args.diagnostic_load_state != "empty" and (args.attachment_plugin is None or not args.attachment_plugin.is_file()):
        parser.error("Load fixture requires an explicit built attachment plugin")
    command = extract_command(args.samples, args.minimum_sim_sec, args.prefix_duration_sec)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    import yaml
    initial = dict(zip(command["payload"]["trajectory"]["joint_names"],
                       command["payload"]["trajectory"]["points"][0]["positions"]))
    initial.update(panda_finger_joint1=0.04, panda_finger_joint2=0.04)
    (out / "initial.yaml").write_text(yaml.safe_dump({"initial_positions": initial}))
    (out / "source_command.json").write_text(json.dumps(command, indent=2))
    world = ET.Element("sdf", version="1.9")
    # The existing spawn/bridge entry point uses this canonical world name.
    w = ET.SubElement(world, "world", name="strawberry_orchard")
    ET.SubElement(w, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(w, "physics", name="default", type="ignored")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1.0"
    # sim.launch's dual-camera path requires exactly one canonical fixed
    # camera include, removes it, then adds the identical robot-mounted rig.
    camera = ET.SubElement(w, "include")
    ET.SubElement(camera, "uri").text = "model://strawberry_rgbd_camera"
    ET.SubElement(camera, "name").text = "strawberry_rgbd_camera"
    for library, name in [("physics", "Physics"), ("user-commands", "UserCommands"),
                          ("scene-broadcaster", "SceneBroadcaster")]:
        ET.SubElement(w, "plugin", filename=f"gz-sim-{library}-system",
                      name=f"gz::sim::systems::{name}")
    plugin = ET.SubElement(w, "plugin", filename=str(args.audit_plugin.resolve()),
                           name="strawberry::CommandAudit")
    ET.SubElement(plugin, "output_file").text = str(out / "ecm_commands.jsonl")
    if args.diagnostic_load_state != "empty":
        ET.SubElement(plugin, "diagnostic_payload_pose_enabled").text = "true"
    if args.diagnostic_load_state != "empty":
        sys.path.insert(0, str(ROOT / "tools/gazebo_command_audit"))
        from payload_fixture import add_payload_fixture, transfer_fixture, motion_metrics
        import xacro
        from ament_index_python.packages import get_package_share_directory
        robot = ET.fromstring(xacro.process_file(str(Path(get_package_share_directory("strawberry_sim")) / "urdf/panda_gz.urdf.xacro"),
                             mappings={"initial_positions_file": str(out / "initial.yaml"), "camera_mount": "dual"}).toxml())
        add_payload_fixture(w, robot, initial, args.attachment_plugin.resolve())
    ET.ElementTree(world).write(out / "world.sdf", encoding="utf-8", xml_declaration=True)
    receipt = {"schema_version": 1, "scope": "EMPTY_ARM_COMMAND_CHAIN_DIAGNOSTIC",
               "formal_acceptance": False, "source_run_id": command["run_id"],
               "source_command_id": command["command_id"],
               "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                                     text=True).strip(),
               "input_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                   for path in [Path(__file__), args.audit_plugin.resolve(), out / "world.sdf",
                                out / "initial.yaml", ROOT / "ros2_ws/src/strawberry_sim/config/panda_controllers.yaml"]},
               "trajectory_sha256": hashlib.sha256(json.dumps(
                   command["payload"]["trajectory"], sort_keys=True).encode()).hexdigest(),
               "outcome": "STARTING"}
    receipt["diagnostic_derivation"] = command.get("diagnostic_derivation")
    receipt.update(diagnostic_load_state=args.diagnostic_load_state,
                   scope="CLEARED_WORLD_COMMAND_CHAIN_DIAGNOSTIC", contact_grasp_proven=False,
                   fixture_has_collision_geometry=False)
    if args.diagnostic_load_state != "empty":
        for path in (args.attachment_plugin.resolve(), ROOT / "tools/gazebo_command_audit/payload_fixture.py"):
            receipt["input_sha256"][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTrajectoryControllerState
    from trajectory_msgs.msg import JointTrajectoryPoint
    rclpy.init(args=[])
    node = Node("empty_arm_command_chain_probe",
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    feedback = []
    latest = {}
    def on_joint(msg):
        latest.update(receipt=time.monotonic(), names=list(msg.name), q=list(msg.position))
    def on_control(msg):
        feedback.append({"sim_sec": msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
                         "receipt_monotonic_sec": time.monotonic(),
                         "reference": list(msg.reference.positions),
                         "actual": list(msg.feedback.positions),
                         "velocities": list(msg.feedback.velocities),
                         "output_positions": list(msg.output.positions)})
    node.create_subscription(JointState, "/joint_states", on_joint, qos_profile_sensor_data)
    node.create_subscription(JointTrajectoryControllerState,
                             "/panda_arm_controller/controller_state", on_control,
                             qos_profile_sensor_data)
    client = ActionClient(node, FollowJointTrajectory,
                          "/panda_arm_controller/follow_joint_trajectory")
    launch = None
    goal_handle = None
    try:
        with (out / "launch.log").open("w") as log:
            launch = subprocess.Popen(["ros2", "launch", "strawberry_sim", "sim.launch.py",
                "headless:=true", "enable_attachment:=false", "camera_mount:=dual",
                "simulation_seed:=45504", f"world_file:={out / 'world.sdf'}",
                f"initial_positions_file:={out / 'initial.yaml'}"], cwd=ROOT,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 60
        while not (latest and client.server_is_ready()):
            if time.monotonic() > deadline or launch.poll() is not None:
                raise RuntimeError("Simulation startup failed or timed out")
            rclpy.spin_once(node, timeout_sec=0.05)
        warmup = time.monotonic() + 4
        while time.monotonic() < warmup:
            rclpy.spin_once(node, timeout_sec=0.05)
        if args.diagnostic_load_state != "empty":
            receipt["fixture_graph_events"] = transfer_fixture(args.diagnostic_load_state, out / "ecm_commands.jsonl")
            settling = time.monotonic() + 1
            while time.monotonic() < settling: rclpy.spin_once(node, timeout_sec=0.05)
        q = [latest["q"][latest["names"].index(name)] for name in initial if name.startswith("panda_joint")]
        planned = command["payload"]["trajectory"]["points"][0]["positions"]
        if time.monotonic() - latest["receipt"] > 1 or max(abs(a-b) for a,b in zip(q,planned)) > 0.005:
            raise RuntimeError("Initial posture did not hold; replay withheld")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = command["payload"]["trajectory"]["joint_names"]
        for raw in command["payload"]["trajectory"]["points"]:
            point = JointTrajectoryPoint()
            for field in ("positions", "velocities", "accelerations", "effort"):
                setattr(point, field, raw[field])
            stamp = raw["time_from_start_ns"]
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(stamp, 1_000_000_000)
            goal.trajectory.points.append(point)
        request = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, request, timeout_sec=5)
        if not request.done() or not request.result().accepted:
            raise RuntimeError("Goal acceptance unknown or rejected")
        goal_handle = request.result()
        result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=30)
        if not result.done():
            goal_handle.cancel_goal_async()
            raise RuntimeError("Replay result timeout; cancellation requested")
        receipt.update(outcome="ACTION_TERMINAL", error_code=result.result().result.error_code,
                       error_string=result.result().result.error_string)
        end = time.monotonic() + 1
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        receipt.update(final_feedback_metrics(
            feedback, command["payload"]["trajectory"]["points"][-1]["positions"], time.monotonic()))
    except Exception as exc:
        receipt.update(outcome="FAILED", error=f"{type(exc).__name__}: {exc}")
    finally:
        if launch is not None:
            # Use the established bounded group cleanup; do not kill unrelated ROS/Gazebo.
            try:
                cleanup = subprocess.run(["bash", "-c",
                    'source scripts/lib/process_group_cleanup.sh; terminate_process_group "$1" "$1"',
                    "cleanup", str(launch.pid)], cwd=ROOT, timeout=60)
                receipt["cleanup"] = "CLEAN" if cleanup.returncode == 0 else "FAILED"
                launch.wait(timeout=10)
            except (subprocess.TimeoutExpired, OSError) as exc:
                receipt.update(cleanup="FAILED", cleanup_error=f"{type(exc).__name__}: {exc}")
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
        if args.diagnostic_load_state != "empty" and receipt.get("fixture_graph_events"):
            try:
                with (out / "ecm_commands.jsonl").open() as stream:
                    receipt["physical_fixture_metrics"] = motion_metrics(
                        [json.loads(line) for line in stream], receipt["fixture_graph_events"], args.diagnostic_load_state)
            except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
                receipt["physical_fixture_metrics"] = {"physical_fixture_status": "INDETERMINATE", "error": str(exc)}
        with (out / "controller_frames.jsonl").open("w") as stream:
            for frame in feedback: stream.write(json.dumps(frame) + "\n")
        (out / "receipt.json").write_text(json.dumps(receipt, indent=2))
        print(json.dumps(receipt), flush=True)
    fixture_valid = args.diagnostic_load_state == "empty" or receipt.get("physical_fixture_metrics", {}).get("physical_fixture_status") == (
        "CARRY_OBSERVED" if args.diagnostic_load_state == "welded" else "RELEASE_FALL_OBSERVED")
    return 0 if (fixture_valid and receipt.get("error_code") == 0 and receipt.get("cleanup") == "CLEAN"
                 and receipt.get("final_endpoint_error_rad", math.inf) <= 0.05
                 and receipt.get("final_stop_observed") is True) else 1

if __name__ == "__main__":
    raise SystemExit(main())
