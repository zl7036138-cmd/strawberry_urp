#!/usr/bin/env python3
"""Isolate one gripper close/reopen cycle at the Blender-v2 grasp pose."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import threading
import time


FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")


def _result_state(state) -> dict[str, dict[str, float]]:
    rows: dict[str, dict[str, float]] = {}
    for index, name in enumerate(state.name):
        row: dict[str, float] = {}
        for field in ("position", "velocity", "effort"):
            values = getattr(state, field)
            if index < len(values):
                row[field] = float(values[index])
        rows[str(name)] = row
    return rows


def main(args=None) -> int:  # pragma: no cover - ROS/Gazebo integration
    from ament_index_python.packages import get_package_share_directory
    from action_msgs.msg import GoalStatus
    from control_msgs.action import ParallelGripperCommand
    from geometry_msgs.msg import PoseStamped
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from strawberry_manipulation.core import (
        Pose,
        hand_pose_for_fruit_center,
        pregrasp_pose_for_fruit_center,
    )
    from strawberry_manipulation.moveit_backend import (
        MoveItBackend,
        gripper_result_allows_command,
    )
    from strawberry_manipulation.moveit_config import build_moveit_config
    from strawberry_sim.core import load_scene_config

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--readiness", type=Path, required=True)
    parser.add_argument("--target-id", type=int, default=1)
    parser.add_argument("--camera-mount", choices=("wrist", "dual"), default="dual")
    parser.add_argument("--action-timeout-sec", type=float, default=10.0)
    parser.add_argument("--contact-settle-sec", type=float, default=1.0)
    options, ros_args = parser.parse_known_args(args)
    if options.output.exists():
        raise FileExistsError(f"refusing to overwrite {options.output}")
    if options.target_id <= 0:
        parser.error("--target-id must be positive")
    if options.action_timeout_sec <= 0.0 or options.contact_settle_sec <= 0.0:
        parser.error("timeouts must be positive")

    readiness = json.loads(options.readiness.read_text(encoding="utf-8"))
    if readiness.get("execution_readiness_passed") is not True:
        raise ValueError("input perception execution readiness did not pass")
    if int(readiness.get("target_id", -1)) != options.target_id:
        raise ValueError("readiness target_id does not match --target-id")
    center_values = readiness.get("estimated_center_xyz_m")
    if not isinstance(center_values, list) or len(center_values) != 3:
        raise ValueError("readiness estimated centre is missing")
    target_center = Pose(*(float(value) for value in center_values))
    geometry = readiness.get("grasp_geometry") or {}
    tool_center_offset_m = float(geometry["tool_center_offset_m"])
    closed_width_m = float(geometry["close_width_m_per_finger"])
    open_width_m = 0.04
    scene_path = (
        Path(get_package_share_directory("strawberry_sim"))
        / "config"
        / "scene.yaml"
    )
    scene = load_scene_config(scene_path)
    fruit_by_id = {fruit.target_id: fruit for fruit in scene.ordered_fruits}
    if options.target_id not in fruit_by_id:
        raise ValueError("target_id is absent from the scene manifest")
    target_model = fruit_by_id[options.target_id].model_name
    fruit_obstacles = {
        fruit.target_id: Pose(
            fruit.initial_pose.x,
            fruit.initial_pose.y,
            fruit.initial_pose.z,
        )
        for fruit in scene.ordered_fruits
    }
    grasp_pose = hand_pose_for_fruit_center(
        target_center,
        quaternion=(1.0, 0.0, 0.0, 0.0),
        tool_center_offset_m=tool_center_offset_m,
    )
    pregrasp_pose = pregrasp_pose_for_fruit_center(
        target_center,
        quaternion=(1.0, 0.0, 0.0, 0.0),
        tool_center_offset_m=tool_center_offset_m,
        pregrasp_offset_m=0.15,
    )
    observation_pose = Pose(
        0.28,
        0.0,
        0.72,
        0.0,
        0.9537169507,
        0.0,
        0.3007057995,
    ).normalized()

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("blender_v2_grasp_pose_gripper_diagnostic")
            self.started = time.monotonic()
            self.latest_joints: dict[str, float] = {}
            self.finger_samples: list[dict[str, object]] = []
            self.truth_samples: list[dict[str, object]] = []
            self.raw_target_contact = {"left": False, "right": False}
            self.processed_target_contact = {"left": False, "right": False}
            self.raw_first_target_contact_sec = {"left": None, "right": None}
            self.processed_first_target_contact_sec = {
                "left": None,
                "right": None,
            }
            self.non_target_contact_seen = False
            self.raw_pairs: dict[str, list[list[str]]] = {
                "left": [],
                "right": [],
            }
            self.gripper = ActionClient(
                self,
                ParallelGripperCommand,
                "/panda_gripper_controller/gripper_cmd",
            )
            self.create_subscription(
                JointState,
                "/joint_states",
                self._on_joints,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PoseStamped,
                f"/strawberry/ground_truth/fruit_{options.target_id}/pose",
                self._on_truth,
                qos_profile_sensor_data,
            )
            prefix = f"/strawberry/sim/fruit_{options.target_id}"
            for side in ("left", "right"):
                self.create_subscription(
                    Contacts,
                    f"/strawberry/sim/gripper/{side}_contacts",
                    lambda message, selected=side: self._on_raw_contact(
                        selected, message
                    ),
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    Bool,
                    f"{prefix}/{side}_contact",
                    lambda message, selected=side: self._on_processed_contact(
                        selected, message
                    ),
                    qos_profile_sensor_data,
                )

        def elapsed(self) -> float:
            return time.monotonic() - self.started

        def _on_joints(self, message: JointState) -> None:
            self.latest_joints.update(
                {
                    str(name): float(position)
                    for name, position in zip(
                        message.name, message.position, strict=False
                    )
                }
            )
            if all(name in self.latest_joints for name in FINGER_JOINTS):
                self.finger_samples.append(
                    {
                        "elapsed_sec": round(self.elapsed(), 6),
                        "positions_m": {
                            name: self.latest_joints[name]
                            for name in FINGER_JOINTS
                        },
                    }
                )

        def _on_truth(self, message: PoseStamped) -> None:
            position = message.pose.position
            self.truth_samples.append(
                {
                    "elapsed_sec": round(self.elapsed(), 6),
                    "position_m": [
                        float(position.x),
                        float(position.y),
                        float(position.z),
                    ],
                }
            )

        def _on_raw_contact(self, side: str, message: Contacts) -> None:
            for contact in message.contacts:
                pair = [
                    str(contact.collision1.name),
                    str(contact.collision2.name),
                ]
                if len(self.raw_pairs[side]) < 20:
                    self.raw_pairs[side].append(pair)
                combined = " ".join(pair)
                if f"{target_model}::" in combined:
                    self.raw_target_contact[side] = True
                    if self.raw_first_target_contact_sec[side] is None:
                        self.raw_first_target_contact_sec[side] = round(
                            self.elapsed(), 6
                        )
                if any(
                    f"{fruit.model_name}::" in combined
                    for fruit in scene.ordered_fruits
                    if fruit.target_id != options.target_id
                ):
                    self.non_target_contact_seen = True

        def _on_processed_contact(self, side: str, message: Bool) -> None:
            if bool(message.data):
                self.processed_target_contact[side] = True
                if self.processed_first_target_contact_sec[side] is None:
                    self.processed_first_target_contact_sec[side] = round(
                        self.elapsed(), 6
                    )

        @staticmethod
        def wait_future(future, timeout_sec: float, description: str):
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            if not event.wait(timeout_sec):
                raise TimeoutError(f"timed out waiting for {description}")
            if future.exception() is not None:
                raise RuntimeError(
                    f"{description} failed: {future.exception()}"
                )
            result = future.result()
            if result is None:
                raise RuntimeError(f"{description} returned no result")
            return result

        def command_gripper(self, position_m: float) -> dict[str, object]:
            before = {
                name: self.latest_joints.get(name) for name in FINGER_JOINTS
            }
            started_sec = self.elapsed()
            goal = ParallelGripperCommand.Goal()
            goal.command.name = [FINGER_JOINTS[0]]
            goal.command.position = [position_m]
            goal.command.effort = [40.0]
            handle = self.wait_future(
                self.gripper.send_goal_async(goal),
                options.action_timeout_sec,
                f"gripper goal acceptance at {position_m}",
            )
            if not handle.accepted:
                raise RuntimeError(f"gripper rejected position {position_m}")
            wrapped = self.wait_future(
                handle.get_result_async(),
                options.action_timeout_sec,
                f"gripper result at {position_m}",
            )
            state = _result_state(wrapped.result.state)
            commanded = state.get(FINGER_JOINTS[0], {})
            observed = commanded.get("position")
            observed_source = "controller_result"
            if observed is None:
                observed = self.latest_joints.get(FINGER_JOINTS[0])
                observed_source = "joint_states_fallback"
            accepted = False
            if observed is not None:
                accepted = gripper_result_allows_command(
                    target_position_m=position_m,
                    observed_position_m=observed,
                    open_position_m=open_width_m,
                    closed_position_m=closed_width_m,
                    stalled=bool(wrapped.result.stalled),
                    reached_goal=bool(wrapped.result.reached_goal),
                )
            return {
                "target_position_m_per_finger": position_m,
                "started_elapsed_sec": round(started_sec, 6),
                "completed_elapsed_sec": round(self.elapsed(), 6),
                "status": int(wrapped.status),
                "status_succeeded": (
                    wrapped.status == GoalStatus.STATUS_SUCCEEDED
                ),
                "stalled": bool(wrapped.result.stalled),
                "reached_goal": bool(wrapped.result.reached_goal),
                "measured_position_validation_passed": accepted,
                "positions_before_m": before,
                "result_state": state,
                "observed_position_m": observed,
                "observed_position_source": observed_source,
            }

    def motion_receipt(outcome) -> dict[str, object]:
        return {
            "success": bool(outcome.success),
            "planning_time_sec": float(outcome.planning_time_sec),
            "execution_time_sec": float(outcome.execution_time_sec),
            "collision": bool(outcome.collision),
        }

    rclpy.init(args=ros_args)
    node = Probe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    backend = None
    payload: dict[str, object] = {
        "schema_version": 1,
        "scope": "NON_ACCEPTANCE_BLENDER_V2_GRASP_POSE_GRIPPER_DIAGNOSTIC",
        "planning_started": False,
        "pick_action_started": False,
        "attachment_command_started": False,
        "place_motion_started": False,
        "target_id": options.target_id,
        "target_center_source": str(options.readiness),
        "target_center_m": [target_center.x, target_center.y, target_center.z],
        "tool_center_offset_m": tool_center_offset_m,
        "open_width_m_per_finger": open_width_m,
        "closed_width_m_per_finger": closed_width_m,
        "errors": [],
    }
    target_collision_prepared = False
    target_contact_open = False
    retreat_complete = False
    home_complete = False
    grasp_motion_execution_started = False
    status = 1
    close_started = None
    close_completed = None
    try:
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            if (
                all(name in node.latest_joints for name in FINGER_JOINTS)
                and node.truth_samples
                and node.gripper.server_is_ready()
            ):
                break
            time.sleep(0.05)
        else:
            raise TimeoutError(
                "joint state, truth pose, or gripper action is unavailable"
            )
        payload["initial_truth_position_m"] = node.truth_samples[-1][
            "position_m"
        ]
        payload["initial_finger_positions_m"] = {
            name: node.latest_joints[name] for name in FINGER_JOINTS
        }
        backend = MoveItBackend(
            node,
            open_width_m=open_width_m,
            closed_width_m=closed_width_m,
            fruit_obstacles=fruit_obstacles,
            fruit_collision_radius_m=scene.fruit_collision_radius_m,
            config_dict=build_moveit_config(camera_mount=options.camera_mount),
        )

        payload["planning_started"] = True
        observation = backend.move_to(
            observation_pose, "WRIST_OBSERVATION"
        )
        payload["observation_motion"] = motion_receipt(observation)
        if not observation.success:
            raise RuntimeError("failed to reach the v3 wrist observation pose")

        initial_open = node.command_gripper(open_width_m)
        payload["initial_open_command"] = initial_open
        if not initial_open["measured_position_validation_passed"]:
            raise RuntimeError("initial open command failed measured validation")

        control_close = node.command_gripper(closed_width_m)
        payload["observation_pose_control_close_command"] = control_close
        time.sleep(0.5)
        payload["observation_pose_control_closed_positions_m"] = {
            name: node.latest_joints[name] for name in FINGER_JOINTS
        }
        control_reopen = node.command_gripper(open_width_m)
        payload["observation_pose_control_reopen_command"] = control_reopen
        time.sleep(0.5)
        if not control_close["measured_position_validation_passed"]:
            raise RuntimeError(
                "free-space close failed at the wrist observation pose"
            )
        if not control_reopen["measured_position_validation_passed"]:
            raise RuntimeError(
                "free-space reopen failed at the wrist observation pose"
            )

        if not backend.prepare_pick(options.target_id, target_center):
            raise RuntimeError("failed to prepare target collision lifecycle")
        target_collision_prepared = True
        approach = backend.move_to(pregrasp_pose, "APPROACH")
        payload["approach_motion"] = motion_receipt(approach)
        if not approach.success:
            raise RuntimeError("failed to reach the v3 pregrasp pose")
        if not backend.allow_target_contact(options.target_id):
            raise RuntimeError("failed to open the target contact corridor")
        target_contact_open = True
        grasp = backend.move_to(grasp_pose, "GRASP_POSE")
        payload["grasp_motion"] = motion_receipt(grasp)
        grasp_motion_execution_started = grasp.execution_time_sec > 0.0
        if not grasp.success:
            raise RuntimeError("failed to reach the v3 grasp pose")

        payload["truth_before_close_m"] = node.truth_samples[-1]["position_m"]
        close_started = node.elapsed()
        close = node.command_gripper(closed_width_m)
        close_completed = node.elapsed()
        payload["close_command"] = close
        time.sleep(options.contact_settle_sec)
        payload["finger_positions_after_close_m"] = {
            name: node.latest_joints[name] for name in FINGER_JOINTS
        }
        payload["truth_after_close_m"] = node.truth_samples[-1]["position_m"]
        payload["raw_target_contact_seen"] = dict(node.raw_target_contact)
        payload["processed_target_contact_seen"] = dict(
            node.processed_target_contact
        )
        payload["raw_first_target_contact_elapsed_sec"] = dict(
            node.raw_first_target_contact_sec
        )
        payload["processed_first_target_contact_elapsed_sec"] = dict(
            node.processed_first_target_contact_sec
        )
        payload["non_target_contact_seen"] = node.non_target_contact_seen
        payload["raw_contact_pairs"] = dict(node.raw_pairs)

        reopen = node.command_gripper(open_width_m)
        payload["reopen_command"] = reopen
        time.sleep(0.5)
        payload["finger_positions_after_reopen_m"] = {
            name: node.latest_joints[name] for name in FINGER_JOINTS
        }
        if not reopen["measured_position_validation_passed"]:
            raise RuntimeError("reopen command failed measured validation")
        retreat = backend.move_to(pregrasp_pose, "RETREAT")
        payload["recovery_retreat_motion"] = motion_receipt(retreat)
        retreat_complete = bool(retreat.success)
        if not retreat_complete:
            raise RuntimeError("failed to retreat after gripper diagnostic")
        if not backend.restore_target_collision(options.target_id):
            raise RuntimeError("failed to restore target collision object")
        target_collision_prepared = False
        target_contact_open = False
        home = backend.move_home()
        payload["recovery_home_succeeded"] = bool(home)
        home_complete = bool(home)
        if not home:
            raise RuntimeError("failed to return home after diagnostic")
    except Exception as exc:
        payload["errors"].append(str(exc))
    finally:
        if backend is not None:
            if target_contact_open and not retreat_complete:
                try:
                    payload["emergency_reopen_succeeded"] = bool(
                        node.command_gripper(open_width_m)[
                            "measured_position_validation_passed"
                        ]
                    )
                except Exception as exc:
                    payload["errors"].append(
                        f"emergency reopen failed: {exc}"
                    )
                try:
                    if not grasp_motion_execution_started:
                        retreat_complete = True
                        payload[
                            "emergency_retreat_not_required"
                        ] = "grasp trajectory execution never started"
                    else:
                        emergency_retreat = backend.move_to(
                            pregrasp_pose, "RETREAT"
                        )
                        payload["emergency_recovery_retreat_motion"] = (
                            motion_receipt(emergency_retreat)
                        )
                        retreat_complete = bool(emergency_retreat.success)
                except Exception as exc:
                    payload["errors"].append(
                        f"emergency retreat failed: {exc}"
                    )
            if target_collision_prepared and (
                retreat_complete or not target_contact_open
            ):
                try:
                    restored = backend.restore_target_collision(
                        options.target_id
                    )
                    payload["emergency_collision_restore_succeeded"] = bool(
                        restored
                    )
                    target_collision_prepared = not restored
                except Exception as exc:
                    payload["errors"].append(
                        f"emergency collision restore failed: {exc}"
                    )
            if not target_collision_prepared and not home_complete:
                try:
                    home_complete = bool(backend.move_home())
                    payload["emergency_home_succeeded"] = home_complete
                except Exception as exc:
                    payload["errors"].append(
                        f"emergency home failed: {exc}"
                    )

        relevant_samples = []
        if close_started is not None:
            end = (
                close_completed + options.contact_settle_sec
                if close_completed is not None
                else node.elapsed()
            )
            relevant_samples = [
                sample
                for sample in node.finger_samples
                if close_started - 0.1
                <= float(sample["elapsed_sec"])
                <= end
            ]
        payload["close_window_finger_samples"] = relevant_samples
        if relevant_samples:
            minima = {
                name: min(
                    float(sample["positions_m"][name])
                    for sample in relevant_samples
                )
                for name in FINGER_JOINTS
            }
            payload["minimum_close_window_finger_positions_m"] = minima
            payload["measured_close_travel_m"] = min(
                open_width_m - minima[name] for name in FINGER_JOINTS
            )
        truth_before = payload.get("truth_before_close_m")
        truth_after = payload.get("truth_after_close_m")
        if isinstance(truth_before, list) and isinstance(truth_after, list):
            payload["fruit_displacement_during_close_m"] = math.dist(
                truth_before, truth_after
            )

        close = payload.get("close_command") or {}
        measured_travel = float(payload.get("measured_close_travel_m", 0.0))
        if (
            close.get("stalled") is True
            and measured_travel < 0.002
        ):
            classification = "FALSE_STALL_NO_MEANINGFUL_TRAVEL"
        elif measured_travel >= 0.002 and all(
            node.raw_target_contact[side]
            or node.processed_target_contact[side]
            for side in ("left", "right")
        ):
            classification = "MEASURED_CLOSE_WITH_BILATERAL_TARGET_CONTACT"
        elif measured_travel >= 0.002:
            classification = "MEASURED_CLOSE_WITHOUT_BILATERAL_TARGET_CONTACT"
        else:
            classification = "GRIPPER_CLOSE_INCONCLUSIVE"
        payload["classification"] = classification
        payload["diagnostic_passed"] = bool(
            not payload["errors"]
            and close.get("measured_position_validation_passed") is True
            and measured_travel >= 0.002
            and payload.get("non_target_contact_seen") is False
            and payload.get("recovery_home_succeeded") is True
            and not target_collision_prepared
        )
        payload["safe_recovery_complete"] = bool(
            home_complete
            and not target_collision_prepared
        )
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(payload, sort_keys=True))
        status = 0 if payload["diagnostic_passed"] else 1
        if backend is not None:
            backend.shutdown()
        executor.shutdown(timeout_sec=10)
        spin_thread.join(timeout=10)
        node.destroy_node()
        rclpy.try_shutdown()
    os._exit(status)


if __name__ == "__main__":
    main()
