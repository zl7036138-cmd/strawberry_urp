#!/usr/bin/env python3
"""Run one real pick-and-place action using Gazebo ground truth as the target."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import time


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-id", type=int, default=1)
    parser.add_argument("--target-topic", default="")
    parser.add_argument(
        "--target-message-type",
        choices=("pose_stamped", "target_pose"),
        default="pose_stamped",
        help="ROS message type published by --target-topic",
    )
    parser.add_argument("--place-x", type=float, default=0.35)
    parser.add_argument("--place-y", type=float, default=-0.45)
    parser.add_argument("--place-z", type=float, default=0.45)
    parser.add_argument("--startup-timeout-sec", type=float, default=45.0)
    parser.add_argument("--action-timeout-sec", type=float, default=180.0)
    parser.add_argument("--stability-samples", type=int, default=10)
    parser.add_argument("--stability-tolerance-m", type=float, default=0.001)
    parser.add_argument(
        "--output",
        type=Path,
        help="optionally write the JSON report to this path",
    )
    return parser


def _wait_future(node, future, timeout_sec: float) -> bool:
    import rclpy

    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    return future.done()


def _emit_report(report: dict, output: Path | None) -> None:
    payload = json.dumps(report, indent=2, sort_keys=True)
    print(payload, flush=True)
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload + "\n", encoding="utf-8")


def main() -> int:  # pragma: no cover - exercised in ROS integration
    arguments = _parser().parse_args()
    if arguments.target_id <= 0:
        raise SystemExit("--target-id must be positive")
    if arguments.startup_timeout_sec <= 0.0 or arguments.action_timeout_sec <= 0.0:
        raise SystemExit("timeouts must be positive")
    if arguments.stability_samples < 2 or arguments.stability_tolerance_m <= 0.0:
        raise SystemExit("stability settings must be positive")
    target_topic = arguments.target_topic or (
        f"/strawberry/ground_truth/fruit_{arguments.target_id}/pose"
    )

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from strawberry_interfaces.action import PickAndPlace
    from strawberry_interfaces.msg import TargetPose
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = Node(
        "strawberry_oracle_pick_test",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
    target_samples = deque(maxlen=arguments.stability_samples)
    sent_target_xyz = None
    action_started_at = None
    current_stage = "WAITING"
    target_displacements = []
    displacement_thresholds_m = deque(
        (0.001, 0.005, 0.01, 0.05, 0.10, 0.25, 0.50, 1.00)
    )
    displacement_events = []
    # Feedback switches to APPROACH only after the guarded pre-grasp motion
    # has completed.  From that point until GRASP finishes, pad geometry is
    # useful for diagnosing the final straight descent as well as closure.
    grasp_path_stages = {"APPROACH", "GRASP"}

    contact_diagnostics = {
        side: {
            "processed_messages": 0,
            "processed_true_messages": 0,
            "processed_seen_true": False,
            "raw_messages": 0,
            "raw_contacts": 0,
            "raw_max_contacts_per_message": 0,
            "raw_contact_pairs": set(),
            "first_processed_true_sec": None,
            "first_raw_contact_sec": None,
        }
        for side in ("left", "right")
    }
    fruit_contact_diagnostics = {
        "raw_messages": 0,
        "raw_contacts": 0,
        "raw_contact_pairs": set(),
        "first_pair_events": [],
    }
    finger_diagnostics = {
        name: {"samples_during_grasp": 0, "min_m": None, "max_m": None, "latest_m": None}
        for name in ("panda_finger_joint1", "panda_finger_joint2")
    }
    pad_geometry = {
        side: {
            "samples_during_grasp_path": 0,
            "first_position_m": None,
            "latest_position_m": None,
            "min_center_distance_m": None,
            "position_at_min_distance_m": None,
            "target_at_min_distance_m": None,
        }
        for side in ("left", "right")
    }
    midpoint_geometry = {
        "samples_during_grasp_path": 0,
        "latest_position_m": None,
        "min_target_distance_m": None,
        "max_target_distance_m": None,
        "midpoint_at_min_target_distance_m": None,
        "target_at_min_distance_m": None,
        "offset_at_min_distance_m": None,
        "hand_pose_at_min_distance": None,
        "min_pad_separation_m": None,
        "max_pad_separation_m": None,
    }
    latest_arm_joint_positions = {}
    arm_joint_sample_count = 0
    initial_arm_joint_positions = None
    initial_finger_positions = None
    latest_attachment_state = None
    initial_attachment_state = None
    attachment_state_events = []

    def on_target(message):
        if arguments.target_message_type == "target_pose":
            if int(message.target_id) != arguments.target_id:
                return
            target = PoseStamped()
            target.header = message.header
            target.pose = message.pose
        else:
            target = message
        target_samples.append(target)
        if sent_target_xyz is not None and action_started_at is not None:
            position = target.pose.position
            displacement = math.dist(
                (position.x, position.y, position.z), sent_target_xyz
            )
            target_displacements.append((current_stage, displacement))
            while (
                displacement_thresholds_m
                and displacement >= displacement_thresholds_m[0]
            ):
                threshold = displacement_thresholds_m.popleft()
                displacement_events.append(
                    {
                        "threshold_m": threshold,
                        "elapsed_sec": elapsed_action_time(),
                        "stage": current_stage,
                        "position_m": [
                            float(position.x),
                            float(position.y),
                            float(position.z),
                        ],
                    }
                )
            if current_stage in grasp_path_stages:
                pad_positions = {}
                for side in ("left", "right"):
                    try:
                        transform = tf_buffer.lookup_transform(
                            "panda_link0",
                            f"panda_{side}_contact_pad",
                            Time(),
                        )
                    except TransformException:
                        continue
                    translation = transform.transform.translation
                    pad_position = (
                        float(translation.x),
                        float(translation.y),
                        float(translation.z),
                    )
                    pad_positions[side] = pad_position
                    diagnostics = pad_geometry[side]
                    diagnostics["samples_during_grasp_path"] += 1
                    if diagnostics["first_position_m"] is None:
                        diagnostics["first_position_m"] = list(pad_position)
                    diagnostics["latest_position_m"] = list(pad_position)
                    distance = math.dist(
                        pad_position,
                        (position.x, position.y, position.z),
                    )
                    if (
                        diagnostics["min_center_distance_m"] is None
                        or distance < diagnostics["min_center_distance_m"]
                    ):
                        diagnostics["min_center_distance_m"] = distance
                        diagnostics["position_at_min_distance_m"] = list(
                            pad_position
                        )
                        diagnostics["target_at_min_distance_m"] = [
                            float(position.x),
                            float(position.y),
                            float(position.z),
                        ]
                if len(pad_positions) == 2:
                    left = pad_positions["left"]
                    right = pad_positions["right"]
                    midpoint = tuple(
                        (left[index] + right[index]) / 2.0 for index in range(3)
                    )
                    midpoint_distance = math.dist(
                        midpoint,
                        (position.x, position.y, position.z),
                    )
                    separation = math.dist(left, right)
                    midpoint_geometry["samples_during_grasp_path"] += 1
                    midpoint_geometry["latest_position_m"] = list(midpoint)
                    previous_minimum = midpoint_geometry["min_target_distance_m"]
                    if previous_minimum is None or midpoint_distance < previous_minimum:
                        midpoint_geometry["midpoint_at_min_target_distance_m"] = list(
                            midpoint
                        )
                        midpoint_geometry["target_at_min_distance_m"] = [
                            float(position.x),
                            float(position.y),
                            float(position.z),
                        ]
                        midpoint_geometry["offset_at_min_distance_m"] = [
                            midpoint[index]
                            - (position.x, position.y, position.z)[index]
                            for index in range(3)
                        ]
                        try:
                            hand_transform = tf_buffer.lookup_transform(
                                "panda_link0", "panda_hand", Time()
                            ).transform
                        except TransformException:
                            hand_transform = None
                        if hand_transform is not None:
                            midpoint_geometry["hand_pose_at_min_distance"] = {
                                "position_m": [
                                    float(hand_transform.translation.x),
                                    float(hand_transform.translation.y),
                                    float(hand_transform.translation.z),
                                ],
                                "orientation_xyzw": [
                                    float(hand_transform.rotation.x),
                                    float(hand_transform.rotation.y),
                                    float(hand_transform.rotation.z),
                                    float(hand_transform.rotation.w),
                                ],
                            }
                    for key, value, function in (
                        ("min_target_distance_m", midpoint_distance, min),
                        ("max_target_distance_m", midpoint_distance, max),
                        ("min_pad_separation_m", separation, min),
                        ("max_pad_separation_m", separation, max),
                    ):
                        previous = midpoint_geometry[key]
                        midpoint_geometry[key] = (
                            value if previous is None else function(previous, value)
                        )

    def elapsed_action_time() -> float | None:
        if action_started_at is None:
            return None
        return round(time.monotonic() - action_started_at, 4)

    def make_processed_contact_callback(side):
        def callback(message: Bool) -> None:
            diagnostics = contact_diagnostics[side]
            diagnostics["processed_messages"] += 1
            if message.data:
                diagnostics["processed_true_messages"] += 1
                diagnostics["processed_seen_true"] = True
                if diagnostics["first_processed_true_sec"] is None:
                    diagnostics["first_processed_true_sec"] = elapsed_action_time()

        return callback

    def make_raw_contact_callback(side):
        def callback(message: Contacts) -> None:
            diagnostics = contact_diagnostics[side]
            pairs = [
                (contact.collision1.name, contact.collision2.name)
                for contact in message.contacts
            ]
            diagnostics["raw_messages"] += 1
            diagnostics["raw_contacts"] += len(pairs)
            diagnostics["raw_max_contacts_per_message"] = max(
                diagnostics["raw_max_contacts_per_message"], len(pairs)
            )
            diagnostics["raw_contact_pairs"].update(pairs)
            if pairs and diagnostics["first_raw_contact_sec"] is None:
                diagnostics["first_raw_contact_sec"] = elapsed_action_time()

        return callback

    def on_fruit_contacts(message: Contacts) -> None:
        pairs = [
            (contact.collision1.name, contact.collision2.name)
            for contact in message.contacts
        ]
        fruit_contact_diagnostics["raw_messages"] += 1
        fruit_contact_diagnostics["raw_contacts"] += len(pairs)
        known_pairs = fruit_contact_diagnostics["raw_contact_pairs"]
        for pair in pairs:
            if pair not in known_pairs:
                fruit_contact_diagnostics["first_pair_events"].append(
                    {
                        "pair": list(pair),
                        "elapsed_sec": elapsed_action_time(),
                        "stage": current_stage,
                        "arm_joint_positions_rad": [
                            latest_arm_joint_positions.get(
                                f"panda_joint{index}"
                            )
                            for index in range(1, 8)
                        ],
                    }
                )
            known_pairs.add(pair)

    def on_joint_state(message: JointState) -> None:
        nonlocal arm_joint_sample_count
        positions = dict(zip(message.name, message.position, strict=False))
        observed_arm_joint = False
        for index in range(1, 8):
            name = f"panda_joint{index}"
            if name in positions:
                observed_arm_joint = True
                latest_arm_joint_positions[name] = float(positions[name])
        if observed_arm_joint:
            arm_joint_sample_count += 1
        for name, diagnostics in finger_diagnostics.items():
            if name not in positions:
                continue
            position = float(positions[name])
            diagnostics["latest_m"] = position
            if current_stage != "GRASP":
                continue
            diagnostics["samples_during_grasp"] += 1
            diagnostics["min_m"] = (
                position
                if diagnostics["min_m"] is None
                else min(diagnostics["min_m"], position)
            )
            diagnostics["max_m"] = (
                position
                if diagnostics["max_m"] is None
                else max(diagnostics["max_m"], position)
            )

    def on_attachment_state(message: String) -> None:
        nonlocal latest_attachment_state
        state = str(message.data).strip().lower()
        if state not in {"attached", "detached"}:
            attachment_state_events.append(
                {
                    "state": state,
                    "attached": None,
                    "stage": current_stage,
                    "elapsed_sec": elapsed_action_time(),
                }
            )
            return
        attached = state == "attached"
        if latest_attachment_state is attached:
            return
        latest_attachment_state = attached
        attachment_state_events.append(
            {
                "state": state,
                "attached": attached,
                "stage": current_stage,
                "elapsed_sec": elapsed_action_time(),
            }
        )

    target_ros_type = (
        TargetPose
        if arguments.target_message_type == "target_pose"
        else PoseStamped
    )
    subscription = node.create_subscription(
        target_ros_type, target_topic, on_target, qos_profile_sensor_data
    )
    diagnostic_subscriptions = [
        node.create_subscription(
            Bool,
            f"/strawberry/sim/fruit_{arguments.target_id}/{side}_contact",
            make_processed_contact_callback(side),
            qos_profile_sensor_data,
        )
        for side in ("left", "right")
    ]
    diagnostic_subscriptions.extend(
        node.create_subscription(
            Contacts,
            f"/strawberry/sim/gripper/{side}_contacts",
            make_raw_contact_callback(side),
            qos_profile_sensor_data,
        )
        for side in ("left", "right")
    )
    diagnostic_subscriptions.append(
        node.create_subscription(
            Contacts,
            "/strawberry/sim/fruit_contacts",
            on_fruit_contacts,
            qos_profile_sensor_data,
        )
    )
    diagnostic_subscriptions.append(
        node.create_subscription(
            JointState,
            "/joint_states",
            on_joint_state,
            qos_profile_sensor_data,
        )
    )
    diagnostic_subscriptions.append(
        node.create_subscription(
            String,
            f"/strawberry/sim/fruit_{arguments.target_id}/attached_state",
            on_attachment_state,
            qos_profile_sensor_data,
        )
    )
    client = ActionClient(node, PickAndPlace, "/strawberry/pick_and_place")
    feedback_log = []

    def diagnostic_report() -> dict:
        contact_report = {}
        for side, diagnostics in contact_diagnostics.items():
            contact_report[side] = {
                **diagnostics,
                "raw_contact_pairs": [
                    list(pair) for pair in sorted(diagnostics["raw_contact_pairs"])
                ],
            }
        all_displacements = [value for _, value in target_displacements]
        grasp_displacements = [
            value for stage, value in target_displacements if stage == "GRASP"
        ]
        displacement_by_stage = {}
        for stage, value in target_displacements:
            displacement_by_stage[stage] = max(
                displacement_by_stage.get(stage, 0.0), value
            )
        initial_arm = initial_arm_joint_positions or {}
        final_arm = dict(latest_arm_joint_positions)
        shared_arm_names = sorted(set(initial_arm) & set(final_arm))
        arm_joint_delta = (
            max(
                abs(final_arm[name] - initial_arm[name])
                for name in shared_arm_names
            )
            if shared_arm_names
            else None
        )
        initial_fingers = initial_finger_positions or {}
        final_fingers = {
            name: diagnostics["latest_m"]
            for name, diagnostics in finger_diagnostics.items()
        }
        final_target = target_samples[-1].pose.position if target_samples else None
        return {
            "contacts": contact_report,
            "fruit_contacts": {
                **fruit_contact_diagnostics,
                "raw_contact_pairs": [
                    list(pair)
                    for pair in sorted(
                        fruit_contact_diagnostics["raw_contact_pairs"]
                    )
                ],
            },
            "fingers": finger_diagnostics,
            "pad_geometry": pad_geometry,
            "pad_midpoint_geometry": midpoint_geometry,
            "target_pose_samples_during_action": len(target_displacements),
            "target_max_displacement_m": (
                max(all_displacements) if all_displacements else None
            ),
            "target_max_displacement_during_grasp_m": (
                max(grasp_displacements) if grasp_displacements else None
            ),
            "target_max_displacement_by_stage_m": displacement_by_stage,
            "target_displacement_events": displacement_events,
            "target_final_position_m": (
                [
                    float(final_target.x),
                    float(final_target.y),
                    float(final_target.z),
                ]
                if final_target is not None
                else None
            ),
            "arm_recovery": {
                "joint_sample_count": arm_joint_sample_count,
                "initial_positions_rad": initial_arm,
                "final_positions_rad": final_arm,
                "maximum_initial_final_delta_rad": arm_joint_delta,
            },
            "gripper_recovery": {
                "initial_positions_m": initial_fingers,
                "final_positions_m": final_fingers,
            },
            "attachment_state": {
                "initial_attached": initial_attachment_state,
                "final_attached": latest_attachment_state,
                "events": attachment_state_events,
            },
        }

    try:
        deadline = time.monotonic() + arguments.startup_timeout_sec
        latest_target = None
        while rclpy.ok() and latest_target is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    f"target did not become stable on {target_topic}"
                )
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
            if len(target_samples) == arguments.stability_samples:
                reference = target_samples[-1].pose.position
                maximum_displacement = max(
                    math.dist(
                        (sample.pose.position.x, sample.pose.position.y, sample.pose.position.z),
                        (reference.x, reference.y, reference.z),
                    )
                    for sample in target_samples
                )
                if maximum_displacement <= arguments.stability_tolerance_m:
                    latest_target = target_samples[-1]
        remaining = max(0.0, deadline - time.monotonic())
        if not client.wait_for_server(timeout_sec=remaining):
            raise RuntimeError("pick-and-place action server is unavailable")
        initial_arm_joint_positions = dict(latest_arm_joint_positions)
        initial_finger_positions = {
            name: diagnostics["latest_m"]
            for name, diagnostics in finger_diagnostics.items()
        }
        initial_attachment_state = latest_attachment_state

        goal = PickAndPlace.Goal()
        goal.target_id = arguments.target_id
        goal.target_pose = latest_target
        goal.place_pose.header.frame_id = "panda_link0"
        goal.place_pose.header.stamp = latest_target.header.stamp
        goal.place_pose.pose.position.x = arguments.place_x
        goal.place_pose.pose.position.y = arguments.place_y
        goal.place_pose.pose.position.z = arguments.place_z
        goal.place_pose.pose.orientation.w = 1.0

        target_position = latest_target.pose.position
        sent_target_xyz = (
            target_position.x,
            target_position.y,
            target_position.z,
        )
        report = {
            "target_id": arguments.target_id,
            "target_topic": target_topic,
            "target_message_type": arguments.target_message_type,
            "target_position_m": [
                target_position.x,
                target_position.y,
                target_position.z,
            ],
            "place_position_m": [
                arguments.place_x,
                arguments.place_y,
                arguments.place_z,
            ],
        }

        def on_feedback(message):
            nonlocal current_stage
            current_stage = message.feedback.stage
            feedback_log.append(
                {
                    "stage": message.feedback.stage,
                    "progress": round(float(message.feedback.progress), 3),
                    "elapsed_sec": elapsed_action_time(),
                }
            )

        action_started_at = time.monotonic()
        send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
        if not _wait_future(node, send_future, arguments.startup_timeout_sec):
            raise RuntimeError("timed out while sending pick-and-place goal")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("pick-and-place goal was rejected")

        result_future = goal_handle.get_result_async()
        if not _wait_future(node, result_future, arguments.action_timeout_sec):
            cancel_future = goal_handle.cancel_goal_async()
            _wait_future(node, cancel_future, 2.0)
            raise RuntimeError("pick-and-place action exceeded its timeout")
        wrapped_result = result_future.result()
        result = wrapped_result.result
        diagnostic_deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < diagnostic_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        action_succeeded = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and bool(result.success)
        )
        report.update(
            {
                "action_status": int(wrapped_result.status),
                "success": action_succeeded,
                "result_success": bool(result.success),
                "failure_code": int(result.failure_code),
                "message": result.message,
                "planning_time_sec": round(float(result.planning_time_sec), 4),
                "execution_time_sec": round(float(result.execution_time_sec), 4),
                "feedback": feedback_log,
                "diagnostics": diagnostic_report(),
            }
        )
        _emit_report(report, arguments.output)
        return 0 if action_succeeded else 1
    except Exception as exc:
        _emit_report(
            {
                "target_id": arguments.target_id,
                "target_topic": target_topic,
                "target_message_type": arguments.target_message_type,
                "success": False,
                "error": str(exc),
                "feedback": feedback_log,
                "diagnostics": diagnostic_report(),
            },
            arguments.output,
        )
        return 2
    finally:
        del subscription
        diagnostic_subscriptions.clear()
        del tf_listener
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
