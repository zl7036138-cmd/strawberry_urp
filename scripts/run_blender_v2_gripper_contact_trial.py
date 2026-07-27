#!/usr/bin/env python3
"""Execute the frozen gripper-only Blender-v2 contact round trip."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
MANIPULATION_ROOT = ROOT / "ros2_ws" / "src" / "strawberry_manipulation"
if str(MANIPULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(MANIPULATION_ROOT))

from strawberry_manipulation.gripper_contact_gate import (  # noqa: E402
    load_contact_contract,
    sha256,
    summarize_contact_trial,
)


ARM_JOINTS = tuple(f"panda_joint{index}" for index in range(1, 8))
FINGER_JOINTS = ("panda_finger_joint1", "panda_finger_joint2")


def _rotate_vector_by_quaternion(
    vector: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    x, y, z = vector
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1.0e-12:
        raise ValueError("hand quaternion is zero")
    qx, qy, qz, qw = (component / norm for component in quaternion)
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def _projection_between(
    fruit: tuple[float, float, float],
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[bool, float]:
    separation = tuple(right[i] - left[i] for i in range(3))
    denominator = sum(value * value for value in separation)
    if denominator <= 1.0e-12:
        return False, math.nan
    projection = sum(
        (fruit[i] - left[i]) * separation[i] for i in range(3)
    ) / denominator
    return 0.0 <= projection <= 1.0, projection


def _fingerprint(path: Path) -> dict[str, object]:
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def main() -> int:  # pragma: no cover - ROS/Gazebo integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    options, ros_args = parser.parse_known_args()
    if options.output.exists():
        raise FileExistsError(f"refusing to overwrite {options.output}")
    contract = load_contact_contract(
        options.contract, options.repository_root
    )

    import rclpy
    from action_msgs.msg import GoalStatus
    from control_msgs.action import ParallelGripperCommand
    from geometry_msgs.msg import PoseStamped
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Contacts, Entity
    from ros_gz_interfaces.srv import SetEntityPose
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener

    parameters = contract["parameters"]
    runtime = contract["runtime"]
    target_id = int(parameters["target_id"])
    target_model = str(parameters["target_model"])
    startup_timeout = float(runtime["startup_timeout_sec"])
    action_timeout = float(runtime["action_timeout_sec"])
    contact_settle = float(runtime["contact_settle_sec"])
    canonical_position = tuple(
        float(value) for value in parameters["canonical_restore_position_m"]
    )

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("blender_v2_gripper_contact_probe")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.latest_joints: dict[str, float] = {}
            self.arm_samples: list[dict[str, float]] = []
            self.truth_position: tuple[float, float, float] | None = None
            self.raw_target_contact = {"left": False, "right": False}
            self.processed_target_contact = {"left": False, "right": False}
            self.non_target_contact_seen = False
            self.raw_pairs: dict[str, list[list[str]]] = {
                "left": [],
                "right": [],
            }
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pose_client = self.create_client(
                SetEntityPose, "/world/strawberry_orchard/set_pose"
            )
            prefix = f"/strawberry/sim/fruit_{target_id}"
            self.attach_client = self.create_client(
                Trigger, f"{prefix}/attach"
            )
            self.detach_client = self.create_client(
                Trigger, f"{prefix}/detach"
            )
            self.gripper_client = ActionClient(
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
                f"/strawberry/ground_truth/fruit_{target_id}/pose",
                self._on_truth,
                qos_profile_sensor_data,
            )
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

        def _on_joints(self, message: JointState) -> None:
            self.latest_joints.update(
                {
                    str(name): float(position)
                    for name, position in zip(
                        message.name, message.position, strict=False
                    )
                }
            )
            if all(name in self.latest_joints for name in ARM_JOINTS):
                self.arm_samples.append(
                    {
                        name: self.latest_joints[name]
                        for name in ARM_JOINTS
                    }
                )

        def _on_truth(self, message: PoseStamped) -> None:
            position = message.pose.position
            self.truth_position = (
                float(position.x),
                float(position.y),
                float(position.z),
            )

        def _on_raw_contact(self, side: str, message: Contacts) -> None:
            for contact in message.contacts:
                pair = (
                    str(contact.collision1.name),
                    str(contact.collision2.name),
                )
                if len(self.raw_pairs[side]) < 20:
                    self.raw_pairs[side].append(list(pair))
                combined = " ".join(pair)
                if f"{target_model}::" in combined:
                    self.raw_target_contact[side] = True
                if any(
                    f"strawberry_{other}::" in combined
                    for other in (2, 3)
                    if other != target_id
                ):
                    self.non_target_contact_seen = True

        def _on_processed_contact(self, side: str, message: Bool) -> None:
            if bool(message.data):
                self.processed_target_contact[side] = True

        def wait_for(self, predicate, timeout: float, description: str) -> None:
            deadline = time.monotonic() + timeout
            while rclpy.ok() and time.monotonic() < deadline:
                if predicate():
                    return
                rclpy.spin_once(self, timeout_sec=0.02)
            if not predicate():
                raise TimeoutError(f"timed out waiting for {description}")

        def wait_future(self, future, timeout: float, description: str):
            self.wait_for(future.done, timeout, description)
            if future.exception() is not None:
                raise RuntimeError(
                    f"{description} failed: {future.exception()}"
                )
            result = future.result()
            if result is None:
                raise RuntimeError(f"{description} returned no result")
            return result

        def transform_position(
            self, frame: str
        ) -> tuple[float, float, float]:
            transform = self.tf_buffer.lookup_transform(
                "panda_link0", frame, rclpy.time.Time()
            ).transform.translation
            return (
                float(transform.x),
                float(transform.y),
                float(transform.z),
            )

        def hand_target_position(self, offset_m: float):
            transform = self.tf_buffer.lookup_transform(
                "panda_link0", "panda_hand", rclpy.time.Time()
            ).transform
            translation = transform.translation
            rotation = transform.rotation
            offset = _rotate_vector_by_quaternion(
                (0.0, 0.0, offset_m),
                (
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
            )
            return tuple(
                value + delta
                for value, delta in zip(
                    (
                        float(translation.x),
                        float(translation.y),
                        float(translation.z),
                    ),
                    offset,
                )
            )

        def set_fruit_pose(
            self, position: tuple[float, float, float]
        ) -> None:
            request = SetEntityPose.Request()
            request.entity.name = target_model
            request.entity.type = Entity.MODEL
            (
                request.pose.position.x,
                request.pose.position.y,
                request.pose.position.z,
            ) = position
            request.pose.orientation.w = 1.0
            response = self.wait_future(
                self.pose_client.call_async(request),
                5.0,
                f"set_pose {target_model}",
            )
            if not response.success:
                raise RuntimeError(f"Gazebo rejected set_pose for {target_model}")

        def call_trigger(self, client, label: str):
            return self.wait_future(
                client.call_async(Trigger.Request()),
                action_timeout,
                label,
            )

        def command_gripper(self, position_m: float) -> dict[str, object]:
            goal = ParallelGripperCommand.Goal()
            goal.command.name = ["panda_finger_joint1"]
            goal.command.position = [position_m]
            goal.command.effort = [
                float(parameters["gripper_effort_n"])
            ]
            handle = self.wait_future(
                self.gripper_client.send_goal_async(goal),
                action_timeout,
                f"accept gripper {position_m}",
            )
            if not handle.accepted:
                raise RuntimeError(f"gripper rejected {position_m}")
            wrapped = self.wait_future(
                handle.get_result_async(),
                action_timeout,
                f"complete gripper {position_m}",
            )
            result = wrapped.result
            state = result.state
            return {
                "command_m_per_finger": position_m,
                "status": int(wrapped.status),
                "status_succeeded": (
                    wrapped.status == GoalStatus.STATUS_SUCCEEDED
                ),
                "reached_goal": bool(result.reached_goal),
                "stalled": bool(result.stalled),
                "result_state": {
                    str(name): float(value)
                    for name, value in zip(
                        state.name, state.position, strict=False
                    )
                },
            }

    rclpy.init(args=ros_args)
    node = Probe()
    runtime_errors: list[str] = []
    observations: dict[str, object] = {
        "planning_started": False,
        "pick_action_started": False,
        "gripper_command_count": 0,
        "non_target_contact_seen": False,
    }
    baseline_arm: dict[str, float] | None = None
    try:
        node.wait_for(
            lambda: (
                all(name in node.latest_joints for name in ARM_JOINTS + FINGER_JOINTS)
                and node.truth_position is not None
                and node.pose_client.service_is_ready()
                and node.attach_client.service_is_ready()
                and node.detach_client.service_is_ready()
                and node.gripper_client.server_is_ready()
            ),
            startup_timeout,
            "joint state, truth, services, and gripper action",
        )
        node.wait_for(
            lambda: all(
                node.tf_buffer.can_transform(
                    "panda_link0", frame, rclpy.time.Time()
                )
                for frame in (
                    "panda_hand",
                    "panda_left_contact_pad",
                    "panda_right_contact_pad",
                )
            ),
            startup_timeout,
            "hand and pad transforms",
        )
        baseline_arm = {
            name: node.latest_joints[name] for name in ARM_JOINTS
        }
        observations["initial_arm_joints_rad"] = dict(baseline_arm)
        observations["initial_finger_positions_m"] = {
            name: node.latest_joints[name] for name in FINGER_JOINTS
        }
        ready = node.call_trigger(node.detach_client, "initial detach readiness")
        observations["initial_detach_ready"] = bool(ready.success)
        observations["initial_detach_message"] = str(ready.message)
        if not ready.success:
            raise RuntimeError(
                f"attachment backend not ready: {ready.message}"
            )

        initial_open = node.command_gripper(
            float(parameters["gripper_open_width_m_per_finger"])
        )
        observations["gripper_command_count"] = 1
        observations["initial_open_action"] = initial_open
        observations["initial_open_succeeded"] = bool(
            initial_open["status_succeeded"]
            and initial_open["reached_goal"]
        )
        open_width = float(parameters["gripper_open_width_m_per_finger"])
        node.wait_for(
            lambda: all(
                abs(node.latest_joints[name] - open_width) <= 0.003
                for name in FINGER_JOINTS
            ),
            action_timeout,
            "initial open joint positions",
        )

        target_position = node.hand_target_position(
            float(parameters["tool_center_offset_m"])
        )
        observations["commanded_target_position_m"] = list(target_position)
        node.set_fruit_pose(target_position)
        node.wait_for(
            lambda: (
                node.truth_position is not None
                and math.dist(node.truth_position, target_position) <= 0.002
            ),
            action_timeout,
            "target fruit at hand centreline",
        )
        observations["observed_target_position_m"] = list(node.truth_position)
        observations["target_pose_error_m"] = math.dist(
            node.truth_position, target_position
        )

        close = node.command_gripper(
            float(parameters["gripper_close_width_m_per_finger"])
        )
        observations["gripper_command_count"] = 2
        observations["close_action"] = close
        observations["close_succeeded_or_stalled"] = bool(
            close["status_succeeded"]
            and (close["reached_goal"] or close["stalled"])
        )
        contact_deadline = time.monotonic() + contact_settle
        while time.monotonic() < contact_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        closed_positions = {
            name: node.latest_joints[name] for name in FINGER_JOINTS
        }
        observations["closed_finger_positions_m"] = closed_positions
        observations["measured_contact_width_m_per_finger"] = (
            sum(closed_positions.values()) / 2.0
        )
        observations["closed_finger_symmetry_error_m"] = abs(
            closed_positions[FINGER_JOINTS[0]]
            - closed_positions[FINGER_JOINTS[1]]
        )
        observations["raw_left_target_contact_seen"] = (
            node.raw_target_contact["left"]
        )
        observations["raw_right_target_contact_seen"] = (
            node.raw_target_contact["right"]
        )
        observations["processed_left_target_contact_seen"] = (
            node.processed_target_contact["left"]
        )
        observations["processed_right_target_contact_seen"] = (
            node.processed_target_contact["right"]
        )
        observations["non_target_contact_seen"] = (
            node.non_target_contact_seen
        )
        observations["raw_contact_pairs"] = dict(node.raw_pairs)

        fruit = node.truth_position
        if fruit is None:
            raise RuntimeError("target truth disappeared after close")
        pads = {
            side: node.transform_position(f"panda_{side}_contact_pad")
            for side in ("left", "right")
        }
        observations["pad_positions_m"] = {
            side: list(position) for side, position in pads.items()
        }
        observations["pad_center_distances_m"] = {
            side: math.dist(fruit, position)
            for side, position in pads.items()
        }
        projected, projection = _projection_between(
            fruit, pads["left"], pads["right"]
        )
        observations["fruit_projected_between_pads"] = projected
        observations["fruit_projection_between_pads"] = projection

        attached = node.call_trigger(node.attach_client, "guarded attach")
        observations["attach_succeeded"] = bool(attached.success)
        observations["attach_message"] = str(attached.message)
        if not attached.success:
            raise RuntimeError(f"guarded attach failed: {attached.message}")
        detached = node.call_trigger(node.detach_client, "guarded detach")
        observations["detach_succeeded"] = bool(detached.success)
        observations["detach_message"] = str(detached.message)
        if not detached.success:
            raise RuntimeError(f"guarded detach failed: {detached.message}")

        node.set_fruit_pose(canonical_position)
        node.wait_for(
            lambda: (
                node.truth_position is not None
                and math.dist(node.truth_position, canonical_position)
                <= 0.002
            ),
            action_timeout,
            "canonical target restore",
        )
        observations["restored_target_position_m"] = list(node.truth_position)
        observations["restore_pose_error_m"] = math.dist(
            node.truth_position, canonical_position
        )

        reopen = node.command_gripper(open_width)
        observations["gripper_command_count"] = 3
        observations["reopen_action"] = reopen
        observations["reopen_succeeded"] = bool(
            reopen["status_succeeded"] and reopen["reached_goal"]
        )
        node.wait_for(
            lambda: all(
                abs(node.latest_joints[name] - open_width) <= 0.003
                for name in FINGER_JOINTS
            ),
            action_timeout,
            "reopened joint positions",
        )
        final_fingers = {
            name: node.latest_joints[name] for name in FINGER_JOINTS
        }
        observations["final_finger_positions_m"] = final_fingers
        observations["open_recovery_error_m"] = max(
            abs(value - open_width) for value in final_fingers.values()
        )
    except (Exception, TransformException) as exc:
        runtime_errors.append(f"contact runtime failed: {exc}")
    finally:
        if baseline_arm is not None and node.arm_samples:
            observations["maximum_arm_joint_delta_rad"] = max(
                abs(sample[name] - baseline_arm[name])
                for sample in node.arm_samples
                for name in ARM_JOINTS
            )
            observations["final_arm_joints_rad"] = {
                name: node.latest_joints.get(name)
                for name in ARM_JOINTS
            }
        else:
            observations["maximum_arm_joint_delta_rad"] = None
        summary = summarize_contact_trial(
            contract=contract,
            observations=observations,
            runtime_errors=runtime_errors,
        )
        summary["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
        summary["contract"] = _fingerprint(
            Path(contract["_contract_path"])
        )
        summary["resolved_inputs"] = {
            label: _fingerprint(Path(path))
            for label, path in contract["_resolved_paths"].items()
        }
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(summary, sort_keys=True), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
