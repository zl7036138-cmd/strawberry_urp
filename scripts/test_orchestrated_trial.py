#!/usr/bin/env python3
"""Run one trial through the T60 service/orchestrator control boundary."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time


REQUIRED_STATES = (
    "IDLE",
    "INIT",
    "ACQUIRE",
    "DETECT",
    "LOCALIZE",
    "SELECT",
    "PLAN",
    "APPROACH",
    "GRASP",
    "RETREAT",
    "PLACE",
    "VERIFY",
    "DONE",
)

NO_PICK_REQUIRED_STATES = (
    "IDLE",
    "INIT",
    "ACQUIRE",
    "DONE",
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--startup-timeout-sec", type=float, default=60.0)
    parser.add_argument("--trial-timeout-sec", type=float, default=180.0)
    parser.add_argument(
        "--control-topic", default="/strawberry/oracle/target_pose"
    )
    parser.add_argument("--detections-topic", default="/strawberry/detections")
    parser.add_argument("--target-source", default="oracle")
    parser.add_argument("--expected-target-id", type=int, default=1)
    parser.add_argument("--scenario-id", default="fixed_world")
    parser.add_argument("--target-model-name", default="")
    parser.add_argument(
        "--target-position-m",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--park-model-pose",
        action="append",
        nargs=5,
        default=[],
        metavar=("MODEL", "TARGET_ID", "X", "Y", "Z"),
        help="Move one non-target model out of the work scene before the trial.",
    )
    parser.add_argument("--pose-settle-wall-time-sec", type=float, default=1.0)
    parser.add_argument("--position-tolerance-m", type=float, default=0.005)
    parser.add_argument(
        "--control-position-tolerance-m",
        type=float,
        help=(
            "Maximum control-to-truth distance used only to reject stale scene "
            "samples; defaults to --position-tolerance-m."
        ),
    )
    parser.add_argument("--expect-shadow", action="store_true")
    parser.add_argument("--expect-no-pick", action="store_true")
    parser.add_argument(
        "--permit-missing-positive-target",
        action="store_true",
        help=(
            "After proving the settled detection stream is live, start a positive "
            "trial even when perception emitted no target so NO_PICK is measured "
            "as a behavioral perception failure rather than infrastructure loss."
        ),
    )
    parser.add_argument(
        "--minimum-detection-frames-after-scene",
        type=int,
        default=0,
        help="Require a settled perception window before starting the trial.",
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser


def _emit(payload: dict, output: Path) -> None:
    rendered = json.dumps(payload, indent=2, sort_keys=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered, flush=True)


def _pose_xyz(message) -> tuple[float, float, float]:
    position = message.pose.position
    return float(position.x), float(position.y), float(position.z)


def _distance(left, right) -> float:
    return math.sqrt(
        sum((float(first) - float(second)) ** 2 for first, second in zip(left, right))
    )


def main() -> int:  # pragma: no cover - exercised in ROS integration
    arguments = _parser().parse_args()
    if arguments.startup_timeout_sec <= 0.0 or arguments.trial_timeout_sec <= 0.0:
        raise SystemExit("timeouts must be positive")
    if arguments.expected_target_id <= 0:
        raise SystemExit("--expected-target-id must be positive")
    if not arguments.scenario_id.strip():
        raise SystemExit("--scenario-id must be non-empty")
    if arguments.pose_settle_wall_time_sec < 0.0:
        raise SystemExit("--pose-settle-wall-time-sec cannot be negative")
    if arguments.position_tolerance_m <= 0.0:
        raise SystemExit("--position-tolerance-m must be positive")
    control_position_tolerance_m = (
        arguments.position_tolerance_m
        if arguments.control_position_tolerance_m is None
        else arguments.control_position_tolerance_m
    )
    if control_position_tolerance_m <= 0.0:
        raise SystemExit("--control-position-tolerance-m must be positive")
    if arguments.minimum_detection_frames_after_scene < 0:
        raise SystemExit(
            "--minimum-detection-frames-after-scene cannot be negative"
        )
    if (
        arguments.permit_missing_positive_target
        and arguments.expect_no_pick
    ):
        raise SystemExit(
            "--permit-missing-positive-target is only valid for positive trials"
        )
    if (
        arguments.permit_missing_positive_target
        and arguments.minimum_detection_frames_after_scene <= 0
    ):
        raise SystemExit(
            "permitted missing targets require a positive settled-frame count"
        )
    configure_position = arguments.target_position_m is not None
    if configure_position != bool(arguments.target_model_name.strip()):
        raise SystemExit(
            "--target-model-name and --target-position-m must be supplied together"
        )
    parked_models = []
    parked_ids = set()
    parked_names = set()
    for raw in arguments.park_model_pose:
        model_name = str(raw[0]).strip()
        try:
            target_id = int(raw[1])
            position = tuple(float(value) for value in raw[2:])
        except ValueError as exc:
            raise SystemExit("--park-model-pose contains a non-numeric value") from exc
        if not model_name or target_id <= 0 or not all(
            math.isfinite(value) for value in position
        ):
            raise SystemExit("--park-model-pose values are invalid")
        if target_id == arguments.expected_target_id:
            raise SystemExit("the selected target cannot also be parked")
        if target_id in parked_ids or model_name in parked_names:
            raise SystemExit("parked model names and target IDs must be unique")
        parked_ids.add(target_id)
        parked_names.add(model_name)
        parked_models.append((model_name, target_id, position))
    configure_scene = configure_position or bool(parked_models)

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from strawberry_interfaces.msg import StrawberryDetectionArray, TargetPose
    from strawberry_interfaces.action import PickAndPlace

    rclpy.init()
    node = Node("strawberry_t60_orchestrated_trial_test")
    status_messages: list[dict] = []
    control_samples = 0
    control_target_counts: dict[int, int] = {}
    detection_frames = 0
    detection_count = 0
    ripe_detection_count = 0
    latest_control_pose = None
    latest_any_control_pose = None
    latest_truth_pose = None
    parked_truth_poses = {}

    def on_status(message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            status_messages.append(payload)

    def on_control_target(message: TargetPose) -> None:
        nonlocal control_samples, latest_control_pose, latest_any_control_pose
        target_id = int(message.target_id)
        latest_any_control_pose = message
        control_target_counts[target_id] = control_target_counts.get(target_id, 0) + 1
        if target_id == arguments.expected_target_id:
            control_samples += 1
            latest_control_pose = message

    def on_detections(message: StrawberryDetectionArray) -> None:
        nonlocal detection_frames, detection_count, ripe_detection_count
        detection_frames += 1
        detection_count += len(message.detections)
        ripe_detection_count += sum(
            item.maturity == item.RIPE for item in message.detections
        )

    def on_truth_pose(message: PoseStamped) -> None:
        nonlocal latest_truth_pose
        latest_truth_pose = message

    def on_parked_truth_pose(target_id: int, message: PoseStamped) -> None:
        parked_truth_poses[target_id] = message

    status_subscription = node.create_subscription(
        String, "/strawberry/trial_status", on_status, 10
    )
    target_subscription = node.create_subscription(
        TargetPose, arguments.control_topic, on_control_target, 10
    )
    detections_subscription = node.create_subscription(
        StrawberryDetectionArray, arguments.detections_topic, on_detections, 10
    )
    truth_subscription = node.create_subscription(
        PoseStamped,
        f"/strawberry/ground_truth/fruit_{arguments.expected_target_id}/pose",
        on_truth_pose,
        qos_profile_sensor_data,
    )
    parked_truth_subscriptions = [
        node.create_subscription(
            PoseStamped,
            f"/strawberry/ground_truth/fruit_{target_id}/pose",
            lambda message, identity=target_id: on_parked_truth_pose(
                identity, message
            ),
            qos_profile_sensor_data,
        )
        for _, target_id, _ in parked_models
    ]
    client = node.create_client(Trigger, "/strawberry/run_trial")
    action_client = ActionClient(node, PickAndPlace, "/strawberry/pick_and_place")
    pose_client = node.create_client(
        SetEntityPose, "/world/strawberry_orchard/set_pose"
    )

    def request_model_pose(
        model_name: str, desired_position: tuple[float, float, float]
    ) -> int:
        maximum_attempts = 2
        for attempt in range(1, maximum_attempts + 1):
            request = SetEntityPose.Request()
            request.entity.name = model_name
            request.entity.type = Entity.MODEL
            request.pose.position.x = desired_position[0]
            request.pose.position.y = desired_position[1]
            request.pose.position.z = desired_position[2]
            request.pose.orientation.w = 1.0
            future = pose_client.call_async(request)
            response_deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < response_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                remove_pending = getattr(pose_client, "remove_pending_request", None)
                if callable(remove_pending):
                    remove_pending(future)
                continue
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(f"Gazebo rejected pose for {model_name}")
            return attempt
        raise RuntimeError(
            f"Gazebo set_pose timed out for {model_name} after "
            f"{maximum_attempts} attempts"
        )

    report: dict = {
        "schema_version": 1,
        "scenario_id": arguments.scenario_id,
        "expected_target_source": arguments.target_source,
        "expected_control_topic": arguments.control_topic,
        "expected_detections_topic": arguments.detections_topic,
        "expected_outcome": (
            "NO_PICK" if arguments.expect_no_pick else "SUCCESS"
        ),
        "expected_target_id": arguments.expected_target_id,
        "requested_target_model_name": arguments.target_model_name or None,
        "requested_target_position_m": arguments.target_position_m,
        "requested_parked_model_poses": [
            {
                "model_name": model_name,
                "target_id": target_id,
                "position_m": position,
            }
            for model_name, target_id, position in parked_models
        ],
        "pose_configuration_attempts": [],
    }
    try:
        startup_deadline = time.monotonic() + arguments.startup_timeout_sec
        while time.monotonic() < startup_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                client.service_is_ready()
                and action_client.server_is_ready()
                and latest_truth_pose is not None
                and all(target_id in parked_truth_poses for target_id in parked_ids)
                and (not configure_scene or pose_client.service_is_ready())
            ):
                break
        if not client.service_is_ready():
            raise RuntimeError("run_trial service did not become ready")
        if not action_client.server_is_ready():
            raise RuntimeError("pick-and-place action did not become ready")
        if latest_truth_pose is None:
            raise RuntimeError("target ground-truth stream did not become ready")
        if configure_scene and not pose_client.service_is_ready():
            raise RuntimeError("Gazebo set_pose service did not become ready")

        configured_truth_position = _pose_xyz(latest_truth_pose)
        configured_control_position = None
        control_samples_before_scene_configuration = control_samples
        detection_frames_before_scene_configuration = detection_frames
        detections_before_scene_configuration = detection_count
        ripe_detections_before_scene_configuration = ripe_detection_count
        configured_parked_positions = []
        for model_name, target_id, desired_position in parked_models:
            attempts = request_model_pose(model_name, desired_position)
            report["pose_configuration_attempts"].append(
                {"model_name": model_name, "attempts": attempts}
            )
            acknowledgement_deadline = time.monotonic() + 5.0
            while time.monotonic() < acknowledgement_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (
                    target_id in parked_truth_poses
                    and _distance(
                        _pose_xyz(parked_truth_poses[target_id]), desired_position
                    )
                    <= arguments.position_tolerance_m
                ):
                    break
            else:
                raise RuntimeError(
                    f"ground truth did not acknowledge parked pose for {model_name}"
                )
            configured_parked_positions.append(
                {
                    "model_name": model_name,
                    "target_id": target_id,
                    "position_m": _pose_xyz(parked_truth_poses[target_id]),
                }
            )
        if configure_position:
            desired_position = tuple(float(value) for value in arguments.target_position_m)
            attempts = request_model_pose(arguments.target_model_name, desired_position)
            report["pose_configuration_attempts"].append(
                {"model_name": arguments.target_model_name, "attempts": attempts}
            )

            acknowledgement_deadline = time.monotonic() + 5.0
            while time.monotonic() < acknowledgement_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (
                    latest_truth_pose is not None
                    and _distance(_pose_xyz(latest_truth_pose), desired_position)
                    <= arguments.position_tolerance_m
                ):
                    break
            else:
                raise RuntimeError("ground truth did not acknowledge target position")

            settle_deadline = time.monotonic() + arguments.pose_settle_wall_time_sec
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(
                    node,
                    timeout_sec=min(0.05, settle_deadline - time.monotonic()),
                )
            configured_truth_position = _pose_xyz(latest_truth_pose)

            # Gravity may settle Z onto the table. X/Y define the diagnostic
            # position and must remain where requested before motion starts.
            if math.hypot(
                configured_truth_position[0] - desired_position[0],
                configured_truth_position[1] - desired_position[1],
            ) > arguments.position_tolerance_m:
                raise RuntimeError("target drifted outside diagnostic X/Y tolerance")

        settled_detection_frame_baseline = detection_frames
        settled_detection_count_baseline = detection_count
        settled_ripe_detection_baseline = ripe_detection_count
        settled_control_sample_baseline = control_samples
        settled_total_control_baseline = sum(control_target_counts.values())

        # Configure the deterministic scene before requiring observations.
        # A positive trial waits for a synchronized control target. A negative
        # trial instead proves the perception process is live by observing a
        # fixed number of post-setup detection-array frames without requiring
        # a ripe target.
        observation_deadline = time.monotonic() + arguments.startup_timeout_sec
        if arguments.expect_no_pick or arguments.permit_missing_positive_target:
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (
                    detection_frames - settled_detection_frame_baseline
                    >= arguments.minimum_detection_frames_after_scene
                ):
                    break
            else:
                raise RuntimeError(
                    "settled scene did not produce enough detection frames"
                )
            if arguments.permit_missing_positive_target and latest_control_pose is not None:
                if not configure_position or (
                    _distance(
                        _pose_xyz(latest_control_pose), configured_truth_position
                    )
                    <= control_position_tolerance_m
                ):
                    configured_control_position = _pose_xyz(latest_control_pose)
        else:
            while time.monotonic() < observation_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if latest_control_pose is None:
                    continue
                if configure_position and (
                    _distance(
                        _pose_xyz(latest_control_pose), configured_truth_position
                    )
                    > control_position_tolerance_m
                ):
                    continue
                configured_control_position = _pose_xyz(latest_control_pose)
                break
            if configured_control_position is None:
                raise RuntimeError(
                    f"{arguments.target_source} control target did not become ready "
                    "after scene configuration"
                )

        report.update(
            {
                "position_configuration_applied": configure_position,
                "parked_model_configuration_applied": (
                    len(configured_parked_positions) == len(parked_models)
                ),
                "configured_parked_model_poses": configured_parked_positions,
                "configured_truth_position_m": configured_truth_position,
                "configured_control_position_m": configured_control_position,
                "position_tolerance_m": arguments.position_tolerance_m,
                "control_position_tolerance_m": control_position_tolerance_m,
                "control_samples_before_scene_configuration": (
                    control_samples_before_scene_configuration
                ),
                "control_samples_after_scene_configuration": (
                    control_samples - control_samples_before_scene_configuration
                ),
                "detection_frames_before_scene_configuration": (
                    detection_frames_before_scene_configuration
                ),
                "detection_frames_after_scene_configuration": (
                    detection_frames - detection_frames_before_scene_configuration
                ),
                "detections_before_scene_configuration": (
                    detections_before_scene_configuration
                ),
                "detections_after_scene_configuration": (
                    detection_count - detections_before_scene_configuration
                ),
                "ripe_detections_before_scene_configuration": (
                    ripe_detections_before_scene_configuration
                ),
                "ripe_detections_after_scene_configuration": (
                    ripe_detection_count - ripe_detections_before_scene_configuration
                ),
                "settled_window_detection_frames": (
                    detection_frames - settled_detection_frame_baseline
                ),
                "settled_window_detection_count": (
                    detection_count - settled_detection_count_baseline
                ),
                "settled_window_ripe_detection_count": (
                    ripe_detection_count - settled_ripe_detection_baseline
                ),
                "settled_window_expected_target_control_samples": (
                    control_samples - settled_control_sample_baseline
                ),
                "settled_window_total_control_samples": (
                    sum(control_target_counts.values())
                    - settled_total_control_baseline
                ),
            }
        )

        request_future = client.call_async(Trigger.Request())
        call_deadline = time.monotonic() + 5.0
        while not request_future.done() and time.monotonic() < call_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not request_future.done():
            raise RuntimeError("run_trial service call timed out")
        response = request_future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"run_trial was rejected: {message}")

        trial_deadline = time.monotonic() + arguments.trial_timeout_sec
        final_status = None
        while time.monotonic() < trial_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if status_messages:
                candidate = status_messages[-1]
                if candidate.get("state") in {"DONE", "FAILED"}:
                    final_status = candidate
                    break
        if final_status is None:
            raise RuntimeError("orchestrated trial did not reach a terminal state")

        history = final_status.get("state_history", [])
        visited = []
        for record in history:
            for key in ("previous", "current"):
                value = record.get(key)
                if value and value not in visited:
                    visited.append(value)
        if final_status.get("state") not in visited:
            visited.append(final_status.get("state"))
        required_states = (
            NO_PICK_REQUIRED_STATES
            if arguments.expect_no_pick
            else REQUIRED_STATES
        )
        state_machine_complete = all(state in visited for state in required_states)
        full_manipulation_state_machine_complete = all(
            state in visited for state in REQUIRED_STATES
        )
        expected_control_target_id = (
            None
            if arguments.expect_no_pick
            or (
                arguments.permit_missing_positive_target
                and final_status.get("outcome") == "NO_PICK"
            )
            else arguments.expected_target_id
        )
        source_isolated = (
            final_status.get("target_source") == arguments.target_source
            and final_status.get("control_target_topic") == arguments.control_topic
            and final_status.get("shadow_target_topic") != arguments.control_topic
            and final_status.get("control_target_id")
            == expected_control_target_id
        )
        safe_no_pick = (
            arguments.expect_no_pick
            and final_status.get("state") == "DONE"
            and final_status.get("outcome") == "NO_PICK"
            and final_status.get("control_target_id") is None
            and source_isolated
            and state_machine_complete
        )
        manipulation_success = (
            not arguments.expect_no_pick
            and final_status.get("state") == "DONE"
            and final_status.get("outcome") == "SUCCESS"
            and source_isolated
            and state_machine_complete
            and (
                not arguments.expect_shadow
                or (
                    final_status.get("shadow_enabled") is True
                    and int(final_status.get("shadow_detection_frames", 0)) > 0
                )
            )
        )
        success = safe_no_pick if arguments.expect_no_pick else manipulation_success
        report.update(
            {
                "success": success,
                "safe_no_pick": safe_no_pick,
                "manipulation_success": manipulation_success,
                "fruit_picked": final_status.get("outcome") == "SUCCESS",
                "unsafe_control_attempt": (
                    final_status.get("control_target_id") is not None
                ),
                "source_isolated": source_isolated,
                "state_machine_complete": state_machine_complete,
                "full_manipulation_state_machine_complete": (
                    full_manipulation_state_machine_complete
                ),
                "visited_states": visited,
                "control_samples_before_trial": control_samples,
                "control_target_counts": dict(sorted(control_target_counts.items())),
                "detection_frames": detection_frames,
                "detection_count": detection_count,
                "ripe_detection_count": ripe_detection_count,
                "planning_time_sec": final_status.get("planning_time_sec"),
                "execution_time_sec": final_status.get("execution_time_sec"),
                "failure_code": final_status.get("failure_code"),
                "failure_message": final_status.get("failure_message", ""),
                "final_status": final_status,
                "status_message_count": len(status_messages),
                "shadow_evidence_required": arguments.expect_shadow,
                "shadow_evidence_present": (
                    final_status.get("shadow_enabled") is True
                    and int(final_status.get("shadow_detection_frames", 0)) > 0
                ),
                "shadow_detection_frames": int(
                    final_status.get("shadow_detection_frames", 0)
                ),
                "shadow_detection_count": int(
                    final_status.get("shadow_detection_count", 0)
                ),
                "shadow_ripe_detection_count": int(
                    final_status.get("shadow_ripe_detection_count", 0)
                ),
                "shadow_target_observations": int(
                    final_status.get("shadow_observations", 0)
                ),
            }
        )
        _emit(report, arguments.output)
        return 0 if success else 1
    except Exception as exc:
        report.update(
            {
                "success": False,
                "error": str(exc),
                "control_samples_before_trial": control_samples,
                "control_target_counts": dict(sorted(control_target_counts.items())),
                "detection_frames": detection_frames,
                "detection_count": detection_count,
                "ripe_detection_count": ripe_detection_count,
                "latest_control_position_m": (
                    _pose_xyz(latest_control_pose)
                    if latest_control_pose is not None
                    else None
                ),
                "latest_any_control_target_id": (
                    int(latest_any_control_pose.target_id)
                    if latest_any_control_pose is not None
                    else None
                ),
                "latest_any_control_position_m": (
                    _pose_xyz(latest_any_control_pose)
                    if latest_any_control_pose is not None
                    else None
                ),
                "latest_truth_position_m": (
                    _pose_xyz(latest_truth_pose)
                    if latest_truth_pose is not None
                    else None
                ),
                "latest_control_to_truth_distance_m": (
                    _distance(
                        _pose_xyz(latest_control_pose),
                        _pose_xyz(latest_truth_pose),
                    )
                    if latest_control_pose is not None
                    and latest_truth_pose is not None
                    else None
                ),
                "status_messages": status_messages,
            }
        )
        _emit(report, arguments.output)
        return 2
    finally:
        del status_subscription
        del target_subscription
        del detections_subscription
        del truth_subscription
        for subscription in parked_truth_subscriptions:
            del subscription
        client.destroy()
        action_client.destroy()
        pose_client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
