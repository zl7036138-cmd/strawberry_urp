"""Run one bounded ADR 0087-E adaptive-candidate execution in simulation.

The probe selects a fresh stable ripe target from ``/strawberry/tracked_targets``
and sends exactly one PickAndPlace goal.  It neither reads simulator truth nor
starts the harvest orchestrator.  The corresponding receipt proves whether a
non-nominal candidate was certified and executed under the physical challenge.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Mapping, Sequence

from .candidate_execution_receipt import (
    qualify_adaptive_candidate_execution_receipt,
)
from .candidate_qualification_probe import (
    StableRipeTrack,
    load_scene_place_position,
    stable_ripe_tracks,
)
from .feasibility_probe import records_from_tracked_message


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_candidate_execution_payload(
    *,
    outcome: str,
    target: StableRipeTrack | None,
    action_result: Mapping[str, object] | None,
    feedback: Sequence[Mapping[str, object]],
    motion_events: Sequence[Mapping[str, object]],
    elapsed_wall_sec: float,
    scene_config_file: Path,
    world_file: Path,
    challenge_obstacle_spec: str,
    git_commit: str,
    place_position_m: tuple[float, float, float],
    timeouts_sec: Mapping[str, float],
) -> dict[str, object]:
    """Build an immutable development receipt and validate its evidence."""

    payload: dict[str, object] = {
        "schema_version": 1,
        "scope": "ADR0087_E_RUNTIME_ADAPTIVE_CANDIDATE_EXECUTION",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "runtime_truth_use": False,
        "trajectory_execution_allowed": True,
        "outcome": str(outcome),
        "elapsed_wall_sec": float(elapsed_wall_sec),
        "timeouts_sec": {key: float(value) for key, value in timeouts_sec.items()},
        "git_commit": str(git_commit),
        "scene_config_file": str(Path(scene_config_file)),
        "world_file": str(Path(world_file)),
        "world_file_sha256": sha256_file(world_file),
        "challenge_obstacle_spec": str(challenge_obstacle_spec),
        "place_position_m": list(place_position_m),
        "target": None if target is None else asdict(target),
        "action_result": None if action_result is None else dict(action_result),
        "feedback": [dict(row) for row in feedback],
        "motion_events": [dict(event) for event in motion_events],
        "execution_dispatched": bool(action_result is not None),
    }
    if target is None:
        payload["receipt_validation"] = {
            "passed": False,
            "errors": ["no stable ripe target was selected"],
            "selected_candidate_id": None,
            "certificate_fingerprint": None,
        }
        return payload
    validation = qualify_adaptive_candidate_execution_receipt(
        payload, target_id=target.track_id
    )
    payload["receipt_validation"] = {
        "passed": validation.passed,
        "errors": list(validation.errors),
        "selected_candidate_id": validation.selected_candidate_id,
        "certificate_fingerprint": validation.certificate_fingerprint,
    }
    return payload


def candidate_execution_probe_exit_code(payload: Mapping[str, object]) -> int:
    validation = payload.get("receipt_validation")
    passed = isinstance(validation, Mapping) and validation.get("passed") is True
    if payload.get("outcome") == "ACTION_SUCCEEDED" and passed:
        return 0
    if str(payload.get("outcome", "")).endswith("TIMEOUT"):
        return 3
    if payload.get("outcome") in {
        "NO_STABLE_RIPE_TARGET",
        "ACTION_REJECTED",
        "ACTION_ABORTED",
        "EVIDENCE_INVALID",
    }:
        return 1
    return 4


def main(argv=None) -> int:  # pragma: no cover - ROS integration only
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--world-file", type=Path, required=True)
    parser.add_argument("--challenge-obstacle-spec", required=True)
    parser.add_argument("--git-commit", required=True)
    parser.add_argument("--tracked-topic", default="/strawberry/tracked_targets")
    parser.add_argument("--action-name", default="/strawberry/pick_and_place")
    parser.add_argument("--evidence-run-id", required=True)
    parser.add_argument("--target-id", type=int, default=0)
    parser.add_argument("--observation-window", type=float, default=15.0)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--action-timeout", type=float, default=480.0)
    parser.add_argument("--evidence-settle-time", type=float, default=1.0)
    parser.add_argument("--hard-timeout", type=float, default=600.0)
    parser.add_argument("--confidence-threshold", type=float, default=0.20524564385414124)
    parser.add_argument("--maximum-sigma", type=float, default=0.015)
    parser.add_argument("--minimum-observations", type=int, default=3)
    options = parser.parse_args(argv)
    if options.output.exists():
        raise FileExistsError(options.output)
    if options.target_id < 0:
        raise ValueError("target ID must not be negative")
    if not str(options.evidence_run_id).strip():
        raise ValueError("evidence run ID is required")
    if not str(options.challenge_obstacle_spec).strip():
        raise ValueError("challenge obstacle specification is required")
    if not str(options.git_commit).strip():
        raise ValueError("git commit is required")
    if not options.world_file.is_file():
        raise FileNotFoundError(options.world_file)
    timeouts = {
        "observation": options.observation_window,
        "startup": options.startup_timeout,
        "action": options.action_timeout,
        "evidence_settle": options.evidence_settle_time,
        "hard": options.hard_timeout,
    }
    if not all(math.isfinite(value) and value > 0.0 for value in timeouts.values()):
        raise ValueError("all probe timeouts must be finite and positive")
    if options.hard_timeout < max(options.startup_timeout, options.action_timeout):
        raise ValueError("hard timeout must cover startup and action timeouts")

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from action_msgs.msg import GoalStatus
        from strawberry_interfaces.action import PickAndPlace
        from strawberry_interfaces.msg import TrackedTargetArray
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    scene = load_scene_config(options.scene_config)
    base_frame = scene.base_frame
    place_position_m = load_scene_place_position(options.scene_config)

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("generalized_candidate_execution_probe")
            self.started = time.monotonic()
            self.first_message_at: float | None = None
            self.selected: StableRipeTrack | None = None
            self.selected_item = None
            self.selected_priority = None
            self.motion_events: list[dict[str, object]] = []
            self.feedback: list[dict[str, object]] = []
            self.client = ActionClient(self, PickAndPlace, options.action_name)
            self.create_subscription(
                TrackedTargetArray, options.tracked_topic, self.on_tracks, 10
            )
            self.create_subscription(
                String, "/strawberry/motion_evidence", self.on_motion_evidence, 100
            )
            self.timer = self.create_timer(0.1, self.tick)
            self.goal_sent = False
            self.goal_handle = None
            self.action_started: float | None = None
            self.action_result: dict[str, object] | None = None
            self.result_received_at: float | None = None
            self.done = False
            self.outcome = "RECORDER_STOPPED"
            self.payload: dict[str, object] | None = None

        def on_tracks(self, message) -> None:
            if self.first_message_at is None:
                self.first_message_at = time.monotonic()
            try:
                ranked = stable_ripe_tracks(
                    records_from_tracked_message(message),
                    requested_target_id=options.target_id,
                    confidence_threshold=options.confidence_threshold,
                    maximum_sigma_m=options.maximum_sigma,
                    minimum_observations=options.minimum_observations,
                )
            except (TypeError, ValueError):
                return
            if not ranked:
                return
            candidate = ranked[0]
            item = next(
                (
                    row
                    for row in message.targets
                    if int(row.track_id) == candidate.track_id
                    and row.header.frame_id == base_frame
                ),
                None,
            )
            if item is None:
                return
            priority = (
                candidate.confidence,
                -candidate.sigma_m,
                candidate.observation_count,
                -candidate.track_id,
            )
            if self.selected_priority is None or priority > self.selected_priority:
                self.selected = candidate
                self.selected_item = item
                self.selected_priority = priority

        def on_motion_evidence(self, message) -> None:
            try:
                event = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                return
            if not isinstance(event, dict) or event.get("run_id") != options.evidence_run_id:
                return
            self.motion_events.append(
                {
                    "received_wall_offset_sec": time.monotonic() - self.started,
                    **event,
                }
            )

        def _dispatch(self) -> None:
            if self.selected is None or self.selected_item is None:
                raise RuntimeError("cannot dispatch without a stable ripe track")
            goal = PickAndPlace.Goal()
            goal.target_id = self.selected.track_id
            goal.target_pose = PoseStamped()
            goal.target_pose.header.frame_id = base_frame
            goal.target_pose.header.stamp = self.selected_item.header.stamp
            goal.target_pose.pose.position.x = float(self.selected_item.pose.position.x)
            goal.target_pose.pose.position.y = float(self.selected_item.pose.position.y)
            goal.target_pose.pose.position.z = float(self.selected_item.pose.position.z)
            goal.target_pose.pose.orientation.w = 1.0
            goal.place_pose = PoseStamped()
            goal.place_pose.header.frame_id = base_frame
            goal.place_pose.pose.position.x = place_position_m[0]
            goal.place_pose.pose.position.y = place_position_m[1]
            goal.place_pose.pose.position.z = place_position_m[2]
            goal.place_pose.pose.orientation.w = 1.0
            self.goal_sent = True
            self.action_started = time.monotonic()
            future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
            future.add_done_callback(self.on_goal_response)

        def on_feedback(self, message) -> None:
            self.feedback.append(
                {
                    "stage": str(message.feedback.stage),
                    "progress": float(message.feedback.progress),
                    "received_wall_offset_sec": time.monotonic() - self.started,
                }
            )

        def on_goal_response(self, future) -> None:
            if self.done:
                return
            try:
                handle = future.result()
            except Exception as exc:
                self.action_result = {"success": False, "error": str(exc)}
                self.result_received_at = time.monotonic()
                self.outcome = "ACTION_REJECTED"
                return
            if handle is None or not handle.accepted:
                self.action_result = {
                    "success": False,
                    "accepted": False,
                    "message": "pick-and-place goal was rejected",
                }
                self.result_received_at = time.monotonic()
                self.outcome = "ACTION_REJECTED"
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.on_action_result)

        def on_action_result(self, future) -> None:
            if self.done:
                return
            try:
                wrapped = future.result()
                result = wrapped.result
                succeeded = (
                    int(wrapped.status) == int(GoalStatus.STATUS_SUCCEEDED)
                    and bool(result.success)
                )
                self.action_result = {
                    "success": succeeded,
                    "accepted": True,
                    "action_status": int(wrapped.status),
                    "result_success": bool(result.success),
                    "failure_code": int(result.failure_code),
                    "recovery_disposition": int(result.recovery_disposition),
                    "message": str(result.message),
                    "planning_time_sec": float(result.planning_time_sec),
                    "execution_time_sec": float(result.execution_time_sec),
                }
                self.outcome = "ACTION_SUCCEEDED" if succeeded else "ACTION_ABORTED"
            except Exception as exc:
                self.action_result = {"success": False, "error": str(exc)}
                self.outcome = "ACTION_ABORTED"
            self.result_received_at = time.monotonic()

        def tick(self) -> None:
            if self.done:
                return
            now = time.monotonic()
            elapsed = now - self.started
            if elapsed >= options.hard_timeout:
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.finish("HARD_TIMEOUT")
                return
            if self.first_message_at is None:
                if elapsed >= options.startup_timeout:
                    self.finish("STARTUP_TIMEOUT")
                return
            if not self.goal_sent:
                if now - self.first_message_at < options.observation_window:
                    return
                if self.selected is None:
                    self.finish("NO_STABLE_RIPE_TARGET")
                    return
                if not self.client.server_is_ready():
                    if elapsed >= options.startup_timeout:
                        self.finish("STARTUP_TIMEOUT")
                    return
                self._dispatch()
                return
            if self.result_received_at is None:
                if (
                    self.action_started is not None
                    and now - self.action_started >= options.action_timeout
                ):
                    if self.goal_handle is not None:
                        self.goal_handle.cancel_goal_async()
                    self.finish("ACTION_TIMEOUT")
                return
            if now - self.result_received_at >= options.evidence_settle_time:
                self.finish(self.outcome)

        def finish(self, outcome: str) -> None:
            if self.done:
                return
            self.done = True
            self.outcome = outcome
            payload = build_candidate_execution_payload(
                outcome=outcome,
                target=self.selected,
                action_result=self.action_result,
                feedback=self.feedback,
                motion_events=self.motion_events,
                elapsed_wall_sec=time.monotonic() - self.started,
                scene_config_file=options.scene_config,
                world_file=options.world_file,
                challenge_obstacle_spec=options.challenge_obstacle_spec,
                git_commit=options.git_commit,
                place_position_m=place_position_m,
                timeouts_sec=timeouts,
            )
            validation = payload["receipt_validation"]
            if (
                outcome == "ACTION_SUCCEEDED"
                and isinstance(validation, Mapping)
                and validation.get("passed") is not True
            ):
                payload["outcome"] = "EVIDENCE_INVALID"
                self.outcome = "EVIDENCE_INVALID"
            self.payload = payload
            options.output.parent.mkdir(parents=True, exist_ok=True)
            temporary = options.output.with_suffix(options.output.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            temporary.replace(options.output)

    rclpy.init()
    node = Probe()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.2)
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    assert node.payload is not None
    validation = node.payload["receipt_validation"]
    print(
        json.dumps(
            {
                "outcome": node.payload["outcome"],
                "receipt_passed": validation["passed"],
                "output": str(options.output),
            },
            sort_keys=True,
        )
    )
    return candidate_execution_probe_exit_code(node.payload)


if __name__ == "__main__":
    raise SystemExit(main())
