"""Capture one zero-motion ADR 0087-D candidate-qualification receipt.

The probe deliberately has no action client and never publishes a controller,
gripper, attachment, or harvest request. It selects one fresh, stable ripe
track from perception, invokes the plan-only qualification service once, and
records the service trace together with the motion-evidence stream needed to
audit that boundary.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Mapping, Sequence

from .candidate_qualification_receipt import qualify_candidate_plan_only_receipt
from .feasibility_probe import records_from_tracked_message


RIPE_MATURITY = 1


@dataclass(frozen=True)
class StableRipeTrack:
    """Perception-only target snapshot accepted by the development probe."""

    track_id: int
    position_m: tuple[float, float, float]
    confidence: float
    sigma_m: float
    observation_count: int
    last_seen_sec: float


def place_position_from_mapping(data: Mapping[str, object]) -> tuple[float, float, float]:
    """Read the configured static collection-bin drop point, fail closed."""

    bin_data = data.get("bin")
    if not isinstance(bin_data, Mapping):
        raise ValueError("scene config lacks a bin mapping")
    raw = bin_data.get("place_pose_m")
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError("bin.place_pose_m must contain exactly three values")
    position = tuple(float(value) for value in raw)
    if not all(math.isfinite(value) for value in position):
        raise ValueError("bin.place_pose_m must be finite")
    return position  # type: ignore[return-value]


def load_scene_place_position(path: Path) -> tuple[float, float, float]:
    """Load the static bin position without accessing simulator ground truth."""

    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment prerequisite
        raise RuntimeError("loading scene config requires PyYAML") from exc
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, Mapping):
        raise ValueError("scene config root must be a mapping")
    return place_position_from_mapping(data)


def stable_ripe_tracks(
    records: Sequence[Mapping[str, object]],
    *,
    requested_target_id: int = 0,
    confidence_threshold: float = 0.20524564385414124,
    maximum_sigma_m: float = 0.015,
    minimum_observations: int = 3,
) -> tuple[StableRipeTrack, ...]:
    """Return deterministic perception-only candidates for a plan-only call."""

    if requested_target_id < 0:
        raise ValueError("requested target ID must not be negative")
    if not (
        math.isfinite(confidence_threshold)
        and 0.0 <= confidence_threshold <= 1.0
        and math.isfinite(maximum_sigma_m)
        and maximum_sigma_m > 0.0
        and minimum_observations > 0
    ):
        raise ValueError("stable-track thresholds are invalid")
    candidates: list[StableRipeTrack] = []
    for record in records:
        try:
            track_id = int(record["track_id"])
            maturity = int(record["maturity"])
            position = tuple(float(value) for value in record["position"])
            confidence = float(record["confidence"])
            sigma_m = float(record["sigma_m"])
            observation_count = int(record["observation_count"])
            last_seen_sec = float(record["last_seen_sec"])
        except (KeyError, TypeError, ValueError):
            continue
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            continue
        if not all(math.isfinite(value) for value in (confidence, sigma_m, last_seen_sec)):
            continue
        if (
            track_id <= 0
            or maturity != RIPE_MATURITY
            or (requested_target_id and track_id != requested_target_id)
            or confidence < confidence_threshold
            or sigma_m > maximum_sigma_m
            or observation_count < minimum_observations
        ):
            continue
        candidates.append(
            StableRipeTrack(
                track_id=track_id,
                position_m=position,  # type: ignore[arg-type]
                confidence=confidence,
                sigma_m=sigma_m,
                observation_count=observation_count,
                last_seen_sec=last_seen_sec,
            )
        )
    return tuple(
        sorted(
            candidates,
            key=lambda target: (
                -target.confidence,
                target.sigma_m,
                -target.observation_count,
                target.track_id,
            ),
        )
    )


def response_to_mapping(response) -> dict[str, object]:
    """Serialize only the public plan-only service response fields."""

    rows = []
    for evaluation in getattr(response, "evaluations", ()):
        rows.append(
            {
                "candidate_id": str(evaluation.candidate_id),
                "geometry_fingerprint": str(evaluation.geometry_fingerprint),
                "result": str(evaluation.result),
                "feasible": bool(evaluation.feasible),
                "allocated_budget_sec": float(evaluation.allocated_budget_sec),
                "planning_time_sec": float(evaluation.planning_time_sec),
                "joint_travel_rad": float(evaluation.joint_travel_rad),
                "stages": list(evaluation.stages),
                "detail": str(evaluation.detail),
            }
        )
    return {
        "feasible": bool(getattr(response, "feasible", False)),
        "plan_only": bool(getattr(response, "plan_only", False)),
        "scene_isolated": bool(getattr(response, "scene_isolated", False)),
        "cleanup_succeeded": bool(getattr(response, "cleanup_succeeded", False)),
        "status": str(getattr(response, "status", "")),
        "selected_candidate_id": str(
            getattr(response, "selected_candidate_id", "")
        ),
        "geometry_fingerprint": str(getattr(response, "geometry_fingerprint", "")),
        "certificate_fingerprint": str(
            getattr(response, "certificate_fingerprint", "")
        ),
        "scene_signature": str(getattr(response, "scene_signature", "")),
        "planning_time_sec": float(getattr(response, "planning_time_sec", 0.0)),
        "joint_travel_rad": float(getattr(response, "joint_travel_rad", 0.0)),
        "evaluations": rows,
        "message": str(getattr(response, "message", "")),
    }


def build_candidate_qualification_payload(
    *,
    outcome: str,
    target: StableRipeTrack | None,
    response: Mapping[str, object] | None,
    motion_events: Sequence[Mapping[str, object]],
    elapsed_wall_sec: float,
    scene_config_file: Path,
    place_position_m: tuple[float, float, float],
    timeouts_sec: Mapping[str, float],
) -> dict[str, object]:
    """Create one self-contained, non-formal runtime receipt."""

    payload: dict[str, object] = {
        "schema_version": 1,
        "scope": "ADR0087_D_PLAN_ONLY_CANDIDATE_QUALIFICATION",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "runtime_truth_use": False,
        "trajectory_execution_allowed": False,
        "outcome": str(outcome),
        "elapsed_wall_sec": float(elapsed_wall_sec),
        "timeouts_sec": {key: float(value) for key, value in timeouts_sec.items()},
        "scene_config_file": str(Path(scene_config_file)),
        "place_position_m": list(place_position_m),
        "target": None if target is None else asdict(target),
        "service_response": None if response is None else dict(response),
        "motion_events": [dict(event) for event in motion_events],
        "execution_dispatched": False,
    }
    if target is None:
        payload["receipt_validation"] = {
            "passed": False,
            "errors": ["no stable ripe target was selected"],
            "status": None,
            "selected_candidate_id": None,
        }
        return payload
    validation = qualify_candidate_plan_only_receipt(
        payload,
        target_id=target.track_id,
    )
    payload["receipt_validation"] = {
        "passed": validation.passed,
        "errors": list(validation.errors),
        "status": validation.status,
        "selected_candidate_id": validation.selected_candidate_id,
    }
    return payload


def candidate_qualification_probe_exit_code(payload: Mapping[str, object]) -> int:
    """Return zero only for a complete plan-only certificate and evidence pass."""

    validation = payload.get("receipt_validation")
    passed = isinstance(validation, Mapping) and validation.get("passed") is True
    if payload.get("outcome") == "PLAN_ONLY_CERTIFIED" and passed:
        return 0
    if str(payload.get("outcome", "")).endswith("TIMEOUT"):
        return 3
    if payload.get("outcome") in {
        "NO_STABLE_RIPE_TARGET",
        "QUALIFICATION_REJECTED",
        "EVIDENCE_INVALID",
    }:
        return 1
    return 4


def main(argv=None) -> int:  # pragma: no cover - exercised in ROS integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--tracked-topic", default="/strawberry/tracked_targets")
    parser.add_argument(
        "--qualification-service", default="/strawberry/qualify_grasp_candidates"
    )
    parser.add_argument("--evidence-run-id", required=True)
    parser.add_argument("--target-id", type=int, default=0)
    parser.add_argument("--observation-window", type=float, default=15.0)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--qualification-timeout", type=float, default=45.0)
    parser.add_argument("--evidence-settle-time", type=float, default=0.5)
    parser.add_argument("--hard-timeout", type=float, default=180.0)
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
    timeouts = {
        "observation": options.observation_window,
        "startup": options.startup_timeout,
        "qualification": options.qualification_timeout,
        "evidence_settle": options.evidence_settle_time,
        "hard": options.hard_timeout,
    }
    if not all(math.isfinite(value) and value > 0.0 for value in timeouts.values()):
        raise ValueError("all probe timeouts must be finite and positive")
    if options.hard_timeout < options.startup_timeout:
        raise ValueError("hard timeout must cover startup timeout")

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from strawberry_interfaces.msg import TrackedTargetArray
        from strawberry_interfaces.srv import QualifyGraspCandidates
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    scene = load_scene_config(options.scene_config)
    base_frame = scene.base_frame
    place_position_m = load_scene_place_position(options.scene_config)

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("generalized_candidate_qualification_probe")
            self.started = time.monotonic()
            self.first_message_at: float | None = None
            self.selected: StableRipeTrack | None = None
            self.selected_item = None
            self.selected_priority = None
            self.motion_events: list[dict[str, object]] = []
            self.client = self.create_client(
                QualifyGraspCandidates, options.qualification_service
            )
            self.create_subscription(
                TrackedTargetArray, options.tracked_topic, self.on_tracks, 10
            )
            self.create_subscription(
                String, "/strawberry/motion_evidence", self.on_motion_evidence, 100
            )
            self.timer = self.create_timer(0.1, self.tick)
            self.called = False
            self.request_started: float | None = None
            self.response_received_at: float | None = None
            self.response: dict[str, object] | None = None
            self.done = False
            self.outcome = "RECORDER_STOPPED"
            self.payload: dict[str, object] | None = None

        def on_tracks(self, message) -> None:
            if self.first_message_at is None:
                self.first_message_at = time.monotonic()
            try:
                records = records_from_tracked_message(message)
                ranked = stable_ripe_tracks(
                    records,
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
                raise RuntimeError("cannot qualify without a stable ripe track")
            request = QualifyGraspCandidates.Request()
            request.target_id = self.selected.track_id
            request.target_pose = PoseStamped()
            request.target_pose.header.frame_id = base_frame
            request.target_pose.header.stamp = self.selected_item.header.stamp
            request.target_pose.pose.position.x = float(
                self.selected_item.pose.position.x
            )
            request.target_pose.pose.position.y = float(
                self.selected_item.pose.position.y
            )
            request.target_pose.pose.position.z = float(
                self.selected_item.pose.position.z
            )
            # A fruit centre has no relevant roll. The planner owns grasp
            # orientation; a canonical finite quaternion avoids depending on
            # a tracker implementation detail.
            request.target_pose.pose.orientation.w = 1.0
            request.place_pose = PoseStamped()
            request.place_pose.header.frame_id = base_frame
            request.place_pose.pose.position.x = place_position_m[0]
            request.place_pose.pose.position.y = place_position_m[1]
            request.place_pose.pose.position.z = place_position_m[2]
            request.place_pose.pose.orientation.w = 1.0
            self.called = True
            self.request_started = time.monotonic()
            future = self.client.call_async(request)
            future.add_done_callback(self.on_response)

        def on_response(self, future) -> None:
            if self.done:
                return
            try:
                response = future.result()
                if response is None:
                    raise RuntimeError("qualification service returned no response")
                self.response = response_to_mapping(response)
            except Exception as exc:
                self.response = {
                    "feasible": False,
                    "plan_only": True,
                    "status": "SERVICE_ERROR",
                    "message": str(exc),
                    "evaluations": [],
                }
            self.response_received_at = time.monotonic()

        def tick(self) -> None:
            if self.done:
                return
            now = time.monotonic()
            elapsed = now - self.started
            if elapsed >= options.hard_timeout:
                self.finish("HARD_TIMEOUT")
                return
            if self.first_message_at is None:
                if elapsed >= options.startup_timeout:
                    self.finish("STARTUP_TIMEOUT")
                return
            if not self.called:
                if now - self.first_message_at < options.observation_window:
                    return
                if self.selected is None:
                    self.finish("NO_STABLE_RIPE_TARGET")
                    return
                if not self.client.service_is_ready():
                    if elapsed >= options.startup_timeout:
                        self.finish("STARTUP_TIMEOUT")
                    return
                self._dispatch()
                return
            if self.response_received_at is None:
                if (
                    self.request_started is not None
                    and now - self.request_started >= options.qualification_timeout
                ):
                    self.finish("QUALIFICATION_TIMEOUT")
                return
            if now - self.response_received_at < options.evidence_settle_time:
                return
            if (
                self.response is not None
                and self.response.get("status") == "PLAN_ONLY_CERTIFIED"
                and self.response.get("feasible") is True
                and self.response.get("plan_only") is True
            ):
                self.finish("PLAN_ONLY_CERTIFIED")
            else:
                self.finish("QUALIFICATION_REJECTED")

        def finish(self, outcome: str) -> None:
            if self.done:
                return
            self.done = True
            self.outcome = outcome
            payload = build_candidate_qualification_payload(
                outcome=outcome,
                target=self.selected,
                response=self.response,
                motion_events=self.motion_events,
                elapsed_wall_sec=time.monotonic() - self.started,
                scene_config_file=options.scene_config,
                place_position_m=place_position_m,
                timeouts_sec=timeouts,
            )
            validation = payload["receipt_validation"]
            if (
                outcome == "PLAN_ONLY_CERTIFIED"
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
    return candidate_qualification_probe_exit_code(node.payload)


if __name__ == "__main__":
    raise SystemExit(main())
