"""Truth-free, zero-motion MoveIt feasibility probe for one development scene."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from .harvest_planning import rank_safe_targets, target_rejection_reasons
from .target_selector import (
    MoveItEvaluation,
    evaluation_failure_is_transient,
    expanded_bin_bounds,
    harvest_candidates_from_records,
)


def seconds(stamp) -> float:
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def records_from_tracked_message(message) -> tuple[dict[str, object], ...]:
    return tuple(
        {
            "track_id": int(item.track_id),
            "maturity": int(item.maturity),
            "position": [
                float(item.pose.position.x),
                float(item.pose.position.y),
                float(item.pose.position.z),
            ],
            "confidence": float(item.detection_confidence),
            "sigma_m": float(item.position_sigma_m),
            "observation_count": int(item.observation_count),
            "last_seen_sec": seconds(item.header.stamp),
        }
        for item in message.targets
        if int(item.track_id) > 0
    )


def build_feasibility_payload(
    *,
    outcome: str,
    records,
    candidates,
    evaluations,
    now_sec: float,
    selection_limits,
    elapsed_sec: float,
    minimum_feasible_ripe: int,
) -> dict[str, object]:
    evaluated = {int(row["track_id"]): dict(row) for row in evaluations}
    candidate_rows = []
    for candidate in candidates:
        reasons = list(
            target_rejection_reasons(
                candidate, now_sec=now_sec, **selection_limits
            )
        )
        candidate_rows.append(
            {
                "track_id": candidate.track_id,
                "maturity": candidate.maturity,
                "position": list(candidate.position),
                "confidence": candidate.confidence,
                "sigma_m": candidate.sigma_m,
                "observation_count": candidate.observation_count,
                "age_sec": now_sec - candidate.last_seen_sec,
                "clearance_m": candidate.clearance_m,
                "data_rejection_reasons": reasons,
                **(
                    {"moveit": evaluated[candidate.track_id]}
                    if candidate.track_id in evaluated
                    else {}
                ),
            }
        )
    feasible_ids = sorted(
        track_id for track_id, row in evaluated.items() if bool(row["feasible"])
    )
    completed = outcome == "COMPLETED"
    eligible = completed and len(feasible_ids) >= int(minimum_feasible_ripe)
    return {
        "schema_version": 1,
        "scope": "GENERALIZED_DEVELOPMENT_ZERO_MOTION_FEASIBILITY",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "runtime_truth_use": False,
        "trajectory_execution_allowed": False,
        "outcome": outcome,
        "elapsed_wall_sec": float(elapsed_sec),
        "minimum_feasible_ripe": int(minimum_feasible_ripe),
        "observed_track_records": [dict(record) for record in records],
        "candidates": candidate_rows,
        "evaluations": [dict(row) for row in evaluations],
        "feasible_ripe_track_ids": feasible_ids,
        "eligible_for_multi_fruit_runtime": eligible,
    }


def probe_exit_code(payload) -> int:
    if payload.get("eligible_for_multi_fruit_runtime") is True:
        return 0
    if payload.get("outcome") == "COMPLETED":
        return 1
    return 3


def main(argv=None) -> int:  # pragma: no cover - exercised in ROS integration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-config", type=Path, required=True)
    parser.add_argument("--tracked-topic", default="/strawberry/tracked_targets")
    parser.add_argument("--evaluation-service", default="/strawberry/evaluate_target")
    parser.add_argument("--observation-window", type=float, default=15.0)
    parser.add_argument("--startup-timeout", type=float, default=90.0)
    parser.add_argument("--evaluation-timeout", type=float, default=30.0)
    parser.add_argument("--hard-timeout", type=float, default=240.0)
    parser.add_argument("--minimum-feasible-ripe", type=int, default=2)
    parser.add_argument("--confidence-threshold", type=float, default=0.20524564385414124)
    parser.add_argument("--maximum-sigma", type=float, default=0.015)
    parser.add_argument("--minimum-observations", type=int, default=3)
    parser.add_argument("--maximum-age", type=float, default=0.50)
    parser.add_argument("--minimum-clearance", type=float, default=0.02)
    parser.add_argument("--fruit-radius", type=float, default=0.026)
    parser.add_argument("--static-obstacle-margin", type=float, default=0.05)
    options = parser.parse_args(argv)
    if options.output.exists():
        raise FileExistsError(options.output)
    finite_positive = (
        options.observation_window,
        options.startup_timeout,
        options.evaluation_timeout,
        options.hard_timeout,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in finite_positive):
        raise ValueError("probe timeouts must be finite and positive")
    if options.hard_timeout < options.startup_timeout:
        raise ValueError("hard timeout must cover startup timeout")
    if options.minimum_feasible_ripe < 2:
        raise ValueError("multi-fruit probe requires at least two feasible ripe targets")

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from strawberry_interfaces.msg import TrackedTargetArray
        from strawberry_interfaces.srv import EvaluateTarget
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    scene = load_scene_config(options.scene_config)
    interior = scene.bin_bounds
    bin_bounds = expanded_bin_bounds(
        (
            interior.min_x,
            interior.max_x,
            interior.min_y,
            interior.max_y,
            interior.min_z,
            interior.max_z,
        )
    )
    limits = {
        "confidence_threshold": options.confidence_threshold,
        "maximum_sigma_m": options.maximum_sigma,
        "minimum_observations": options.minimum_observations,
        "maximum_age_sec": options.maximum_age,
        "minimum_clearance_m": options.minimum_clearance,
        "excluded_track_ids": set(),
    }

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("generalized_feasibility_probe")
            self.started = time.monotonic()
            self.first_message_wall = None
            self.latest_message = None
            self.records = ()
            self.candidates = ()
            self.messages_by_id = {}
            self.ranked = ()
            self.evaluations = []
            self.index = 0
            self.current = None
            self.request_started = None
            self.retry_count = 0
            self.retry_after = 0.0
            self.done = False
            self.outcome = "RECORDER_STOPPED"
            self.payload = None
            self.client = self.create_client(EvaluateTarget, options.evaluation_service)
            self.create_subscription(
                TrackedTargetArray, options.tracked_topic, self.on_tracks, 10
            )
            self.timer = self.create_timer(0.1, self.tick)

        def on_tracks(self, message) -> None:
            self.latest_message = message
            if self.first_message_wall is None:
                self.first_message_wall = time.monotonic()

        def prepare(self) -> None:
            if self.latest_message is None:
                return
            self.records = records_from_tracked_message(self.latest_message)
            self.messages_by_id = {
                int(item.track_id): item for item in self.latest_message.targets
            }
            try:
                candidates = harvest_candidates_from_records(
                    self.records,
                    fruit_radius_m=options.fruit_radius,
                    bin_bounds=bin_bounds,
                    static_obstacle_margin_m=options.static_obstacle_margin,
                )
            except (KeyError, TypeError, ValueError):
                self.finish("INVALID_TRACK_SNAPSHOT")
                return
            now_sec = max(
                (candidate.last_seen_sec for candidate in candidates), default=0.0
            )
            self.candidates = tuple(candidates)
            self.ranked = tuple(
                rank_safe_targets(candidates, now_sec=now_sec, **limits)
            )
            if not self.ranked:
                self.finish("COMPLETED")

        def tick(self) -> None:
            if self.done:
                return
            now = time.monotonic()
            elapsed = now - self.started
            if elapsed >= options.hard_timeout:
                self.finish("HARD_TIMEOUT")
                return
            if self.first_message_wall is None:
                if elapsed >= options.startup_timeout:
                    self.finish("STARTUP_TIMEOUT")
                return
            if not self.candidates:
                if now - self.first_message_wall < options.observation_window:
                    return
                self.prepare()
                if self.done:
                    return
            if self.current is not None:
                if (
                    self.request_started is not None
                    and now - self.request_started >= options.evaluation_timeout
                ):
                    self.finish("EVALUATION_TIMEOUT")
                return
            if now < self.retry_after:
                return
            if self.index >= len(self.ranked):
                self.finish("COMPLETED")
                return
            if not self.client.service_is_ready():
                return
            self.current = self.ranked[self.index]
            item = self.messages_by_id[self.current.track_id]
            request = EvaluateTarget.Request()
            request.target_id = self.current.track_id
            request.target_pose = PoseStamped()
            request.target_pose.header = item.header
            request.target_pose.pose.position = item.pose.position
            request.target_pose.pose.orientation.w = 1.0
            self.request_started = now
            future = self.client.call_async(request)
            future.add_done_callback(self.on_evaluation)

        def on_evaluation(self, future) -> None:
            candidate = self.current
            if self.done or candidate is None:
                return
            try:
                response = future.result()
                if response is None:
                    raise RuntimeError("target evaluation service failed")
                evaluation = MoveItEvaluation(
                    position=candidate.position,
                    feasible=bool(response.feasible),
                    collision=bool(response.collision),
                    planning_time_sec=float(response.planning_time_sec),
                    joint_travel_rad=float(response.joint_travel_rad),
                    message=str(response.message),
                )
            except Exception as exc:
                evaluation = MoveItEvaluation(
                    position=candidate.position,
                    feasible=False,
                    collision=False,
                    planning_time_sec=0.0,
                    joint_travel_rad=0.0,
                    message=f"target evaluation service failed: {exc}",
                )
            if not evaluation.feasible and evaluation_failure_is_transient(
                evaluation.message
            ):
                self.retry_count += 1
                self.current = None
                self.request_started = None
                if self.retry_count > 3:
                    self.finish("EVALUATION_DEFERRED")
                else:
                    self.retry_after = time.monotonic() + 0.5
                return
            self.evaluations.append(
                {
                    "track_id": candidate.track_id,
                    "feasible": evaluation.feasible,
                    "collision": evaluation.collision,
                    "planning_time_sec": evaluation.planning_time_sec,
                    "joint_travel_rad": evaluation.joint_travel_rad,
                    "message": evaluation.message,
                }
            )
            self.index += 1
            self.current = None
            self.request_started = None
            self.retry_count = 0

        def finish(self, outcome: str) -> None:
            if self.done:
                return
            self.done = True
            self.outcome = outcome
            now_sec = max(
                (candidate.last_seen_sec for candidate in self.candidates),
                default=0.0,
            )
            self.payload = build_feasibility_payload(
                outcome=outcome,
                records=self.records,
                candidates=self.candidates,
                evaluations=self.evaluations,
                now_sec=now_sec,
                selection_limits=limits,
                elapsed_sec=time.monotonic() - self.started,
                minimum_feasible_ripe=options.minimum_feasible_ripe,
            )
            options.output.parent.mkdir(parents=True, exist_ok=True)
            temporary = options.output.with_suffix(options.output.suffix + ".tmp")
            temporary.write_text(
                json.dumps(self.payload, indent=2, sort_keys=True) + "\n",
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
    print(
        json.dumps(
            {
                "eligible": node.payload["eligible_for_multi_fruit_runtime"],
                "output": str(options.output),
                "outcome": node.outcome,
            },
            sort_keys=True,
        )
    )
    return probe_exit_code(node.payload)


if __name__ == "__main__":
    raise SystemExit(main())
