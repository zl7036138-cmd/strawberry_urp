"""ROS 2 generalized target selector without oracle or fixed ROI dependencies."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from typing import Mapping

from .harvest_planning import (
    HarvestCandidate,
    ViewAssessment,
    generate_dynamic_views,
    hand_pose_for_optical_view,
    rank_dynamic_views,
    rank_safe_targets,
    target_rejection_reasons,
)


REACH_BOUNDS = (0.30, 0.72, -0.34, 0.34, 0.48, 0.66)
OBSERVATION_REACH_BOUNDS = (0.12, 0.78, -0.55, 0.55, 0.30, 0.90)


@dataclass(frozen=True)
class MoveItEvaluation:
    """A reusable target-path assessment tied to one measured fruit pose."""

    position: tuple[float, float, float]
    feasible: bool
    collision: bool
    planning_time_sec: float
    joint_travel_rad: float
    message: str

    def __post_init__(self) -> None:
        if len(self.position) != 3 or not all(
            math.isfinite(float(value)) for value in self.position
        ):
            raise ValueError("evaluation position must be a finite 3-D point")
        if (
            not math.isfinite(float(self.planning_time_sec))
            or float(self.planning_time_sec) < 0.0
            or not math.isfinite(float(self.joint_travel_rad))
            or float(self.joint_travel_rad) < 0.0
        ):
            raise ValueError(
                "evaluation motion metrics must be finite and non-negative"
            )


@dataclass
class MoveItSelectionBatch:
    """Dependency-light coordinator for one sequential MoveIt preflight batch.

    ROS callbacks only transport requests and responses.  This object owns the
    ordering, epoch guard, pending identity, rejection audit, and terminal
    decision so the asynchronous safety behavior can be tested directly.
    """

    epoch: int
    now_sec: float
    candidates: tuple[HarvestCandidate, ...]
    ranked: tuple[HarvestCandidate, ...]
    messages: Mapping[int, object]
    selection_limits: Mapping[str, object]
    index: int = 0
    group: tuple[float, float, float] | None = None
    feasible: list[HarvestCandidate] = field(default_factory=list)
    evaluations: dict[int, MoveItEvaluation] = field(default_factory=dict)
    evaluated_track_ids: list[int] = field(default_factory=list)
    moveit_rejections: list[dict[str, object]] = field(default_factory=list)
    pending_track_id: int | None = None
    active: bool = True

    def __post_init__(self) -> None:
        if self.epoch <= 0:
            raise ValueError("selection batch epoch must be positive")
        if not math.isfinite(float(self.now_sec)):
            raise ValueError("selection batch time must be finite")
        self.candidates = tuple(self.candidates)
        self.ranked = tuple(self.ranked)
        if not self.ranked:
            raise ValueError("selection batch requires ranked candidates")
        self.group = moveit_priority_group(self.ranked[0])

    def accepts(self, epoch: int, track_id: int | None = None) -> bool:
        if not self.active or int(epoch) != self.epoch:
            return False
        return track_id is None or self.pending_track_id == int(track_id)

    def next_step(
        self, epoch: int
    ) -> tuple[str, HarvestCandidate | None]:
        """Return EVALUATE, SELECT, NO_PICK, WAIT, or STALE."""

        if not self.accepts(epoch):
            return "STALE", None
        if self.pending_track_id is not None:
            return "WAIT", None
        if self.feasible and (
            self.index >= len(self.ranked)
            or moveit_priority_group(self.ranked[self.index]) != self.group
        ):
            return "SELECT", None
        if self.index >= len(self.ranked):
            return "NO_PICK", None
        if (
            not self.feasible
            and moveit_priority_group(self.ranked[self.index]) != self.group
        ):
            self.group = moveit_priority_group(self.ranked[self.index])
        candidate = self.ranked[self.index]
        self.index += 1
        self.pending_track_id = candidate.track_id
        return "EVALUATE", candidate

    def record(
        self,
        epoch: int,
        candidate: HarvestCandidate,
        evaluation: MoveItEvaluation,
        *,
        cached: bool,
    ) -> bool:
        """Record only the currently pending result; reject stale callbacks."""

        if not self.accepts(epoch, candidate.track_id):
            return False
        self.pending_track_id = None
        self.evaluated_track_ids.append(candidate.track_id)
        self.evaluations[candidate.track_id] = evaluation
        assessed = apply_moveit_evaluation(candidate, evaluation)
        if evaluation.feasible:
            self.feasible.append(assessed)
        else:
            self.moveit_rejections.append(
                {
                    "track_id": candidate.track_id,
                    "collision": evaluation.collision,
                    "planning_time_sec": evaluation.planning_time_sec,
                    "joint_travel_rad": evaluation.joint_travel_rad,
                    "message": evaluation.message,
                    "cached": bool(cached),
                }
            )
        return True

    def selected(self) -> HarvestCandidate | None:
        ranked = rank_safe_targets(
            self.feasible,
            now_sec=self.now_sec,
            **self.selection_limits,
        )
        return ranked[0] if ranked else None

    def invalidate(self) -> None:
        self.active = False
        self.pending_track_id = None


def apply_moveit_evaluation(
    candidate: HarvestCandidate,
    evaluation: MoveItEvaluation,
) -> HarvestCandidate:
    """Replace geometric reachability with the authoritative MoveIt result."""

    return replace(
        candidate,
        joint_travel_rad=float(evaluation.joint_travel_rad),
        pregrasp_feasible=bool(evaluation.feasible),
        grasp_feasible=bool(evaluation.feasible),
        retreat_feasible=bool(evaluation.feasible),
    )


def moveit_priority_group(candidate: HarvestCandidate) -> tuple[float, float, float]:
    """The ranking prefix that is independent of MoveIt's joint-travel result."""

    return (
        -candidate.clearance_m,
        candidate.sigma_m,
        -candidate.confidence,
    )


def evaluation_matches_position(
    evaluation: MoveItEvaluation,
    position,
    *,
    maximum_drift_m: float,
) -> bool:
    """Only reuse a path proof while the tracked fruit stayed at that pose."""

    limit = float(maximum_drift_m)
    return (
        math.isfinite(limit)
        and limit >= 0.0
        and len(position) == 3
        and all(math.isfinite(float(value)) for value in position)
        and math.dist(evaluation.position, position) <= limit
    )


def evaluation_failure_is_transient(message: str) -> bool:
    """Distinguish backend availability failures from deterministic path failures."""

    detail = str(message).strip().lower()
    return any(
        marker in detail
        for marker in (
            "motion backend is busy",
            "service failed",
            "service is unavailable",
            "target evaluation request is invalid",
            "failed to synchronize perception collision scene",
            "failed to open target contact corridor for evaluation",
            "failed to restore the target collision object",
            "target feasibility evaluation failed",
        )
    )


def register_completed_track(excluded: set[int], target_id: int) -> bool:
    """Record a completed identity without replaying a pre-completion cache."""

    identity = int(target_id)
    if identity <= 0 or identity in excluded:
        return False
    excluded.add(identity)
    return True


def geometric_clearance(position, neighbours, fruit_radius_m: float) -> float:
    distances = [
        math.dist(position, neighbour) - 2.0 * fruit_radius_m
        for neighbour in neighbours
    ]
    return max(0.0, min(distances)) if distances else 1.0


def axis_aligned_box_clearance(position, bounds) -> float:
    """Euclidean point clearance from an axis-aligned forbidden volume."""

    if len(position) != 3 or len(bounds) != 6:
        raise ValueError("position and bounds must contain three and six values")
    min_x, max_x, min_y, max_y, min_z, max_z = (float(value) for value in bounds)
    if min_x >= max_x or min_y >= max_y or min_z >= max_z:
        raise ValueError("axis-aligned bounds are invalid")
    x, y, z = (float(value) for value in position)
    deltas = (
        max(min_x - x, 0.0, x - max_x),
        max(min_y - y, 0.0, y - max_y),
        max(min_z - z, 0.0, z - max_z),
    )
    return math.sqrt(sum(value * value for value in deltas))


def expanded_bin_bounds(interior_bounds) -> tuple[float, ...]:
    """Convert the manifest's interior volume into a conservative outer box."""

    if len(interior_bounds) != 6:
        raise ValueError("bin interior bounds must contain six values")
    min_x, max_x, min_y, max_y, min_z, max_z = (
        float(value) for value in interior_bounds
    )
    return (
        min_x - 0.05,
        max_x + 0.05,
        min_y - 0.05,
        max_y + 0.05,
        min_z - 0.03,
        max_z,
    )


def inside_conservative_reach(position) -> bool:
    min_x, max_x, min_y, max_y, min_z, max_z = REACH_BOUNDS
    x, y, z = position
    return min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z


def inside_observation_reach(position) -> bool:
    """Reject only wrist views that are certainly outside Panda's workspace.

    Observation poses naturally sit between the arm base and the fruit, so the
    narrower fruit-center bounds must not be reused here. MoveIt remains the
    authority for IK, joint-limit, and collision feasibility.
    """

    min_x, max_x, min_y, max_y, min_z, max_z = OBSERVATION_REACH_BOUNDS
    x, y, z = position
    return min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z


def harvest_candidates_from_records(
    records,
    *,
    fruit_radius_m: float,
    bin_bounds,
    static_obstacle_margin_m: float,
) -> list[HarvestCandidate]:
    """Build selector-identical candidates from dependency-light track records."""

    radius = float(fruit_radius_m)
    margin = float(static_obstacle_margin_m)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError("fruit radius must be finite and positive")
    if not math.isfinite(margin) or margin < 0.0:
        raise ValueError("static obstacle margin must be finite and non-negative")
    normalized = tuple(dict(record) for record in records)
    identities = [int(record["track_id"]) for record in normalized]
    if any(identity <= 0 for identity in identities) or len(identities) != len(
        set(identities)
    ):
        raise ValueError("track IDs must be positive and unique")
    positions = {
        int(record["track_id"]): tuple(float(value) for value in record["position"])
        for record in normalized
    }
    if any(
        len(position) != 3 or not all(math.isfinite(value) for value in position)
        for position in positions.values()
    ):
        raise ValueError("track positions must be finite 3-D points")
    candidates = []
    for record in normalized:
        track_id = int(record["track_id"])
        position = positions[track_id]
        neighbours = [
            value for other_id, value in positions.items() if other_id != track_id
        ]
        reachable = inside_conservative_reach(position)
        clearance = min(
            geometric_clearance(position, neighbours, radius),
            max(
                0.0,
                axis_aligned_box_clearance(position, bin_bounds) - margin,
            ),
        )
        candidates.append(
            HarvestCandidate(
                track_id=track_id,
                maturity=int(record["maturity"]),
                position=position,
                confidence=float(record["confidence"]),
                sigma_m=float(record["sigma_m"]),
                observation_count=int(record["observation_count"]),
                last_seen_sec=float(record["last_seen_sec"]),
                clearance_m=clearance,
                joint_travel_rad=math.dist((0.0, 0.0, 0.55), position),
                pregrasp_feasible=reachable,
                grasp_feasible=reachable,
                retreat_feasible=reachable,
            )
        )
    return candidates


def main(args=None) -> None:  # pragma: no cover - exercised in ROS integration
    try:
        import rclpy
        from geometry_msgs.msg import Pose, PoseStamped
        from rclpy.node import Node
        from std_msgs.msg import String, UInt32
        from strawberry_interfaces.msg import (
            ObservationPlan,
            TargetPose,
            TrackedTargetArray,
        )
        from strawberry_interfaces.srv import EvaluateTarget
        from strawberry_sim.core import load_scene_config
    except ImportError as exc:
        raise RuntimeError("ROS 2 runtime dependencies are not installed") from exc

    class TargetSelector(Node):
        def __init__(self) -> None:
            super().__init__("strawberry_target_selector")
            self.declare_parameter(
                "tracked_targets_topic", "/strawberry/tracked_targets"
            )
            self.declare_parameter("selected_target_topic", "/strawberry/target_pose")
            self.declare_parameter(
                "observation_pose_topic", "/strawberry/wrist_observation_pose"
            )
            self.declare_parameter(
                "observation_plan_topic", "/strawberry/wrist_observation_plan"
            )
            self.declare_parameter("status_topic", "/strawberry/selection_status")
            self.declare_parameter("confidence_threshold", 0.60)
            self.declare_parameter("maximum_sigma_m", 0.015)
            self.declare_parameter("minimum_observations", 3)
            self.declare_parameter("maximum_age_sec", 0.50)
            self.declare_parameter("minimum_clearance_m", 0.02)
            self.declare_parameter("fruit_radius_m", 0.026)
            self.declare_parameter("static_obstacle_margin_m", 0.05)
            self.declare_parameter(
                "target_evaluation_service", "/strawberry/evaluate_target"
            )
            self.declare_parameter("maximum_evaluation_pose_drift_m", 0.005)
            self.declare_parameter("scene_config_file", "")
            scene_path = str(self.get_parameter("scene_config_file").value)
            if not scene_path:
                raise RuntimeError(
                    "scene_config_file is required for static-obstacle clearance"
                )
            scene = load_scene_config(scene_path)
            bounds = scene.bin_bounds
            self._bin_bounds = expanded_bin_bounds(
                (
                    bounds.min_x,
                    bounds.max_x,
                    bounds.min_y,
                    bounds.max_y,
                    bounds.min_z,
                    bounds.max_z,
                )
            )
            self._excluded = set()
            self._latest_targets_message = None
            self._evaluation_epoch = 0
            self._evaluation_active = False
            self._evaluation_batch = None
            self._evaluation_cache = {}
            self._selected_track_id = None
            self._evaluation_client = self.create_client(
                EvaluateTarget,
                str(self.get_parameter("target_evaluation_service").value),
            )
            self._target_publisher = self.create_publisher(
                TargetPose, str(self.get_parameter("selected_target_topic").value), 10
            )
            self._view_publisher = self.create_publisher(
                PoseStamped, str(self.get_parameter("observation_pose_topic").value), 10
            )
            self._view_plan_publisher = self.create_publisher(
                ObservationPlan,
                str(self.get_parameter("observation_plan_topic").value),
                10,
            )
            self._status_publisher = self.create_publisher(
                String, str(self.get_parameter("status_topic").value), 10
            )
            self.create_subscription(
                TrackedTargetArray,
                str(self.get_parameter("tracked_targets_topic").value),
                self._on_targets,
                10,
            )
            self.create_subscription(
                UInt32,
                "/strawberry/completed_track_id",
                self._on_completed_track,
                10,
            )

        def _on_completed_track(self, message) -> None:
            # Selection resumes only when the localizer publishes a new
            # tracking snapshot. Replaying the pre-completion cache can race
            # the batch state transition and can never prove a fresh rescan.
            target_id = int(message.data)
            if not register_completed_track(self._excluded, target_id):
                return
            # Feasibility includes a connected route from the current robot
            # state. A completed or skipped attempt changes that state, so no
            # assessment for any remaining fruit may survive the tombstone.
            self._evaluation_cache.clear()
            if self._selected_track_id == target_id:
                self._selected_track_id = None
            # Any in-flight response was computed before the completion
            # tombstone and must never publish after it.
            self._evaluation_epoch += 1
            self._evaluation_active = False
            self._evaluation_batch = None

        @staticmethod
        def _seconds(stamp) -> float:
            return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

        def _on_targets(self, message) -> None:
            self._latest_targets_message = message
            if self._evaluation_active:
                return
            self._select_targets(message)

        def _select_targets(self, message) -> None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            candidates, messages_by_track_id = self._build_candidates(message)
            selection_limits = self._selection_limits()
            ranked = rank_safe_targets(
                candidates,
                now_sec=now_sec,
                **selection_limits,
            )
            if not ranked:
                self._publish_no_pick(
                    candidates,
                    now_sec=now_sec,
                    selection_limits=selection_limits,
                )
                return

            # Keep a preflighted identity selected until the orchestrator
            # publishes its completion tombstone. Fresh poses are still used
            # for every publication, but no MoveIt request competes with arm
            # motion after hand-off.
            if self._selected_track_id is not None:
                selected = next(
                    (
                        candidate
                        for candidate in ranked
                        if candidate.track_id == self._selected_track_id
                    ),
                    None,
                )
                evaluation = self._evaluation_cache.get(self._selected_track_id)
                maximum_drift_m = float(
                    self.get_parameter("maximum_evaluation_pose_drift_m").value
                )
                if (
                    selected is not None
                    and evaluation is not None
                    and evaluation.feasible
                    and evaluation_matches_position(
                        evaluation,
                        selected.position,
                        maximum_drift_m=maximum_drift_m,
                    )
                ):
                    self._publish_selected_target(
                        apply_moveit_evaluation(selected, evaluation),
                        messages_by_track_id[selected.track_id],
                        evaluation=evaluation,
                        evaluated_track_ids=(selected.track_id,),
                        moveit_rejections=(),
                        cached=True,
                    )
                    return
                self._selected_track_id = None

            self._evaluation_epoch += 1
            epoch = self._evaluation_epoch
            self._evaluation_active = True
            self._evaluation_batch = MoveItSelectionBatch(
                epoch=epoch,
                now_sec=now_sec,
                candidates=tuple(candidates),
                ranked=tuple(ranked),
                messages=messages_by_track_id,
                selection_limits=selection_limits,
            )
            self._advance_evaluation(epoch)

        def _build_candidates(self, message):
            messages_by_track_id = {
                int(item.track_id): item for item in message.targets
            }
            records = (
                {
                    "track_id": int(item.track_id),
                    "maturity": int(item.maturity),
                    "position": (
                        float(item.pose.position.x),
                        float(item.pose.position.y),
                        float(item.pose.position.z),
                    ),
                    "confidence": float(item.detection_confidence),
                    "sigma_m": float(item.position_sigma_m),
                    "observation_count": int(item.observation_count),
                    "last_seen_sec": self._seconds(item.header.stamp),
                }
                for item in message.targets
            )
            candidates = harvest_candidates_from_records(
                records,
                fruit_radius_m=float(self.get_parameter("fruit_radius_m").value),
                bin_bounds=self._bin_bounds,
                static_obstacle_margin_m=float(
                    self.get_parameter("static_obstacle_margin_m").value
                ),
            )
            return candidates, messages_by_track_id

        def _selection_limits(self):
            return {
                "confidence_threshold": float(
                    self.get_parameter("confidence_threshold").value
                ),
                "maximum_sigma_m": float(self.get_parameter("maximum_sigma_m").value),
                "minimum_observations": int(
                    self.get_parameter("minimum_observations").value
                ),
                "maximum_age_sec": float(self.get_parameter("maximum_age_sec").value),
                "minimum_clearance_m": float(
                    self.get_parameter("minimum_clearance_m").value
                ),
                "excluded_track_ids": self._excluded,
            }

        def _publish_no_pick(
            self,
            candidates,
            *,
            now_sec,
            selection_limits,
            reason=None,
            moveit_rejections=(),
            evaluated_track_ids=(),
        ) -> None:
            moveit_by_id = {
                int(entry["track_id"]): entry for entry in moveit_rejections
            }
            payload = {
                "schema_version": 1,
                "outcome": "NO_PICK",
                "candidate_count": len(candidates),
                "moveit_evaluated_track_ids": list(evaluated_track_ids),
                "rejections": [
                    {
                        "track_id": candidate.track_id,
                        "reasons": list(
                            target_rejection_reasons(
                                candidate,
                                now_sec=now_sec,
                                **selection_limits,
                            )
                        )
                        + (
                            ["MOVEIT_PATH_INFEASIBLE"]
                            if candidate.track_id in moveit_by_id
                            else []
                        ),
                        "confidence": candidate.confidence,
                        "sigma_m": candidate.sigma_m,
                        "observation_count": candidate.observation_count,
                        "age_sec": now_sec - candidate.last_seen_sec,
                        "clearance_m": candidate.clearance_m,
                        **(
                            {"moveit": moveit_by_id[candidate.track_id]}
                            if candidate.track_id in moveit_by_id
                            else {}
                        ),
                    }
                    for candidate in candidates
                ],
            }
            if reason:
                payload["reason"] = str(reason)
            status = String()
            status.data = json.dumps(payload, separators=(",", ":"))
            self._status_publisher.publish(status)

        def _advance_evaluation(self, epoch: int) -> None:
            if (
                not self._evaluation_active
                or self._evaluation_batch is None
                or not self._evaluation_batch.accepts(epoch)
            ):
                return
            batch = self._evaluation_batch
            step, candidate = batch.next_step(epoch)
            if step == "SELECT":
                self._finish_evaluation_with_selection(epoch)
                return
            if step == "NO_PICK":
                self._finish_evaluation_without_selection(epoch)
                return
            if step != "EVALUATE" or candidate is None:
                return
            maximum_drift_m = float(
                self.get_parameter("maximum_evaluation_pose_drift_m").value
            )
            cached = self._evaluation_cache.get(candidate.track_id)
            if cached is not None and evaluation_matches_position(
                cached,
                candidate.position,
                maximum_drift_m=maximum_drift_m,
            ):
                self._record_evaluation(epoch, candidate, cached, cached=True)
                return
            if not self._evaluation_client.service_is_ready():
                self._defer_evaluation(
                    epoch,
                    "target feasibility evaluation service is unavailable",
                )
                return

            item = batch.messages[candidate.track_id]
            request = EvaluateTarget.Request()
            request.target_id = candidate.track_id
            request.target_pose = PoseStamped()
            request.target_pose.header = item.header
            request.target_pose.pose.position = item.pose.position
            request.target_pose.pose.orientation.w = 1.0
            future = self._evaluation_client.call_async(request)
            future.add_done_callback(
                lambda completed, token=epoch, row=candidate: (
                    self._on_evaluation_result(token, row, completed)
                )
            )

        def _on_evaluation_result(self, epoch, candidate, future) -> None:
            if (
                not self._evaluation_active
                or self._evaluation_batch is None
                or not self._evaluation_batch.accepts(epoch, candidate.track_id)
            ):
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
                self._defer_evaluation(epoch, str(exc))
                return
            if not evaluation.feasible and evaluation_failure_is_transient(
                evaluation.message
            ):
                self._defer_evaluation(epoch, evaluation.message)
                return
            self._evaluation_cache[candidate.track_id] = evaluation
            self._record_evaluation(epoch, candidate, evaluation, cached=False)

        def _record_evaluation(
            self,
            epoch,
            candidate,
            evaluation,
            *,
            cached,
        ) -> None:
            if (
                not self._evaluation_active
                or self._evaluation_batch is None
            ):
                return
            batch = self._evaluation_batch
            if not batch.record(
                epoch, candidate, evaluation, cached=bool(cached)
            ):
                return
            self._advance_evaluation(epoch)

        def _finish_evaluation_with_selection(self, epoch) -> None:
            batch = self._evaluation_batch
            if (
                not self._evaluation_active
                or batch is None
                or not batch.accepts(epoch)
            ):
                return
            selected = batch.selected()
            if selected is None:
                self._finish_evaluation_without_selection(epoch)
                return
            evaluation = batch.evaluations[selected.track_id]

            latest = self._latest_targets_message
            latest_now = self.get_clock().now().nanoseconds * 1e-9
            latest_candidates, latest_messages = self._build_candidates(latest)
            latest_by_id = {
                candidate.track_id: candidate for candidate in latest_candidates
            }
            latest_selected = latest_by_id.get(selected.track_id)
            latest_limits = self._selection_limits()
            maximum_drift_m = float(
                self.get_parameter("maximum_evaluation_pose_drift_m").value
            )
            latest_reasons = (
                ("TARGET_DISAPPEARED",)
                if latest_selected is None
                else target_rejection_reasons(
                    latest_selected,
                    now_sec=latest_now,
                    **latest_limits,
                )
            )
            if (
                latest_selected is None
                or latest_reasons
                or not evaluation_matches_position(
                    evaluation,
                    latest_selected.position,
                    maximum_drift_m=maximum_drift_m,
                )
            ):
                self._evaluation_cache.pop(selected.track_id, None)
                self._evaluation_epoch += 1
                self._evaluation_active = False
                self._evaluation_batch = None
                self._selected_track_id = None
                self._select_targets(latest)
                return

            evaluated_track_ids = tuple(batch.evaluated_track_ids)
            moveit_rejections = tuple(batch.moveit_rejections)
            batch.invalidate()
            self._evaluation_active = False
            self._evaluation_batch = None
            self._selected_track_id = selected.track_id
            self._publish_selected_target(
                apply_moveit_evaluation(latest_selected, evaluation),
                latest_messages[selected.track_id],
                evaluation=evaluation,
                evaluated_track_ids=evaluated_track_ids,
                moveit_rejections=moveit_rejections,
                cached=False,
            )

        def _finish_evaluation_without_selection(self, epoch) -> None:
            batch = self._evaluation_batch
            if (
                not self._evaluation_active
                or batch is None
                or not batch.accepts(epoch)
            ):
                return
            batch.invalidate()
            self._evaluation_active = False
            self._evaluation_batch = None
            self._selected_track_id = None
            self._publish_no_pick(
                batch.candidates,
                now_sec=batch.now_sec,
                selection_limits=batch.selection_limits,
                reason="NO_MOVEIT_FEASIBLE_TARGET",
                moveit_rejections=batch.moveit_rejections,
                evaluated_track_ids=batch.evaluated_track_ids,
            )

        def _defer_evaluation(self, epoch, detail) -> None:
            batch = self._evaluation_batch
            if (
                not self._evaluation_active
                or batch is None
                or not batch.accepts(epoch)
            ):
                return
            batch.invalidate()
            self._evaluation_epoch += 1
            self._evaluation_active = False
            self._evaluation_batch = None
            self._publish_no_pick(
                batch.candidates,
                now_sec=batch.now_sec,
                selection_limits=batch.selection_limits,
                reason="MOVEIT_EVALUATION_DEFERRED",
                moveit_rejections=batch.moveit_rejections,
                evaluated_track_ids=batch.evaluated_track_ids,
            )
            self.get_logger().debug(
                f"Target selection deferred until MoveIt is idle: {detail}"
            )

        def _publish_selected_target(
            self,
            selected,
            selected_message,
            *,
            evaluation,
            evaluated_track_ids,
            moveit_rejections,
            cached,
        ) -> None:
            status = String()

            def assess_view(view):
                reachable = inside_observation_reach(view.position)
                return ViewAssessment(
                    ik_reachable=reachable,
                    collision_free=reachable,
                    target_in_view=True,
                    clearance_m=selected.clearance_m,
                    joint_travel_rad=math.dist((0.0, 0.0, 0.55), view.position),
                )

            ranked_views = rank_dynamic_views(
                generate_dynamic_views(selected.track_id, selected.position),
                assess_view,
            )
            if not ranked_views:
                status.data = json.dumps(
                    {
                        "schema_version": 1,
                        "outcome": "NO_PICK",
                        "reason": "NO_SAFE_WRIST_VIEW",
                        "track_id": selected.track_id,
                    }
                )
                self._status_publisher.publish(status)
                return
            _, assessment = ranked_views[0]
            hand_views = tuple(
                hand_pose_for_optical_view(candidate_view)
                for candidate_view, _ in ranked_views
            )
            hand_view = hand_views[0]
            target = TargetPose()
            target.header = selected_message.header
            target.target_id = selected.track_id
            target.pose.position.x, target.pose.position.y, target.pose.position.z = (
                selected.position
            )
            target.pose.orientation.w = 1.0
            target.detection_confidence = selected.confidence
            target.position_sigma_m = selected.sigma_m
            view_message = PoseStamped()
            view_message.header = selected_message.header
            (
                view_message.pose.position.x,
                view_message.pose.position.y,
                view_message.pose.position.z,
            ) = hand_view.position
            (
                view_message.pose.orientation.x,
                view_message.pose.orientation.y,
                view_message.pose.orientation.z,
                view_message.pose.orientation.w,
            ) = hand_view.quaternion_xyzw
            view_plan = ObservationPlan()
            view_plan.header = selected_message.header
            view_plan.target_id = selected.track_id
            for candidate in hand_views:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = candidate.position
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ) = candidate.quaternion_xyzw
                view_plan.hand_poses.append(pose)
            self._view_plan_publisher.publish(view_plan)
            self._view_publisher.publish(view_message)
            self._target_publisher.publish(target)
            status.data = json.dumps(
                {
                    "schema_version": 1,
                    "outcome": "TARGET_SELECTED",
                    "track_id": selected.track_id,
                    "maturity": selected.maturity,
                    "position_m": list(selected.position),
                    "confidence": selected.confidence,
                    "sigma_m": selected.sigma_m,
                    "clearance_m": selected.clearance_m,
                    "view_joint_travel_rad": assessment.joint_travel_rad,
                    "observation_candidate_count": len(hand_views),
                    "moveit_preflight": {
                        "feasible": True,
                        "planning_time_sec": evaluation.planning_time_sec,
                        "joint_travel_rad": evaluation.joint_travel_rad,
                        "message": evaluation.message,
                        "cached": bool(cached),
                    },
                    "moveit_evaluated_track_ids": list(evaluated_track_ids),
                    "moveit_rejections": list(moveit_rejections),
                },
                separators=(",", ":"),
            )
            self._status_publisher.publish(status)

    rclpy.init(args=args)
    node = TargetSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
