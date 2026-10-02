"""Plan-only runtime qualification for ADR 0087-D.

This module joins the deterministic candidate set, ADR 0086 evaluation, and
ADR 0087-C certificate contract without importing ROS or commanding a robot.
The caller owns the live PlanningScene lifecycle; this coordinator only reads
its signature before and after candidate evaluation and fails closed if it
changes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Callable, Mapping

from .grasp_authorization import AuthorizedGraspPlan, authorize_grasp_candidate
from .grasp_candidate_search import (
    CandidateEvaluation,
    CandidateEvaluator,
    CandidateSearchResult,
    CandidateSearchStatus,
    FirstFeasibleCandidateSearch,
)
from .grasp_candidates import GraspCandidate


class CandidateQualificationStatus(str, Enum):
    """Stable outcomes for the no-motion runtime qualification boundary."""

    PLAN_ONLY_CERTIFIED = "PLAN_ONLY_CERTIFIED"
    NO_FEASIBLE_CANDIDATE = "NO_FEASIBLE_CANDIDATE"
    SEARCH_TIME_BUDGET_EXCEEDED = "SEARCH_TIME_BUDGET_EXCEEDED"
    CANDIDATE_SET_INVALID = "CANDIDATE_SET_INVALID"
    SCENE_CHANGED = "SCENE_CHANGED"
    SCENE_UNAVAILABLE = "SCENE_UNAVAILABLE"
    PAYLOAD_NOT_EMPTY = "PAYLOAD_NOT_EMPTY"
    EVALUATION_ERROR = "EVALUATION_ERROR"


EventSink = Callable[[str, Mapping[str, object]], None]


@dataclass(frozen=True)
class CandidateQualificationResult:
    """Auditable outcome of one plan-only candidate search.

    ``certificate`` is a proof that was valid while the caller-held planning
    scene signature remained unchanged. A plan-only caller must not cache or
    execute it after restoring that scene; ADR 0087-E will re-qualify within
    the same live lifecycle immediately before authorized execution.
    """

    status: CandidateQualificationStatus
    search: CandidateSearchResult | None
    scene_signature_before: str | None
    scene_signature_after: str | None
    certificate: AuthorizedGraspPlan | None
    detail: str = ""

    def __post_init__(self) -> None:
        if self.status is CandidateQualificationStatus.PLAN_ONLY_CERTIFIED:
            if self.search is None or self.certificate is None:
                raise ValueError("certified qualification requires search and certificate")
            if not self.scene_isolated:
                raise ValueError("certified qualification requires an unchanged scene")
            if self.search.status is not CandidateSearchStatus.FOUND:
                raise ValueError("certified qualification requires a found candidate")
        elif self.certificate is not None:
            raise ValueError("only a certified qualification may carry a certificate")

    @property
    def feasible(self) -> bool:
        return self.status is CandidateQualificationStatus.PLAN_ONLY_CERTIFIED

    @property
    def scene_isolated(self) -> bool:
        return (
            isinstance(self.scene_signature_before, str)
            and bool(self.scene_signature_before)
            and self.scene_signature_before == self.scene_signature_after
        )

    @property
    def evaluations(self) -> tuple[CandidateEvaluation, ...]:
        return () if self.search is None else self.search.evaluations

    @property
    def planning_time_sec(self) -> float:
        return sum(row.result.planning_time_sec for row in self.evaluations)

    @property
    def joint_travel_rad(self) -> float:
        return sum(row.result.joint_travel_rad for row in self.evaluations)


@dataclass
class PlanOnlyCandidateQualifier:
    """Evaluate candidates in a read-only scene and issue a temporary proof."""

    evaluator: CandidateEvaluator
    scene_signature_provider: Callable[[], str]
    per_candidate_budget_sec: float = 2.0
    total_budget_sec: float = 12.0
    clock: Callable[[], float] = field(default=time.perf_counter, repr=False)
    certificate_clock_ns: Callable[[], int] = field(
        default=time.monotonic_ns, repr=False
    )
    event_sink: EventSink | None = field(default=None, repr=False)
    qualification_mode: str = "PLAN_ONLY"

    def __post_init__(self) -> None:
        for value, label in (
            (self.per_candidate_budget_sec, "per-candidate budget"),
            (self.total_budget_sec, "total budget"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} must be finite and positive")
        if not isinstance(self.qualification_mode, str) or not self.qualification_mode:
            raise ValueError("qualification mode must be a non-empty string")

    def _emit(self, event_type: str, payload: Mapping[str, object]) -> None:
        if self.event_sink is not None:
            self.event_sink(event_type, dict(payload))

    def _scene_signature(self) -> str:
        signature = self.scene_signature_provider()
        if not isinstance(signature, str) or not signature:
            raise ValueError("planning-scene signature must be a non-empty string")
        return signature

    @staticmethod
    def _status_from_search(
        status: CandidateSearchStatus,
    ) -> CandidateQualificationStatus:
        return {
            CandidateSearchStatus.NO_FEASIBLE_CANDIDATE:
                CandidateQualificationStatus.NO_FEASIBLE_CANDIDATE,
            CandidateSearchStatus.SEARCH_TIME_BUDGET_EXCEEDED:
                CandidateQualificationStatus.SEARCH_TIME_BUDGET_EXCEEDED,
            CandidateSearchStatus.CANDIDATE_SET_INVALID:
                CandidateQualificationStatus.CANDIDATE_SET_INVALID,
        }.get(status, CandidateQualificationStatus.EVALUATION_ERROR)

    def qualify(
        self,
        *,
        target_id: int,
        candidates: tuple[GraspCandidate, ...],
        payload_state: str = "EMPTY",
    ) -> CandidateQualificationResult:
        """Run a bounded no-motion search and create a temporary certificate."""

        if payload_state != "EMPTY":
            return CandidateQualificationResult(
                CandidateQualificationStatus.PAYLOAD_NOT_EMPTY,
                None,
                None,
                None,
                None,
                f"plan-only qualification requires EMPTY payload, got {payload_state}",
            )
        try:
            before = self._scene_signature()
        except Exception as exc:
            return CandidateQualificationResult(
                CandidateQualificationStatus.SCENE_UNAVAILABLE,
                None,
                None,
                None,
                None,
                f"cannot snapshot scene before candidate qualification: {exc}",
            )

        start_payload = {
            "target_id": target_id,
            # The coordinator itself is always non-motion.  A runtime caller
            # may nevertheless label this qualification as a pre-execution
            # boundary so the evidence cannot be misread as a plan-only run.
            "mode": self.qualification_mode,
            "live_payload_state": payload_state,
            "candidate_count": len(candidates),
            "candidate_ids": [candidate.candidate_id for candidate in candidates],
            "candidate_geometry_fingerprints": [
                candidate.geometry_fingerprint for candidate in candidates
            ],
            "scene_signature_before": before,
            "controller_commands_during_evaluation": 0,
            "gripper_commands_during_evaluation": 0,
            "physical_attach_during_evaluation": 0,
        }
        try:
            self._emit("CANDIDATE_QUALIFICATION_STARTED", start_payload)
            qualifier = FirstFeasibleCandidateSearch(
                evaluator=_EvidenceRecordingEvaluator(
                    self.evaluator,
                    target_id=target_id,
                    event_sink=self._emit,
                ),
                per_candidate_budget_sec=self.per_candidate_budget_sec,
                total_budget_sec=self.total_budget_sec,
                clock=self.clock,
            )
            search = qualifier.search(candidates)
        except Exception as exc:
            return self._after_failure(
                CandidateQualificationStatus.EVALUATION_ERROR,
                None,
                before,
                f"candidate qualification raised: {exc}",
                target_id=target_id,
                payload_state=payload_state,
            )

        try:
            after = self._scene_signature()
        except Exception as exc:
            return self._emit_result(
                CandidateQualificationResult(
                    CandidateQualificationStatus.SCENE_UNAVAILABLE,
                    search,
                    before,
                    None,
                    None,
                    f"cannot snapshot scene after candidate qualification: {exc}",
                ),
                target_id=target_id,
                payload_state=payload_state,
            )
        if before != after:
            return self._emit_result(
                CandidateQualificationResult(
                    CandidateQualificationStatus.SCENE_CHANGED,
                    search,
                    before,
                    after,
                    None,
                    "live planning-scene signature changed during qualification",
                ),
                target_id=target_id,
                payload_state=payload_state,
            )
        if search.status is not CandidateSearchStatus.FOUND:
            return self._emit_result(
                CandidateQualificationResult(
                    self._status_from_search(search.status),
                    search,
                    before,
                    after,
                    None,
                    search.detail,
                ),
                target_id=target_id,
                payload_state=payload_state,
            )

        selected = search.selected_candidate
        selected_evaluation = search.evaluations[-1] if search.evaluations else None
        if (
            selected is None
            or selected_evaluation is None
            or selected_evaluation.candidate_id != selected.candidate_id
            or selected_evaluation.geometry_fingerprint
            != selected.geometry_fingerprint
            or not selected_evaluation.result.feasible
        ):
            return self._emit_result(
                CandidateQualificationResult(
                    CandidateQualificationStatus.EVALUATION_ERROR,
                    search,
                    before,
                    after,
                    None,
                    "first-feasible result is internally inconsistent",
                ),
                target_id=target_id,
                payload_state=payload_state,
            )
        try:
            certificate = authorize_grasp_candidate(
                selected,
                selected_evaluation.result,
                target_id=target_id,
                scene_signature=before,
                certificate_timestamp_ns=int(self.certificate_clock_ns()),
            )
        except Exception as exc:
            return self._emit_result(
                CandidateQualificationResult(
                    CandidateQualificationStatus.EVALUATION_ERROR,
                    search,
                    before,
                    after,
                    None,
                    f"cannot create candidate certificate: {exc}",
                ),
                target_id=target_id,
                payload_state=payload_state,
            )
        return self._emit_result(
            CandidateQualificationResult(
                CandidateQualificationStatus.PLAN_ONLY_CERTIFIED,
                search,
                before,
                after,
                certificate,
                "first feasible candidate certified for plan-only qualification",
            ),
            target_id=target_id,
            payload_state=payload_state,
        )

    def _after_failure(
        self,
        status: CandidateQualificationStatus,
        search: CandidateSearchResult | None,
        before: str,
        detail: str,
        *,
        target_id: int,
        payload_state: str,
    ) -> CandidateQualificationResult:
        try:
            after = self._scene_signature()
        except Exception:
            after = None
        return self._emit_result(
            CandidateQualificationResult(status, search, before, after, None, detail),
            target_id=target_id,
            payload_state=payload_state,
        )

    def _emit_result(
        self,
        result: CandidateQualificationResult,
        *,
        target_id: int,
        payload_state: str,
    ) -> CandidateQualificationResult:
        certificate = result.certificate
        self._emit(
            "CANDIDATE_QUALIFICATION_RESULT",
            {
                "target_id": target_id,
                "status": result.status.value,
                "feasible": result.feasible,
                "mode": self.qualification_mode,
                "live_payload_state": payload_state,
                "scene_signature_before": result.scene_signature_before,
                "scene_signature_after": result.scene_signature_after,
                "scene_isolated": result.scene_isolated,
                "selected_candidate_id": (
                    "" if certificate is None else certificate.candidate.candidate_id
                ),
                "geometry_fingerprint": (
                    "" if certificate is None else certificate.candidate.geometry_fingerprint
                ),
                "certificate_fingerprint": (
                    "" if certificate is None else certificate.certificate_fingerprint
                ),
                # This coordinator never sends the certificate to the executor.
                "execution_dispatched": False,
                "planning_time_sec": result.planning_time_sec,
                "joint_travel_rad": result.joint_travel_rad,
                "evaluation_count": len(result.evaluations),
                "detail": result.detail,
                "controller_commands_during_evaluation": 0,
                "gripper_commands_during_evaluation": 0,
                "physical_attach_during_evaluation": 0,
            },
        )
        return result


@dataclass(frozen=True)
class _EvidenceRecordingEvaluator:
    """Forward each actual evaluation while retaining event-time ordering."""

    evaluator: CandidateEvaluator
    target_id: int
    event_sink: Callable[[str, Mapping[str, object]], None]

    def evaluate(
        self, candidate: GraspCandidate, time_budget_sec: float
    ):
        result = self.evaluator.evaluate(candidate, time_budget_sec)
        self.event_sink(
            "CANDIDATE_EVALUATION_RESULT",
            {
                "target_id": self.target_id,
                "candidate_id": candidate.candidate_id,
                "geometry_fingerprint": candidate.geometry_fingerprint,
                "allocated_budget_sec": time_budget_sec,
                "result": result.code.value,
                "feasible": result.feasible,
                "planning_time_sec": result.planning_time_sec,
                "joint_travel_rad": result.joint_travel_rad,
                "stages": list(result.stages),
                "detail": result.detail,
            },
        )
        return result
