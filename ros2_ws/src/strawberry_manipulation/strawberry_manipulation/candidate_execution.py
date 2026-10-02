"""Runtime bridge from ADR 0087 candidate search to exact execution.

This module deliberately has no ROS or MoveIt imports.  It owns the narrow
ADR 0087-E policy boundary:

``prepare live target -> copied-scene candidate search -> restore ->
re-prepare and verify the same scene -> execute exactly the certificate``.

It never executes a rejected candidate, and the executor independently checks
the live scene signature after its second preparation step before it may send
an arm or gripper command.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Mapping, Protocol

from .candidate_qualification import (
    CandidateQualificationResult,
    PlanOnlyCandidateQualifier,
)
from .core import (
    ExecutionResult,
    FailureCode,
    PayloadState,
    PickAndPlaceExecutor,
    Pose,
    RecoveryDisposition,
)
from .grasp_candidate_search import CandidateEvaluator
from .grasp_candidates import GraspCandidate


class CandidateExecutionBackend(Protocol):
    """Minimal live-scene lifecycle needed around a candidate certificate."""

    def prepare_pick(self, target_id: int, target_pose: Pose) -> bool: ...

    def restore_target_collision(self, target_id: int) -> bool: ...

    def authorization_scene_fingerprint(self) -> str: ...


EventSink = Callable[[str, Mapping[str, object]], None]
FeedbackSink = Callable[[str, float], None]


@dataclass
class RuntimeCandidateExecutionCoordinator:
    """Perform a fresh bounded search immediately before certified execution.

    The first ``prepare_pick`` exists solely to materialize the same solid
    target collision state that ADR 0086 evaluates.  It is always undone
    before the executor is called.  The executor then performs its own fresh
    preparation and refuses to move unless its resulting live-scene
    fingerprint matches the certificate.
    """

    executor: PickAndPlaceExecutor
    backend: CandidateExecutionBackend
    evaluator: CandidateEvaluator
    per_candidate_budget_sec: float = 2.0
    total_budget_sec: float = 12.0
    event_sink: EventSink | None = field(default=None, repr=False)

    def _emit(self, event_type: str, payload: Mapping[str, object]) -> None:
        if self.event_sink is None:
            return
        try:
            self.event_sink(event_type, dict(payload))
        except Exception:
            # Evidence transport is diagnostic.  It cannot be allowed to
            # silently alter a safety decision or trigger an unsafe fallback.
            pass

    @staticmethod
    def _failure(
        message: str,
        stages: tuple[str, ...],
        *,
        planning_time_sec: float = 0.0,
    ) -> ExecutionResult:
        return ExecutionResult(
            False,
            FailureCode.PLANNING_FAILED,
            message,
            planning_time_sec,
            0.0,
            stages,
            RecoveryDisposition.HOME_REQUIRED,
            PayloadState.EMPTY,
        )

    def execute(
        self,
        *,
        target_id: int,
        target_pose: Pose,
        place_pose: Pose,
        candidates: tuple[GraspCandidate, ...],
        feedback: FeedbackSink | None = None,
    ) -> ExecutionResult:
        """Search once, then execute only the exact fresh certificate."""

        if not isinstance(target_id, int) or isinstance(target_id, bool) or target_id <= 0:
            return self._failure(
                "adaptive candidate execution requires a positive target ID",
                ("CANDIDATE_EXECUTION_DENIED",),
            )
        if self.executor.payload_state is not PayloadState.EMPTY:
            return self._failure(
                "adaptive candidate execution requires an EMPTY payload",
                ("CANDIDATE_EXECUTION_DENIED",),
            )

        self._emit(
            "ADAPTIVE_CANDIDATE_EXECUTION_STARTED",
            {
                "target_id": target_id,
                "candidate_count": len(candidates),
                "candidate_ids": [candidate.candidate_id for candidate in candidates],
                "candidate_geometry_fingerprints": [
                    candidate.geometry_fingerprint for candidate in candidates
                ],
                "live_payload_state": self.executor.payload_state.name,
                "controller_commands_before_authorization": 0,
                "gripper_commands_before_authorization": 0,
                "physical_attach_before_authorization": 0,
            },
        )
        if feedback is not None:
            feedback("CANDIDATE_SEARCH", 0.04)

        prepared = False
        cleanup_succeeded = False
        qualification: CandidateQualificationResult | None = None
        try:
            if not self.backend.prepare_pick(target_id, target_pose):
                self._emit(
                    "ADAPTIVE_CANDIDATE_EXECUTION_DENIED",
                    {
                        "target_id": target_id,
                        "reason": "PREPARE_PICK_FAILED",
                        "execution_dispatched": False,
                    },
                )
                return self._failure(
                    "adaptive candidate execution could not prepare the target "
                    "collision scene",
                    ("CANDIDATE_EXECUTION_SETUP_FAILED",),
                )
            prepared = True
            qualification = PlanOnlyCandidateQualifier(
                evaluator=self.evaluator,
                scene_signature_provider=self.backend.authorization_scene_fingerprint,
                per_candidate_budget_sec=self.per_candidate_budget_sec,
                total_budget_sec=self.total_budget_sec,
                event_sink=self.event_sink,
                qualification_mode="PRE_EXECUTION_QUALIFICATION",
            ).qualify(
                target_id=target_id,
                candidates=candidates,
                payload_state=self.executor.payload_state.name,
            )
        finally:
            if prepared:
                try:
                    cleanup_succeeded = bool(
                        self.backend.restore_target_collision(target_id)
                    )
                except Exception:
                    cleanup_succeeded = False

        assert qualification is not None
        self._emit(
            "ADAPTIVE_CANDIDATE_QUALIFICATION_CLEANUP",
            {
                "target_id": target_id,
                "qualification_status": qualification.status.value,
                "target_collision_restored": cleanup_succeeded,
                "execution_dispatched": False,
            },
        )
        if not cleanup_succeeded:
            self._emit(
                "ADAPTIVE_CANDIDATE_EXECUTION_DENIED",
                {
                    "target_id": target_id,
                    "reason": "TARGET_COLLISION_CLEANUP_FAILED",
                    "qualification_status": qualification.status.value,
                    "execution_dispatched": False,
                },
            )
            return self._failure(
                "adaptive candidate qualification cleanup failed; execution is "
                "withheld",
                ("CANDIDATE_SEARCH", "CLEANUP_FAILED"),
                planning_time_sec=qualification.planning_time_sec,
            )
        if not qualification.feasible or qualification.certificate is None:
            self._emit(
                "ADAPTIVE_CANDIDATE_EXECUTION_DENIED",
                {
                    "target_id": target_id,
                    "reason": qualification.status.value,
                    "qualification_status": qualification.status.value,
                    "scene_isolated": qualification.scene_isolated,
                    "evaluation_count": len(qualification.evaluations),
                    "execution_dispatched": False,
                },
            )
            return self._failure(
                "adaptive candidate qualification rejected before execution: "
                f"{qualification.status.value}; {qualification.detail}",
                ("CANDIDATE_SEARCH", qualification.status.value),
                planning_time_sec=qualification.planning_time_sec,
            )

        certificate = qualification.certificate
        self._emit(
            "AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED",
            {
                "target_id": target_id,
                "candidate_id": certificate.candidate.candidate_id,
                "geometry_fingerprint": certificate.candidate.geometry_fingerprint,
                "certificate_fingerprint": certificate.certificate_fingerprint,
                "scene_signature": certificate.scene_signature,
                "execution_dispatched": True,
            },
        )
        result = self.executor.execute_authorized(
            target_id,
            target_pose,
            place_pose,
            certificate,
            execution_identity=certificate.execution_identity,
            scene_signature_provider=self.backend.authorization_scene_fingerprint,
            feedback=feedback,
        )
        self._emit(
            "AUTHORIZED_CANDIDATE_EXECUTION_RESULT",
            {
                "target_id": target_id,
                "candidate_id": certificate.candidate.candidate_id,
                "geometry_fingerprint": certificate.candidate.geometry_fingerprint,
                "certificate_fingerprint": certificate.certificate_fingerprint,
                "success": result.success,
                "failure_code": result.failure_code.name,
                "payload_state": result.payload_state.name,
                "stages": list(result.stages),
            },
        )
        return result
