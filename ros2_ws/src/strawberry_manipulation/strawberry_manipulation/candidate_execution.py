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

from dataclasses import dataclass, field, replace
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
from .grasp_candidates import (
    CONTACT_CENTERING_SCALE_SEQUENCE,
    GraspCandidate,
    generate_recentered_grasp_candidates,
)


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

    def _correction_event_sink(
        self,
        event_type: str,
        payload: Mapping[str, object],
    ) -> None:
        translated = {
            "CANDIDATE_QUALIFICATION_STARTED":
                "CONTACT_CENTERING_QUALIFICATION_STARTED",
            "CANDIDATE_EVALUATION_RESULT":
                "CONTACT_CENTERING_EVALUATION_RESULT",
            "CANDIDATE_QUALIFICATION_RESULT":
                "CONTACT_CENTERING_QUALIFICATION_RESULT",
        }.get(event_type, f"CONTACT_CENTERING_{event_type}")
        self._emit(translated, {**dict(payload), "reauthorization_attempt": 1})

    def _execute_contact_centering_reauthorization(
        self,
        *,
        target_id: int,
        target_pose: Pose,
        place_pose: Pose,
        source_candidate: GraspCandidate,
        remaining_candidates: tuple[GraspCandidate, ...],
        initial_result: ExecutionResult,
        feedback: FeedbackSink | None,
    ) -> ExecutionResult:
        """Qualify and execute at most one evidence-derived correction."""

        hint = initial_result.contact_centering_hint
        if (
            hint is None
            or initial_result.success
            or initial_result.failure_code is not FailureCode.GRASP_FAILED
            or initial_result.payload_state is not PayloadState.EMPTY
            or initial_result.recovery_disposition is not RecoveryDisposition.AT_HOME
            or hint.source_candidate_id != source_candidate.candidate_id
            or hint.source_geometry_fingerprint
            != source_candidate.geometry_fingerprint
        ):
            return initial_result

        try:
            # A plan-only FEASIBLE grasp can still expose a physical contact
            # obstruction that the simplified collision geometry does not
            # model (the controlled G12 challenge pinned one finger fully
            # open).  In that case, prefer the next frozen orientations that
            # the first-feasible search did not reach.  This is still exactly
            # one reauthorization and at most four candidates.  Only when no
            # untried orientation remains do we use the measured translation
            # ladder.
            untried_orientations = tuple(remaining_candidates[:4])
            if untried_orientations:
                corrected_candidates = untried_orientations
                reauthorization_strategy = "NEXT_UNTRIED_ORIENTATION"
                correction_scales: tuple[float, ...] = ()
            else:
                generated_corrections = generate_recentered_grasp_candidates(
                    source_candidate,
                    measured_local_y_offset_m=hint.local_y_offset_m,
                )
                corrected_candidates = tuple(
                    candidate
                    for candidate, scale in zip(
                        generated_corrections,
                        CONTACT_CENTERING_SCALE_SEQUENCE,
                        strict=True,
                    )
                    if abs(hint.local_y_offset_m * scale)
                    >= self.executor.minimum_grasp_centering_correction_m
                )
                reauthorization_strategy = "CENTERING_TRANSLATION"
                correction_scales = CONTACT_CENTERING_SCALE_SEQUENCE[
                    :len(corrected_candidates)
                ]
            if not corrected_candidates:
                raise ValueError("no correction remains above the minimum bound")
        except Exception as exc:
            self._emit(
                "CONTACT_CENTERING_REAUTHORIZATION_DENIED",
                {
                    "target_id": target_id,
                    "reason": "CORRECTED_CANDIDATE_INVALID",
                    "detail": str(exc),
                    "execution_dispatched": False,
                },
            )
            return replace(
                initial_result,
                message=(
                    f"{initial_result.message}; contact-centering candidate "
                    "could not be constructed safely"
                ),
            )

        self._emit(
            "CONTACT_CENTERING_REAUTHORIZATION_STARTED",
            {
                "target_id": target_id,
                "reauthorization_attempt": 1,
                "maximum_reauthorizations": 1,
                "reauthorization_strategy": reauthorization_strategy,
                "source_candidate_id": source_candidate.candidate_id,
                "source_geometry_fingerprint":
                    source_candidate.geometry_fingerprint,
                "contact_class": hint.contact_class,
                "local_y_offset_m": hint.local_y_offset_m,
                "corrected_candidate_id": corrected_candidates[0].candidate_id,
                "corrected_geometry_fingerprint":
                    corrected_candidates[0].geometry_fingerprint,
                "corrected_candidate_ids": [
                    candidate.candidate_id for candidate in corrected_candidates
                ],
                "corrected_geometry_fingerprints": [
                    candidate.geometry_fingerprint
                    for candidate in corrected_candidates
                ],
                "correction_scale_factors": list(correction_scales),
                "escape_rejoins_source_candidate": (
                    reauthorization_strategy == "CENTERING_TRANSLATION"
                ),
                "live_payload_state": initial_result.payload_state.name,
                "recovery_disposition":
                    initial_result.recovery_disposition.name,
            },
        )
        if feedback is not None:
            feedback("CONTACT_CENTERING_REAUTHORIZATION", 0.44)

        prepared = False
        cleanup_succeeded = False
        qualification: CandidateQualificationResult | None = None
        try:
            if not self.backend.prepare_pick(target_id, target_pose):
                self._emit(
                    "CONTACT_CENTERING_REAUTHORIZATION_DENIED",
                    {
                        "target_id": target_id,
                        "reason": "PREPARE_PICK_FAILED",
                        "execution_dispatched": False,
                    },
                )
                return replace(
                    initial_result,
                    message=(
                        f"{initial_result.message}; contact-centering "
                        "reauthorization could not prepare the target scene"
                    ),
                )
            prepared = True
            qualification = PlanOnlyCandidateQualifier(
                evaluator=self.evaluator,
                scene_signature_provider=self.backend.authorization_scene_fingerprint,
                per_candidate_budget_sec=self.per_candidate_budget_sec,
                total_budget_sec=self.total_budget_sec,
                required_first_candidate_id=corrected_candidates[0].candidate_id,
                event_sink=self._correction_event_sink,
                qualification_mode="CONTACT_CENTERING_REAUTHORIZATION",
            ).qualify(
                target_id=target_id,
                candidates=corrected_candidates,
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
            "CONTACT_CENTERING_REAUTHORIZATION_CLEANUP",
            {
                "target_id": target_id,
                "reauthorization_attempt": 1,
                "qualification_status": qualification.status.value,
                "target_collision_restored": cleanup_succeeded,
                "execution_dispatched": False,
            },
        )
        if not cleanup_succeeded:
            self._emit(
                "CONTACT_CENTERING_REAUTHORIZATION_DENIED",
                {
                    "target_id": target_id,
                    "reason": "TARGET_COLLISION_CLEANUP_FAILED",
                    "execution_dispatched": False,
                },
            )
            return replace(
                initial_result,
                failure_code=FailureCode.PLANNING_FAILED,
                message=(
                    f"{initial_result.message}; contact-centering "
                    "qualification cleanup failed, so further motion is withheld"
                ),
                stages=initial_result.stages
                + ("CONTACT_CENTERING_CLEANUP_FAILED",),
                recovery_disposition=RecoveryDisposition.MOTION_WITHHELD,
                contact_centering_hint=None,
            )
        if not qualification.feasible or qualification.certificate is None:
            self._emit(
                "CONTACT_CENTERING_REAUTHORIZATION_DENIED",
                {
                    "target_id": target_id,
                    "reason": qualification.status.value,
                    "qualification_status": qualification.status.value,
                    "scene_isolated": qualification.scene_isolated,
                    "execution_dispatched": False,
                },
            )
            return replace(
                initial_result,
                message=(
                    f"{initial_result.message}; corrected candidate was rejected "
                    f"before motion: {qualification.status.value}"
                ),
                stages=initial_result.stages
                + ("CONTACT_CENTERING_REAUTHORIZATION_REJECTED",),
                contact_centering_hint=None,
            )

        certificate = qualification.certificate
        if (
            not any(
                certificate.candidate.candidate_id == candidate.candidate_id
                and certificate.candidate.geometry_fingerprint
                == candidate.geometry_fingerprint
                for candidate in corrected_candidates
            )
        ):
            self._emit(
                "CONTACT_CENTERING_REAUTHORIZATION_DENIED",
                {
                    "target_id": target_id,
                    "reason": "CERTIFICATE_IDENTITY_MISMATCH",
                    "execution_dispatched": False,
                },
            )
            return replace(
                initial_result,
                failure_code=FailureCode.PLANNING_FAILED,
                message=(
                    f"{initial_result.message}; corrected candidate certificate "
                    "identity was inconsistent"
                ),
                stages=initial_result.stages
                + ("CONTACT_CENTERING_IDENTITY_MISMATCH",),
                contact_centering_hint=None,
            )

        self._emit(
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_DISPATCHED",
            {
                "target_id": target_id,
                "reauthorization_attempt": 1,
                "candidate_id": certificate.candidate.candidate_id,
                "geometry_fingerprint":
                    certificate.candidate.geometry_fingerprint,
                "certificate_fingerprint": certificate.certificate_fingerprint,
                "scene_signature": certificate.scene_signature,
                "source_candidate_id": source_candidate.candidate_id,
                "source_geometry_fingerprint":
                    source_candidate.geometry_fingerprint,
                "local_y_offset_m": hint.local_y_offset_m,
                "reauthorization_strategy": reauthorization_strategy,
                "execution_dispatched": True,
            },
        )
        corrected_result = self.executor.execute_authorized(
            target_id,
            target_pose,
            place_pose,
            certificate,
            execution_identity=certificate.execution_identity,
            scene_signature_provider=self.backend.authorization_scene_fingerprint,
            feedback=feedback,
        )
        self._emit(
            "CONTACT_CENTERING_AUTHORIZED_EXECUTION_RESULT",
            {
                "target_id": target_id,
                "reauthorization_attempt": 1,
                "candidate_id": certificate.candidate.candidate_id,
                "geometry_fingerprint":
                    certificate.candidate.geometry_fingerprint,
                "certificate_fingerprint": certificate.certificate_fingerprint,
                "success": corrected_result.success,
                "failure_code": corrected_result.failure_code.name,
                "payload_state": corrected_result.payload_state.name,
                "recovery_disposition":
                    corrected_result.recovery_disposition.name,
                "stages": list(corrected_result.stages),
                "further_reauthorization_permitted": False,
            },
        )
        combined_message = corrected_result.message
        if corrected_result.success:
            combined_message = (
                "pick-and-place completed after one separately certified "
                "contact-driven candidate reauthorization"
            )
        else:
            combined_message = (
                f"{corrected_result.message}; the single permitted "
                "contact-driven reauthorization has been consumed"
            )
        return replace(
            corrected_result,
            message=combined_message,
            planning_time_sec=(
                initial_result.planning_time_sec
                + qualification.planning_time_sec
                + corrected_result.planning_time_sec
            ),
            execution_time_sec=(
                initial_result.execution_time_sec
                + corrected_result.execution_time_sec
            ),
            stages=(
                initial_result.stages
                + ("CONTACT_CENTERING_REAUTHORIZED",)
                + corrected_result.stages
            ),
            contact_centering_hint=None,
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
        centering_hint = result.contact_centering_hint
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
                "recovery_disposition": result.recovery_disposition.name,
                "contact_centering_source_candidate_id": (
                    ""
                    if centering_hint is None
                    else centering_hint.source_candidate_id
                ),
                "contact_centering_source_geometry_fingerprint": (
                    ""
                    if centering_hint is None
                    else centering_hint.source_geometry_fingerprint
                ),
                "contact_centering_contact_class": (
                    "" if centering_hint is None else centering_hint.contact_class
                ),
                "contact_centering_local_y_offset_m": (
                    None
                    if centering_hint is None
                    else centering_hint.local_y_offset_m
                ),
                "stages": list(result.stages),
            },
        )
        return self._execute_contact_centering_reauthorization(
            target_id=target_id,
            target_pose=target_pose,
            place_pose=place_pose,
            source_candidate=certificate.candidate,
            remaining_candidates=tuple(
                candidates[
                    next(
                        index
                        for index, candidate in enumerate(candidates)
                        if candidate.candidate_id
                        == certificate.candidate.candidate_id
                        and candidate.geometry_fingerprint
                        == certificate.candidate.geometry_fingerprint
                    )
                    + 1:
                ]
            ),
            initial_result=result,
            feedback=feedback,
        )
