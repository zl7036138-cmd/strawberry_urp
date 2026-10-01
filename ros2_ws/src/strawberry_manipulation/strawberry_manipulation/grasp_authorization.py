"""Certificate-to-execution identity contract for ADR 0087-C.

This is deliberately pure: it defines what may be executed, but it does not
move a robot or inspect a PlanningScene.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import hmac
import json

from .grasp_candidates import GraspCandidate
from .whole_chain import ChainEvaluation, ChainFailureCode


class ExecutionAuthorizationError(ValueError):
    """Raised when a certificate cannot authorize the requested geometry."""


@dataclass(frozen=True)
class ExecutionIdentity:
    target_id: int
    candidate_id: str
    geometry_fingerprint: str
    scene_signature: str

    def __post_init__(self) -> None:
        if (
            not isinstance(self.target_id, int)
            or isinstance(self.target_id, bool)
            or self.target_id <= 0
        ):
            raise ValueError("execution target ID must be a positive integer")
        if not all(
            isinstance(value, str) and value
            for value in (
                self.candidate_id,
                self.geometry_fingerprint,
                self.scene_signature,
            )
        ):
            raise ValueError("execution identity fields must be non-empty")


@dataclass(frozen=True)
class AuthorizedGraspPlan:
    """A feasible ADR 0086 certificate bound to one exact candidate geometry."""

    target_id: int
    candidate: GraspCandidate
    whole_chain_result: ChainEvaluation
    scene_signature: str
    certificate_timestamp_ns: int
    certificate_fingerprint: str

    def __post_init__(self) -> None:
        if (
            not isinstance(self.target_id, int)
            or isinstance(self.target_id, bool)
            or self.target_id <= 0
        ):
            raise ValueError("certificate target ID must be a positive integer")
        if not isinstance(self.candidate, GraspCandidate):
            raise ValueError("certificate candidate must be a GraspCandidate")
        if not isinstance(self.whole_chain_result, ChainEvaluation):
            raise ValueError("certificate result must be a ChainEvaluation")
        if self.whole_chain_result.code is not ChainFailureCode.FEASIBLE:
            raise ValueError("only a feasible whole-chain result may authorize execution")
        if not isinstance(self.scene_signature, str) or not self.scene_signature:
            raise ValueError("scene signature must be non-empty")
        if (
            not isinstance(self.certificate_timestamp_ns, int)
            or isinstance(self.certificate_timestamp_ns, bool)
            or self.certificate_timestamp_ns <= 0
        ):
            raise ValueError("certificate timestamp must be a positive integer")
        if (
            not isinstance(self.certificate_fingerprint, str)
            or not self.certificate_fingerprint
        ):
            raise ValueError("certificate fingerprint must be non-empty")
        expected = _certificate_fingerprint(
            self.target_id,
            self.candidate,
            self.whole_chain_result,
            self.scene_signature,
            self.certificate_timestamp_ns,
        )
        if not hmac.compare_digest(self.certificate_fingerprint, expected):
            raise ValueError("certificate fingerprint does not match authorization")

    @property
    def execution_identity(self) -> ExecutionIdentity:
        return ExecutionIdentity(
            self.target_id,
            self.candidate.candidate_id,
            self.candidate.geometry_fingerprint,
            self.scene_signature,
        )


def _certificate_fingerprint(
    target_id: int,
    candidate: GraspCandidate,
    result: ChainEvaluation,
    scene_signature: str,
    certificate_timestamp_ns: int,
) -> str:
    payload = {
        "target_id": target_id,
        "candidate_id": candidate.candidate_id,
        "candidate_geometry_fingerprint": candidate.geometry_fingerprint,
        "scene_signature": scene_signature,
        "certificate_timestamp_ns": certificate_timestamp_ns,
        "whole_chain_code": result.code.value,
        "whole_chain_stages": result.stages,
        "whole_chain_detail": result.detail,
        "planning_time_sec": result.planning_time_sec,
        "joint_travel_rad": result.joint_travel_rad,
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def authorize_grasp_candidate(
    candidate: GraspCandidate,
    result: ChainEvaluation,
    *,
    target_id: int,
    scene_signature: str,
    certificate_timestamp_ns: int,
) -> AuthorizedGraspPlan:
    """Create an immutable execution authority only from FEASIBLE evidence."""

    if not isinstance(candidate, GraspCandidate):
        raise ValueError("candidate must be a GraspCandidate")
    if not isinstance(result, ChainEvaluation):
        raise ValueError("result must be a ChainEvaluation")
    if result.code is not ChainFailureCode.FEASIBLE:
        raise ExecutionAuthorizationError(
            f"candidate {candidate.candidate_id} lacks a feasible certificate"
        )
    identity = ExecutionIdentity(
        target_id,
        candidate.candidate_id,
        candidate.geometry_fingerprint,
        scene_signature,
    )
    return AuthorizedGraspPlan(
        target_id=identity.target_id,
        candidate=candidate,
        whole_chain_result=result,
        scene_signature=identity.scene_signature,
        certificate_timestamp_ns=certificate_timestamp_ns,
        certificate_fingerprint=_certificate_fingerprint(
            identity.target_id,
            candidate,
            result,
            identity.scene_signature,
            certificate_timestamp_ns,
        ),
    )


def require_execution_identity(
    authorized_plan: AuthorizedGraspPlan | None,
    requested_identity: ExecutionIdentity | None,
    *,
    target_id: int | None = None,
) -> AuthorizedGraspPlan:
    """Fail closed unless execution requests exactly the certified geometry."""

    if not isinstance(authorized_plan, AuthorizedGraspPlan):
        raise ExecutionAuthorizationError("execution requires an authorized grasp plan")
    if not isinstance(requested_identity, ExecutionIdentity):
        raise ExecutionAuthorizationError("execution requires a complete identity")
    if target_id is not None and (
        not isinstance(target_id, int)
        or isinstance(target_id, bool)
        or target_id <= 0
    ):
        raise ExecutionAuthorizationError("requested pick target ID must be positive")
    expected = authorized_plan.execution_identity
    if requested_identity.target_id != expected.target_id:
        raise ExecutionAuthorizationError("execution target ID differs from certificate")
    if target_id is not None and requested_identity.target_id != target_id:
        raise ExecutionAuthorizationError("execution target ID differs from requested pick")
    if requested_identity.candidate_id != expected.candidate_id:
        raise ExecutionAuthorizationError("execution candidate ID differs from certificate")
    if requested_identity.geometry_fingerprint != expected.geometry_fingerprint:
        raise ExecutionAuthorizationError(
            "execution geometry fingerprint differs from certificate"
        )
    if requested_identity.scene_signature != expected.scene_signature:
        raise ExecutionAuthorizationError("execution scene signature differs from certificate")
    return authorized_plan
