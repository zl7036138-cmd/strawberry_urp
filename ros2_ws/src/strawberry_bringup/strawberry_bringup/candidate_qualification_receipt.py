"""Receipt checks for ADR 0087-D plan-only runtime qualification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


QUALIFICATION_START = "CANDIDATE_QUALIFICATION_STARTED"
EVALUATION_RESULT = "CANDIDATE_EVALUATION_RESULT"
QUALIFICATION_RESULT = "CANDIDATE_QUALIFICATION_RESULT"
QUALIFICATION_CLEANUP = "CANDIDATE_QUALIFICATION_CLEANUP"
TARGET_COLLISION_RESTORED = "TARGET_COLLISION_RESTORED"
FORBIDDEN_PLAN_ONLY_EVENT_FRAGMENTS = (
    # A normal backend command emits COMMAND_PREPARED first, but the receipt
    # checker is intentionally broader: any command diagnostic inside the
    # plan-only interval is a failed safety boundary, even if its producer
    # emitted a malformed or incomplete command lifecycle.
    "COMMAND_",
    "ACTION_ACCEPTED",
    "GRIPPER_",
    "PHYSICAL_ATTACH",
    "CONTACT",
    "HOLDING",
)


@dataclass(frozen=True)
class CandidateQualificationReceiptResult:
    passed: bool
    errors: tuple[str, ...]
    status: str | None
    selected_candidate_id: str | None


def qualify_candidate_plan_only_receipt(
    receipt: Mapping[str, object],
    *,
    target_id: int,
    expected_status: str = "PLAN_ONLY_CERTIFIED",
) -> CandidateQualificationReceiptResult:
    """Validate one complete no-motion candidate-qualification receipt."""

    errors: list[str] = []
    motion_events = receipt.get("motion_events")
    if not isinstance(motion_events, list):
        return CandidateQualificationReceiptResult(
            False, ("motion_events missing",), None, None
        )
    ordered = sorted(
        (event for event in motion_events if isinstance(event, Mapping)),
        key=lambda event: event.get("event_time_monotonic_ns", -1),
    )

    def matching(kind: str) -> list[Mapping[str, object]]:
        return [
            event
            for event in ordered
            if event.get("event_type") == kind
            and isinstance(event.get("payload"), Mapping)
            and event["payload"].get("target_id") == target_id
        ]

    starts = matching(QUALIFICATION_START)
    results = matching(QUALIFICATION_RESULT)
    cleanups = matching(QUALIFICATION_CLEANUP)
    if len(starts) != 1 or len(results) != 1 or len(cleanups) != 1:
        return CandidateQualificationReceiptResult(
            False,
            ("expected exactly one start/result/cleanup for target",),
            None,
            None,
        )
    start, result, cleanup = starts[0], results[0], cleanups[0]
    start_payload = start["payload"]
    result_payload = result["payload"]
    cleanup_payload = cleanup["payload"]
    start_ns = start.get("event_time_monotonic_ns")
    result_ns = result.get("event_time_monotonic_ns")
    cleanup_ns = cleanup.get("event_time_monotonic_ns")
    if not (
        isinstance(start_ns, int)
        and isinstance(result_ns, int)
        and isinstance(cleanup_ns, int)
        and start_ns < result_ns < cleanup_ns
    ):
        errors.append("start/result/cleanup timestamps are invalid")

    for label, payload in (("start", start_payload), ("result", result_payload)):
        if payload.get("mode") != "PLAN_ONLY":
            errors.append(f"{label} is not marked PLAN_ONLY")
        if payload.get("live_payload_state") != "EMPTY":
            errors.append(f"{label} payload was not EMPTY")
        for key in (
            "controller_commands_during_evaluation",
            "gripper_commands_during_evaluation",
            "physical_attach_during_evaluation",
        ):
            if payload.get(key) != 0:
                errors.append(f"{label} reports non-zero {key}")
    if result_payload.get("status") != expected_status:
        errors.append("unexpected qualification status")
    if expected_status == "PLAN_ONLY_CERTIFIED":
        if result_payload.get("feasible") is not True:
            errors.append("certified result was not feasible")
        if result_payload.get("scene_isolated") is not True:
            errors.append("planning scene changed during qualification")
        for key in (
            "selected_candidate_id",
            "geometry_fingerprint",
            "certificate_fingerprint",
        ):
            if not isinstance(result_payload.get(key), str) or not result_payload[key]:
                errors.append(f"certified result lacks {key}")
    if result_payload.get("execution_dispatched") is not False:
        errors.append("plan-only qualification dispatched execution")

    evaluations = [
        event
        for event in matching(EVALUATION_RESULT)
        if isinstance(start_ns, int)
        and isinstance(result_ns, int)
        and start_ns < event.get("event_time_monotonic_ns", -1) < result_ns
    ]
    candidate_ids = start_payload.get("candidate_ids")
    candidate_fingerprints = start_payload.get("candidate_geometry_fingerprints")
    if not isinstance(candidate_ids, list) or not isinstance(candidate_fingerprints, list):
        errors.append("start lacks candidate identity sequence")
    elif len(candidate_ids) != len(candidate_fingerprints):
        errors.append("candidate identity sequence lengths differ")
    else:
        expected_prefix = candidate_ids[:len(evaluations)]
        observed_ids = [event["payload"].get("candidate_id") for event in evaluations]
        if observed_ids != expected_prefix:
            errors.append("evaluation sequence differs from frozen candidate order")
        for index, event in enumerate(evaluations):
            payload = event["payload"]
            if payload.get("geometry_fingerprint") != candidate_fingerprints[index]:
                errors.append("evaluation geometry fingerprint differs from start")
                break
        feasible_indices = [
            index
            for index, event in enumerate(evaluations)
            if event["payload"].get("feasible") is True
        ]
        if expected_status == "PLAN_ONLY_CERTIFIED":
            if feasible_indices != [len(evaluations) - 1]:
                errors.append("first-feasible search did not stop at one final feasible row")
            elif result_payload.get("selected_candidate_id") != observed_ids[-1]:
                errors.append("selected candidate differs from final feasible row")

    forbidden_events = [
        event.get("event_type", "")
        for event in ordered
        if isinstance(start_ns, int)
        and isinstance(cleanup_ns, int)
        and start_ns < event.get("event_time_monotonic_ns", -1) < cleanup_ns
        and any(
            fragment in event.get("event_type", "")
            for fragment in FORBIDDEN_PLAN_ONLY_EVENT_FRAGMENTS
        )
    ]
    if forbidden_events:
        errors.append("plan-only qualification emitted motion/gripper/attach evidence")

    restorations = [
        event
        for event in matching(TARGET_COLLISION_RESTORED)
        if isinstance(result_ns, int)
        and isinstance(cleanup_ns, int)
        and result_ns < event.get("event_time_monotonic_ns", -1) < cleanup_ns
        # Restoration must be explicitly attested. Treating a missing field
        # as success would turn an incomplete diagnostic into a false proof.
        and event["payload"].get("restored") is True
    ]
    if not restorations:
        errors.append("target collision was not restored after qualification")
    if cleanup_payload.get("target_collision_restored") is not True:
        errors.append("cleanup does not attest target collision restoration")
    if cleanup_payload.get("execution_dispatched") is not False:
        errors.append("cleanup does not preserve plan-only boundary")

    return CandidateQualificationReceiptResult(
        not errors,
        tuple(errors),
        str(result_payload.get("status")),
        (
            str(result_payload.get("selected_candidate_id"))
            if result_payload.get("selected_candidate_id")
            else None
        ),
    )
