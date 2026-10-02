"""Strict evidence checks for ADR 0087-E adaptive candidate execution."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence


ADAPTIVE_START = "ADAPTIVE_CANDIDATE_EXECUTION_STARTED"
QUALIFICATION_START = "CANDIDATE_QUALIFICATION_STARTED"
EVALUATION = "CANDIDATE_EVALUATION_RESULT"
QUALIFICATION_RESULT = "CANDIDATE_QUALIFICATION_RESULT"
TARGET_RESTORED = "TARGET_COLLISION_RESTORED"
QUALIFICATION_CLEANUP = "ADAPTIVE_CANDIDATE_QUALIFICATION_CLEANUP"
EXECUTION_DISPATCHED = "AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED"
EXECUTION_RESULT = "AUTHORIZED_CANDIDATE_EXECUTION_RESULT"
COMMAND_EVENTS = {"COMMAND_PREPARED", "ACTION_ACCEPTED"}


@dataclass(frozen=True)
class CandidateExecutionReceiptResult:
    passed: bool
    errors: tuple[str, ...]
    selected_candidate_id: str | None
    certificate_fingerprint: str | None


def _payload(event: Mapping[str, object]) -> Mapping[str, object] | None:
    value = event.get("payload")
    return value if isinstance(value, Mapping) else None


def _event_time(event: Mapping[str, object]) -> int | None:
    value = event.get("event_time_monotonic_ns")
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def _target_matches(event: Mapping[str, object], target_id: int) -> bool:
    payload = _payload(event)
    return payload is not None and payload.get("target_id") == target_id


def _one(
    events: Sequence[Mapping[str, object]], event_type: str, target_id: int
) -> Mapping[str, object] | None:
    matching = [
        event
        for event in events
        if event.get("event_type") == event_type and _target_matches(event, target_id)
    ]
    return matching[0] if len(matching) == 1 else None


def qualify_adaptive_candidate_execution_receipt(
    receipt: Mapping[str, object], *, target_id: int
) -> CandidateExecutionReceiptResult:
    """Verify ``G00 rejected -> Gi certified -> exact Gi dispatched``.

    This is intentionally stronger than a generic action-success check.  It
    binds the selected candidate, geometry fingerprint, certificate and scene
    signature across qualification, cleanup and physical dispatch.  It also
    proves that no arm command occurred before that authorization boundary.
    """

    errors: list[str] = []
    raw_events = receipt.get("motion_events")
    if not isinstance(raw_events, list):
        return CandidateExecutionReceiptResult(
            False, ("motion_events missing",), None, None
        )
    events = tuple(event for event in raw_events if isinstance(event, Mapping))
    if not events:
        return CandidateExecutionReceiptResult(
            False, ("motion_events is empty",), None, None
        )

    required = {
        name: _one(events, name, target_id)
        for name in (
            ADAPTIVE_START,
            QUALIFICATION_START,
            QUALIFICATION_RESULT,
            QUALIFICATION_CLEANUP,
            EXECUTION_DISPATCHED,
            EXECUTION_RESULT,
        )
    }
    for name, event in required.items():
        if event is None:
            errors.append(f"expected exactly one {name} event for target")
    if errors:
        return CandidateExecutionReceiptResult(False, tuple(errors), None, None)

    start = required[ADAPTIVE_START]
    qualification_start = required[QUALIFICATION_START]
    qualification_result = required[QUALIFICATION_RESULT]
    cleanup = required[QUALIFICATION_CLEANUP]
    dispatched = required[EXECUTION_DISPATCHED]
    execution_result = required[EXECUTION_RESULT]
    assert all(
        event is not None
        for event in (
            start,
            qualification_start,
            qualification_result,
            cleanup,
            dispatched,
            execution_result,
        )
    )
    start_payload = _payload(start)
    qualification_start_payload = _payload(qualification_start)
    qualification_result_payload = _payload(qualification_result)
    cleanup_payload = _payload(cleanup)
    dispatched_payload = _payload(dispatched)
    execution_result_payload = _payload(execution_result)
    if not all(
        payload is not None
        for payload in (
            start_payload,
            qualification_start_payload,
            qualification_result_payload,
            cleanup_payload,
            dispatched_payload,
            execution_result_payload,
        )
    ):
        return CandidateExecutionReceiptResult(
            False, ("required event payload missing",), None, None
        )
    assert start_payload is not None
    assert qualification_start_payload is not None
    assert qualification_result_payload is not None
    assert cleanup_payload is not None
    assert dispatched_payload is not None
    assert execution_result_payload is not None

    for label, payload in (
        ("adaptive start", start_payload),
        ("qualification start", qualification_start_payload),
    ):
        if payload.get("live_payload_state") != "EMPTY":
            errors.append(f"{label} payload was not EMPTY")
        for counter in (
            "controller_commands_before_authorization",
            "gripper_commands_before_authorization",
            "physical_attach_before_authorization",
        ):
            if counter in payload and payload.get(counter) != 0:
                errors.append(f"{label} reports non-zero {counter}")
        for counter in (
            "controller_commands_during_evaluation",
            "gripper_commands_during_evaluation",
            "physical_attach_during_evaluation",
        ):
            if counter in payload and payload.get(counter) != 0:
                errors.append(f"{label} reports non-zero {counter}")
    if qualification_start_payload.get("mode") != "PRE_EXECUTION_QUALIFICATION":
        errors.append("qualification was not labelled PRE_EXECUTION_QUALIFICATION")
    if qualification_result_payload.get("status") != "PLAN_ONLY_CERTIFIED":
        errors.append("candidate qualification did not certify a result")
    if qualification_result_payload.get("feasible") is not True:
        errors.append("candidate qualification result was not feasible")
    if qualification_result_payload.get("scene_isolated") is not True:
        errors.append("candidate qualification changed the live planning scene")
    if qualification_result_payload.get("execution_dispatched") is not False:
        errors.append("qualification result claimed execution before cleanup")

    timestamped = {
        name: _event_time(event)
        for name, event in required.items()
        if event is not None
    }
    if any(value is None for value in timestamped.values()):
        errors.append("required event timestamps are invalid")
    elif not (
        timestamped[ADAPTIVE_START]
        < timestamped[QUALIFICATION_START]
        < timestamped[QUALIFICATION_RESULT]
        < timestamped[QUALIFICATION_CLEANUP]
        < timestamped[EXECUTION_DISPATCHED]
        < timestamped[EXECUTION_RESULT]
    ):
        errors.append("adaptive authorization event order is invalid")

    qualification_start_ns = _event_time(qualification_start)
    qualification_result_ns = _event_time(qualification_result)
    evaluation_rows = [
        event
        for event in events
        if event.get("event_type") == EVALUATION and _target_matches(event, target_id)
    ]
    evaluation_rows.sort(key=lambda event: _event_time(event) or -1)
    if not evaluation_rows:
        errors.append("candidate evaluation trace is missing")
    elif (
        qualification_start_ns is None
        or qualification_result_ns is None
        or any(
            (time_ns := _event_time(event)) is None
            or not qualification_start_ns < time_ns < qualification_result_ns
            for event in evaluation_rows
        )
    ):
        errors.append("candidate evaluation trace is outside qualification interval")

    selected_candidate = qualification_result_payload.get("selected_candidate_id")
    selected_fingerprint = qualification_result_payload.get("geometry_fingerprint")
    certificate_fingerprint = qualification_result_payload.get("certificate_fingerprint")
    scene_signature = qualification_result_payload.get("scene_signature_before")
    # CandidateQualificationResult emits the scene identity as
    # ``scene_signature_before`` only in the diagnostic event.  Its response
    # and the later dispatch both use the same signed value.
    if not isinstance(selected_candidate, str) or not selected_candidate:
        errors.append("qualification result lacks selected candidate ID")
        selected_candidate = None
    if not isinstance(selected_fingerprint, str) or not selected_fingerprint:
        errors.append("qualification result lacks selected geometry fingerprint")
        selected_fingerprint = None
    if not isinstance(certificate_fingerprint, str) or not certificate_fingerprint:
        errors.append("qualification result lacks certificate fingerprint")
        certificate_fingerprint = None
    if not isinstance(scene_signature, str) or not scene_signature:
        errors.append("qualification result lacks planning-scene signature")
        scene_signature = None

    if evaluation_rows and selected_candidate is not None:
        rows = []
        for event in evaluation_rows:
            payload = _payload(event)
            if payload is None:
                errors.append("candidate evaluation payload missing")
                continue
            rows.append(payload)
        if not rows or rows[0].get("candidate_id") != "G00":
            errors.append("candidate trace did not begin with nominal G00")
        selected_indices = [
            index
            for index, payload in enumerate(rows)
            if payload.get("candidate_id") == selected_candidate
        ]
        if len(selected_indices) != 1:
            errors.append("selected candidate is absent or duplicated in trace")
        else:
            selected_index = selected_indices[0]
            if selected_index == 0:
                errors.append("adaptive execution did not replace nominal G00")
            if selected_index != len(rows) - 1:
                errors.append("candidate search evaluated after the selected candidate")
            selected_row = rows[selected_index]
            if selected_row.get("feasible") is not True or selected_row.get("result") != "FEASIBLE":
                errors.append("selected candidate was not FEASIBLE")
            if selected_row.get("geometry_fingerprint") != selected_fingerprint:
                errors.append("selected candidate geometry differs from certificate")
            if any(payload.get("feasible") is True for payload in rows[:selected_index]):
                errors.append("candidate search skipped an earlier feasible candidate")

    cleanup_ns = _event_time(cleanup)
    restore_events = [
        event
        for event in events
        if event.get("event_type") == TARGET_RESTORED
        and _target_matches(event, target_id)
        and _payload(event) is not None
        and _payload(event).get("restored") is True
        and qualification_result_ns is not None
        and cleanup_ns is not None
        and qualification_result_ns < (_event_time(event) or -1) < cleanup_ns
    ]
    if not restore_events:
        errors.append("target collision was not restored before adaptive dispatch")
    if cleanup_payload.get("target_collision_restored") is not True:
        errors.append("adaptive cleanup did not report restored target collision")
    if cleanup_payload.get("execution_dispatched") is not False:
        errors.append("adaptive cleanup claimed execution before dispatch")

    for key, expected in (
        ("candidate_id", selected_candidate),
        ("geometry_fingerprint", selected_fingerprint),
        ("certificate_fingerprint", certificate_fingerprint),
        ("scene_signature", scene_signature),
    ):
        if expected is not None and dispatched_payload.get(key) != expected:
            errors.append(f"dispatch {key} differs from certificate")
    if dispatched_payload.get("execution_dispatched") is not True:
        errors.append("certificate was not explicitly dispatched")
    for key, expected in (
        ("candidate_id", selected_candidate),
        ("geometry_fingerprint", selected_fingerprint),
        ("certificate_fingerprint", certificate_fingerprint),
    ):
        if expected is not None and execution_result_payload.get(key) != expected:
            errors.append(f"execution result {key} differs from certificate")
    if execution_result_payload.get("success") is not True:
        errors.append("authorized candidate execution did not succeed")

    dispatch_ns = _event_time(dispatched)
    if dispatch_ns is not None:
        premature_commands = [
            event
            for event in events
            if event.get("event_type") in COMMAND_EVENTS
            and (_event_time(event) is None or _event_time(event) < dispatch_ns)
        ]
        if premature_commands:
            errors.append("controller command occurred before candidate dispatch")
    denied_events = [
        event
        for event in events
        if event.get("event_type") in {
            "ADAPTIVE_CANDIDATE_EXECUTION_DENIED",
            "AUTHORIZED_GRASP_DENIED",
        }
        and _target_matches(event, target_id)
    ]
    if denied_events:
        errors.append("adaptive path emitted a denial event")

    action_result = receipt.get("action_result")
    if not isinstance(action_result, Mapping) or action_result.get("success") is not True:
        errors.append("action result was not successful")

    return CandidateExecutionReceiptResult(
        not errors,
        tuple(errors),
        selected_candidate,
        certificate_fingerprint,
    )
