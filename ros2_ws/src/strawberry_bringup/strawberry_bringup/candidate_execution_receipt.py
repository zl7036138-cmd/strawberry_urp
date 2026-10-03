"""Strict evidence checks for ADR 0087-E adaptive candidate execution."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


ADAPTIVE_START = "ADAPTIVE_CANDIDATE_EXECUTION_STARTED"
SCENE_LEASE_STARTED = "ADAPTIVE_COLLISION_SCENE_LEASE_STARTED"
SCENE_LEASE_ENDED = "ADAPTIVE_COLLISION_SCENE_LEASE_ENDED"
QUALIFICATION_START = "CANDIDATE_QUALIFICATION_STARTED"
EVALUATION = "CANDIDATE_EVALUATION_RESULT"
QUALIFICATION_RESULT = "CANDIDATE_QUALIFICATION_RESULT"
TARGET_RESTORED = "TARGET_COLLISION_RESTORED"
QUALIFICATION_CLEANUP = "ADAPTIVE_CANDIDATE_QUALIFICATION_CLEANUP"
EXECUTION_DISPATCHED = "AUTHORIZED_CANDIDATE_EXECUTION_DISPATCHED"
EXECUTION_RESULT = "AUTHORIZED_CANDIDATE_EXECUTION_RESULT"
CENTERING_START = "CONTACT_CENTERING_REAUTHORIZATION_STARTED"
CENTERING_QUALIFICATION_START = "CONTACT_CENTERING_QUALIFICATION_STARTED"
CENTERING_EVALUATION = "CONTACT_CENTERING_EVALUATION_RESULT"
CENTERING_QUALIFICATION_RESULT = "CONTACT_CENTERING_QUALIFICATION_RESULT"
CENTERING_CLEANUP = "CONTACT_CENTERING_REAUTHORIZATION_CLEANUP"
CENTERING_DISPATCHED = "CONTACT_CENTERING_AUTHORIZED_EXECUTION_DISPATCHED"
CENTERING_RESULT = "CONTACT_CENTERING_AUTHORIZED_EXECUTION_RESULT"
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
            SCENE_LEASE_STARTED,
            ADAPTIVE_START,
            QUALIFICATION_START,
            QUALIFICATION_RESULT,
            QUALIFICATION_CLEANUP,
            EXECUTION_DISPATCHED,
            EXECUTION_RESULT,
            SCENE_LEASE_ENDED,
        )
    }
    for name, event in required.items():
        if event is None:
            errors.append(f"expected exactly one {name} event for target")
    if errors:
        return CandidateExecutionReceiptResult(False, tuple(errors), None, None)

    start = required[ADAPTIVE_START]
    scene_lease_started = required[SCENE_LEASE_STARTED]
    qualification_start = required[QUALIFICATION_START]
    qualification_result = required[QUALIFICATION_RESULT]
    cleanup = required[QUALIFICATION_CLEANUP]
    dispatched = required[EXECUTION_DISPATCHED]
    execution_result = required[EXECUTION_RESULT]
    scene_lease_ended = required[SCENE_LEASE_ENDED]
    assert all(
        event is not None
        for event in (
            start,
            scene_lease_started,
            qualification_start,
            qualification_result,
            cleanup,
            dispatched,
            execution_result,
            scene_lease_ended,
        )
    )
    start_payload = _payload(start)
    scene_lease_started_payload = _payload(scene_lease_started)
    qualification_start_payload = _payload(qualification_start)
    qualification_result_payload = _payload(qualification_result)
    cleanup_payload = _payload(cleanup)
    dispatched_payload = _payload(dispatched)
    execution_result_payload = _payload(execution_result)
    scene_lease_ended_payload = _payload(scene_lease_ended)
    if not all(
        payload is not None
        for payload in (
            start_payload,
            scene_lease_started_payload,
            qualification_start_payload,
            qualification_result_payload,
            cleanup_payload,
            dispatched_payload,
            execution_result_payload,
            scene_lease_ended_payload,
        )
    ):
        return CandidateExecutionReceiptResult(
            False, ("required event payload missing",), None, None
        )
    assert start_payload is not None
    assert scene_lease_started_payload is not None
    assert qualification_start_payload is not None
    assert qualification_result_payload is not None
    assert cleanup_payload is not None
    assert dispatched_payload is not None
    assert execution_result_payload is not None
    assert scene_lease_ended_payload is not None

    if scene_lease_started_payload.get("truth_source") is not False:
        errors.append("adaptive collision-scene lease was not perception-only")
    lease_duration = scene_lease_started_payload.get("maximum_duration_sec")
    if (
        not isinstance(lease_duration, (int, float))
        or isinstance(lease_duration, bool)
        or not math.isfinite(float(lease_duration))
        or float(lease_duration) <= 0.0
    ):
        errors.append("adaptive collision-scene lease duration is invalid")
    if scene_lease_ended_payload.get("released") is not True:
        errors.append("adaptive collision-scene lease was not released")

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
        timestamped[SCENE_LEASE_STARTED]
        < timestamped[ADAPTIVE_START]
        < timestamped[QUALIFICATION_START]
        < timestamped[QUALIFICATION_RESULT]
        < timestamped[QUALIFICATION_CLEANUP]
        < timestamped[EXECUTION_DISPATCHED]
        < timestamped[EXECUTION_RESULT]
        < timestamped[SCENE_LEASE_ENDED]
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
    initial_execution_succeeded = execution_result_payload.get("success") is True

    correction_event_types = {
        CENTERING_START,
        CENTERING_QUALIFICATION_START,
        CENTERING_EVALUATION,
        CENTERING_QUALIFICATION_RESULT,
        CENTERING_CLEANUP,
        CENTERING_DISPATCHED,
        CENTERING_RESULT,
    }
    correction_events = [
        event
        for event in events
        if event.get("event_type") in correction_event_types
        and _target_matches(event, target_id)
    ]
    if initial_execution_succeeded:
        if correction_events:
            errors.append("successful initial execution unexpectedly retried centering")
    elif not correction_events:
        errors.append("authorized candidate execution did not succeed")
    else:
        correction_required = {
            name: _one(events, name, target_id)
            for name in (
                CENTERING_START,
                CENTERING_QUALIFICATION_START,
                CENTERING_QUALIFICATION_RESULT,
                CENTERING_CLEANUP,
                CENTERING_DISPATCHED,
                CENTERING_RESULT,
            )
        }
        for name, event in correction_required.items():
            if event is None:
                errors.append(f"expected exactly one {name} event for target")
        if all(event is not None for event in correction_required.values()):
            correction_payloads = {
                name: _payload(event)
                for name, event in correction_required.items()
                if event is not None
            }
            if any(payload is None for payload in correction_payloads.values()):
                errors.append("contact-centering event payload missing")
            else:
                centering_start = correction_required[CENTERING_START]
                centering_qualification_start = correction_required[
                    CENTERING_QUALIFICATION_START
                ]
                centering_qualification_result = correction_required[
                    CENTERING_QUALIFICATION_RESULT
                ]
                centering_cleanup = correction_required[CENTERING_CLEANUP]
                centering_dispatched = correction_required[CENTERING_DISPATCHED]
                centering_result = correction_required[CENTERING_RESULT]
                assert centering_start is not None
                assert centering_qualification_start is not None
                assert centering_qualification_result is not None
                assert centering_cleanup is not None
                assert centering_dispatched is not None
                assert centering_result is not None
                cp = correction_payloads
                start_payload = cp[CENTERING_START]
                cqs_payload = cp[CENTERING_QUALIFICATION_START]
                cqr_payload = cp[CENTERING_QUALIFICATION_RESULT]
                ccleanup_payload = cp[CENTERING_CLEANUP]
                cdispatch_payload = cp[CENTERING_DISPATCHED]
                cresult_payload = cp[CENTERING_RESULT]
                assert start_payload is not None
                assert cqs_payload is not None
                assert cqr_payload is not None
                assert ccleanup_payload is not None
                assert cdispatch_payload is not None
                assert cresult_payload is not None

                correction_times = [
                    _event_time(event)
                    for event in (
                        centering_start,
                        centering_qualification_start,
                        centering_qualification_result,
                        centering_cleanup,
                        centering_dispatched,
                        centering_result,
                    )
                ]
                initial_result_ns = _event_time(execution_result)
                if (
                    initial_result_ns is None
                    or any(value is None for value in correction_times)
                    or not (
                        initial_result_ns
                        < correction_times[0]
                        < correction_times[1]
                        < correction_times[2]
                        < correction_times[3]
                        < correction_times[4]
                        < correction_times[5]
                        < timestamped[SCENE_LEASE_ENDED]
                    )
                ):
                    errors.append("contact-centering authorization order is invalid")

                raw_candidate_ids = start_payload.get("corrected_candidate_ids")
                raw_fingerprints = start_payload.get(
                    "corrected_geometry_fingerprints"
                )
                raw_scales = start_payload.get("correction_scale_factors")
                candidate_ids = (
                    list(raw_candidate_ids)
                    if isinstance(raw_candidate_ids, list)
                    else []
                )
                candidate_fingerprints = (
                    list(raw_fingerprints)
                    if isinstance(raw_fingerprints, list)
                    else []
                )
                correction_scales = (
                    list(raw_scales) if isinstance(raw_scales, list) else []
                )
                reauthorization_strategy = start_payload.get(
                    "reauthorization_strategy", "CENTERING_TRANSLATION"
                )
                if (
                    not 1 <= len(candidate_ids) <= 4
                    or len(candidate_fingerprints) != len(candidate_ids)
                    or any(not isinstance(value, str) or not value for value in candidate_ids)
                    or any(
                        not isinstance(value, str) or not value
                        for value in candidate_fingerprints
                    )
                    or len(set(candidate_ids)) != len(candidate_ids)
                    or len(set(candidate_fingerprints)) != len(candidate_fingerprints)
                ):
                    errors.append("contact-centering candidate ladder is invalid")
                    candidate_ids = []
                    candidate_fingerprints = []
                if reauthorization_strategy == "CENTERING_TRANSLATION":
                    frozen_scales = [1.0, 0.75, 0.5, 0.25][
                        :len(correction_scales)
                    ]
                    if (
                        len(correction_scales) != len(candidate_ids)
                        or correction_scales != frozen_scales
                    ):
                        errors.append(
                            "contact-centering scale order differs from contract"
                        )
                elif reauthorization_strategy == "NEXT_UNTRIED_ORIENTATION":
                    if correction_scales:
                        errors.append(
                            "untried-orientation fallback reported translation scales"
                        )
                    initial_ids = qualification_start_payload.get("candidate_ids")
                    expected_remaining: list[object] = []
                    if isinstance(initial_ids, list) and selected_candidate in initial_ids:
                        selected_index = initial_ids.index(selected_candidate)
                        expected_remaining = initial_ids[selected_index + 1:selected_index + 5]
                    if candidate_ids != expected_remaining:
                        errors.append(
                            "untried-orientation fallback differs from frozen suffix"
                        )
                else:
                    errors.append("contact reauthorization strategy is invalid")
                first_corrected_candidate = (
                    candidate_ids[0] if candidate_ids else None
                )
                correction_offset = start_payload.get("local_y_offset_m")
                if start_payload.get("reauthorization_attempt") != 1:
                    errors.append("contact-centering attempt was not exactly one")
                if start_payload.get("maximum_reauthorizations") != 1:
                    errors.append("contact-centering retry bound was not one")
                if start_payload.get("live_payload_state") != "EMPTY":
                    errors.append("contact-centering started with non-empty payload")
                if start_payload.get("recovery_disposition") != "AT_HOME":
                    errors.append("contact-centering started before safe home recovery")
                if start_payload.get("source_candidate_id") != selected_candidate:
                    errors.append("contact-centering source candidate differs from certificate")
                if (
                    start_payload.get("source_geometry_fingerprint")
                    != selected_fingerprint
                ):
                    errors.append("contact-centering source geometry differs from certificate")
                if start_payload.get("contact_class") not in {
                    "LEFT_SINGLE_FRUIT",
                    "RIGHT_SINGLE_FRUIT",
                }:
                    errors.append("contact-centering lacks unique single-sided contact")
                if (
                    not isinstance(correction_offset, (int, float))
                    or isinstance(correction_offset, bool)
                    or not math.isfinite(float(correction_offset))
                    or not 0.001 <= abs(float(correction_offset)) <= 0.010
                ):
                    errors.append("contact-centering offset is outside frozen bounds")
                if (
                    first_corrected_candidate is not None
                    and start_payload.get("corrected_candidate_id")
                    != first_corrected_candidate
                ):
                    errors.append("contact-centering first candidate alias is inconsistent")
                if (
                    candidate_fingerprints
                    and start_payload.get("corrected_geometry_fingerprint")
                    != candidate_fingerprints[0]
                ):
                    errors.append("contact-centering first fingerprint alias is inconsistent")
                if selected_candidate in candidate_ids:
                    errors.append("contact-centering reused the source candidate ID")
                if selected_fingerprint in candidate_fingerprints:
                    errors.append("contact-centering reused the source geometry fingerprint")

                for key, expected in (
                    ("source_candidate_id", selected_candidate),
                    ("source_geometry_fingerprint", selected_fingerprint),
                    ("contact_class", start_payload.get("contact_class")),
                    ("local_y_offset_m", correction_offset),
                ):
                    initial_key = {
                        "source_candidate_id":
                            "contact_centering_source_candidate_id",
                        "source_geometry_fingerprint":
                            "contact_centering_source_geometry_fingerprint",
                        "contact_class": "contact_centering_contact_class",
                        "local_y_offset_m":
                            "contact_centering_local_y_offset_m",
                    }[key]
                    if execution_result_payload.get(initial_key) != expected:
                        errors.append(f"initial execution {key} differs from centering evidence")
                if execution_result_payload.get("failure_code") != "GRASP_FAILED":
                    errors.append("centering was not triggered by GRASP_FAILED")
                if execution_result_payload.get("payload_state") != "EMPTY":
                    errors.append("failed initial grasp did not retain EMPTY payload")
                if execution_result_payload.get("recovery_disposition") != "AT_HOME":
                    errors.append("failed initial grasp did not recover home")

                if cqs_payload.get("mode") != "CONTACT_CENTERING_REAUTHORIZATION":
                    errors.append("corrected candidate used the wrong qualification mode")
                if cqs_payload.get("live_payload_state") != "EMPTY":
                    errors.append("corrected candidate qualification payload was not EMPTY")
                if (
                    cqs_payload.get("required_first_candidate_id")
                    != first_corrected_candidate
                ):
                    errors.append("corrected qualification did not bind its first candidate")
                if cqs_payload.get("candidate_ids") != candidate_ids:
                    errors.append("corrected qualification candidate IDs differ from ladder")
                if (
                    cqs_payload.get("candidate_geometry_fingerprints")
                    != candidate_fingerprints
                ):
                    errors.append("corrected qualification geometry differs from ladder")
                if cqs_payload.get("candidate_count") != len(candidate_ids):
                    errors.append("corrected qualification candidate count is inconsistent")
                for counter in (
                    "controller_commands_during_evaluation",
                    "gripper_commands_during_evaluation",
                    "physical_attach_during_evaluation",
                ):
                    if cqs_payload.get(counter) != 0:
                        errors.append(f"corrected qualification reports non-zero {counter}")

                corrected_certificate = cqr_payload.get("certificate_fingerprint")
                corrected_scene = cqr_payload.get("scene_signature_before")
                corrected_candidate = cqr_payload.get("selected_candidate_id")
                corrected_fingerprint = cqr_payload.get("geometry_fingerprint")
                if (
                    cqr_payload.get("status") != "PLAN_ONLY_CERTIFIED"
                    or cqr_payload.get("feasible") is not True
                    or cqr_payload.get("scene_isolated") is not True
                    or cqr_payload.get("execution_dispatched") is not False
                ):
                    errors.append("corrected qualification was not isolated and certified")
                if corrected_candidate not in candidate_ids:
                    errors.append("corrected certificate selected a candidate outside ladder")
                    corrected_candidate = None
                selected_correction_index = (
                    candidate_ids.index(corrected_candidate)
                    if corrected_candidate in candidate_ids
                    else None
                )
                if (
                    selected_correction_index is None
                    or corrected_fingerprint
                    != candidate_fingerprints[selected_correction_index]
                ):
                    errors.append("corrected certificate geometry is inconsistent")
                    corrected_fingerprint = None

                correction_start_ns = _event_time(centering_qualification_start)
                correction_result_ns = _event_time(centering_qualification_result)
                correction_evaluations = [
                    event
                    for event in events
                    if event.get("event_type") == CENTERING_EVALUATION
                    and _target_matches(event, target_id)
                    and correction_start_ns is not None
                    and correction_result_ns is not None
                    and correction_start_ns
                    < (_event_time(event) or -1)
                    < correction_result_ns
                ]
                correction_evaluations.sort(
                    key=lambda event: _event_time(event) or -1
                )
                evaluation_payloads = [
                    _payload(event) for event in correction_evaluations
                ]
                if (
                    not evaluation_payloads
                    or any(payload is None for payload in evaluation_payloads)
                ):
                    errors.append("corrected candidate evaluation trace is missing")
                else:
                    evaluated_ids = [
                        payload.get("candidate_id")
                        for payload in evaluation_payloads
                        if payload is not None
                    ]
                    evaluated_fingerprints = [
                        payload.get("geometry_fingerprint")
                        for payload in evaluation_payloads
                        if payload is not None
                    ]
                    if evaluated_ids != candidate_ids[:len(evaluated_ids)]:
                        errors.append("corrected candidates were not evaluated in frozen order")
                    if (
                        evaluated_fingerprints
                        != candidate_fingerprints[:len(evaluated_fingerprints)]
                    ):
                        errors.append("corrected evaluation fingerprints differ from ladder")
                    if corrected_candidate is not None:
                        if evaluated_ids[-1] != corrected_candidate:
                            errors.append("corrected search did not stop at selected candidate")
                        elif (
                            evaluation_payloads[-1].get("result") != "FEASIBLE"
                            or evaluation_payloads[-1].get("feasible") is not True
                        ):
                            errors.append("selected corrected candidate was not FEASIBLE")
                    if any(
                        payload.get("feasible") is True
                        for payload in evaluation_payloads[:-1]
                        if payload is not None
                    ):
                        errors.append("corrected search skipped an earlier feasible candidate")
                if not isinstance(corrected_certificate, str) or not corrected_certificate:
                    errors.append("corrected certificate fingerprint is missing")
                    corrected_certificate = None
                if not isinstance(corrected_scene, str) or not corrected_scene:
                    errors.append("corrected scene signature is missing")
                    corrected_scene = None

                correction_cleanup_ns = _event_time(centering_cleanup)
                correction_restore_events = [
                    event
                    for event in events
                    if event.get("event_type") == TARGET_RESTORED
                    and _target_matches(event, target_id)
                    and _payload(event) is not None
                    and _payload(event).get("restored") is True
                    and correction_result_ns is not None
                    and correction_cleanup_ns is not None
                    and correction_result_ns
                    < (_event_time(event) or -1)
                    < correction_cleanup_ns
                ]
                if not correction_restore_events:
                    errors.append("corrected target collision was not restored before dispatch")
                if ccleanup_payload.get("target_collision_restored") is not True:
                    errors.append("corrected qualification cleanup did not restore target")
                if ccleanup_payload.get("execution_dispatched") is not False:
                    errors.append("corrected cleanup claimed early execution")

                for payload, label in (
                    (cdispatch_payload, "corrected dispatch"),
                    (cresult_payload, "corrected execution result"),
                ):
                    for key, expected in (
                        ("candidate_id", corrected_candidate),
                        ("geometry_fingerprint", corrected_fingerprint),
                        ("certificate_fingerprint", corrected_certificate),
                    ):
                        if expected is not None and payload.get(key) != expected:
                            errors.append(f"{label} {key} differs from certificate")
                if (
                    corrected_scene is not None
                    and cdispatch_payload.get("scene_signature") != corrected_scene
                ):
                    errors.append("corrected dispatch scene differs from certificate")
                if cdispatch_payload.get("execution_dispatched") is not True:
                    errors.append("corrected certificate was not dispatched")
                if cresult_payload.get("success") is not True:
                    errors.append("corrected authorized execution did not succeed")
                if cresult_payload.get("further_reauthorization_permitted") is not False:
                    errors.append("corrected execution did not close the retry bound")

                centering_start_ns = _event_time(centering_start)
                centering_dispatch_ns = _event_time(centering_dispatched)
                if centering_start_ns is not None and centering_dispatch_ns is not None:
                    centering_premature_commands = [
                        event
                        for event in events
                        if event.get("event_type") in COMMAND_EVENTS
                        and (
                            _event_time(event) is None
                            or centering_start_ns
                            < _event_time(event)
                            < centering_dispatch_ns
                        )
                    ]
                    if centering_premature_commands:
                        errors.append(
                            "controller command occurred during centering qualification"
                        )
                if corrected_candidate is not None:
                    selected_candidate = corrected_candidate
                if corrected_certificate is not None:
                    certificate_fingerprint = corrected_certificate

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
            "CONTACT_CENTERING_REAUTHORIZATION_DENIED",
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
