"""Deterministic receipt checks for ADR 0086 runtime qualification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


EVALUATION_START = "WHOLE_CHAIN_EVALUATION_STARTED"
EVALUATION_RESULT = "WHOLE_CHAIN_EVALUATION_RESULT"
COMMAND_EVENTS = {"COMMAND_PREPARED", "ACTION_ACCEPTED"}


@dataclass(frozen=True)
class QualificationResult:
    passed: bool
    errors: tuple[str, ...]
    evaluation_result: str | None
    first_command_after_evaluation_ns: int | None


def qualify_whole_chain_receipt(
    receipt: Mapping[str, object],
    *,
    expected_result: str,
    expect_motion: bool,
    target_id: int | None = None,
) -> QualificationResult:
    """Check one authorization attempt in an immutable development receipt.

    A continuous harvest can evaluate several targets.  ``target_id`` selects
    one paired evaluation without discarding the surrounding receipt, so the
    time ordering against real controller evidence remains auditable.
    """

    errors: list[str] = []
    motion_events = receipt.get("motion_events")
    if not isinstance(motion_events, list):
        return QualificationResult(False, ("motion_events missing",), None, None)
    def is_selected_evaluation(event: object, event_type: str) -> bool:
        if not isinstance(event, Mapping) or event.get("event_type") != event_type:
            return False
        if target_id is None:
            return True
        payload = event.get("payload")
        return isinstance(payload, Mapping) and payload.get("target_id") == target_id

    starts = [event for event in motion_events
              if is_selected_evaluation(event, EVALUATION_START)]
    results = [event for event in motion_events
               if is_selected_evaluation(event, EVALUATION_RESULT)]
    if len(starts) != 1 or len(results) != 1:
        return QualificationResult(False, ("expected exactly one evaluation start/result",), None, None)
    start, result = starts[0], results[0]
    start_payload = start.get("payload")
    result_payload = result.get("payload")
    if not isinstance(start_payload, Mapping) or not isinstance(result_payload, Mapping):
        return QualificationResult(False, ("evaluation payload missing",), None, None)
    for label, payload in (("start", start_payload), ("result", result_payload)):
        if payload.get("live_payload_state") != "EMPTY":
            errors.append(f"{label} payload was not EMPTY")
        for key in (
            "controller_commands_during_evaluation",
            "gripper_commands_during_evaluation",
            "physical_attach_during_evaluation",
        ):
            if payload.get(key) != 0:
                errors.append(f"{label} reports non-zero {key}")
    if result_payload.get("result") != expected_result:
        errors.append("unexpected evaluation result")
    if result_payload.get("scene_isolated") is not True:
        errors.append("live scene changed during virtual evaluation")
    start_ns = start.get("event_time_monotonic_ns")
    result_ns = result.get("event_time_monotonic_ns")
    if not isinstance(start_ns, int) or not isinstance(result_ns, int) or result_ns <= start_ns:
        errors.append("evaluation timestamps are invalid")
        result_ns = None
    ordered = sorted(
        (event for event in motion_events if isinstance(event, Mapping)),
        key=lambda event: event.get("event_time_monotonic_ns", -1),
    )
    between = [
        event for event in ordered
        if isinstance(start_ns, int) and isinstance(result_ns, int)
        and start_ns < event.get("event_time_monotonic_ns", -1) < result_ns
        and event.get("event_type") in COMMAND_EVENTS
    ]
    if between:
        errors.append("controller/gripper command occurred during evaluation")
    after = [
        event for event in ordered
        if isinstance(result_ns, int)
        and event.get("event_time_monotonic_ns", -1) > result_ns
        and event.get("event_type") in COMMAND_EVENTS
    ]
    first_command_ns = (
        after[0].get("event_time_monotonic_ns") if after else None
    )
    if expect_motion and not isinstance(first_command_ns, int):
        errors.append("feasible authorization had no subsequent command")
    # A rejected action may be followed by an independent recovery-home
    # command.  The receipt cannot attribute that command to the rejected
    # action, so runtime qualification asserts the hard boundary (no command
    # *during* evaluation) and payload/scene invariants here.  The executor's
    # direct unit contract separately proves that an authorization rejection
    # sends no gripper or manipulation command.
    return QualificationResult(
        not errors,
        tuple(errors),
        str(result_payload.get("result")),
        first_command_ns if isinstance(first_command_ns, int) else None,
    )
