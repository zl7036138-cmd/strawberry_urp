"""Strict scoring for the non-formal five-scene multi-fruit runtime gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


MOTION_OUTCOME_PREFIXES = (
    "OBSERVATION_MOTION_",
    "REOBSERVATION_MOTION_",
    "PICK_SENT",
)
TERMINAL_OUTCOMES = {"SUCCESS", "PARTIAL_SUCCESS", "FAILED", "NO_PICK"}
EVIDENCE_PASS = "PASS"
EVIDENCE_FAIL = "FAIL"
EVIDENCE_INDETERMINATE = "INDETERMINATE"


def _mapping(value, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _positive_int(value, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a positive integer")
    result = int(value)
    if result <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return result


def _position(value, label: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} must contain three coordinates")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must be finite")
    return result


def _scene_fruits(scene: Mapping[str, object]) -> dict[int, dict]:
    rows = scene.get("fruits")
    if not isinstance(rows, list) or not rows:
        raise ValueError("scene fruits must be a non-empty list")
    fruits = {}
    for index, raw in enumerate(rows):
        row = _mapping(raw, f"fruits[{index}]")
        target_id = _positive_int(row.get("target_id"), "fruit target_id")
        maturity = str(row.get("maturity", "")).upper()
        if maturity not in {"RIPE", "UNRIPE"}:
            raise ValueError("fruit maturity must be RIPE or UNRIPE")
        reachable = row.get("reachable_by_construction")
        if not isinstance(reachable, bool):
            raise ValueError(
                "generalized development scoring requires reachability truth"
            )
        if target_id in fruits:
            raise ValueError("scene fruit target IDs must be unique")
        fruits[target_id] = {
            "target_id": target_id,
            "maturity": maturity,
            "reachable_by_construction": reachable,
            "position_m": _position(
                row.get("initial_pose_m"), f"fruit {target_id} initial_pose_m"
            ),
        }
    return fruits


def match_runtime_tracks_to_scene(
    selection_events: Sequence[Mapping[str, object]],
    scene: Mapping[str, object],
    *,
    track_ids: Sequence[int],
    maximum_distance_m: float = 0.05,
) -> tuple[dict[int, int], dict[int, float]]:
    """Greedily bind acted runtime tracks to distinct truth fruit for scoring.

    This function is called only after the run.  Its result never reaches a
    runtime topic, service, action, or selection decision.
    """

    if not math.isfinite(maximum_distance_m) or maximum_distance_m <= 0.0:
        raise ValueError("maximum scoring association distance must be positive")
    wanted = {_positive_int(value, "runtime track_id") for value in track_ids}
    first_position = {}
    for raw in selection_events:
        event = _mapping(raw, "selection event")
        if event.get("outcome") != "TARGET_SELECTED":
            continue
        track_id = _positive_int(event.get("track_id"), "selection track_id")
        if track_id in wanted and track_id not in first_position:
            first_position[track_id] = _position(
                event.get("position_m"), "selection position_m"
            )
    fruits = _scene_fruits(scene)
    pairs = sorted(
        (
            math.dist(position, fruit["position_m"]),
            track_id,
            target_id,
        )
        for track_id, position in first_position.items()
        for target_id, fruit in fruits.items()
    )
    mapping = {}
    distances = {}
    assigned_fruits = set()
    for distance, track_id, target_id in pairs:
        if distance > maximum_distance_m:
            break
        if track_id in mapping or target_id in assigned_fruits:
            continue
        mapping[track_id] = target_id
        distances[track_id] = distance
        assigned_fruits.add(target_id)
    return mapping, distances


def _unique_ids(events, event_name: str) -> list[int]:
    values = []
    for raw in events:
        event = _mapping(raw, "ground-truth score event")
        if event.get("event") != event_name:
            continue
        if event.get("scope") != "SIMULATION_GROUND_TRUTH_SCORING_ONLY":
            raise ValueError("ground-truth score event has an invalid scope")
        target_id = _positive_int(event.get("target_id"), "score target_id")
        if target_id not in values:
            values.append(target_id)
    return values


def _combined_evidence_status(statuses: Sequence[str]) -> str:
    if EVIDENCE_FAIL in statuses:
        return EVIDENCE_FAIL
    if statuses and all(status == EVIDENCE_PASS for status in statuses):
        return EVIDENCE_PASS
    return EVIDENCE_INDETERMINATE


def _threshold_evidence_status(
    threshold_passed: bool, evidence_statuses: Sequence[str]
) -> str:
    """Combine a numeric/check result with the evidence needed to claim it."""

    if not threshold_passed:
        return EVIDENCE_FAIL
    return _combined_evidence_status(evidence_statuses)


def _physical_harvests(
    events: Sequence[Mapping[str, object]],
) -> tuple[list[int], list[str]]:
    """Return ordered completed physical lifecycles and their contradictions."""

    completed = []
    errors = []
    pending_contact = None
    for index, raw in enumerate(events):
        event = _mapping(raw, f"ground-truth score event[{index}]")
        event_name = event.get("event")
        if event_name not in {"CONTACT_RESOLVED", "PLACED"}:
            continue
        if event.get("scope") != "SIMULATION_GROUND_TRUTH_SCORING_ONLY":
            raise ValueError("ground-truth score event has an invalid scope")
        target_id = _positive_int(event.get("target_id"), "score target_id")
        if event_name == "CONTACT_RESOLVED":
            if pending_contact is not None:
                errors.append(
                    "physical contact lifecycle restarted before placement"
                )
            pending_contact = target_id
            continue
        if pending_contact is None:
            errors.append("physical placement lacks preceding contact resolution")
        elif pending_contact != target_id:
            errors.append("physical contact and placement identities disagree")
        completed.append(target_id)
        pending_contact = None
    if pending_contact is not None:
        errors.append("physical contact lifecycle lacks terminal placement state")
    return completed, errors


def score_runtime_run(
    receipt: Mapping[str, object],
    scene: Mapping[str, object],
    cleanup: Mapping[str, object],
    truth_audit: Mapping[str, object],
    *,
    maximum_association_distance_m: float = 0.05,
) -> dict:
    """Return one fail-closed, machine-readable development run score."""

    if int(receipt.get("schema_version", 0)) != 3:
        raise ValueError("runtime receipt must use schema v3")
    events = receipt.get("events")
    selection_events = receipt.get("selection_events")
    score_events = receipt.get("ground_truth_score_events")
    if not all(isinstance(value, list) for value in (events, selection_events, score_events)):
        raise ValueError("runtime receipt event collections must be lists")
    if receipt.get("ground_truth_score_events_used_for_control") is not False:
        raise ValueError("score-event truth/control isolation is not explicit")
    terminal = events[-1] if events else None
    terminal_received = bool(
        receipt.get("terminal_status_received") is True
        and isinstance(terminal, Mapping)
        and terminal.get("state") == "DONE"
    )
    terminal = {} if terminal is None else _mapping(terminal, "terminal event")
    terminal_errors = []
    terminal_unknowns = []
    terminal_outcome = terminal.get("outcome")
    if not terminal_received:
        terminal_errors.append("receipt lacks a final DONE terminal event")
    if not isinstance(terminal_outcome, str) or not terminal_outcome:
        terminal_unknowns.append("terminal outcome is missing")
    elif terminal_outcome not in TERMINAL_OUTCOMES:
        terminal_errors.append("terminal outcome is not a batch terminal outcome")
    recorder_outcome = receipt.get("recorder_outcome")
    if not isinstance(recorder_outcome, str) or not recorder_outcome:
        terminal_unknowns.append("recorder outcome is missing")
    elif recorder_outcome != terminal_outcome:
        terminal_errors.append("recorder and terminal outcomes disagree")
    terminal_history = terminal.get("state_history")
    if not isinstance(terminal_history, list) or not terminal_history:
        terminal_unknowns.append("terminal state history is missing")
    else:
        terminal_transition = _mapping(
            terminal_history[-1], "terminal state transition"
        )
        if terminal_transition.get("state") != "DONE":
            terminal_errors.append("state history lacks a final DONE transition")
        if terminal_transition.get("outcome") != terminal_outcome:
            terminal_errors.append("terminal event and state history outcomes disagree")
    accepted_track_ids = []
    motion_track_ids = []
    for raw in events:
        event = _mapping(raw, "harvest event")
        outcome = str(event.get("outcome", ""))
        target_id = event.get("current_target_id")
        if target_id is None:
            continue
        target_id = _positive_int(target_id, "harvest current_target_id")
        if outcome == "PICK_SENT" and target_id not in accepted_track_ids:
            accepted_track_ids.append(target_id)
        if outcome.startswith(MOTION_OUTCOME_PREFIXES) and target_id not in motion_track_ids:
            motion_track_ids.append(target_id)
    mapping, distances = match_runtime_tracks_to_scene(
        selection_events,
        scene,
        track_ids=motion_track_ids,
        maximum_distance_m=maximum_association_distance_m,
    )
    fruits = _scene_fruits(scene)
    unmatched_motion_track_ids = sorted(set(motion_track_ids) - set(mapping))
    unsafe_motion_tracks = [
        track_id
        for track_id in motion_track_ids
        if track_id in mapping
        and (
            fruits[mapping[track_id]]["maturity"] != "RIPE"
            or not fruits[mapping[track_id]]["reachable_by_construction"]
        )
    ]
    contact_ids = _unique_ids(score_events, "CONTACT_RESOLVED")
    placed_ids = _unique_ids(score_events, "PLACED")
    physical_harvest_ids, physical_lifecycle_errors = _physical_harvests(
        score_events
    )
    invalid_physical_ids = sorted(
        (set(contact_ids) | set(placed_ids)) - set(fruits)
    )
    declared_maturity_errors = []
    for raw in score_events:
        event = _mapping(raw, "ground-truth score event")
        if event.get("event") not in {"CONTACT_RESOLVED", "PLACED"}:
            continue
        target_id = _positive_int(event.get("target_id"), "score target_id")
        declared_maturity = str(event.get("maturity", "")).upper()
        if target_id in fruits and declared_maturity != fruits[target_id]["maturity"]:
            declared_maturity_errors.append(
                f"physical event maturity disagrees with scene for fruit {target_id}"
            )
    unripe_pick_ids = [
        target_id
        for target_id in contact_ids
        if target_id in fruits and fruits[target_id]["maturity"] != "RIPE"
    ]
    failures = terminal.get("failures", [])
    if not isinstance(failures, list):
        raise ValueError("terminal failures must be a list")
    safe_collision_rejection_target_ids = set()
    for raw in events:
        event = _mapping(raw, "harvest event")
        state_history = event.get("state_history", [])
        if not isinstance(state_history, list):
            raise ValueError("harvest event state_history must be a list")
        for raw_transition in state_history:
            transition = _mapping(raw_transition, "harvest state transition")
            if transition.get("outcome") != "FINAL_PICK_FEASIBILITY_FAILED_SKIPPED":
                continue
            safe_collision_rejection_target_ids.add(
                _positive_int(
                    transition.get("target_id"), "collision rejection target_id"
                )
            )
    collision_count = 0
    collision_rejection_count = 0
    joint_limit_violation_count = 0
    for raw in failures:
        failure = _mapping(raw, "failure")
        if int(failure.get("failure_code", 0)) == 7:
            target_id = _positive_int(
                failure.get("target_id"), "collision failure target_id"
            )
            if target_id in safe_collision_rejection_target_ids:
                collision_rejection_count += 1
            else:
                collision_count += 1
        if "joint limit" in str(failure.get("message", "")).lower():
            joint_limit_violation_count += 1
    reobservations = {}
    for raw in events:
        event = _mapping(raw, "harvest event")
        if event.get("outcome") != "CACHED_DISTINCT_REOBSERVATION":
            continue
        diagnostics = _mapping(event.get("diagnostics", {}), "diagnostics")
        target_id = _positive_int(
            diagnostics.get("target_id"), "reobservation target_id"
        )
        reobservations[target_id] = reobservations.get(target_id, 0) + 1
    harvested_track_ids = terminal.get("harvested_target_ids", [])
    if not isinstance(harvested_track_ids, list):
        raise ValueError("terminal harvested_target_ids must be a list")
    harvested_track_ids = [
        _positive_int(value, "harvested track_id") for value in harvested_track_ids
    ]
    skipped_targets = terminal.get("skipped_targets")
    if skipped_targets is None:
        terminal_unknowns.append("terminal skipped-target state is missing")
        skipped_target_count = None
    elif not isinstance(skipped_targets, Mapping):
        terminal_errors.append("terminal skipped-target state is not a mapping")
        skipped_target_count = None
    else:
        skipped_target_count = len(skipped_targets)
    if skipped_target_count is not None:
        if harvested_track_ids and skipped_target_count:
            expected_terminal_outcome = "PARTIAL_SUCCESS"
        elif harvested_track_ids:
            expected_terminal_outcome = "SUCCESS"
        elif skipped_target_count:
            expected_terminal_outcome = "FAILED"
        else:
            expected_terminal_outcome = "NO_PICK"
        if terminal_outcome != expected_terminal_outcome:
            terminal_errors.append(
                "terminal outcome contradicts harvested and skipped target state"
            )

    physical_harvest_correspondence = []
    correspondence_count = max(
        len(harvested_track_ids), len(physical_harvest_ids)
    )
    for index in range(correspondence_count):
        track_id = (
            harvested_track_ids[index]
            if index < len(harvested_track_ids)
            else None
        )
        physical_target_id = (
            physical_harvest_ids[index]
            if index < len(physical_harvest_ids)
            else None
        )
        mapped_target_id = mapping.get(track_id) if track_id is not None else None
        physical_harvest_correspondence.append(
            {
                "runtime_track_id": track_id,
                "mapped_sim_target_id": mapped_target_id,
                "physical_sim_target_id": physical_target_id,
                "identity_match": bool(
                    track_id is not None
                    and mapped_target_id is not None
                    and physical_target_id is not None
                    and mapped_target_id == physical_target_id
                ),
            }
        )
    mapped_harvested_ids = [
        mapping[track_id]
        for track_id in harvested_track_ids
        if track_id in mapping
    ]
    identity_correct_harvest_ids = sorted(
        {
            int(row["physical_sim_target_id"])
            for row in physical_harvest_correspondence
            if row["identity_match"]
        }
    )
    accepted_success_rate = (
        len(identity_correct_harvest_ids) / len(accepted_track_ids)
        if accepted_track_ids
        else None
    )
    identity_errors = []
    if unmatched_motion_track_ids:
        identity_errors.append("acted runtime tracks could not be scored")
    if invalid_physical_ids:
        identity_errors.append("physical score events reference unknown fruit")
    identity_errors.extend(declared_maturity_errors)
    if not set(placed_ids).issubset(contact_ids):
        identity_errors.append("placed fruit lacks contact-resolved evidence")
    if len(harvested_track_ids) != len(set(harvested_track_ids)):
        identity_errors.append("terminal repeats a harvested runtime track")
    if len(physical_harvest_ids) != len(set(physical_harvest_ids)):
        identity_errors.append("physical score events repeat a harvested fruit")
    if len(harvested_track_ids) != len(physical_harvest_ids):
        identity_errors.append(
            "runtime harvest count disagrees with physical harvest lifecycles"
        )
    if not set(accepted_track_ids).issuperset(harvested_track_ids):
        identity_errors.append("harvested track lacks a recorded pick attempt")
    if any(
        not row["identity_match"] for row in physical_harvest_correspondence
    ):
        identity_errors.append(
            "mapped runtime track and physical harvest identities disagree"
        )
    if set(mapped_harvested_ids) != set(placed_ids):
        identity_errors.append(
            "mapped harvested fruit set disagrees with physical placement set"
        )

    terminal_evidence_status = (
        EVIDENCE_FAIL
        if terminal_errors
        else EVIDENCE_INDETERMINATE
        if terminal_unknowns
        else EVIDENCE_PASS
    )
    physical_identity_status = (
        EVIDENCE_FAIL if identity_errors else EVIDENCE_PASS
    )
    reported_recovery_failure = any(
        _mapping(raw, "harvest event").get("outcome")
        == "RECOVERY_HOME_FAILED"
        for raw in events
    )
    # Legacy failure records remain useful diagnostics, but absence of a
    # terminal string is not independent proof of collision-free motion,
    # joint-limit compliance, or a verified home/stop state.
    collision_evidence_status = (
        EVIDENCE_FAIL if collision_count else EVIDENCE_INDETERMINATE
    )
    joint_limit_evidence_status = (
        EVIDENCE_FAIL
        if joint_limit_violation_count
        else EVIDENCE_INDETERMINATE
    )
    home_stop_evidence_status = (
        EVIDENCE_FAIL
        if reported_recovery_failure
        else EVIDENCE_INDETERMINATE
    )
    if physical_lifecycle_errors or (
        harvested_track_ids
        and len(harvested_track_ids) != len(physical_harvest_ids)
    ):
        attachment_terminal_evidence_status = EVIDENCE_FAIL
    elif harvested_track_ids:
        # PLACED is emitted only after the attachment manager independently
        # observes detachment and stable collection-bin contact.
        attachment_terminal_evidence_status = EVIDENCE_PASS
    else:
        attachment_terminal_evidence_status = EVIDENCE_INDETERMINATE
    # The v3 recorder has no independent terminal scene-state snapshot.
    scene_terminal_evidence_status = EVIDENCE_INDETERMINATE
    # Schema-v3 PICK_SENT proves a command was emitted, but not that the
    # production planner accepted it or that physical motion began.
    planner_acceptance_evidence_status = EVIDENCE_INDETERMINATE
    pick_start_evidence_status = EVIDENCE_INDETERMINATE
    physical_state_evidence_status = _combined_evidence_status(
        (
            attachment_terminal_evidence_status,
            scene_terminal_evidence_status,
        )
    )
    safety_evidence_status = _combined_evidence_status(
        (
            terminal_evidence_status,
            physical_identity_status,
            collision_evidence_status,
            joint_limit_evidence_status,
            home_stop_evidence_status,
            attachment_terminal_evidence_status,
            scene_terminal_evidence_status,
        )
    )
    if unripe_pick_ids or unsafe_motion_tracks or max(
        reobservations.values(), default=0
    ) > 1:
        safety_evidence_status = EVIDENCE_FAIL

    evidence_errors = [
        *terminal_errors,
        *physical_lifecycle_errors,
        *identity_errors,
    ]
    evidence_unknowns = [*terminal_unknowns]
    if collision_evidence_status == EVIDENCE_INDETERMINATE:
        evidence_unknowns.append("independent collision telemetry is missing")
    if joint_limit_evidence_status == EVIDENCE_INDETERMINATE:
        evidence_unknowns.append("independent joint-limit telemetry is missing")
    if home_stop_evidence_status == EVIDENCE_INDETERMINATE:
        evidence_unknowns.append("independent home/stop telemetry is missing")
    if attachment_terminal_evidence_status == EVIDENCE_INDETERMINATE:
        evidence_unknowns.append("terminal attachment state is missing")
    if scene_terminal_evidence_status == EVIDENCE_INDETERMINATE:
        evidence_unknowns.append("independent scene terminal state is missing")
    evidence_status = (
        EVIDENCE_FAIL
        if evidence_errors or safety_evidence_status == EVIDENCE_FAIL
        else EVIDENCE_INDETERMINATE
        if evidence_unknowns or safety_evidence_status == EVIDENCE_INDETERMINATE
        else EVIDENCE_PASS
    )
    score = {
        "schema_version": 2,
        "scope": "GENERALIZED_DEVELOPMENT_RUNTIME_SCORE",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "terminal_status_received": terminal_received,
        "terminal_outcome": terminal_outcome,
        "terminal_evidence_status": terminal_evidence_status,
        "cleanup_clean": cleanup.get("outcome") == "CLEAN",
        "truth_isolation_pass": truth_audit.get("overall_pass") is True,
        "accepted_track_ids": accepted_track_ids,
        "pick_commanded_track_ids": accepted_track_ids,
        "accepted_target_count_semantics": "LEGACY_PICK_SENT_COMMAND_COUNT",
        "planner_accepted_target_count": None,
        "planner_acceptance_evidence_status": planner_acceptance_evidence_status,
        "pick_started_target_count": None,
        "pick_start_evidence_status": pick_start_evidence_status,
        "motion_track_ids": motion_track_ids,
        "runtime_to_sim_target_id": {
            str(key): value for key, value in sorted(mapping.items())
        },
        "association_distance_m": {
            str(key): value for key, value in sorted(distances.items())
        },
        "contact_resolved_sim_target_ids": contact_ids,
        "placed_sim_target_ids": placed_ids,
        "physical_harvest_sim_target_ids": physical_harvest_ids,
        "harvested_track_ids": harvested_track_ids,
        "mapped_harvested_sim_target_ids": mapped_harvested_ids,
        "physical_harvest_correspondence": physical_harvest_correspondence,
        "physical_identity_status": physical_identity_status,
        "harvested_distinct_count": len(set(harvested_track_ids)),
        "identity_correct_physical_harvest_ids": identity_correct_harvest_ids,
        "identity_correct_physical_harvest_count": len(
            identity_correct_harvest_ids
        ),
        "terminally_confirmed_harvest_count": None,
        "terminally_confirmed_harvest_status": EVIDENCE_INDETERMINATE,
        "accepted_target_count": len(accepted_track_ids),
        "accepted_target_success_rate": accepted_success_rate,
        "accepted_target_success_rate_status": EVIDENCE_INDETERMINATE,
        "unripe_pick_count": len(unripe_pick_ids),
        "unripe_pick_sim_target_ids": unripe_pick_ids,
        "collision_count": collision_count,
        "collision_rejection_count": collision_rejection_count,
        "collision_evidence_status": collision_evidence_status,
        "joint_limit_violation_count": joint_limit_violation_count,
        "joint_limit_evidence_status": joint_limit_evidence_status,
        "home_stop_evidence_status": home_stop_evidence_status,
        "attachment_terminal_evidence_status": (
            attachment_terminal_evidence_status
        ),
        "scene_terminal_evidence_status": scene_terminal_evidence_status,
        "physical_state_evidence_status": physical_state_evidence_status,
        "safety_evidence_status": safety_evidence_status,
        "unsafe_motion_attempt_count": len(unsafe_motion_tracks),
        "unsafe_motion_track_ids": unsafe_motion_tracks,
        "reobservations_by_track_id": {
            str(key): value for key, value in sorted(reobservations.items())
        },
        "maximum_reobservations_for_one_target": max(
            reobservations.values(), default=0
        ),
        "evidence_errors": evidence_errors,
        "evidence_unknowns": evidence_unknowns,
        "evidence_status": evidence_status,
        "evidence_integrity_pass": evidence_status == EVIDENCE_PASS,
    }
    score["run_safety_pass"] = bool(
        terminal_received
        and score["cleanup_clean"]
        and score["truth_isolation_pass"]
        and score["evidence_integrity_pass"]
        and score["safety_evidence_status"] == EVIDENCE_PASS
        and score["unripe_pick_count"] == 0
        and score["collision_count"] == 0
        and score["joint_limit_violation_count"] == 0
        and score["unsafe_motion_attempt_count"] == 0
        and score["maximum_reobservations_for_one_target"] <= 1
    )
    return score


def summarize_runtime_gate(scores: Sequence[Mapping[str, object]]) -> dict:
    if len(scores) != 5:
        raise ValueError("the development runtime gate requires exactly five runs")
    for index, row in enumerate(scores):
        if int(row.get("schema_version", 0)) != 2:
            raise ValueError(f"development score[{index}] must use schema v2")
    accepted = sum(int(row["accepted_target_count"]) for row in scores)
    harvested = sum(
        int(row["identity_correct_physical_harvest_count"]) for row in scores
    )
    success_rate = harvested / accepted if accepted else 0.0
    raw_threshold_checks = {
        "five_terminal_and_clean": all(
            row.get("terminal_status_received") is True
            and row.get("cleanup_clean") is True
            for row in scores
        ),
        "truth_isolation": all(row.get("truth_isolation_pass") is True for row in scores),
        "evidence_integrity": all(row.get("evidence_integrity_pass") is True for row in scores),
        "run_safety_evidence": all(
            row.get("run_safety_pass") is True
            and row.get("safety_evidence_status") == EVIDENCE_PASS
            for row in scores
        ),
        "zero_unripe_picks": sum(int(row["unripe_pick_count"]) for row in scores) == 0,
        "zero_collisions": sum(int(row["collision_count"]) for row in scores) == 0,
        "zero_joint_limit_violations": sum(
            int(row["joint_limit_violation_count"]) for row in scores
        ) == 0,
        "zero_unsafe_motion_attempts": sum(
            int(row["unsafe_motion_attempt_count"]) for row in scores
        ) == 0,
        "four_of_five_multi_fruit": sum(
            int(row["identity_correct_physical_harvest_count"]) >= 2
            for row in scores
        ) >= 4,
        "accepted_target_success_rate": accepted > 0 and success_rate >= 0.80,
        "bounded_reobservation": all(
            int(row["maximum_reobservations_for_one_target"]) <= 1
            for row in scores
        ),
    }
    gate_statuses = {
        "five_terminal_and_clean": (
            EVIDENCE_PASS
            if raw_threshold_checks["five_terminal_and_clean"]
            else EVIDENCE_FAIL
        ),
        "truth_isolation": (
            EVIDENCE_PASS
            if raw_threshold_checks["truth_isolation"]
            else EVIDENCE_FAIL
        ),
        "evidence_integrity": _combined_evidence_status(
            [str(row.get("evidence_status")) for row in scores]
        ),
        "run_safety_evidence": _combined_evidence_status(
            [str(row.get("safety_evidence_status")) for row in scores]
        ),
        "zero_unripe_picks": _threshold_evidence_status(
            raw_threshold_checks["zero_unripe_picks"],
            [str(row.get("scene_terminal_evidence_status")) for row in scores],
        ),
        "zero_collisions": _threshold_evidence_status(
            raw_threshold_checks["zero_collisions"],
            [str(row.get("collision_evidence_status")) for row in scores],
        ),
        "zero_joint_limit_violations": _threshold_evidence_status(
            raw_threshold_checks["zero_joint_limit_violations"],
            [str(row.get("joint_limit_evidence_status")) for row in scores],
        ),
        "zero_unsafe_motion_attempts": _threshold_evidence_status(
            raw_threshold_checks["zero_unsafe_motion_attempts"],
            [str(row.get("scene_terminal_evidence_status")) for row in scores],
        ),
        "four_of_five_multi_fruit": _threshold_evidence_status(
            raw_threshold_checks["four_of_five_multi_fruit"],
            [
                _combined_evidence_status(
                    (
                        str(row.get("physical_identity_status")),
                        str(row.get("attachment_terminal_evidence_status")),
                        str(row.get("home_stop_evidence_status")),
                        str(row.get("scene_terminal_evidence_status")),
                    )
                )
                for row in scores
            ],
        ),
        "accepted_target_success_rate": _threshold_evidence_status(
            raw_threshold_checks["accepted_target_success_rate"],
            [
                _combined_evidence_status(
                    (
                        str(row.get("planner_acceptance_evidence_status")),
                        str(row.get("pick_start_evidence_status")),
                        str(row.get("terminally_confirmed_harvest_status")),
                    )
                )
                for row in scores
            ],
        ),
        "bounded_reobservation": _threshold_evidence_status(
            raw_threshold_checks["bounded_reobservation"],
            [EVIDENCE_INDETERMINATE for _ in scores],
        ),
    }
    gates = {
        name: status == EVIDENCE_PASS for name, status in gate_statuses.items()
    }
    overall_status = _combined_evidence_status(tuple(gate_statuses.values()))
    return {
        "schema_version": 2,
        "scope": "GENERALIZED_DEVELOPMENT_MULTI_FRUIT_GATE",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "run_count": len(scores),
        "runs_with_two_or_more_harvests": sum(
            int(row["identity_correct_physical_harvest_count"]) >= 2
            for row in scores
        ),
        "accepted_target_count": accepted,
        "identity_correct_physical_harvest_count": harvested,
        "terminally_confirmed_harvest_count": None,
        "accepted_target_success_rate": success_rate,
        "accepted_target_success_rate_status": gate_statuses[
            "accepted_target_success_rate"
        ],
        "raw_threshold_checks": raw_threshold_checks,
        "gate_statuses": gate_statuses,
        "gates": gates,
        "overall_status": overall_status,
        "overall_pass": overall_status == EVIDENCE_PASS,
    }


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    return dict(_mapping(value, str(path)))


def load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment dependency
        raise RuntimeError("development scoring requires PyYAML") from exc
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    return dict(_mapping(value, str(path)))


def write_new_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--cleanup", type=Path, required=True)
    parser.add_argument("--truth-audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args(argv)
    score = score_runtime_run(
        load_json(options.receipt),
        load_yaml(options.scene),
        load_json(options.cleanup),
        load_json(options.truth_audit),
    )
    write_new_json(options.output, score)
    print(json.dumps(score, sort_keys=True))
    return 0 if score["run_safety_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
