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
    invalid_physical_ids = sorted(
        (set(contact_ids) | set(placed_ids)) - set(fruits)
    )
    unripe_pick_ids = [
        target_id
        for target_id in contact_ids
        if target_id in fruits and fruits[target_id]["maturity"] != "RIPE"
    ]
    failures = terminal.get("failures", [])
    if not isinstance(failures, list):
        raise ValueError("terminal failures must be a list")
    collision_count = 0
    joint_limit_violation_count = 0
    for raw in failures:
        failure = _mapping(raw, "failure")
        if int(failure.get("failure_code", 0)) == 7:
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
    accepted_success_rate = (
        len(set(harvested_track_ids)) / len(accepted_track_ids)
        if accepted_track_ids
        else 1.0
    )
    evidence_errors = []
    if unmatched_motion_track_ids:
        evidence_errors.append("acted runtime tracks could not be scored")
    if invalid_physical_ids:
        evidence_errors.append("physical score events reference unknown fruit")
    if not set(placed_ids).issubset(contact_ids):
        evidence_errors.append("placed fruit lacks contact-resolved evidence")
    if len(set(harvested_track_ids)) != len(placed_ids):
        evidence_errors.append("runtime harvest count disagrees with physical placement")
    if not set(accepted_track_ids).issuperset(harvested_track_ids):
        evidence_errors.append("harvested track lacks a recorded pick attempt")
    score = {
        "schema_version": 1,
        "scope": "GENERALIZED_DEVELOPMENT_RUNTIME_SCORE",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "terminal_status_received": terminal_received,
        "terminal_outcome": terminal.get("outcome"),
        "cleanup_clean": cleanup.get("outcome") == "CLEAN",
        "truth_isolation_pass": truth_audit.get("overall_pass") is True,
        "accepted_track_ids": accepted_track_ids,
        "motion_track_ids": motion_track_ids,
        "runtime_to_sim_target_id": {
            str(key): value for key, value in sorted(mapping.items())
        },
        "association_distance_m": {
            str(key): value for key, value in sorted(distances.items())
        },
        "contact_resolved_sim_target_ids": contact_ids,
        "placed_sim_target_ids": placed_ids,
        "harvested_track_ids": harvested_track_ids,
        "harvested_distinct_count": len(set(harvested_track_ids)),
        "accepted_target_count": len(accepted_track_ids),
        "accepted_target_success_rate": accepted_success_rate,
        "unripe_pick_count": len(unripe_pick_ids),
        "unripe_pick_sim_target_ids": unripe_pick_ids,
        "collision_count": collision_count,
        "joint_limit_violation_count": joint_limit_violation_count,
        "unsafe_motion_attempt_count": len(unsafe_motion_tracks),
        "unsafe_motion_track_ids": unsafe_motion_tracks,
        "reobservations_by_track_id": {
            str(key): value for key, value in sorted(reobservations.items())
        },
        "maximum_reobservations_for_one_target": max(
            reobservations.values(), default=0
        ),
        "evidence_errors": evidence_errors,
        "evidence_integrity_pass": not evidence_errors,
    }
    score["run_safety_pass"] = bool(
        terminal_received
        and score["cleanup_clean"]
        and score["truth_isolation_pass"]
        and score["evidence_integrity_pass"]
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
    accepted = sum(int(row["accepted_target_count"]) for row in scores)
    harvested = sum(int(row["harvested_distinct_count"]) for row in scores)
    success_rate = harvested / accepted if accepted else 0.0
    gates = {
        "five_terminal_and_clean": all(
            row.get("terminal_status_received") is True
            and row.get("cleanup_clean") is True
            for row in scores
        ),
        "truth_isolation": all(row.get("truth_isolation_pass") is True for row in scores),
        "evidence_integrity": all(row.get("evidence_integrity_pass") is True for row in scores),
        "zero_unripe_picks": sum(int(row["unripe_pick_count"]) for row in scores) == 0,
        "zero_collisions": sum(int(row["collision_count"]) for row in scores) == 0,
        "zero_joint_limit_violations": sum(
            int(row["joint_limit_violation_count"]) for row in scores
        ) == 0,
        "zero_unsafe_motion_attempts": sum(
            int(row["unsafe_motion_attempt_count"]) for row in scores
        ) == 0,
        "four_of_five_multi_fruit": sum(
            int(row["harvested_distinct_count"]) >= 2 for row in scores
        ) >= 4,
        "accepted_target_success_rate": success_rate >= 0.80,
        "bounded_reobservation": all(
            int(row["maximum_reobservations_for_one_target"]) <= 1
            for row in scores
        ),
    }
    return {
        "schema_version": 1,
        "scope": "GENERALIZED_DEVELOPMENT_MULTI_FRUIT_GATE",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "run_count": len(scores),
        "runs_with_two_or_more_harvests": sum(
            int(row["harvested_distinct_count"]) >= 2 for row in scores
        ),
        "accepted_target_count": accepted,
        "harvested_target_count": harvested,
        "accepted_target_success_rate": success_rate,
        "gates": gates,
        "overall_pass": all(gates.values()),
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
