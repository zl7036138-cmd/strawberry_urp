#!/usr/bin/env python3
"""Validate the frozen single Blender-v2 Oracle execution round trip."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping


EXPECTED_GATE = "blender_v2_oracle_execution_round_trip_v1"
EXPECTED_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_ORACLE_EXECUTION"
EXPECTED_STAGES = [
    "PLAN",
    "APPROACH",
    "GRASP",
    "RETREAT",
    "PLACE",
    "VERIFY",
    "DONE",
]
READY_JOINTS = {
    "panda_joint1": 0.0,
    "panda_joint2": -0.785,
    "panda_joint3": 0.0,
    "panda_joint4": -2.356,
    "panda_joint5": 0.0,
    "panda_joint6": 1.571,
    "panda_joint7": 0.785,
}
EXPECTED_BINDINGS = {
    "accepted_pregrasp_decision",
    "pregrasp_result",
    "contact_result",
    "scene_manifest",
    "grasp_geometry_config",
    "grasp_geometry_loader",
    "executor_core",
    "action_server",
    "moveit_backend",
    "system_launch",
    "sim_launch",
    "oracle_client",
    "validator",
    "runner",
}
SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprint(path: Path) -> dict[str, object]:
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def bound_path(
    root: Path,
    record: Mapping[str, object],
    label: str,
) -> Path:
    root = root.resolve(strict=True)
    path = (root / str(record.get("path"))).resolve(strict=True)
    if root not in path.parents:
        raise ValueError(f"{label} escapes repository")
    if (
        path.stat().st_size != int(record.get("size_bytes", -1))
        or sha256(path) != record.get("sha256")
    ):
        raise ValueError(f"{label} binding changed")
    return path


def load_contract(path: Path, root: Path) -> dict:
    contract_path = path.resolve(strict=True)
    root = root.resolve(strict=True)
    raw = json.loads(contract_path.read_text(encoding="utf-8"))
    if (
        raw.get("schema_version") != 1
        or raw.get("gate_id") != EXPECTED_GATE
        or raw.get("scope") != EXPECTED_SCOPE
        or raw.get("status") != "FROZEN_SINGLE_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected Oracle execution contract")
    resolved = {
        "decision_record": bound_path(
            root, raw["decision_record"], "decision_record"
        )
    }
    if set(raw.get("bindings", {})) != EXPECTED_BINDINGS:
        raise ValueError("Oracle execution bindings changed")
    for label, record in raw["bindings"].items():
        resolved[label] = bound_path(root, record, label)
    if raw.get("parameters") != {
        "target_id": 1,
        "target_topic": "/strawberry/ground_truth/fruit_1/pose",
        "camera_mount": "dual",
        "grasp_profile": "blender_v2_26mm",
        "tool_center_offset_m": 0.0964,
        "gripper_open_width_m_per_finger": 0.04,
        "gripper_closed_width_m_per_finger": 0.022,
        "place_position_m": [0.35, -0.45, 0.45],
        "stability_samples": 10,
        "stability_tolerance_m": 0.001,
    }:
        raise ValueError("Oracle execution parameters changed")
    if raw.get("thresholds") != {
        "maximum_planning_time_sec": 5.0,
        "maximum_ready_joint_error_rad": 0.02,
        "maximum_home_return_delta_rad": 0.02,
        "maximum_open_recovery_error_m": 0.003,
        "require_processed_dual_contact": True,
        "require_raw_dual_contact": True,
        "require_attach_detach_round_trip": True,
        "require_clean_shutdown": True,
    }:
        raise ValueError("Oracle execution thresholds changed")
    safety = raw.get("safety", {})
    if any(
        safety.get(key) is not False
        for key in (
            "formal_acceptance",
            "held_out_test_access_authorized",
            "perception_authorized",
            "localization_authorized",
            "oracle_provider_authorized",
            "orchestrator_authorized",
            "pose_control_authorized",
            "repeated_execution_authorized",
            "general_pick_authorized",
        )
    ) or any(
        safety.get(key) is not True
        for key in (
            "single_oracle_action_authorized",
            "robot_motion_authorized",
            "gripper_command_authorized",
            "attachment_authorized",
        )
    ):
        raise ValueError("unsafe Oracle execution authorization")
    if raw.get("execution") != {
        "maximum_trials": 1,
        "retry_authorized": False,
        "output_directory": (
            "results/development/"
            "blender_v2_oracle_execution_round_trip_v1"
        ),
    }:
        raise ValueError("Oracle execution boundary changed")
    raw["_contract_path"] = str(contract_path)
    raw["_contract_sha256"] = sha256(contract_path)
    raw["_resolved_paths"] = {
        label: str(value) for label, value in resolved.items()
    }
    return raw


def unexpected_fruit_contacts(trial: Mapping[str, object]) -> list[dict]:
    diagnostics = trial.get("diagnostics") or {}
    events = (diagnostics.get("fruit_contacts") or {}).get(
        "first_pair_events", []
    )
    unexpected = []
    for event in events:
        pair = event.get("pair", [])
        stage = str(event.get("stage", "UNKNOWN"))
        finger_contact = any(
            "panda_leftfinger" in item or "panda_rightfinger" in item
            for item in pair
        )
        released_bin_contact = (
            any("collection_bin" in item for item in pair)
            and stage in {"VERIFY", "DONE"}
        )
        if not finger_contact and not released_bin_contact:
            unexpected.append({"pair": pair, "stage": stage})
    return unexpected


def _attachment_round_trip(events: list[dict]) -> bool:
    states = [
        event.get("attached")
        for event in events
        if isinstance(event.get("attached"), bool)
    ]
    try:
        attached_index = states.index(True)
    except ValueError:
        return False
    return False in states[attached_index + 1 :]


def evaluate_execution(
    contract: Mapping[str, object],
    trial: Mapping[str, object],
    launch_text: str,
) -> tuple[list[str], dict[str, object]]:
    violations: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            violations.append(message)

    parameters = contract["parameters"]
    thresholds = contract["thresholds"]
    require(trial.get("success") is True, "action did not succeed")
    require(trial.get("result_success") is True, "action result was false")
    require(int(trial.get("action_status", -1)) == 4, "action status changed")
    require(int(trial.get("failure_code", -1)) == 0, "failure code was nonzero")
    stages = [entry.get("stage") for entry in trial.get("feedback", [])]
    require(stages == EXPECTED_STAGES, "execution stage sequence changed")
    planning_time = float(trial.get("planning_time_sec", math.inf))
    require(
        planning_time <= float(thresholds["maximum_planning_time_sec"]),
        "planning time exceeded threshold",
    )
    require(
        int(trial.get("target_id", 0)) == int(parameters["target_id"]),
        "target identity changed",
    )
    require(
        trial.get("target_topic") == parameters["target_topic"],
        "target topic changed",
    )
    require(
        trial.get("place_position_m") == parameters["place_position_m"],
        "place position changed",
    )

    diagnostics = trial.get("diagnostics") or {}
    contacts = diagnostics.get("contacts") or {}
    for side in ("left", "right"):
        contact = contacts.get(side) or {}
        require(
            contact.get("processed_seen_true") is True,
            f"{side} processed contact missing",
        )
        require(
            int(contact.get("raw_contacts", 0)) > 0,
            f"{side} raw contact missing",
        )
    unexpected = unexpected_fruit_contacts(trial)
    require(not unexpected, "unexpected fruit contact occurred")

    attachment = diagnostics.get("attachment_state") or {}
    attachment_events = attachment.get("events") or []
    require(
        attachment.get("initial_attached") is False,
        "target was not initially detached",
    )
    require(
        _attachment_round_trip(attachment_events),
        "attach/detach round trip was not observed",
    )
    require(
        attachment.get("final_attached") is False,
        "target remained attached",
    )

    arm = diagnostics.get("arm_recovery") or {}
    initial = arm.get("initial_positions_rad") or {}
    final = arm.get("final_positions_rad") or {}
    require(set(initial) == set(READY_JOINTS), "initial arm state incomplete")
    require(set(final) == set(READY_JOINTS), "final arm state incomplete")
    ready_error = (
        max(abs(float(initial[name]) - value) for name, value in READY_JOINTS.items())
        if set(initial) == set(READY_JOINTS)
        else math.inf
    )
    require(
        ready_error <= float(thresholds["maximum_ready_joint_error_rad"]),
        "initial arm state was not ready",
    )
    home_delta = float(
        arm.get("maximum_initial_final_delta_rad", math.inf)
    )
    require(
        home_delta <= float(thresholds["maximum_home_return_delta_rad"]),
        "final arm state did not return home",
    )

    gripper = diagnostics.get("gripper_recovery") or {}
    final_fingers = gripper.get("final_positions_m") or {}
    expected_open = float(parameters["gripper_open_width_m_per_finger"])
    open_error = (
        max(
            abs(float(final_fingers[name]) - expected_open)
            for name in ("panda_finger_joint1", "panda_finger_joint2")
        )
        if all(
            isinstance(final_fingers.get(name), (int, float))
            for name in ("panda_finger_joint1", "panda_finger_joint2")
        )
        else math.inf
    )
    require(
        open_error <= float(thresholds["maximum_open_recovery_error_m"]),
        "gripper did not reopen",
    )

    profile_marker = (
        "Loaded grasp geometry profile blender_v2_26mm: "
        "tool_center_offset_m=0.0964, "
        "gripper_closed_width_m_per_finger=0.022"
    )
    require(profile_marker in launch_text, "runtime grasp profile was not confirmed")
    lower_log = launch_text.lower()
    shutdown_clean = not any(
        marker in lower_log for marker in SHUTDOWN_ERROR_MARKERS
    )
    require(shutdown_clean, "launch shutdown was not clean")
    return violations, {
        "stages": stages,
        "planning_time_sec": planning_time,
        "unexpected_fruit_contacts": unexpected,
        "attachment_events": attachment_events,
        "initial_ready_error_rad": ready_error,
        "home_return_delta_rad": home_delta,
        "open_recovery_error_m": open_error,
        "shutdown_clean": shutdown_clean,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--launch-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    contract = load_contract(args.contract, args.repository_root)
    trial = json.loads(args.trial.read_text(encoding="utf-8"))
    launch_text = args.launch_log.read_text(
        encoding="utf-8", errors="replace"
    )
    violations, observations = evaluate_execution(
        contract, trial, launch_text
    )
    summary = {
        "schema_version": 1,
        "gate": contract["gate_id"],
        "scope": contract["scope"],
        "passed": not violations,
        "formal_acceptance": False,
        "held_out_test_accessed": False,
        "pick_authorized": False,
        "single_oracle_action_authorized": True,
        "robot_motion_started": True,
        "gripper_command_started": True,
        "attachment_started": True,
        "perception_started": False,
        "localization_started": False,
        "oracle_provider_started": False,
        "manipulation_started": True,
        "orchestrator_started": False,
        "pose_control_started": False,
        "pick_action_started": True,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "contract": fingerprint(Path(contract["_contract_path"])),
        "resolved_inputs": {
            label: fingerprint(Path(path))
            for label, path in contract["_resolved_paths"].items()
        },
        "trial_result": fingerprint(args.trial),
        "observations": observations,
        "action": {
            "success": trial.get("success"),
            "status": trial.get("action_status"),
            "failure_code": trial.get("failure_code"),
            "message": trial.get("message"),
            "planning_time_sec": trial.get("planning_time_sec"),
            "execution_time_sec": trial.get("execution_time_sec"),
        },
        "contacts": (trial.get("diagnostics") or {}).get("contacts"),
        "violations": violations,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, sort_keys=True), flush=True)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
