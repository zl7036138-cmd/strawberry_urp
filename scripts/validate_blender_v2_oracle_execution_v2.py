#!/usr/bin/env python3
"""Validate the measurement-repaired Blender-v2 Oracle round trip."""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Mapping


EXPECTED_GATE = "blender_v2_oracle_execution_round_trip_v2"
EXPECTED_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_ORACLE_EXECUTION_MEASUREMENT_V2"
EXPECTED_BINDINGS = {
    "v1_contract",
    "v1_failed_summary",
    "v1_trial",
    "scene_manifest",
    "grasp_geometry_config",
    "executor_core",
    "action_server",
    "moveit_backend",
    "system_launch",
    "sim_launch",
    "oracle_client",
    "v1_validator",
    "initial_state_probe",
    "validator",
    "runner",
}


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
        or raw.get("status") != "FROZEN_MEASUREMENT_REQUALIFICATION_AUTHORIZED"
    ):
        raise ValueError("unexpected v2 Oracle execution contract")
    resolved = {
        "decision_record": bound_path(
            root, raw["decision_record"], "decision_record"
        )
    }
    if set(raw.get("bindings", {})) != EXPECTED_BINDINGS:
        raise ValueError("v2 Oracle execution bindings changed")
    for label, record in raw["bindings"].items():
        resolved[label] = bound_path(root, record, label)
    expected_parameters = {
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
        "initial_state_samples": 10,
    }
    if raw.get("parameters") != expected_parameters:
        raise ValueError("v2 Oracle execution parameters changed")
    expected_thresholds = {
        "maximum_planning_time_sec": 5.0,
        "maximum_ready_joint_error_rad": 0.02,
        "maximum_home_return_delta_rad": 0.02,
        "maximum_open_recovery_error_m": 0.003,
        "require_processed_dual_contact": True,
        "require_raw_dual_contact": True,
        "require_attach_detach_round_trip": True,
        "require_clean_shutdown": True,
    }
    if raw.get("thresholds") != expected_thresholds:
        raise ValueError("v2 Oracle execution thresholds changed")
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
            "single_measurement_requalification_authorized",
            "robot_motion_authorized",
            "gripper_command_authorized",
            "attachment_authorized",
        )
    ):
        raise ValueError("unsafe v2 Oracle execution authorization")
    if raw.get("execution") != {
        "maximum_trials": 1,
        "retry_authorized": False,
        "output_directory": (
            "results/development/"
            "blender_v2_oracle_execution_round_trip_v2"
        ),
    }:
        raise ValueError("v2 Oracle execution boundary changed")
    raw["_contract_path"] = str(contract_path)
    raw["_resolved_paths"] = {
        label: str(value) for label, value in resolved.items()
    }
    return raw


def _load_v1_validator(root: Path):
    path = root / "scripts" / "validate_blender_v2_oracle_execution.py"
    spec = importlib.util.spec_from_file_location(
        "_frozen_v1_oracle_validator", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def evaluate_v2(
    contract: Mapping[str, object],
    preflight: Mapping[str, object],
    trial: Mapping[str, object],
    launch_text: str,
    *,
    repository_root: Path,
) -> tuple[list[str], dict[str, object]]:
    violations = []
    required_samples = int(contract["parameters"]["initial_state_samples"])
    if preflight.get("passed") is not True:
        violations.append("initial-state preflight did not pass")
    if preflight.get("violations") != []:
        violations.append("initial-state preflight reported violations")
    if int(preflight.get("control_command_count", -1)) != 0:
        violations.append("initial-state preflight sent a control command")
    if int(preflight.get("complete_joint_sample_count", 0)) < required_samples:
        violations.append("insufficient initial-state samples")
    if preflight.get("target_attached") is not False:
        violations.append("target was not initially detached")

    prepared = deepcopy(trial)
    diagnostics = prepared.setdefault("diagnostics", {})
    arm = diagnostics.setdefault("arm_recovery", {})
    gripper = diagnostics.setdefault("gripper_recovery", {})
    attachment = diagnostics.setdefault("attachment_state", {})
    initial_arm = preflight.get("arm_positions_rad") or {}
    final_arm = arm.get("final_positions_rad") or {}
    arm["initial_positions_rad"] = initial_arm
    if set(initial_arm) == set(final_arm) and initial_arm:
        try:
            delta = max(
                abs(float(final_arm[name]) - float(initial_arm[name]))
                for name in initial_arm
            )
            arm["maximum_initial_final_delta_rad"] = (
                delta if math.isfinite(delta) else math.inf
            )
        except (TypeError, ValueError):
            arm["maximum_initial_final_delta_rad"] = math.inf
    else:
        arm["maximum_initial_final_delta_rad"] = math.inf
    gripper["initial_positions_m"] = (
        preflight.get("finger_positions_m") or {}
    )
    attachment["initial_attached"] = preflight.get("target_attached")
    if not isinstance(prepared.get("planning_time_sec"), (int, float)):
        prepared["planning_time_sec"] = math.inf

    v1 = _load_v1_validator(repository_root)
    try:
        base_violations, observations = v1.evaluate_execution(
            contract, prepared, launch_text
        )
    except Exception as exc:
        base_violations = [
            "frozen base validator rejected malformed evidence: "
            f"{type(exc).__name__}: {exc}"
        ]
        observations = {}
    violations.extend(base_violations)
    observations["preflight_ready_error_rad"] = preflight.get(
        "ready_error_rad"
    )
    observations["preflight_complete_joint_sample_count"] = preflight.get(
        "complete_joint_sample_count"
    )
    observations["preflight_target_attached"] = preflight.get(
        "target_attached"
    )
    return violations, observations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--preflight", type=Path, required=True)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--launch-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    root = args.repository_root.resolve(strict=True)
    contract = load_contract(args.contract, root)
    preflight = json.loads(args.preflight.read_text(encoding="utf-8"))
    trial = json.loads(args.trial.read_text(encoding="utf-8"))
    launch_text = args.launch_log.read_text(
        encoding="utf-8", errors="replace"
    )
    violations, observations = evaluate_v2(
        contract,
        preflight,
        trial,
        launch_text,
        repository_root=root,
    )
    summary = {
        "schema_version": 1,
        "gate": contract["gate_id"],
        "scope": contract["scope"],
        "passed": not violations,
        "formal_acceptance": False,
        "held_out_test_accessed": False,
        "pick_authorized": False,
        "single_measurement_requalification_authorized": True,
        "robot_motion_started": bool(trial.get("feedback")),
        "gripper_command_started": bool(trial.get("feedback")),
        "attachment_started": bool(trial.get("feedback")),
        "perception_started": False,
        "localization_started": False,
        "oracle_provider_started": False,
        "manipulation_started": True,
        "orchestrator_started": False,
        "pose_control_started": False,
        "pick_action_started": bool(trial.get("feedback")),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "contract": fingerprint(Path(contract["_contract_path"])),
        "resolved_inputs": {
            label: fingerprint(Path(path))
            for label, path in contract["_resolved_paths"].items()
        },
        "preflight_result": fingerprint(args.preflight),
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
