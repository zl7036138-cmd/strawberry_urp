#!/usr/bin/env python3
"""Summarize the frozen five-world Blender-v2 Oracle execution repeat."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Mapping


EXPECTED_GATE = "blender_v2_oracle_execution_repeat_5_v1"
EXPECTED_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_ORACLE_EXECUTION_REPEAT"
EXPECTED_BINDINGS = {
    "single_v2_contract",
    "single_v2_summary",
    "scene_manifest",
    "grasp_geometry_config",
    "executor_core",
    "action_server",
    "moveit_backend",
    "system_launch",
    "sim_launch",
    "oracle_client",
    "initial_state_probe",
    "single_v2_validator",
    "runner",
    "summarizer",
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
        or raw.get("status") != "FROZEN_FIVE_TRIAL_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected five-world repeat contract")
    resolved = {
        "decision_record": bound_path(
            root, raw["decision_record"], "decision_record"
        )
    }
    if set(raw.get("bindings", {})) != EXPECTED_BINDINGS:
        raise ValueError("five-world repeat bindings changed")
    for label, record in raw["bindings"].items():
        resolved[label] = bound_path(root, record, label)
    if raw.get("parameters") != {
        "trial_count": 5,
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
    }:
        raise ValueError("five-world repeat parameters changed")
    if raw.get("thresholds") != {
        "required_pass_count": 5,
        "maximum_planning_time_sec": 5.0,
        "maximum_ready_joint_error_rad": 0.02,
        "maximum_home_return_delta_rad": 0.02,
        "maximum_open_recovery_error_m": 0.003,
        "require_processed_dual_contact": True,
        "require_raw_dual_contact": True,
        "require_attach_detach_round_trip": True,
        "require_clean_shutdown": True,
    }:
        raise ValueError("five-world repeat thresholds changed")
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
            "general_pick_authorized",
        )
    ) or safety.get("five_trial_oracle_repeat_authorized") is not True:
        raise ValueError("unsafe five-world repeat authorization")
    if raw.get("execution") != {
        "maximum_trials": 5,
        "per_trial_retry_authorized": False,
        "output_directory": (
            "results/development/"
            "blender_v2_oracle_execution_repeat_5_v1"
        ),
    }:
        raise ValueError("five-world repeat execution boundary changed")
    raw["_contract_path"] = str(contract_path)
    raw["_resolved_paths"] = {
        label: str(value) for label, value in resolved.items()
    }
    return raw


def aggregate_passes(records: list[Mapping[str, object]], expected: int) -> bool:
    return len(records) == expected and all(
        record.get("passed") is True for record in records
    )


def _load_single_validator(root: Path):
    path = root / "scripts" / "validate_blender_v2_oracle_execution_v2.py"
    spec = importlib.util.spec_from_file_location(
        "_single_v2_execution_validator", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    root = args.repository_root.resolve(strict=True)
    contract = load_contract(args.contract, root)
    validator = _load_single_validator(root)
    records = []
    for index in range(1, 6):
        label = f"trial_{index:02d}"
        trial_dir = args.results / label
        preflight_path = trial_dir / "initial_state.json"
        trial_path = trial_dir / "trial.json"
        launch_path = trial_dir / "launch.log"
        violations = []
        observations = {}
        if not all(
            path.exists()
            for path in (preflight_path, trial_path, launch_path)
        ):
            violations.append("trial evidence is incomplete")
            preflight = {}
            trial = {}
        else:
            preflight = json.loads(
                preflight_path.read_text(encoding="utf-8")
            )
            trial = json.loads(trial_path.read_text(encoding="utf-8"))
            launch_text = launch_path.read_text(
                encoding="utf-8", errors="replace"
            )
            violations, observations = validator.evaluate_v2(
                contract,
                preflight,
                trial,
                launch_text,
                repository_root=root,
            )
        records.append(
            {
                "trial_id": label,
                "passed": not violations,
                "action_success": trial.get("success"),
                "planning_time_sec": trial.get("planning_time_sec"),
                "execution_time_sec": trial.get("execution_time_sec"),
                "observations": observations,
                "preflight_result": (
                    fingerprint(preflight_path)
                    if preflight_path.exists()
                    else None
                ),
                "trial_result": (
                    fingerprint(trial_path) if trial_path.exists() else None
                ),
                "violations": violations,
            }
        )
    passed = aggregate_passes(records, 5)
    summary = {
        "schema_version": 1,
        "gate": contract["gate_id"],
        "scope": contract["scope"],
        "passed": passed,
        "formal_acceptance": False,
        "held_out_test_accessed": False,
        "pick_authorized": False,
        "five_trial_oracle_repeat_authorized": True,
        "perception_started": False,
        "localization_started": False,
        "oracle_provider_started": False,
        "manipulation_started": True,
        "orchestrator_started": False,
        "pose_control_started": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "contract": fingerprint(Path(contract["_contract_path"])),
        "resolved_inputs": {
            label: fingerprint(Path(path))
            for label, path in contract["_resolved_paths"].items()
        },
        "trial_count": 5,
        "pass_count": sum(record["passed"] for record in records),
        "required_pass_count": 5,
        "trials": records,
        "violations": (
            []
            if passed
            else ["one or more fixed fresh-world trials failed"]
        ),
    }
    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, sort_keys=True), flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
