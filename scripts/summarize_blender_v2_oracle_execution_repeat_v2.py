#!/usr/bin/env python3
"""Summarize repeat-v2 using the frozen repeat-v1 evaluation logic."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Mapping


EXPECTED_GATE = "blender_v2_oracle_execution_repeat_5_v2"
EXPECTED_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_ORACLE_EXECUTION_REPEAT_V2"
EXPECTED_BINDINGS = {
    "repeat_v1_contract",
    "repeat_v1_infrastructure_failure",
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
    "repeat_v1_runner",
    "repeat_v1_summarizer",
    "runner",
    "summarizer",
}


def _load_base(root: Path):
    path = root / "scripts" / "summarize_blender_v2_oracle_execution_repeat.py"
    spec = importlib.util.spec_from_file_location(
        "_frozen_repeat_v1_summary", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_contract(path: Path, root: Path, base) -> dict:
    contract_path = path.resolve(strict=True)
    root = root.resolve(strict=True)
    raw = json.loads(contract_path.read_text(encoding="utf-8"))
    if (
        raw.get("schema_version") != 1
        or raw.get("gate_id") != EXPECTED_GATE
        or raw.get("scope") != EXPECTED_SCOPE
        or raw.get("status") != "FROZEN_FIVE_TRIAL_EXECUTION_V2_AUTHORIZED"
    ):
        raise ValueError("unexpected five-world repeat-v2 contract")
    resolved = {
        "decision_record": base.bound_path(
            root, raw["decision_record"], "decision_record"
        )
    }
    if set(raw.get("bindings", {})) != EXPECTED_BINDINGS:
        raise ValueError("five-world repeat-v2 bindings changed")
    for label, record in raw["bindings"].items():
        resolved[label] = base.bound_path(root, record, label)
    expected_parameters = {
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
    }
    if raw.get("parameters") != expected_parameters:
        raise ValueError("five-world repeat-v2 parameters changed")
    expected_thresholds = {
        "required_pass_count": 5,
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
        raise ValueError("five-world repeat-v2 thresholds changed")
    if raw.get("execution") != {
        "maximum_trials": 5,
        "per_trial_retry_authorized": False,
        "output_directory": (
            "results/development/"
            "blender_v2_oracle_execution_repeat_5_v2"
        ),
    }:
        raise ValueError("five-world repeat-v2 execution boundary changed")
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
    ) or safety.get("five_trial_oracle_repeat_v2_authorized") is not True:
        raise ValueError("unsafe five-world repeat-v2 authorization")
    raw["_contract_path"] = str(contract_path)
    raw["_resolved_paths"] = {
        label: str(value) for label, value in resolved.items()
    }
    return raw


def main() -> int:
    repository_root = Path(__file__).resolve().parents[1]
    base = _load_base(repository_root)
    original_loader = base.load_contract

    def v2_loader(path: Path, root: Path) -> Mapping[str, object]:
        return load_contract(path, root, base)

    base.load_contract = v2_loader
    try:
        return base.main()
    finally:
        base.load_contract = original_loader


if __name__ == "__main__":
    raise SystemExit(main())
