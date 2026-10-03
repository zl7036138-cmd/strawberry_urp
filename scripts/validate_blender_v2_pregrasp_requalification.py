#!/usr/bin/env python3
"""Validate the frozen controller-free v2 pre-grasp planning result."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping


EXPECTED_GATE = "blender_v2_pregrasp_planning_post_contact_v1"
EXPECTED_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_PREGRASP_PLANNING"
EXPECTED_BINDINGS = {
    "acceptance_decision",
    "contact_runtime_result",
    "grasp_geometry_config",
    "grasp_geometry_loader",
    "pregrasp_shadow",
    "oracle_handoff",
    "validator",
    "runner",
}
EXPECTED_COLLISIONS = {
    "collection_bin",
    "strawberry_fruit_1",
    "strawberry_fruit_2",
    "strawberry_fruit_3",
    "strawberry_plant_crown",
    "strawberry_planter",
    "work_table",
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
        or raw.get("status") != "FROZEN_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected pre-grasp requalification contract")
    resolved = {
        "decision_record": bound_path(
            root, raw["decision_record"], "decision_record"
        )
    }
    if set(raw.get("bindings", {})) != EXPECTED_BINDINGS:
        raise ValueError("pre-grasp bindings changed")
    for label, record in raw["bindings"].items():
        resolved[label] = bound_path(root, record, label)
    if raw.get("parameters") != {
        "expected_target_id": 1,
        "target_topic": "/strawberry/oracle/target_pose",
        "camera_mount": "dual",
        "expected_grasp_profile": "blender_v2_26mm",
        "expected_tool_center_offset_m": 0.0964,
        "pregrasp_offset_m": 0.15,
        "planning_attempts": 3,
        "minimum_target_samples": 15,
        "minimum_joint_samples": 10,
        "maximum_target_age_sec": 0.5,
        "maximum_joint_delta_rad": 0.002,
    }:
        raise ValueError("pre-grasp parameters changed")
    if raw.get("thresholds") != {
        "required_collision_count": 7,
        "maximum_moveit_state_delta_rad": 0.002,
        "require_selected_collision_retained": True,
        "require_trajectory_discarded": True,
        "required_control_command_count": 0,
    }:
        raise ValueError("pre-grasp thresholds changed")
    safety = raw.get("safety", {})
    if any(
        safety.get(key) is not False
        for key in (
            "formal_acceptance",
            "held_out_test_access_authorized",
            "robot_motion_authorized",
            "gripper_command_authorized",
            "attachment_authorized",
            "perception_authorized",
            "localization_authorized",
            "manipulation_authorized",
            "orchestrator_authorized",
            "trajectory_execution_authorized",
            "pick_action_authorized",
        )
    ) or safety.get("oracle_target_authorized") is not True:
        raise ValueError("unsafe pre-grasp authorization")
    if raw.get("execution") != {
        "maximum_trials": 1,
        "retry_authorized": False,
        "output_directory": (
            "results/development/"
            "blender_v2_pregrasp_planning_post_contact_v1"
        ),
    }:
        raise ValueError("pre-grasp execution boundary changed")
    raw["_contract_path"] = str(contract_path)
    raw["_contract_sha256"] = sha256(contract_path)
    raw["_resolved_paths"] = {
        label: str(value) for label, value in resolved.items()
    }
    return raw


def evaluate_pregrasp(
    contract: Mapping[str, object],
    pregrasp: Mapping[str, object],
) -> list[str]:
    violations: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            violations.append(message)

    parameters = contract["parameters"]
    thresholds = contract["thresholds"]
    require(pregrasp.get("planning_passed") is True, "planning did not pass")
    require(pregrasp.get("pick_authorized") is False, "pick was authorized")
    require(
        int(pregrasp.get("expected_target_id", 0))
        == int(parameters["expected_target_id"]),
        "target identity changed",
    )
    require(
        pregrasp.get("observed_target_ids") == [1],
        "observed target identity changed",
    )
    require(
        int(pregrasp.get("target_sample_count", 0))
        >= int(parameters["minimum_target_samples"]),
        "insufficient target samples",
    )
    require(
        int(pregrasp.get("joint_sample_count", 0))
        >= int(parameters["minimum_joint_samples"]),
        "insufficient joint samples",
    )
    require(
        pregrasp.get("grasp_geometry_profile")
        == parameters["expected_grasp_profile"],
        "wrong grasp geometry profile",
    )
    pose = pregrasp.get("pregrasp_pose") or {}
    require(
        float(pose.get("tool_center_offset_m", math.nan))
        == float(parameters["expected_tool_center_offset_m"]),
        "wrong tool-centre offset",
    )
    require(
        float(pose.get("pregrasp_offset_m", math.nan))
        == float(parameters["pregrasp_offset_m"]),
        "wrong pre-grasp offset",
    )
    collision = pregrasp.get("collision_scene") or {}
    expected_ids = set(collision.get("expected_ids", []))
    before_ids = set(collision.get("before_ids", []))
    after_ids = set(collision.get("after_ids", []))
    require(
        expected_ids == EXPECTED_COLLISIONS,
        "expected collision identity set changed",
    )
    require(
        before_ids == EXPECTED_COLLISIONS
        and after_ids == EXPECTED_COLLISIONS,
        "collision objects were missing before or after planning",
    )
    require(
        collision.get("selected_fruit_collision_retained") is True,
        "selected fruit collision was removed",
    )
    require(
        float(pregrasp.get("observed_joint_delta_rad", math.inf))
        <= float(parameters["maximum_joint_delta_rad"]),
        "observed joints moved",
    )
    require(
        float(pregrasp.get("moveit_state_delta_rad", math.inf))
        <= float(thresholds["maximum_moveit_state_delta_rad"]),
        "MoveIt state changed during planning",
    )
    require(
        pregrasp.get("trajectory_generated") is True,
        "no valid trajectory was generated",
    )
    require(
        pregrasp.get("trajectory_discarded") is True,
        "trajectory was not explicitly discarded",
    )
    require(
        pregrasp.get("trajectory_executed") is False,
        "trajectory unexpectedly executed",
    )
    require(
        int(pregrasp.get("control_command_count", -1))
        == int(thresholds["required_control_command_count"]),
        "control command count changed",
    )
    require(
        pregrasp.get("controller_configuration_keys") == [],
        "controller configuration leaked into Shadow",
    )
    require(
        pregrasp.get("control_interface_created") is False,
        "control interface was created",
    )
    require(
        pregrasp.get("gripper_command_sent") is False,
        "gripper command was sent",
    )
    require(
        pregrasp.get("pick_action_called") is False,
        "pick action was called",
    )
    require(pregrasp.get("violations") == [], "Shadow reported violations")
    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--pregrasp", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    contract = load_contract(args.contract, args.repository_root)
    pregrasp = json.loads(args.pregrasp.read_text(encoding="utf-8"))
    violations = evaluate_pregrasp(contract, pregrasp)
    summary = {
        "schema_version": 1,
        "gate": contract["gate_id"],
        "scope": contract["scope"],
        "passed": not violations,
        "formal_acceptance": False,
        "held_out_test_accessed": False,
        "pick_authorized": False,
        "robot_motion_started": False,
        "gripper_command_started": False,
        "attachment_started": False,
        "perception_started": False,
        "localization_started": False,
        "manipulation_started": False,
        "orchestrator_started": False,
        "oracle_target_started": True,
        "planning_started": True,
        "trajectory_execution_started": False,
        "pick_action_started": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "contract": fingerprint(Path(contract["_contract_path"])),
        "resolved_inputs": {
            label: fingerprint(Path(path))
            for label, path in contract["_resolved_paths"].items()
        },
        "pregrasp_result": fingerprint(args.pregrasp),
        "planning_attempts": pregrasp.get("planning_attempts"),
        "pregrasp_pose": pregrasp.get("pregrasp_pose"),
        "collision_scene": pregrasp.get("collision_scene"),
        "observed_joint_delta_rad": pregrasp.get(
            "observed_joint_delta_rad"
        ),
        "moveit_state_delta_rad": pregrasp.get("moveit_state_delta_rad"),
        "trajectory_generated": pregrasp.get("trajectory_generated"),
        "trajectory_discarded": pregrasp.get("trajectory_discarded"),
        "control_command_count": pregrasp.get("control_command_count"),
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
