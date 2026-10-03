#!/usr/bin/env python3
"""Validate the frozen Blender-v2 perception handoff and planning gate."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping


EXPECTED_EXECUTION = {
    "blender_v2_perception_handoff_pregrasp_v1": {
        "maximum_trials": 1,
        "retry_authorized": False,
        "ros_domain_id": 230,
        "output_directory": (
            "results/development/"
            "blender_v2_perception_handoff_pregrasp_v1"
        ),
    },
    "blender_v2_perception_handoff_pregrasp_v2": {
        "maximum_trials": 1,
        "retry_authorized": False,
        "ros_domain_id": 231,
        "output_directory": (
            "results/development/"
            "blender_v2_perception_handoff_pregrasp_v2"
        ),
    },
    "blender_v2_perception_handoff_pregrasp_v3": {
        "maximum_trials": 1,
        "retry_authorized": False,
        "ros_domain_id": 232,
        "output_directory": (
            "results/development/"
            "blender_v2_perception_handoff_pregrasp_v3"
        ),
    },
}
EXPECTED_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_PERCEPTION_HANDOFF_PREGRASP"
EXPECTED_BINDINGS_V1 = {
    "perception_waiver",
    "model",
    "runner",
    "observation_motion",
    "observation_selector",
    "sequence_summary",
    "perception_node",
    "localization_node",
    "handoff_shadow",
    "pregrasp_shadow",
    "grasp_geometry_loader",
    "grasp_geometry_config",
    "scene_config",
}
EXPECTED_BINDINGS = {
    "blender_v2_perception_handoff_pregrasp_v1": EXPECTED_BINDINGS_V1,
    "blender_v2_perception_handoff_pregrasp_v2": (
        EXPECTED_BINDINGS_V1
        | {
            "moveit_backend",
            "validator",
            "failed_v1_summary",
            "failed_v1_observation_log",
        }
    ),
    "blender_v2_perception_handoff_pregrasp_v3": (
        EXPECTED_BINDINGS_V1
        | {
            "moveit_backend",
            "validator",
            "failed_v1_summary",
            "failed_v1_observation_log",
            "failed_v2_summary",
            "failed_v2_wrist_window",
            "failed_v2_runtime_topics",
        }
    ),
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


def bound_path(root: Path, record: Mapping[str, object], label: str) -> Path:
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
    gate_id = raw.get("gate_id")
    if (
        raw.get("schema_version") != 1
        or gate_id not in EXPECTED_EXECUTION
        or raw.get("scope") != EXPECTED_SCOPE
        or raw.get("status") != "FROZEN_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected perception handoff contract")
    resolved = {
        "decision_record": bound_path(
            root, raw["decision_record"], "decision_record"
        )
    }
    if set(raw.get("bindings", {})) != EXPECTED_BINDINGS[gate_id]:
        raise ValueError("perception handoff bindings changed")
    for label, record in raw["bindings"].items():
        resolved[label] = bound_path(root, record, label)
    expected_parameters = {
        "expected_target_id": 1,
        "expected_wrist_preset": "lower",
        "detections_topic": "/strawberry/shadow/detections",
        "target_topic": "/strawberry/shadow/target_pose",
        "camera_mount": "dual",
        "confidence_threshold": 0.58,
        "base_measurement_frames": 60,
        "minimum_base_support_frames": 48,
        "wrist_measurement_frames": 60,
        "required_matching_wrist_target_pose_frames": 60,
        "minimum_ready_consecutive_frames": 15,
        "minimum_target_samples": 15,
        "minimum_joint_samples": 10,
        "maximum_target_age_sec": 0.5,
        "maximum_joint_delta_rad": 0.002,
        "expected_grasp_profile": "blender_v2_26mm",
        "expected_tool_center_offset_m": 0.0964,
        "pregrasp_offset_m": 0.15,
        "planning_attempts": 3,
    }
    if raw.get("parameters") != expected_parameters:
        raise ValueError("perception handoff parameters changed")
    expected_safety = {
        "formal_acceptance": False,
        "held_out_test_access_authorized": False,
        "oracle_target_authorized": False,
        "perception_authorized": True,
        "localization_authorized": True,
        "observation_pose_motion_authorized": True,
        "manipulation_authorized": False,
        "orchestrator_authorized": False,
        "attachment_authorized": False,
        "gripper_command_authorized": False,
        "trajectory_execution_authorized": False,
        "pick_action_authorized": False,
    }
    if raw.get("safety") != expected_safety:
        raise ValueError("unsafe perception handoff authorization")
    if raw.get("execution") != EXPECTED_EXECUTION[gate_id]:
        raise ValueError("perception handoff execution boundary changed")
    raw["_contract_path"] = str(contract_path)
    raw["_contract_sha256"] = sha256(contract_path)
    raw["_resolved_paths"] = {
        label: str(value) for label, value in resolved.items()
    }
    return raw


def _require(
    violations: list[str], condition: bool, message: str
) -> None:
    if not condition:
        violations.append(message)


def evaluate(
    contract: Mapping[str, object],
    sequence: Mapping[str, object],
    observation_motion: Mapping[str, object],
    wrist_window: Mapping[str, object],
    handoff: Mapping[str, object],
    pregrasp: Mapping[str, object],
    runtime_nodes: list[str],
    runtime_topics: list[str],
) -> list[str]:
    violations: list[str] = []
    parameters = contract["parameters"]
    target_id = int(parameters["expected_target_id"])
    target_topic = str(parameters["target_topic"])

    _require(
        violations,
        observation_motion.get("success") is True,
        "wrist observation-pose motion did not pass",
    )
    _require(
        violations,
        observation_motion.get("camera_mount") == parameters["camera_mount"],
        "observation motion used the wrong camera mount",
    )
    _require(
        violations,
        observation_motion.get("robot_motion_started") is True
        and observation_motion.get("fruit_manipulation_started") is False,
        "observation motion crossed the fruit-manipulation boundary",
    )
    _require(
        violations,
        all(
            receipt.get("collision") is False
            for receipt in observation_motion.get("planning_attempts", [])
        ),
        "observation motion encountered a collision",
    )
    wrist_summary = wrist_window.get("summary") or {}
    readiness = wrist_window.get("readiness_gate") or {}
    _require(
        violations,
        wrist_window.get("completed") is True,
        "wrist measurement window did not complete",
    )
    _require(
        violations,
        int(wrist_summary.get("frame_count", 0))
        == int(parameters["wrist_measurement_frames"])
        and int(wrist_summary.get("frames_with_target_pose", 0))
        == int(parameters["required_matching_wrist_target_pose_frames"]),
        "wrist receipt does not prove complete target-pose coverage",
    )
    _require(
        violations,
        readiness.get("satisfied") is True
        and int(readiness.get("required_consecutive_target_pose_frames", 0))
        == int(parameters["minimum_ready_consecutive_frames"]),
        "wrist readiness gate did not pass",
    )
    _require(
        violations,
        all(
            frame.get("target_pose_ids") == [target_id]
            for frame in wrist_window.get("frames", [])
        ),
        "wrist receipt contains an unexpected target identity",
    )
    _require(
        violations,
        wrist_window.get("formal_acceptance") is False
        and wrist_window.get("held_out_test_consumed") is False,
        "wrist receipt crossed an acceptance boundary",
    )
    _require(
        violations,
        sequence.get("sequence_passed") is True,
        "dual-camera sequence did not pass",
    )
    _require(
        violations,
        int(sequence.get("candidate_target_id", 0)) == target_id,
        "base camera selected the wrong target",
    )
    _require(
        violations,
        sequence.get("selected_preset")
        == parameters["expected_wrist_preset"],
        "wrist preset changed",
    )
    _require(
        violations,
        int(sequence.get("base_frame_count", 0))
        == int(parameters["base_measurement_frames"]),
        "base measurement frame count changed",
    )
    _require(
        violations,
        int(sequence.get("base_support_frames", 0))
        >= int(parameters["minimum_base_support_frames"]),
        "base target support is insufficient",
    )
    _require(
        violations,
        int(sequence.get("wrist_frame_count", 0))
        == int(parameters["wrist_measurement_frames"]),
        "wrist measurement frame count changed",
    )
    _require(
        violations,
        int(sequence.get("wrist_matching_target_pose_frames", 0))
        == int(parameters["required_matching_wrist_target_pose_frames"]),
        "wrist target pose coverage is incomplete",
    )
    _require(
        violations,
        sequence.get("handoff_shadow_passed") is True,
        "sequence handoff did not pass",
    )
    _require(
        violations,
        sequence.get("pregrasp_shadow_passed") is True,
        "sequence pre-grasp planning did not pass",
    )
    _require(
        violations,
        sequence.get("pick_authorized") is False,
        "sequence authorized a pick",
    )
    _require(
        violations,
        sequence.get("formal_acceptance") is False,
        "sequence claimed formal acceptance",
    )
    _require(
        violations,
        sequence.get("held_out_test_consumed") is False,
        "sequence consumed the held-out test",
    )
    _require(
        violations,
        sequence.get("violations") == [],
        "sequence reported violations",
    )

    _require(
        violations,
        handoff.get("handoff_passed") is True,
        "handoff did not pass",
    )
    _require(
        violations,
        int(handoff.get("expected_target_id", 0)) == target_id
        and handoff.get("observed_target_ids") == [target_id],
        "handoff target identity changed",
    )
    _require(
        violations,
        handoff.get("target_topic") == target_topic,
        "handoff target topic changed",
    )
    _require(
        violations,
        int(handoff.get("target_sample_count", 0))
        >= int(parameters["minimum_target_samples"]),
        "handoff target samples are insufficient",
    )
    _require(
        violations,
        int(handoff.get("joint_sample_count", 0))
        >= int(parameters["minimum_joint_samples"]),
        "handoff joint samples are insufficient",
    )
    age = handoff.get("target_age_sec") or {}
    _require(
        violations,
        float(age.get("maximum", math.inf))
        <= float(parameters["maximum_target_age_sec"]),
        "handoff target became stale",
    )
    _require(
        violations,
        float(handoff.get("observed_joint_delta_rad", math.inf))
        <= float(parameters["maximum_joint_delta_rad"])
        and float(handoff.get("moveit_state_delta_rad", math.inf))
        <= float(parameters["maximum_joint_delta_rad"]),
        "arm moved during handoff",
    )
    handoff_collision = handoff.get("collision_scene") or {}
    _require(
        violations,
        set(handoff_collision.get("observed_ids", []))
        == EXPECTED_COLLISIONS,
        "handoff collision scene is incomplete",
    )
    _require(
        violations,
        handoff_collision.get("selected_fruit_collision_retained") is True,
        "handoff removed selected-fruit collision",
    )
    for field in (
        "pick_action_called",
        "trajectory_command_sent",
        "gripper_command_sent",
        "pick_authorized",
    ):
        _require(
            violations,
            handoff.get(field) is False,
            f"handoff safety field {field} changed",
        )
    _require(
        violations,
        handoff.get("violations") == [],
        "handoff reported violations",
    )

    _require(
        violations,
        pregrasp.get("planning_passed") is True,
        "pre-grasp planning did not pass",
    )
    _require(
        violations,
        int(pregrasp.get("expected_target_id", 0)) == target_id
        and pregrasp.get("observed_target_ids") == [target_id],
        "planning target identity changed",
    )
    _require(
        violations,
        pregrasp.get("target_topic") == target_topic,
        "planning target topic changed",
    )
    _require(
        violations,
        pregrasp.get("grasp_geometry_profile")
        == parameters["expected_grasp_profile"],
        "wrong grasp geometry profile",
    )
    pose = pregrasp.get("pregrasp_pose") or {}
    _require(
        violations,
        float(pose.get("tool_center_offset_m", math.nan))
        == float(parameters["expected_tool_center_offset_m"]),
        "wrong tool-centre offset",
    )
    _require(
        violations,
        float(pose.get("pregrasp_offset_m", math.nan))
        == float(parameters["pregrasp_offset_m"]),
        "wrong pre-grasp offset",
    )
    pregrasp_collision = pregrasp.get("collision_scene") or {}
    _require(
        violations,
        set(pregrasp_collision.get("before_ids", []))
        == EXPECTED_COLLISIONS
        and set(pregrasp_collision.get("after_ids", []))
        == EXPECTED_COLLISIONS,
        "planning collision scene is incomplete",
    )
    _require(
        violations,
        pregrasp_collision.get("selected_fruit_collision_retained") is True,
        "planning removed selected-fruit collision",
    )
    _require(
        violations,
        pregrasp.get("trajectory_generated") is True
        and pregrasp.get("trajectory_discarded") is True
        and pregrasp.get("trajectory_executed") is False
        and pregrasp.get("trajectory_execution_requested") is False,
        "planning trajectory crossed the execution boundary",
    )
    _require(
        violations,
        int(pregrasp.get("control_command_count", -1)) == 0
        and pregrasp.get("control_interface_created") is False
        and pregrasp.get("controller_configuration_keys") == []
        and pregrasp.get("pick_action_called") is False
        and pregrasp.get("gripper_command_sent") is False
        and pregrasp.get("pick_authorized") is False,
        "planning created a control path",
    )
    _require(
        violations,
        pregrasp.get("violations") == [],
        "pre-grasp planning reported violations",
    )

    _require(
        violations,
        "/strawberry/oracle/target_pose" not in runtime_topics,
        "Oracle target topic existed at runtime",
    )
    _require(
        violations,
        not any(
            "oracle" in node.lower() or "orchestrator" in node.lower()
            for node in runtime_nodes
        ),
        "Oracle or orchestrator node existed at runtime",
    )
    return list(dict.fromkeys(violations))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository-root", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    contract = load_contract(args.contract, args.repository_root)
    inputs = {
        "sequence": args.run_directory / "sequence_summary.json",
        "observation_motion": (
            args.run_directory / "observation_motion.json"
        ),
        "wrist_window": args.run_directory / "wrist_window.json",
        "handoff": args.run_directory / "handoff_shadow.json",
        "pregrasp": args.run_directory / "pregrasp_shadow.json",
        "runtime_nodes": args.run_directory / "wrist_runtime_nodes.txt",
        "runtime_topics": args.run_directory / "wrist_runtime_topics.txt",
    }
    missing = [
        f"missing {label}: {path}"
        for label, path in inputs.items()
        if not path.is_file()
    ]
    sequence = (
        json.loads(inputs["sequence"].read_text(encoding="utf-8"))
        if inputs["sequence"].is_file()
        else {}
    )
    observation_motion = (
        json.loads(
            inputs["observation_motion"].read_text(encoding="utf-8")
        )
        if inputs["observation_motion"].is_file()
        else {}
    )
    wrist_window = (
        json.loads(inputs["wrist_window"].read_text(encoding="utf-8"))
        if inputs["wrist_window"].is_file()
        else {}
    )
    handoff = (
        json.loads(inputs["handoff"].read_text(encoding="utf-8"))
        if inputs["handoff"].is_file()
        else {}
    )
    pregrasp = (
        json.loads(inputs["pregrasp"].read_text(encoding="utf-8"))
        if inputs["pregrasp"].is_file()
        else {}
    )
    runtime_nodes = (
        inputs["runtime_nodes"].read_text(encoding="utf-8").splitlines()
        if inputs["runtime_nodes"].is_file()
        else []
    )
    runtime_topics = (
        inputs["runtime_topics"].read_text(encoding="utf-8").splitlines()
        if inputs["runtime_topics"].is_file()
        else []
    )
    violations = missing or evaluate(
        contract,
        sequence,
        observation_motion,
        wrist_window,
        handoff,
        pregrasp,
        runtime_nodes,
        runtime_topics,
    )
    summary = {
        "schema_version": 1,
        "gate": contract["gate_id"],
        "scope": contract["scope"],
        "passed": not violations,
        "formal_acceptance": False,
        "held_out_test_accessed": False,
        "pick_authorized": False,
        "oracle_target_started": False,
        "perception_started": True,
        "localization_started": True,
        "observation_pose_motion_started": True,
        "manipulation_started": False,
        "orchestrator_started": False,
        "attachment_started": False,
        "trajectory_execution_started": False,
        "pick_action_started": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "contract": fingerprint(Path(contract["_contract_path"])),
        "resolved_inputs": {
            label: fingerprint(Path(path))
            for label, path in contract["_resolved_paths"].items()
        },
        "run_inputs": {
            label: fingerprint(path)
            for label, path in inputs.items()
            if path.is_file()
        },
        "candidate_target_id": sequence.get("candidate_target_id"),
        "selected_preset": sequence.get("selected_preset"),
        "observation_motion": observation_motion,
        "wrist_window_summary": wrist_window.get("summary"),
        "wrist_readiness_gate": wrist_window.get("readiness_gate"),
        "base_support_frames": sequence.get("base_support_frames"),
        "wrist_matching_target_pose_frames": sequence.get(
            "wrist_matching_target_pose_frames"
        ),
        "handoff_target_sample_count": handoff.get("target_sample_count"),
        "handoff_maximum_target_age_sec": (
            handoff.get("target_age_sec") or {}
        ).get("maximum"),
        "handoff_observed_joint_delta_rad": handoff.get(
            "observed_joint_delta_rad"
        ),
        "handoff_moveit_state_delta_rad": handoff.get(
            "moveit_state_delta_rad"
        ),
        "grasp_geometry_profile": pregrasp.get("grasp_geometry_profile"),
        "pregrasp_pose": pregrasp.get("pregrasp_pose"),
        "planning_attempts": pregrasp.get("planning_attempts"),
        "trajectory_generated": pregrasp.get("trajectory_generated"),
        "trajectory_discarded": pregrasp.get("trajectory_discarded"),
        "control_command_count": pregrasp.get("control_command_count"),
        "runtime_nodes": runtime_nodes,
        "runtime_topics": runtime_topics,
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
