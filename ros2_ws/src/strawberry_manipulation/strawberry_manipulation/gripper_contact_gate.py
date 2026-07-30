"""Pure contract and summary logic for the v2 gripper contact round trip."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Mapping


GATE_ID = "blender_v2_gripper_contact_round_trip_v1"
GATE_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_GRIPPER_CONTACT_RUNTIME"
EXPECTED_BINDINGS = {
    "scene_manifest",
    "panda_xacro",
    "attachment_manager",
    "sim_launch",
    "controller_config",
    "geometry_contract",
    "geometry_result",
    "contact_gate_logic",
    "runtime_probe",
    "runtime_runner",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bound_path(
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


def load_contact_contract(path: Path, repository_root: Path) -> dict:
    contract_path = path.resolve(strict=True)
    root = repository_root.resolve(strict=True)
    if root not in contract_path.parents:
        raise ValueError("contact contract must be inside repository")
    raw = json.loads(contract_path.read_text(encoding="utf-8"))
    if (
        raw.get("schema_version") != 1
        or raw.get("gate_id") != GATE_ID
        or raw.get("scope") != GATE_SCOPE
        or raw.get("status") != "FROZEN_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected gripper contact contract")
    resolved = {
        "decision_record": _bound_path(
            root, raw["decision_record"], "decision_record"
        )
    }
    if set(raw.get("bindings", {})) != EXPECTED_BINDINGS:
        raise ValueError("gripper contact bindings changed")
    for label, record in raw["bindings"].items():
        resolved[label] = _bound_path(root, record, label)

    parameters = raw.get("parameters", {})
    if parameters != {
        "target_id": 1,
        "target_model": "strawberry_1",
        "tool_center_offset_m": 0.0964,
        "predicted_contact_width_m_per_finger": 0.025857178634063095,
        "gripper_open_width_m_per_finger": 0.04,
        "gripper_close_width_m_per_finger": 0.022,
        "gripper_effort_n": 40.0,
        "geometric_contact_limit_m": 0.029,
        "canonical_restore_position_m": [
            0.419064753,
            -0.053479813,
            0.546111838,
        ],
        "camera_mount": "dual",
    }:
        raise ValueError("gripper contact parameters changed")
    thresholds = raw.get("thresholds", {})
    if thresholds != {
        "maximum_arm_joint_delta_rad": 0.002,
        "maximum_finger_symmetry_error_m": 0.002,
        "maximum_contact_width_error_m": 0.003,
        "maximum_pad_center_distance_m": 0.029,
        "maximum_target_pose_error_m": 0.002,
        "maximum_restore_pose_error_m": 0.002,
        "maximum_open_recovery_error_m": 0.003,
        "required_gripper_command_count": 3,
    }:
        raise ValueError("gripper contact thresholds changed")
    runtime = raw.get("runtime", {})
    if runtime != {
        "headless": True,
        "startup_timeout_sec": 60.0,
        "action_timeout_sec": 20.0,
        "contact_settle_sec": 2.0,
        "shutdown_timeout_sec": 20.0,
    }:
        raise ValueError("gripper contact runtime changed")
    safety = raw.get("safety", {})
    true_keys = (
        "simulated_fruit_motion_authorized",
        "gripper_command_authorized",
        "attachment_authorized",
    )
    false_keys = (
        "formal_acceptance",
        "formal_held_out_test_access_authorized",
        "robot_arm_motion_authorized",
        "planning_authorized",
        "perception_authorized",
        "localization_authorized",
        "orchestrator_authorized",
        "pick_action_authorized",
    )
    if any(safety.get(key) is not True for key in true_keys) or any(
        safety.get(key) is not False for key in false_keys
    ):
        raise ValueError("unsafe gripper contact authorization")
    execution = raw.get("execution", {})
    if execution != {
        "maximum_trials": 1,
        "retry_authorized": False,
        "output_directory": (
            "results/development/blender_v2_gripper_contact_round_trip_v1"
        ),
    }:
        raise ValueError("gripper contact execution boundary changed")

    raw["_contract_path"] = str(contract_path)
    raw["_contract_sha256"] = sha256(contract_path)
    raw["_resolved_paths"] = {
        label: str(resolved_path)
        for label, resolved_path in resolved.items()
    }
    return raw


def summarize_contact_trial(
    *,
    contract: Mapping[str, object],
    observations: Mapping[str, object],
    runtime_errors: list[str],
) -> dict[str, object]:
    thresholds = contract["thresholds"]
    parameters = contract["parameters"]
    violations = list(runtime_errors)

    def require(condition: bool, message: str) -> None:
        if not condition:
            violations.append(message)

    arm_delta = observations.get("maximum_arm_joint_delta_rad")
    require(
        isinstance(arm_delta, (int, float))
        and math.isfinite(float(arm_delta))
        and float(arm_delta)
        <= float(thresholds["maximum_arm_joint_delta_rad"]),
        "arm joint drift exceeded the frozen no-motion limit",
    )
    require(
        int(observations.get("gripper_command_count", -1))
        == int(thresholds["required_gripper_command_count"]),
        "gripper command count changed",
    )
    require(
        observations.get("initial_open_succeeded") is True,
        "initial open command did not succeed",
    )
    require(
        observations.get("close_succeeded_or_stalled") is True,
        "close command did not succeed or stall on contact",
    )
    require(
        observations.get("reopen_succeeded") is True,
        "reopen command did not succeed",
    )
    symmetry = observations.get("closed_finger_symmetry_error_m")
    require(
        isinstance(symmetry, (int, float))
        and float(symmetry)
        <= float(thresholds["maximum_finger_symmetry_error_m"]),
        "closed fingers were asymmetric",
    )
    measured_width = observations.get("measured_contact_width_m_per_finger")
    require(
        isinstance(measured_width, (int, float))
        and abs(
            float(measured_width)
            - float(parameters["predicted_contact_width_m_per_finger"])
        )
        <= float(thresholds["maximum_contact_width_error_m"]),
        "measured contact width disagreed with mesh prediction",
    )
    require(
        observations.get("raw_left_target_contact_seen") is True,
        "raw left-finger target contact was not observed",
    )
    require(
        observations.get("raw_right_target_contact_seen") is True,
        "raw right-finger target contact was not observed",
    )
    require(
        observations.get("non_target_contact_seen") is False,
        "a non-target fruit contact was observed",
    )
    pad_distances = observations.get("pad_center_distances_m", {})
    require(
        isinstance(pad_distances, Mapping)
        and set(pad_distances) == {"left", "right"}
        and all(
            float(pad_distances[side])
            <= float(thresholds["maximum_pad_center_distance_m"])
            for side in ("left", "right")
        ),
        "pad centres exceeded the v2 contact envelope",
    )
    require(
        observations.get("fruit_projected_between_pads") is True,
        "fruit centre did not project between the pads",
    )
    require(
        float(observations.get("target_pose_error_m", math.inf))
        <= float(thresholds["maximum_target_pose_error_m"]),
        "target set-pose verification failed",
    )
    require(
        observations.get("attach_succeeded") is True,
        "guarded attach did not succeed",
    )
    require(
        observations.get("detach_succeeded") is True,
        "guarded detach did not succeed",
    )
    require(
        float(observations.get("restore_pose_error_m", math.inf))
        <= float(thresholds["maximum_restore_pose_error_m"]),
        "canonical fruit restore failed",
    )
    require(
        float(observations.get("open_recovery_error_m", math.inf))
        <= float(thresholds["maximum_open_recovery_error_m"]),
        "gripper did not recover to the open width",
    )
    require(
        observations.get("planning_started") is False,
        "planning unexpectedly started",
    )
    require(
        observations.get("pick_action_started") is False,
        "pick action unexpectedly started",
    )

    return {
        "schema_version": 1,
        "gate": contract["gate_id"],
        "scope": contract["scope"],
        "passed": not violations,
        "formal_acceptance": False,
        "formal_held_out_test_accessed": False,
        "pick_authorized": False,
        "robot_arm_motion_started": False,
        "simulated_fruit_motion_started": True,
        "gripper_command_started": True,
        "attachment_started": True,
        "planning_started": False,
        "perception_started": False,
        "localization_started": False,
        "orchestrator_started": False,
        "pick_action_started": False,
        "observations": dict(observations),
        "violations": violations,
    }
