"""Frozen contract helpers for the Blender-v2 localization accuracy gate."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


INITIAL_GATE_ID = "blender_v2_localization_accuracy_100_v1"
POST_WINDING_GATE_ID = (
    "blender_v2_localization_accuracy_100_post_winding_v1"
)
EXPECTED_SCOPE = "NON_ACCEPTANCE_BLENDER_V2_LOCALIZATION_GEOMETRY"
EXPECTED_X_M = (0.44, 0.47, 0.50, 0.53, 0.56)
EXPECTED_Y_M = (-0.12, -0.075, -0.03, 0.015, 0.06)
EXPECTED_Z_M = (0.51, 0.53, 0.55, 0.57)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _repository_path(root: Path, value: object, label: str) -> Path:
    root = root.resolve(strict=True)
    path = (root / str(value)).resolve(strict=True)
    if path != root and root not in path.parents:
        raise ValueError(f"{label} escapes the repository")
    return path


def _bound_path(root: Path, record: Mapping[str, object], label: str) -> Path:
    path = _repository_path(root, record.get("path"), label)
    if (
        path.stat().st_size != int(record.get("size_bytes", -1))
        or sha256(path) != record.get("sha256")
    ):
        raise ValueError(f"{label} binding changed")
    return path


def _finite_tuple(
    value: object,
    *,
    length: int,
    label: str,
) -> tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != length
    ):
        raise ValueError(f"{label} must contain {length} values")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{label} must be finite")
    return result


def position_grid(contract: Mapping[str, object]) -> tuple[tuple[float, float, float], ...]:
    grid = contract["position_grid"]
    x_values = _finite_tuple(grid["x_m"], length=5, label="position_grid.x_m")
    y_values = _finite_tuple(grid["y_m"], length=5, label="position_grid.y_m")
    z_values = _finite_tuple(grid["z_m"], length=4, label="position_grid.z_m")
    if (
        x_values != EXPECTED_X_M
        or y_values != EXPECTED_Y_M
        or z_values != EXPECTED_Z_M
        or grid.get("order") != "z_then_y_then_x"
    ):
        raise ValueError("Blender-v2 position grid changed")
    positions = tuple(
        (x_value, y_value, z_value)
        for z_value in z_values
        for y_value in y_values
        for x_value in x_values
    )
    if (
        len(positions) != 100
        or len(set(positions)) != 100
        or int(grid.get("expected_position_count", 0)) != 100
    ):
        raise ValueError("Blender-v2 position grid must contain 100 distinct poses")
    return positions


def load_contract(path: Path, repository_root: Path) -> dict:
    contract_path = path.resolve(strict=True)
    root = repository_root.resolve(strict=True)
    if root not in contract_path.parents:
        raise ValueError("gate contract must be inside the repository")
    raw = json.loads(contract_path.read_text(encoding="utf-8"))
    gate_id = raw.get("gate_id")
    if (
        raw.get("schema_version") != 1
        or gate_id not in {INITIAL_GATE_ID, POST_WINDING_GATE_ID}
        or raw.get("scope") != EXPECTED_SCOPE
        or raw.get("status") != "FROZEN_EXECUTION_AUTHORIZED"
    ):
        raise ValueError("unexpected Blender-v2 localization gate contract")

    resolved = {
        "decision_record": _bound_path(
            root, raw["decision_record"], "decision record"
        )
    }
    for label, record in raw.get("bindings", {}).items():
        resolved[label] = _bound_path(root, record, label)
    expected_bindings = {
        "base_world",
        "scene_manifest",
        "localization_config",
        "target_model_sdf",
        "target_visual_mesh",
    }
    if gate_id == POST_WINDING_GATE_ID:
        expected_bindings.update({"unripe_visual_mesh", "repair_receipt"})
    if set(raw.get("bindings", {})) != expected_bindings:
        raise ValueError("gate bindings changed")

    target = raw.get("target", {})
    if (
        int(target.get("target_id", 0)) != 1
        or target.get("model_name") != "strawberry_1"
        or target.get("maturity") != "RIPE"
        or float(target.get("fruit_radius_m", 0.0)) != 0.026
        or _finite_tuple(
            target.get("orientation_xyzw"),
            length=4,
            label="target.orientation_xyzw",
        )
        != (0.0, 0.0, 0.0, 1.0)
    ):
        raise ValueError("Blender-v2 target contract changed")

    position_grid(raw)
    runtime = raw.get("runtime", {})
    if runtime != {
        "attempts_per_position": 3,
        "settled_depth_frames": 3,
        "sensor_timeout_sec": 5.0,
        "runner_timeout_sec": 360.0,
        "headless": True,
    }:
        raise ValueError("Blender-v2 gate runtime changed")
    thresholds = raw.get("thresholds", {})
    if thresholds != {
        "minimum_positions": 100,
        "median_error_mm_max": 15.0,
        "p95_error_mm_max": 30.0,
        "require_all_measurements": True,
    }:
        raise ValueError("Blender-v2 gate thresholds changed")

    safety = raw.get("safety", {})
    false_keys = (
        "formal_acceptance",
        "formal_real_test_access_authorized",
        "formal_simulator_matrix_authorized",
        "perception_model_started",
        "robot_motion_authorized",
        "gripper_command_authorized",
        "attachment_authorized",
        "planning_authorized",
        "pick_action_authorized",
    )
    if any(safety.get(key) is not False for key in false_keys):
        raise ValueError("unsafe Blender-v2 localization authorization")
    if safety.get("simulated_fruit_pose_motion_authorized") is not True:
        raise ValueError("fruit pose motion must be explicitly authorized")
    execution = raw.get("execution", {})
    if (
        int(execution.get("maximum_claims", 0)) != 1
        or execution.get("retry_authorized") is not False
        or execution.get("output_directory")
        != (
            "results/development/"
            + (
                "blender_v2_localization_accuracy_100_v1"
                if gate_id == INITIAL_GATE_ID
                else "blender_v2_localization_accuracy_100_post_winding_v1"
            )
        )
    ):
        raise ValueError("Blender-v2 execution boundary changed")

    scene = raw.get("scene_materialization", {})
    if (
        scene.get("plant_model") != "strawberry_plant"
        or set(scene.get("parked_fruit_poses_xyz_rpy", {}))
        != {"strawberry_2", "strawberry_3"}
        or set(scene.get("canonical_restore_positions_m", {}))
        != {"strawberry_1", "strawberry_2", "strawberry_3"}
    ):
        raise ValueError("Blender-v2 materialized scene identities changed")
    _finite_tuple(
        scene.get("plant_park_pose_xyz_rpy"),
        length=6,
        label="plant park pose",
    )
    _finite_tuple(
        scene.get("target_initial_pose_xyz_rpy"),
        length=6,
        label="target initial pose",
    )
    for model_name, pose in scene["parked_fruit_poses_xyz_rpy"].items():
        _finite_tuple(pose, length=6, label=f"{model_name} park pose")
    for model_name, position in scene["canonical_restore_positions_m"].items():
        _finite_tuple(position, length=3, label=f"{model_name} restore position")

    raw["_resolved_paths"] = {
        key: str(value) for key, value in resolved.items()
    }
    raw["_contract_path"] = str(contract_path)
    raw["_contract_sha256"] = sha256(contract_path)
    return raw


def validate_world_receipt(
    receipt_path: Path,
    materialized_world: Path,
    contract: Mapping[str, object],
) -> dict:
    receipt_path = receipt_path.resolve(strict=True)
    world_path = materialized_world.resolve(strict=True)
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    if (
        receipt.get("kind")
        != "blender_v2_localization_camera_clear_world_receipt"
        or receipt.get("gate_id") != contract["gate_id"]
        or receipt.get("contract_sha256") != contract["_contract_sha256"]
        or receipt.get("plant_parked") is not True
        or receipt.get("non_target_fruits_parked") is not True
        or receipt.get("robot_motion_started") is not False
    ):
        raise ValueError("invalid Blender-v2 materialized-world receipt")
    world = receipt.get("materialized_world", {})
    if (
        Path(str(world.get("path"))).resolve() != world_path
        or world_path.stat().st_size != int(world.get("size_bytes", -1))
        or sha256(world_path) != world.get("sha256")
    ):
        raise ValueError("materialized Blender-v2 world binding changed")
    return receipt
