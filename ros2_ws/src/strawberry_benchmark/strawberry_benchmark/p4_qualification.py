"""Frozen P4 simulator-only perception qualification contract and metrics."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable


EXPECTED_POSITIONS = {
    "near_left": (0.44, -0.10, 0.48),
    "near_right": (0.50, -0.05, 0.52),
    "center": (0.47, -0.075, 0.50),
    "far_left": (0.44, 0.00, 0.52),
    "far_right": (0.47, -0.025, 0.48),
}
EXPECTED_CONDITIONS = {
    "nominal_none": ("nominal", "none"),
    "dim_none": ("dim", "none"),
    "nominal_heavy": ("nominal", "heavy"),
}
EXPECTED_ACCEPTANCE = {
    "minimum_no_occlusion_ripe_frame_rate_each_condition": 0.95,
    "minimum_no_occlusion_target_pose_frame_rate_each_condition": 0.95,
    "minimum_heavy_ripe_frame_rate_overall": 0.60,
    "minimum_heavy_target_pose_frame_rate_overall": 0.60,
    "minimum_heavy_ripe_frame_rate_each_position": 0.50,
    "minimum_heavy_target_pose_frame_rate_each_position": 0.50,
    "maximum_unripe_false_ripe_frame_rate_overall": 0.05,
    "minimum_unripe_correct_observation_frame_rate_overall": 0.50,
    "all_scenarios_complete": True,
    "all_infrastructure_valid": True,
    "robot_motion_must_remain_false": True,
}


@dataclass(frozen=True)
class QualificationScenario:
    order_index: int
    ros_domain_id: int
    scenario_id: str
    maturity: str
    position_label: str
    position_m: tuple[float, float, float]
    condition_label: str
    lighting: str
    occlusion: str
    target_model_name: str
    target_id: int
    parked_models: tuple[dict, ...]


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _bound_file(root: Path, record: dict, prefix: str) -> Path:
    path = (root / str(record[f"{prefix}_path"])).resolve(strict=True)
    metadata_prefix = prefix.removesuffix("_relative")
    resolved_root = root.resolve()
    if path != resolved_root and resolved_root not in path.parents:
        raise ValueError(f"{prefix} path escapes repository")
    if path.stat().st_size != int(record[f"{metadata_prefix}_size_bytes"]):
        raise ValueError(f"{prefix} size mismatch")
    if sha256_file(path) != str(record[f"{metadata_prefix}_sha256"]):
        raise ValueError(f"{prefix} hash mismatch")
    return path


def _vector(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain three values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result  # type: ignore[return-value]


def load_qualification_contract(path: Path, repository_root: Path) -> dict:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if int(raw.get("schema_version", 0)) != 1:
        raise ValueError("unsupported P4 qualification schema")
    if raw.get("qualification_id") != "p4_sim_adapt_qualification_v1":
        raise ValueError("unexpected qualification ID")
    if raw.get("scope") != "SIMULATOR_ONLY_NO_MOTION_QUALIFICATION":
        raise ValueError("qualification scope changed")
    if raw.get("formal_acceptance") or raw.get("held_out_test_consumed"):
        raise ValueError("qualification cannot assert formal or held-out acceptance")
    if raw.get("single_execution") is not True:
        raise ValueError("qualification must remain single execution")

    intervention = raw["intervention"]
    _bound_file(repository_root, intervention, "decision")
    model = _bound_file(repository_root, intervention, "model_relative")
    if intervention.get("only_change") != "simulator_perception_checkpoint_substitution":
        raise ValueError("intervention boundary changed")
    if float(intervention.get("confidence_threshold", -1)) != 0.80:
        raise ValueError("candidate threshold must remain 0.80")
    if intervention.get("real_validation_status") != "REJECTED_REGRESSION":
        raise ValueError("real-validation rejection warning is missing")
    if any(
        intervention.get(key) is not False
        for key in ("new_training_allowed", "asset_change_allowed", "runtime_motion_change_allowed")
    ):
        raise ValueError("qualification permits an additional intervention")

    trigger = raw["trigger_evidence"]
    trigger_path = _bound_file(repository_root, trigger, "formal_p3_summary")
    trigger_payload = json.loads(trigger_path.read_text(encoding="utf-8"))
    if trigger_payload["metrics"].get("failure_by_category") != {
        "grasp": 12,
        "perception": 84,
    }:
        raise ValueError("formal trigger failure counts changed")
    if trigger_payload.get("formal_p3_simulator_gate_passed") is not False:
        raise ValueError("P4 qualification requires the failed formal baseline")

    prior = raw["prior_candidate_evidence"]
    synthetic_path = _bound_file(repository_root, prior, "synthetic_summary")
    real_path = _bound_file(repository_root, prior, "real_summary")
    synthetic = json.loads(synthetic_path.read_text(encoding="utf-8"))
    real = json.loads(real_path.read_text(encoding="utf-8"))
    if synthetic.get("synthetic_gate", {}).get("actual") != 1.0:
        raise ValueError("synthetic candidate evidence changed")
    if real.get("real_nonregression_passed") is not False:
        raise ValueError("real regression must remain explicit")
    if prior.get("held_out_real_test_accessed") is not False:
        raise ValueError("held-out test boundary changed")

    runtime = raw["runtime"]
    if runtime.get("headless") is not True or runtime.get("robot_motion_started") is not False:
        raise ValueError("qualification must remain headless and no-motion")
    if any(runtime.get(key) is not False for key in ("start_manipulation", "start_orchestrator", "enable_attachment")):
        raise ValueError("motion-capable process requested")
    base_domain = int(runtime.get("base_ros_domain_id", -1))
    if not 0 <= base_domain <= 203:
        raise ValueError("30 ROS domains must fit in [0, 232]")

    condition = raw["condition_contract"]
    _bound_file(repository_root, condition, "scene_config_relative")
    _bound_file(repository_root, condition, "base_world_relative")
    measurement = raw["measurement"]
    if measurement != {
        "condition_probe_frames": 5,
        "settled_frames_after_pose": 10,
        "fixed_detection_frames": 60,
        "post_window_wait_sec": 1.0,
        "scenario_count": 30,
    }:
        raise ValueError("qualification measurement contract changed")
    if raw.get("acceptance") != EXPECTED_ACCEPTANCE:
        raise ValueError("qualification acceptance thresholds changed")

    positions = {
        str(item["label"]): _vector(item["position_m"], "position_m")
        for item in raw.get("positions", [])
    }
    if positions != EXPECTED_POSITIONS:
        raise ValueError("qualification positions changed")
    conditions = {
        str(item["label"]): (str(item["lighting"]), str(item["occlusion"]))
        for item in raw.get("conditions", [])
    }
    if conditions != EXPECTED_CONDITIONS:
        raise ValueError("qualification conditions changed")
    scenes = raw.get("scenes", {})
    if set(scenes) != {"RIPE", "UNRIPE"}:
        raise ValueError("qualification must include ripe and unripe scenes")
    for maturity, expected_target in (("RIPE", ("strawberry_1", 1)), ("UNRIPE", ("strawberry_2", 2))):
        scene = scenes[maturity]
        if (str(scene["target_model_name"]), int(scene["target_id"])) != expected_target:
            raise ValueError(f"{maturity} target identity changed")
        parked = scene.get("parked_models", [])
        if len(parked) != 2:
            raise ValueError(f"{maturity} scene must park two models")
        identities = {int(scene["target_id"]), *(int(item["target_id"]) for item in parked)}
        names = {str(scene["target_model_name"]), *(str(item["model_name"]) for item in parked)}
        if identities != {1, 2, 3} or names != {"strawberry_1", "strawberry_2", "strawberry_3"}:
            raise ValueError(f"{maturity} scene identities changed")

    raw["_resolved_model_path"] = str(model)
    return raw


def generate_qualification_schedule(contract: dict) -> list[QualificationScenario]:
    candidates = []
    for maturity, scene in contract["scenes"].items():
        for position in contract["positions"]:
            for condition in contract["conditions"]:
                scenario_id = (
                    f"{maturity.lower()}__{position['label']}__{condition['label']}"
                )
                candidates.append((scenario_id, maturity, scene, position, condition))
    seed = int(contract["runtime"]["schedule_seed"])
    candidates.sort(
        key=lambda item: hashlib.sha256(f"{seed}:{item[0]}".encode()).hexdigest()
    )
    base = int(contract["runtime"]["base_ros_domain_id"])
    return [
        QualificationScenario(
            order_index=index,
            ros_domain_id=base + index - 1,
            scenario_id=scenario_id,
            maturity=maturity,
            position_label=str(position["label"]),
            position_m=_vector(position["position_m"], "position_m"),
            condition_label=str(condition["label"]),
            lighting=str(condition["lighting"]),
            occlusion=str(condition["occlusion"]),
            target_model_name=str(scene["target_model_name"]),
            target_id=int(scene["target_id"]),
            parked_models=tuple(scene["parked_models"]),
        )
        for index, (scenario_id, maturity, scene, position, condition) in enumerate(candidates, 1)
    ]


def _rate(numerator: int, denominator: int) -> float:
    return numerator / denominator if denominator else 0.0


def evaluate_qualification_records(contract: dict, records: Iterable[dict]) -> dict:
    records = list(records)
    expected_ids = {item.scenario_id for item in generate_qualification_schedule(contract)}
    actual_ids = [str(item.get("scenario_id")) for item in records]
    complete = len(records) == 30 and len(set(actual_ids)) == 30 and set(actual_ids) == expected_ids
    infrastructure = complete and all(bool(item.get("infrastructure_valid")) for item in records)
    no_motion = complete and all(item.get("robot_motion_started") is False for item in records)

    ripe = [item for item in records if item.get("maturity") == "RIPE"]
    unripe = [item for item in records if item.get("maturity") == "UNRIPE"]
    no_occ = defaultdict(lambda: {"frames": 0, "ripe": 0, "target": 0})
    heavy = defaultdict(lambda: {"frames": 0, "ripe": 0, "target": 0})
    for item in ripe:
        bucket = heavy[item["position_label"]] if item.get("occlusion") == "heavy" else no_occ[item["condition_label"]]
        bucket["frames"] += int(item.get("frame_count", 0))
        bucket["ripe"] += int(item.get("ripe_frames", 0))
        bucket["target"] += int(item.get("target_pose_frames", 0))

    no_occ_metrics = {
        key: {
            **value,
            "ripe_frame_rate": _rate(value["ripe"], value["frames"]),
            "target_pose_frame_rate": _rate(value["target"], value["frames"]),
        }
        for key, value in sorted(no_occ.items())
    }
    heavy_metrics = {
        key: {
            **value,
            "ripe_frame_rate": _rate(value["ripe"], value["frames"]),
            "target_pose_frame_rate": _rate(value["target"], value["frames"]),
        }
        for key, value in sorted(heavy.items())
    }
    heavy_total = {
        key: sum(value[key] for value in heavy.values())
        for key in ("frames", "ripe", "target")
    }
    unripe_frames = sum(int(item.get("frame_count", 0)) for item in unripe)
    unripe_false_ripe = sum(int(item.get("ripe_frames", 0)) for item in unripe)
    unripe_correct = sum(int(item.get("unripe_frames", 0)) for item in unripe)
    thresholds = contract["acceptance"]
    checks = {
        "all_scenarios_complete": complete,
        "all_infrastructure_valid": infrastructure,
        "robot_motion_remained_false": no_motion,
        "no_occlusion_ripe_rates": len(no_occ_metrics) == 2 and all(
            value["ripe_frame_rate"] >= thresholds["minimum_no_occlusion_ripe_frame_rate_each_condition"]
            for value in no_occ_metrics.values()
        ),
        "no_occlusion_target_pose_rates": len(no_occ_metrics) == 2 and all(
            value["target_pose_frame_rate"] >= thresholds["minimum_no_occlusion_target_pose_frame_rate_each_condition"]
            for value in no_occ_metrics.values()
        ),
        "heavy_ripe_overall": _rate(heavy_total["ripe"], heavy_total["frames"]) >= thresholds["minimum_heavy_ripe_frame_rate_overall"],
        "heavy_target_overall": _rate(heavy_total["target"], heavy_total["frames"]) >= thresholds["minimum_heavy_target_pose_frame_rate_overall"],
        "heavy_ripe_each_position": len(heavy_metrics) == 5 and all(
            value["ripe_frame_rate"] >= thresholds["minimum_heavy_ripe_frame_rate_each_position"]
            for value in heavy_metrics.values()
        ),
        "heavy_target_each_position": len(heavy_metrics) == 5 and all(
            value["target_pose_frame_rate"] >= thresholds["minimum_heavy_target_pose_frame_rate_each_position"]
            for value in heavy_metrics.values()
        ),
        "unripe_false_ripe_rate": _rate(unripe_false_ripe, unripe_frames) <= thresholds["maximum_unripe_false_ripe_frame_rate_overall"],
        "unripe_correct_observation_rate": _rate(unripe_correct, unripe_frames) >= thresholds["minimum_unripe_correct_observation_frame_rate_overall"],
    }
    return {
        "qualification_passed": all(checks.values()),
        "checks": checks,
        "scenario_count": len(records),
        "no_occlusion": no_occ_metrics,
        "heavy_by_position": heavy_metrics,
        "heavy_overall": {
            **heavy_total,
            "ripe_frame_rate": _rate(heavy_total["ripe"], heavy_total["frames"]),
            "target_pose_frame_rate": _rate(heavy_total["target"], heavy_total["frames"]),
        },
        "unripe": {
            "frames": unripe_frames,
            "false_ripe_frames": unripe_false_ripe,
            "false_ripe_frame_rate": _rate(unripe_false_ripe, unripe_frames),
            "correct_unripe_frames": unripe_correct,
            "correct_unripe_frame_rate": _rate(unripe_correct, unripe_frames),
        },
    }
