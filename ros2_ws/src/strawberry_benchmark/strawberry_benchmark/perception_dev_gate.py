"""Contract for the non-formal repeated perception-control development gate."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from .perception_control import (
    PerceptionControlWaiver,
    SceneModel,
    load_perception_control_waiver,
    sha256_file,
    verify_perception_control_waiver,
)


@dataclass(frozen=True)
class DevGateScene:
    scenario_id: str
    expected_outcome: str
    target: SceneModel
    target_maturity: str
    parked_models: tuple[SceneModel, ...]


@dataclass(frozen=True)
class PerceptionRepeatedDevGate:
    gate_id: str
    waiver_relative_path: str
    waiver_sha256: str
    base_ros_domain_id: int
    minimum_settled_detection_frames: int
    seed: int
    positive_trials: int
    minimum_positive_success_rate: float
    negative_trials: int
    minimum_negative_no_pick_rate: float
    maximum_negative_false_pick_rate: float
    maximum_negative_control_attempts: int
    planning_time_p95_limit_sec: float
    positive_scene: DevGateScene
    negative_scene: DevGateScene


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def _relative_path(value: Any, name: str) -> str:
    path = Path(str(value))
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise ValueError(f"{name} must be a repository-relative path")
    return path.as_posix()


def _sha256(value: Any, name: str) -> str:
    rendered = str(value).lower()
    if len(rendered) != 64 or any(
        character not in "0123456789abcdef" for character in rendered
    ):
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return rendered


def _finite_float(value: Any, name: str) -> float:
    try:
        rendered = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(rendered):
        raise ValueError(f"{name} must be finite")
    return rendered


def _positive_integer(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _scene_model(value: Any, name: str) -> SceneModel:
    raw = _mapping(value, name)
    model_name = str(raw.get("model_name", "")).strip()
    target_id = _positive_integer(raw.get("target_id"), f"{name}.target_id")
    position = raw.get("position_m")
    if not model_name:
        raise ValueError(f"{name}.model_name must be non-empty")
    if not isinstance(position, list) or len(position) != 3:
        raise ValueError(f"{name}.position_m must contain three coordinates")
    xyz = tuple(
        _finite_float(coordinate, f"{name}.position_m")
        for coordinate in position
    )
    return SceneModel(model_name, target_id, xyz)


def _scene(value: Any, name: str, expected_outcome: str) -> DevGateScene:
    raw = _mapping(value, name)
    scenario_id = str(raw.get("scenario_id", "")).strip()
    if not scenario_id:
        raise ValueError(f"{name}.scenario_id must be non-empty")
    if raw.get("expected_outcome") != expected_outcome:
        raise ValueError(f"{name} must expect {expected_outcome}")
    target_raw = _mapping(raw.get("target"), f"{name}.target")
    maturity = str(target_raw.get("maturity", "")).strip().upper()
    target = _scene_model(target_raw, f"{name}.target")
    parked_raw = raw.get("parked_models")
    if not isinstance(parked_raw, list) or len(parked_raw) != 2:
        raise ValueError(f"{name} must park exactly two models")
    parked = tuple(
        _scene_model(item, f"{name}.parked_models[{index}]")
        for index, item in enumerate(parked_raw)
    )
    ids = {target.target_id, *(item.target_id for item in parked)}
    names = {target.model_name, *(item.model_name for item in parked)}
    if len(ids) != 3 or len(names) != 3:
        raise ValueError(f"{name} model names and target IDs must be unique")
    return DevGateScene(
        scenario_id=scenario_id,
        expected_outcome=expected_outcome,
        target=target,
        target_maturity=maturity,
        parked_models=parked,
    )


def load_perception_repeated_dev_gate(path: Path) -> PerceptionRepeatedDevGate:
    """Load and strictly validate the repeated development-gate manifest."""

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read repeated gate manifest: {exc}") from exc
    root = _mapping(raw, "repeated gate manifest")
    if root.get("schema_version") != 1:
        raise ValueError("repeated gate schema_version must be 1")
    if root.get("scope") != "NON_FORMAL_SIMPLE_SCENE_DEVELOPMENT_GATE":
        raise ValueError("repeated gate scope must remain non-formal")
    for key in (
        "formal_acceptance",
        "formal_135_plus_30_matrix_consumed",
        "held_out_real_test_consumed",
    ):
        if root.get(key) is not False:
            raise ValueError(f"{key} must remain false")

    binding = _mapping(root.get("waiver_binding"), "waiver_binding")
    execution = _mapping(root.get("execution"), "execution")
    acceptance = _mapping(root.get("acceptance"), "acceptance")
    conditions = _mapping(execution.get("conditions"), "execution.conditions")
    if execution.get("fresh_world_per_trial") is not True:
        raise ValueError("every repeated trial must use a fresh world")
    if execution.get("headless") is not True:
        raise ValueError("the repeated gate must remain headless")
    if conditions.get("lighting") != "nominal":
        raise ValueError("development gate lighting must remain nominal")
    if conditions.get("occlusion") != "none":
        raise ValueError("development gate must remain unobstructed")

    base_domain = execution.get("base_ros_domain_id")
    if not isinstance(base_domain, int) or isinstance(base_domain, bool):
        raise ValueError("base_ros_domain_id must be an integer")
    minimum_frames = _positive_integer(
        execution.get("minimum_settled_detection_frames"),
        "minimum_settled_detection_frames",
    )
    seed = conditions.get("seed")
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise ValueError("development-gate seed must be an integer")

    positive_trials = _positive_integer(
        acceptance.get("positive_trials"), "positive_trials"
    )
    negative_trials = _positive_integer(
        acceptance.get("negative_trials"), "negative_trials"
    )
    minimum_positive = _finite_float(
        acceptance.get("minimum_positive_success_rate"),
        "minimum_positive_success_rate",
    )
    minimum_no_pick = _finite_float(
        acceptance.get("minimum_negative_no_pick_rate"),
        "minimum_negative_no_pick_rate",
    )
    maximum_false_pick = _finite_float(
        acceptance.get("maximum_negative_false_pick_rate"),
        "maximum_negative_false_pick_rate",
    )
    maximum_control_attempts = acceptance.get(
        "maximum_negative_control_attempts"
    )
    planning_limit = _finite_float(
        acceptance.get("planning_time_p95_limit_sec"),
        "planning_time_p95_limit_sec",
    )
    if (positive_trials, negative_trials) != (10, 10):
        raise ValueError("development gate must retain 10 positive and 10 negative trials")
    if not math.isclose(minimum_positive, 0.8) or not math.isclose(
        minimum_no_pick, 1.0
    ):
        raise ValueError("development-gate success/no-pick rates were altered")
    if not math.isclose(maximum_false_pick, 0.05):
        raise ValueError("negative false-pick limit must remain 0.05")
    if maximum_control_attempts != 0:
        raise ValueError("negative control-attempt limit must remain zero")
    if not math.isclose(planning_limit, 5.0):
        raise ValueError("planning p95 limit must remain five seconds")
    if acceptance.get("require_all_infrastructure_valid") is not True:
        raise ValueError("all repeated-trial infrastructure must be valid")
    if acceptance.get("require_all_shutdown_clean") is not True:
        raise ValueError("all repeated-trial shutdowns must be clean")
    if not 0 <= base_domain <= 232 - positive_trials - negative_trials + 1:
        raise ValueError("repeated gate ROS domain range exceeds [0, 232]")

    positive_scene = _scene(
        root.get("positive_scene"), "positive_scene", "SUCCESS"
    )
    negative_scene = _scene(
        root.get("negative_scene"), "negative_scene", "NO_PICK"
    )
    if positive_scene.target_maturity != "RIPE":
        raise ValueError("positive target must be RIPE")
    if negative_scene.target_maturity != "UNRIPE":
        raise ValueError("negative target must be UNRIPE")
    if positive_scene.target.position_m != negative_scene.target.position_m:
        raise ValueError("positive and negative targets must share the same pose")

    return PerceptionRepeatedDevGate(
        gate_id=str(root.get("gate_id", "")),
        waiver_relative_path=_relative_path(
            binding.get("relative_path"), "waiver_binding.relative_path"
        ),
        waiver_sha256=_sha256(binding.get("sha256"), "waiver_binding.sha256"),
        base_ros_domain_id=base_domain,
        minimum_settled_detection_frames=minimum_frames,
        seed=seed,
        positive_trials=positive_trials,
        minimum_positive_success_rate=minimum_positive,
        negative_trials=negative_trials,
        minimum_negative_no_pick_rate=minimum_no_pick,
        maximum_negative_false_pick_rate=maximum_false_pick,
        maximum_negative_control_attempts=maximum_control_attempts,
        planning_time_p95_limit_sec=planning_limit,
        positive_scene=positive_scene,
        negative_scene=negative_scene,
    )


def verify_perception_repeated_dev_gate(
    contract: PerceptionRepeatedDevGate,
    repository_root: Path,
    model_path: Path | None = None,
) -> tuple[PerceptionControlWaiver, dict[str, Path]]:
    """Verify the bound waiver and every model/evidence byte it authorizes."""

    root = repository_root.resolve()
    waiver_path = (root / contract.waiver_relative_path).resolve()
    if root not in waiver_path.parents or not waiver_path.is_file():
        raise ValueError("bound perception-control waiver is missing or unsafe")
    if sha256_file(waiver_path) != contract.waiver_sha256:
        raise ValueError("bound perception-control waiver digest differs")
    waiver = load_perception_control_waiver(waiver_path)
    verified = verify_perception_control_waiver(
        waiver, root, model_path=model_path
    )
    if contract.positive_scene.target != waiver.target:
        raise ValueError("positive scene differs from the accepted smoke target")
    if contract.positive_scene.parked_models != waiver.parked_models:
        raise ValueError("positive parked scene differs from the accepted smoke")
    return waiver, verified
