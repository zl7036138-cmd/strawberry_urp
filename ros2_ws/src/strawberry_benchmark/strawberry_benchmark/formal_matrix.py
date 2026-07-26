"""Strict contract and deterministic schedule for the formal P3 simulator matrix."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping

from .models import ScenarioKind
from .perception_control import (
    PerceptionControlWaiver,
    SceneModel,
    load_perception_control_waiver,
    sha256_file,
    verify_perception_control_waiver,
)
from .scenarios import (
    EXPECTED_NEGATIVE_TRIALS,
    EXPECTED_POSITIVE_TRIALS,
    generate_scenarios,
    load_benchmark_spec,
)


EXPECTED_POSITIONS = (
    ("near_left", "camera_clear_01", (0.44, -0.10, 0.48)),
    ("near_right", "camera_clear_03", (0.50, -0.05, 0.52)),
    ("center", "camera_clear_02", (0.47, -0.075, 0.50)),
    ("far_left", "camera_clear_05", (0.44, 0.00, 0.52)),
    ("far_right", "camera_clear_04", (0.47, -0.025, 0.48)),
)
EXPECTED_SIMULATION_SEEDS = (20260710, 20260711, 20260712)
EXPECTED_LIGHTING = ("dim", "nominal", "bright")
EXPECTED_OCCLUSION = ("none", "partial", "heavy")
EXPECTED_FAILURE_CODES = {
    "NO_TARGET",
    "LOW_CONFIDENCE",
    "DEPTH_INVALID",
    "TF_TIMEOUT",
    "STALE_DATA",
    "UNREACHABLE",
    "PLANNING_FAILED",
    "COLLISION",
    "GRASP_FAILED",
    "PLACE_FAILED",
}


@dataclass(frozen=True)
class BoundFile:
    relative_path: str
    size_bytes: int
    sha256: str


@dataclass(frozen=True)
class FormalPosition:
    label: str
    source: str
    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class FormalScene:
    target_model_name: str
    target_id: int
    target_maturity: str
    parked_models: tuple[SceneModel, ...]


@dataclass(frozen=True)
class FormalMatrixContract:
    matrix_id: str
    bindings: Mapping[str, BoundFile]
    model: BoundFile
    confidence_threshold: float
    image_size: int
    positions: tuple[FormalPosition, ...]
    simulation_seeds: tuple[int, ...]
    base_ros_domain_id: int
    schedule_seed: int
    materialization_id: int
    minimum_positive_detection_frames: int
    minimum_negative_detection_frames: int
    condition_probe_frames: int
    startup_timeout_sec: float
    trial_timeout_sec: float
    outer_timeout_sec: float
    control_position_tolerance_m: float
    maximum_infrastructure_attempts: int
    minimum_positive_success_rate: float
    minimum_negative_no_pick_rate: float
    maximum_negative_pick_attempt_rate: float
    maximum_negative_fruit_picked_rate: float
    maximum_planning_p95_sec: float
    positive_scene: FormalScene
    negative_scene: FormalScene
    failure_taxonomy: Mapping[str, str]
    claim_relative_path: str
    preflight_relative_dir: str
    default_result_relative_dir: str


@dataclass(frozen=True)
class ScheduledFormalScenario:
    order_index: int
    ros_domain_id: int
    trial_id: str
    kind: str
    lighting: str
    occlusion: str
    position: FormalPosition
    simulation_seed: int
    expected_maturity: str
    expected_outcome: str
    target_model_name: str
    target_id: int
    parked_models: tuple[SceneModel, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "order_index": self.order_index,
            "ros_domain_id": self.ros_domain_id,
            "trial_id": self.trial_id,
            "kind": self.kind,
            "lighting": self.lighting,
            "occlusion": self.occlusion,
            "position": self.position.label,
            "position_source": self.position.source,
            "target_position_m": list(self.position.position_m),
            "simulation_seed": self.simulation_seed,
            "expected_maturity": self.expected_maturity,
            "expected_outcome": self.expected_outcome,
            "target_model_name": self.target_model_name,
            "target_id": self.target_id,
            "parked_models": [
                {
                    "model_name": item.model_name,
                    "target_id": item.target_id,
                    "position_m": list(item.position_m),
                }
                for item in self.parked_models
            ],
        }


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _strict_relative_path(value: Any, name: str) -> str:
    path = Path(str(value))
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise ValueError(f"{name} must be repository-relative")
    return path.as_posix()


def _digest(value: Any, name: str) -> str:
    text = str(value).lower()
    if len(text) != 64 or any(character not in "0123456789abcdef" for character in text):
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return text


def _positive_int(value: Any, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _bound_file(value: Any, name: str) -> BoundFile:
    raw = _mapping(value, name)
    return BoundFile(
        relative_path=_strict_relative_path(raw.get("relative_path"), f"{name}.relative_path"),
        size_bytes=_positive_int(raw.get("size_bytes"), f"{name}.size_bytes"),
        sha256=_digest(raw.get("sha256"), f"{name}.sha256"),
    )


def _position(value: Any, name: str) -> FormalPosition:
    raw = _mapping(value, name)
    coordinates = raw.get("position_m")
    if not isinstance(coordinates, list) or len(coordinates) != 3:
        raise ValueError(f"{name}.position_m must contain three coordinates")
    return FormalPosition(
        label=str(raw.get("label", "")).strip(),
        source=str(raw.get("source", "")).strip(),
        position_m=tuple(_finite(item, f"{name}.position_m") for item in coordinates),  # type: ignore[arg-type]
    )


def _scene_model(value: Any, name: str) -> SceneModel:
    raw = _mapping(value, name)
    coordinates = raw.get("position_m")
    if not isinstance(coordinates, list) or len(coordinates) != 3:
        raise ValueError(f"{name}.position_m must contain three coordinates")
    model_name = str(raw.get("model_name", "")).strip()
    if not model_name:
        raise ValueError(f"{name}.model_name must be non-empty")
    return SceneModel(
        model_name=model_name,
        target_id=_positive_int(raw.get("target_id"), f"{name}.target_id"),
        position_m=tuple(_finite(item, f"{name}.position_m") for item in coordinates),  # type: ignore[arg-type]
    )


def _scene(value: Any, name: str, expected_maturity: str) -> FormalScene:
    raw = _mapping(value, name)
    target = _mapping(raw.get("target"), f"{name}.target")
    maturity = str(target.get("maturity", "")).upper()
    if maturity != expected_maturity:
        raise ValueError(f"{name} target maturity must be {expected_maturity}")
    target_name = str(target.get("model_name", "")).strip()
    target_id = _positive_int(target.get("target_id"), f"{name}.target.target_id")
    parked_raw = raw.get("parked_models")
    if not isinstance(parked_raw, list) or len(parked_raw) != 2:
        raise ValueError(f"{name} must park exactly two models")
    parked = tuple(
        _scene_model(item, f"{name}.parked_models[{index}]")
        for index, item in enumerate(parked_raw)
    )
    if len({target_id, *(item.target_id for item in parked)}) != 3:
        raise ValueError(f"{name} target IDs must be unique")
    if len({target_name, *(item.model_name for item in parked)}) != 3:
        raise ValueError(f"{name} model names must be unique")
    return FormalScene(target_name, target_id, maturity, parked)


def load_formal_matrix_contract(path: Path) -> FormalMatrixContract:
    """Load and strictly validate the formal matrix manifest."""

    try:
        root = _mapping(json.loads(path.read_text(encoding="utf-8")), "formal matrix")
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read formal matrix manifest: {exc}") from exc
    if root.get("schema_version") != 1:
        raise ValueError("formal matrix schema_version must be 1")
    if root.get("matrix_id") != "p3_formal_matrix_v1":
        raise ValueError("unexpected formal matrix ID")
    if root.get("scope") != "FORMAL_P3_SIMULATOR_MATRIX":
        raise ValueError("formal matrix scope was altered")
    if root.get("formal_acceptance_eligible") is not True:
        raise ValueError("formal matrix must remain acceptance eligible")
    if root.get("held_out_real_test_consumed") is not False:
        raise ValueError("formal matrix cannot consume the held-out real test")
    if root.get("engineering_waiver_active") is not True:
        raise ValueError("ADR-0026 engineering waiver must remain explicit")

    raw_bindings = _mapping(root.get("bindings"), "bindings")
    expected_binding_names = {
        "waiver", "repeated_gate_handoff", "benchmark", "project",
        "scene_conditions", "base_world", "simulation_launch", "system_launch",
    }
    if set(raw_bindings) != expected_binding_names:
        raise ValueError("formal matrix binding set differs from the frozen set")
    bindings = {
        name: _bound_file(raw_bindings[name], f"bindings.{name}")
        for name in sorted(raw_bindings)
    }

    model_raw = _mapping(root.get("model"), "model")
    model = BoundFile(
        relative_path=_strict_relative_path(model_raw.get("relative_path"), "model.relative_path"),
        size_bytes=_positive_int(model_raw.get("size_bytes"), "model.size_bytes"),
        sha256=_digest(model_raw.get("sha256"), "model.sha256"),
    )
    confidence = _finite(model_raw.get("confidence_threshold"), "confidence_threshold")
    if not math.isclose(confidence, 0.58):
        raise ValueError("formal model confidence threshold must remain 0.58")
    if _positive_int(model_raw.get("image_size"), "image_size") != 640:
        raise ValueError("formal model image size must remain 640")
    if model_raw.get("numeric_t30_gate_passed") is not False:
        raise ValueError("formal manifest cannot rewrite the failed numeric T30 gate")
    if not math.isclose(_finite(model_raw.get("numeric_macro_f1"), "numeric_macro_f1"), 0.8006746846582575):
        raise ValueError("formal manifest changed the measured macro-F1")
    if not math.isclose(_finite(model_raw.get("required_macro_f1"), "required_macro_f1"), 0.85):
        raise ValueError("formal manifest changed the original macro-F1 threshold")

    matrix = _mapping(root.get("matrix"), "matrix")
    if matrix.get("positive_trials") != EXPECTED_POSITIVE_TRIALS or matrix.get("negative_trials") != EXPECTED_NEGATIVE_TRIALS:
        raise ValueError("formal matrix must contain 135 positive and 30 negative trials")
    if tuple(matrix.get("lighting_levels", ())) != EXPECTED_LIGHTING:
        raise ValueError("formal lighting order must remain dim, nominal, bright")
    if tuple(matrix.get("occlusion_levels", ())) != EXPECTED_OCCLUSION:
        raise ValueError("formal occlusion order must remain none, partial, heavy")
    seeds = tuple(int(value) for value in matrix.get("simulation_seeds", ()))
    if seeds != EXPECTED_SIMULATION_SEEDS:
        raise ValueError("formal simulation seeds were altered")
    if matrix.get("negative_design") != "deterministic_balanced_marginals":
        raise ValueError("formal negative design was altered")
    positions = tuple(
        _position(item, f"matrix.positions[{index}]")
        for index, item in enumerate(matrix.get("positions", ()))
    )
    if tuple((item.label, item.source, item.position_m) for item in positions) != EXPECTED_POSITIONS:
        raise ValueError("formal positions differ from the frozen reachable candidates")

    execution = _mapping(root.get("execution"), "execution")
    exact_execution = {
        "fresh_world_per_trial": True,
        "headless": True,
        "distinct_ros_domain_per_trial": True,
        "schedule_method": "sha256_rank",
        "simulation_seed_launch_argument": "simulation_seed",
        "simulation_seed_runtime_binding": "gz sim --seed",
        "condition_materialization_id_role": "world_receipt_only_not_simulation_random_seed",
        "condition_probe_required": True,
        "infrastructure_retry_only_before_trial_service_acceptance": True,
        "behavioral_retry_allowed": False,
        "preserve_every_attempt": True,
    }
    for key, expected in exact_execution.items():
        if execution.get(key) != expected:
            raise ValueError(f"execution.{key} differs from the frozen contract")
    base_domain = execution.get("base_ros_domain_id")
    if not isinstance(base_domain, int) or isinstance(base_domain, bool):
        raise ValueError("base_ros_domain_id must be an integer")
    if not 0 <= base_domain <= 232 - 164:
        raise ValueError("165 distinct formal ROS domains must fit within [0, 232]")
    schedule_seed = execution.get("schedule_seed")
    materialization_id = execution.get("condition_materialization_id")
    if schedule_seed != 20260713 or materialization_id != 20260710:
        raise ValueError("formal schedule/materialization identifiers were altered")
    if execution.get("maximum_infrastructure_attempts_per_trial") != 2:
        raise ValueError("formal trials allow at most two infrastructure attempts")
    condition_frames = _positive_int(execution.get("condition_probe_frames"), "condition_probe_frames")
    positive_frames = _positive_int(execution.get("minimum_positive_detection_frames"), "minimum_positive_detection_frames")
    negative_frames = _positive_int(execution.get("minimum_negative_detection_frames"), "minimum_negative_detection_frames")
    if condition_frames != 5 or positive_frames != 10 or negative_frames != 10:
        raise ValueError("formal liveness/probe frame counts were altered")

    acceptance = _mapping(root.get("acceptance"), "acceptance")
    required_flags = (
        "require_all_scenarios_complete", "require_all_infrastructure_valid",
        "require_all_failures_attributed", "require_all_shutdowns_clean",
    )
    if any(acceptance.get(key) is not True for key in required_flags):
        raise ValueError("all formal completeness/infrastructure flags must remain true")
    thresholds = (
        (_finite(acceptance.get("minimum_positive_end_to_end_success_rate"), "positive rate"), 0.80),
        (_finite(acceptance.get("minimum_negative_safe_no_pick_rate"), "negative no-pick rate"), 0.95),
        (_finite(acceptance.get("maximum_negative_pick_attempt_rate"), "negative attempt rate"), 0.05),
        (_finite(acceptance.get("maximum_negative_fruit_picked_rate"), "negative picked rate"), 0.05),
        (_finite(acceptance.get("maximum_planning_time_p95_sec"), "planning p95"), 5.0),
    )
    if any(not math.isclose(observed, expected) for observed, expected in thresholds):
        raise ValueError("formal acceptance thresholds were altered")

    failure_taxonomy = _mapping(root.get("failure_taxonomy"), "failure_taxonomy")
    if set(failure_taxonomy) != EXPECTED_FAILURE_CODES:
        raise ValueError("formal failure taxonomy is incomplete or altered")
    if any(not str(value).strip() for value in failure_taxonomy.values()):
        raise ValueError("formal failure taxonomy categories must be non-empty")
    positive_scene = _scene(root.get("positive_scene"), "positive_scene", "RIPE")
    negative_scene = _scene(root.get("negative_scene"), "negative_scene", "UNRIPE")
    if (positive_scene.target_model_name, positive_scene.target_id) != ("strawberry_1", 1):
        raise ValueError("formal positive target must remain strawberry_1 / ID 1")
    if (negative_scene.target_model_name, negative_scene.target_id) != ("strawberry_2", 2):
        raise ValueError("formal negative target must remain strawberry_2 / ID 2")

    claim = _mapping(root.get("claim_policy"), "claim_policy")
    for key in (
        "single_claim", "no_model_threshold_code_or_config_changes_after_claim",
        "resume_same_claim_for_missing_prebehavior_attempts_only",
        "held_out_real_test_must_remain_sealed",
    ):
        if claim.get(key) is not True:
            raise ValueError(f"claim_policy.{key} must remain true")

    return FormalMatrixContract(
        matrix_id=str(root["matrix_id"]), bindings=bindings, model=model,
        confidence_threshold=confidence, image_size=640, positions=positions,
        simulation_seeds=seeds, base_ros_domain_id=base_domain,
        schedule_seed=int(schedule_seed), materialization_id=int(materialization_id),
        minimum_positive_detection_frames=positive_frames,
        minimum_negative_detection_frames=negative_frames,
        condition_probe_frames=condition_frames,
        startup_timeout_sec=_finite(execution.get("startup_timeout_sec"), "startup_timeout_sec"),
        trial_timeout_sec=_finite(execution.get("trial_timeout_sec"), "trial_timeout_sec"),
        outer_timeout_sec=_finite(execution.get("outer_timeout_sec"), "outer_timeout_sec"),
        control_position_tolerance_m=_finite(execution.get("control_position_tolerance_m"), "control_position_tolerance_m"),
        maximum_infrastructure_attempts=2,
        minimum_positive_success_rate=thresholds[0][0],
        minimum_negative_no_pick_rate=thresholds[1][0],
        maximum_negative_pick_attempt_rate=thresholds[2][0],
        maximum_negative_fruit_picked_rate=thresholds[3][0],
        maximum_planning_p95_sec=thresholds[4][0], positive_scene=positive_scene,
        negative_scene=negative_scene, failure_taxonomy=dict(failure_taxonomy),
        claim_relative_path=_strict_relative_path(claim.get("claim_relative_path"), "claim_relative_path"),
        preflight_relative_dir=_strict_relative_path(claim.get("preflight_relative_dir"), "preflight_relative_dir"),
        default_result_relative_dir=_strict_relative_path(claim.get("default_result_relative_dir"), "default_result_relative_dir"),
    )


def _verify_bound_file(binding: BoundFile, root: Path, name: str) -> Path:
    path = (root / binding.relative_path).resolve()
    if root not in path.parents or not path.is_file():
        raise ValueError(f"bound {name} is missing or escapes the repository")
    if path.stat().st_size != binding.size_bytes:
        raise ValueError(f"bound {name} size differs")
    if sha256_file(path) != binding.sha256:
        raise ValueError(f"bound {name} digest differs")
    return path


def verify_formal_matrix_contract(
    contract: FormalMatrixContract, repository_root: Path, model_path: Path | None = None
) -> dict[str, Path]:
    """Verify every frozen byte, the waiver, previous gate, matrix, and test seal."""

    root = repository_root.resolve()
    verified = {
        name: _verify_bound_file(binding, root, name)
        for name, binding in contract.bindings.items()
    }
    selected_model = model_path.resolve(strict=True) if model_path else (root / contract.model.relative_path).resolve(strict=True)
    if root not in selected_model.parents:
        raise ValueError("formal model must remain inside the repository")
    if selected_model.stat().st_size != contract.model.size_bytes or sha256_file(selected_model) != contract.model.sha256:
        raise ValueError("formal model bytes differ from the frozen checkpoint")

    waiver: PerceptionControlWaiver = load_perception_control_waiver(verified["waiver"])
    waiver_verified = verify_perception_control_waiver(waiver, root, model_path=selected_model)
    if not math.isclose(waiver.confidence_threshold, contract.confidence_threshold):
        raise ValueError("formal threshold differs from the ADR-0026 waiver")
    previous = json.loads(verified["repeated_gate_handoff"].read_text(encoding="utf-8"))
    if previous.get("status") != "DEVELOPMENT_GATE_PASSED_ACCEPTED_WITH_WAIVER":
        raise ValueError("the repeated perception development gate is not accepted")
    if previous.get("result", {}).get("development_gate_passed") is not True:
        raise ValueError("the repeated development-gate result did not pass")
    if previous.get("safety", {}).get("held_out_test_receipt_exists") is not False:
        raise ValueError("previous gate does not preserve the held-out-test seal")

    benchmark = load_benchmark_spec(verified["benchmark"])
    if benchmark.positions != tuple(item.label for item in contract.positions):
        raise ValueError("benchmark position labels differ from formal coordinates")
    if benchmark.seeds != contract.simulation_seeds:
        raise ValueError("benchmark seeds differ from formal Gazebo seeds")
    if benchmark.lighting_levels != EXPECTED_LIGHTING or benchmark.occlusion_levels != EXPECTED_OCCLUSION:
        raise ValueError("benchmark condition order differs from the formal contract")
    scenarios = generate_scenarios(benchmark)
    if len(scenarios) != 165:
        raise ValueError("formal scenario generator did not produce 165 trials")

    # Import lazily so the benchmark package remains usable without strawberry_sim.
    from strawberry_sim.scene_conditions import (  # type: ignore[import-not-found]
        load_scene_condition_config,
        resolve_occluder_parameters,
        validate_benchmark_condition_mapping,
    )

    scene = load_scene_condition_config(verified["scene_conditions"])
    validate_benchmark_condition_mapping(
        scene,
        lighting_levels=benchmark.lighting_levels,
        occlusion_levels=benchmark.occlusion_levels,
    )
    if int(scene["validation_scene"]["seed"]) != contract.materialization_id:
        raise ValueError("formal materialization ID differs from the scene contract")
    for position in contract.positions:
        for occlusion in EXPECTED_OCCLUSION:
            resolve_occluder_parameters(scene, occlusion, position.position_m)

    sim_launch = verified["simulation_launch"].read_text(encoding="utf-8")
    system_launch = verified["system_launch"].read_text(encoding="utf-8")
    if 'gz_command.extend(["--seed", str(simulation_seed)])' not in sim_launch:
        raise ValueError("simulation launch does not pass the formal seed to Gazebo")
    if '"simulation_seed": simulation_seed' not in system_launch:
        raise ValueError("system launch does not forward the formal simulation seed")
    if waiver_verified["held_out_receipt"].exists():
        raise ValueError("held-out real test receipt exists; formal simulator preflight refuses")

    return {
        **verified,
        "model": selected_model,
        "held_out_receipt": waiver_verified["held_out_receipt"],
        "claim": root / contract.claim_relative_path,
        "preflight_dir": root / contract.preflight_relative_dir,
        "default_result_dir": root / contract.default_result_relative_dir,
    }


def generate_formal_schedule(
    contract: FormalMatrixContract, benchmark_path: Path
) -> list[ScheduledFormalScenario]:
    """Generate the frozen matrix, deterministically interleave it, and assign domains."""

    spec = load_benchmark_spec(benchmark_path)
    positions = {item.label: item for item in contract.positions}
    scenarios = generate_scenarios(spec)
    ranked = sorted(
        scenarios,
        key=lambda item: (
            hashlib.sha256(f"{contract.schedule_seed}:{item.trial_id}".encode("utf-8")).hexdigest(),
            item.trial_id,
        ),
    )
    schedule = []
    for order_index, scenario in enumerate(ranked, start=1):
        positive = scenario.kind is ScenarioKind.POSITIVE
        scene = contract.positive_scene if positive else contract.negative_scene
        schedule.append(
            ScheduledFormalScenario(
                order_index=order_index,
                ros_domain_id=contract.base_ros_domain_id + order_index - 1,
                trial_id=scenario.trial_id,
                kind=scenario.kind.value,
                lighting=scenario.lighting,
                occlusion=scenario.occlusion,
                position=positions[scenario.position],
                simulation_seed=scenario.seed,
                expected_maturity=scenario.expected_maturity.value,
                expected_outcome="SUCCESS" if positive else "NO_PICK",
                target_model_name=scene.target_model_name,
                target_id=scene.target_id,
                parked_models=scene.parked_models,
            )
        )
    if len(schedule) != 165 or len({item.trial_id for item in schedule}) != 165:
        raise AssertionError("formal schedule is incomplete or contains duplicate IDs")
    if len({item.ros_domain_id for item in schedule}) != 165:
        raise AssertionError("formal schedule does not use distinct ROS domains")
    negatives = [item for item in schedule if item.kind == "NEGATIVE"]
    checks = (
        Counter(item.lighting for item in negatives) == {value: 10 for value in EXPECTED_LIGHTING},
        Counter(item.occlusion for item in negatives) == {value: 10 for value in EXPECTED_OCCLUSION},
        Counter(item.position.label for item in negatives) == {value[0]: 6 for value in EXPECTED_POSITIONS},
        Counter(item.simulation_seed for item in negatives) == {value: 10 for value in EXPECTED_SIMULATION_SEEDS},
    )
    if not all(checks):
        raise AssertionError("formal negative schedule lost its balanced marginals")
    return schedule
