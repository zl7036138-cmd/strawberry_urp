"""Contracts for bounded, non-acceptance downstream diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping


SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class PositionDiagnosticScenario:
    scenario_id: str
    position_label: str
    target_model_name: str
    target_position_m: tuple[float, float, float]

    def __post_init__(self) -> None:
        for name in ("scenario_id", "position_label", "target_model_name"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
        if len(self.target_position_m) != 3 or not all(
            math.isfinite(value) for value in self.target_position_m
        ):
            raise ValueError("target_position_m must contain three finite values")

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "PositionDiagnosticScenario":
        return cls(
            scenario_id=str(value["scenario_id"]),
            position_label=str(value["position_label"]),
            target_model_name=str(value["target_model_name"]),
            target_position_m=tuple(
                float(component) for component in value["target_position_m"]
            ),
        )


@dataclass(frozen=True)
class ParkedDiagnosticModel:
    model_name: str
    target_id: int
    position_m: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not isinstance(self.model_name, str) or not self.model_name.strip():
            raise ValueError("parked model_name must be non-empty")
        if self.target_id <= 0:
            raise ValueError("parked target_id must be positive")
        if len(self.position_m) != 3 or not all(
            math.isfinite(value) for value in self.position_m
        ):
            raise ValueError("parked position_m must contain three finite values")

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "ParkedDiagnosticModel":
        return cls(
            model_name=str(value["model_name"]),
            target_id=int(value["target_id"]),
            position_m=tuple(float(component) for component in value["position_m"]),
        )


@dataclass(frozen=True)
class OracleShadowDiagnosticManifest:
    diagnostic_id: str
    control_topic: str
    control_target_id: int
    shadow_model_role: str
    shadow_model_relative_path: str
    shadow_model_sha256: str
    confidence_threshold: float
    shadow_detections_topic: str
    shadow_target_topic: str
    occlusion: str
    lighting: str
    seed: int
    scenarios: tuple[PositionDiagnosticScenario, ...]
    scene_mode: str = "multi_fruit_fixed_world"
    parked_models: tuple[ParkedDiagnosticModel, ...] = ()
    fruit_collision_radius_m: float = 0.035
    schema_version: int = 1
    scope: str = "NON_ACCEPTANCE_DIAGNOSTIC"
    formal_acceptance: bool = False
    held_out_test_consumed: bool = False
    formal_position_labels: bool = False

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise ValueError("unsupported diagnostic schema_version")
        if self.scope != "NON_ACCEPTANCE_DIAGNOSTIC":
            raise ValueError("diagnostic scope must remain NON_ACCEPTANCE_DIAGNOSTIC")
        if self.formal_acceptance or self.held_out_test_consumed:
            raise ValueError("diagnostics cannot assert acceptance or consume held-out test")
        if self.formal_position_labels:
            raise ValueError("diagnostic positions cannot masquerade as formal positions")
        if not self.diagnostic_id.strip():
            raise ValueError("diagnostic_id must be non-empty")
        if self.control_topic != "/strawberry/oracle/target_pose":
            raise ValueError("oracle control topic is frozen")
        if self.control_target_id <= 0:
            raise ValueError("control_target_id must be positive")
        if self.shadow_model_role != "baseline__best":
            raise ValueError("only baseline__best may serve as the primary shadow model")
        if not self.shadow_model_relative_path or Path(
            self.shadow_model_relative_path
        ).is_absolute():
            raise ValueError("shadow model path must be a non-empty repository-relative path")
        if not SHA256_PATTERN.fullmatch(self.shadow_model_sha256):
            raise ValueError("shadow model SHA-256 must be 64 lowercase hex characters")
        if not math.isfinite(self.confidence_threshold) or not (
            0.0 <= self.confidence_threshold <= 1.0
        ):
            raise ValueError("confidence_threshold must be finite and in [0, 1]")
        for topic in (self.shadow_detections_topic, self.shadow_target_topic):
            if not topic.startswith("/") or topic == "/":
                raise ValueError("shadow topics must be absolute")
            if topic == self.control_topic:
                raise ValueError("shadow topics must differ from the control topic")
        if self.occlusion != "none" or self.lighting != "nominal":
            raise ValueError("v1 position diagnostics require unmodified clear lighting")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise ValueError("seed must be an integer")
        if len(self.scenarios) != 5:
            raise ValueError("v1 position diagnostic must contain exactly five scenarios")
        scenario_ids = [scenario.scenario_id for scenario in self.scenarios]
        labels = [scenario.position_label for scenario in self.scenarios]
        positions = [scenario.target_position_m for scenario in self.scenarios]
        if len(set(scenario_ids)) != len(scenario_ids):
            raise ValueError("diagnostic scenario IDs must be unique")
        if len(set(labels)) != len(labels):
            raise ValueError("diagnostic position labels must be unique")
        if len(set(positions)) != len(positions):
            raise ValueError("diagnostic positions must be unique")
        if any(scenario.target_model_name != "strawberry_1" for scenario in self.scenarios):
            raise ValueError("v1 diagnostics are frozen to ripe target strawberry_1")
        if self.scene_mode not in {
            "multi_fruit_fixed_world",
            "single_target_isolation",
            "multi_fruit_clearance",
        }:
            raise ValueError("unsupported diagnostic scene_mode")
        if not math.isfinite(self.fruit_collision_radius_m) or (
            self.fruit_collision_radius_m <= 0.0
        ):
            raise ValueError("fruit_collision_radius_m must be positive and finite")
        if self.scene_mode == "multi_fruit_fixed_world" and self.parked_models:
            raise ValueError("fixed-world diagnostics cannot park models")
        if self.scene_mode in {
            "single_target_isolation",
            "multi_fruit_clearance",
        }:
            if len(self.parked_models) != 2:
                raise ValueError("configured scene must position exactly two non-target models")
            model_names = [model.model_name for model in self.parked_models]
            target_ids = [model.target_id for model in self.parked_models]
            if set(model_names) != {"strawberry_2", "strawberry_3"}:
                raise ValueError("configured scene must position strawberry_2 and strawberry_3")
            if set(target_ids) != {2, 3}:
                raise ValueError("parked target IDs must be 2 and 3")
            if len(set(model.position_m for model in self.parked_models)) != 2:
                raise ValueError("parked model positions must be unique")
        if self.scene_mode == "multi_fruit_clearance":
            diameter = 2.0 * self.fruit_collision_radius_m
            non_target_distance = math.dist(
                self.parked_models[0].position_m,
                self.parked_models[1].position_m,
            )
            if non_target_distance <= diameter:
                raise ValueError("clearance fruit collision spheres must not overlap")
            surface_clearances = []
            for scenario in self.scenarios:
                minimum_center_distance = min(
                    math.dist(scenario.target_position_m, model.position_m)
                    for model in self.parked_models
                )
                if minimum_center_distance <= diameter:
                    raise ValueError("target and non-target collision spheres must not overlap")
                surface_clearances.append(minimum_center_distance - diameter)
            if not all(
                first > second
                for first, second in zip(surface_clearances, surface_clearances[1:])
            ):
                raise ValueError(
                    "clearance scenarios must have strictly decreasing surface clearance"
                )

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "OracleShadowDiagnosticManifest":
        control = raw["control"]
        shadow = raw["shadow"]
        conditions = raw["conditions"]
        scene_setup = raw.get("scene_setup", {})
        if str(control.get("source", "")) != "oracle":
            raise ValueError("diagnostic motion control must use oracle")
        return cls(
            schema_version=int(raw.get("schema_version", 1)),
            diagnostic_id=str(raw["diagnostic_id"]),
            scope=str(raw["scope"]),
            formal_acceptance=raw["formal_acceptance"],
            held_out_test_consumed=raw["held_out_test_consumed"],
            control_topic=str(control["topic"]),
            control_target_id=int(control["target_id"]),
            shadow_model_role=str(shadow["model_role"]),
            shadow_model_relative_path=str(shadow["model_relative_path"]),
            shadow_model_sha256=str(shadow["model_sha256"]),
            confidence_threshold=float(shadow["confidence_threshold"]),
            shadow_detections_topic=str(shadow["detections_topic"]),
            shadow_target_topic=str(shadow["target_topic"]),
            occlusion=str(conditions["occlusion"]),
            lighting=str(conditions["lighting"]),
            seed=int(conditions["seed"]),
            formal_position_labels=conditions["formal_position_labels"],
            scene_mode=str(
                scene_setup.get("mode", "multi_fruit_fixed_world")
            ),
            fruit_collision_radius_m=float(
                scene_setup.get("fruit_collision_radius_m", 0.035)
            ),
            parked_models=tuple(
                ParkedDiagnosticModel.from_dict(value)
                for value in scene_setup.get("parked_models", ())
            ),
            scenarios=tuple(
                PositionDiagnosticScenario.from_dict(value)
                for value in raw["scenarios"]
            ),
        )


def load_oracle_shadow_diagnostic_manifest(
    path: str | Path,
) -> OracleShadowDiagnosticManifest:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("diagnostic manifest must contain a JSON object")
    return OracleShadowDiagnosticManifest.from_dict(raw)


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_shadow_model(
    manifest: OracleShadowDiagnosticManifest,
    repository_root: str | Path,
    model_path: str | Path | None = None,
) -> Path:
    root = Path(repository_root).resolve()
    candidate = (
        Path(model_path).expanduser().resolve()
        if model_path is not None
        else (root / manifest.shadow_model_relative_path).resolve()
    )
    if not candidate.is_file():
        raise ValueError(f"shadow model does not exist: {candidate}")
    actual = sha256_file(candidate)
    if actual != manifest.shadow_model_sha256:
        raise ValueError(
            "shadow model SHA-256 differs from the frozen baseline__best binding"
        )
    return candidate
