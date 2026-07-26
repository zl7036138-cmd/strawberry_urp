"""Deterministic world materialization and metrics for scene-condition injection."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from statistics import mean
from typing import Mapping, Sequence
import xml.etree.ElementTree as ET


LIGHTING_LEVELS = ("dim", "nominal", "bright")
OCCLUSION_LEVELS = ("none", "partial", "heavy")
OCCLUDER_MODEL_NAME = "benchmark_visual_occluder"


def fingerprint(path: str | Path) -> dict[str, object]:
    candidate = Path(path)
    content = candidate.read_bytes()
    return {
        "path": str(candidate.resolve()),
        "size_bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def _finite_vector(value: object, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be a numeric sequence")
    normalized = tuple(float(component) for component in value)
    if len(normalized) != length or not all(math.isfinite(item) for item in normalized):
        raise ValueError(f"{name} must contain {length} finite values")
    return normalized


def _rgba(value: object, name: str) -> tuple[float, float, float, float]:
    normalized = _finite_vector(value, 4, name)
    if not all(0.0 <= item <= 1.0 for item in normalized):
        raise ValueError(f"{name} values must be in [0, 1]")
    return normalized  # type: ignore[return-value]


def load_scene_condition_config(path: str | Path) -> dict[str, object]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("scene-condition config must contain a JSON object")
    schema_version = int(raw.get("schema_version", 0))
    if schema_version not in (1, 2, 3):
        raise ValueError("unsupported scene-condition schema version")
    expected_scope = {
        1: "NON_ACCEPTANCE_SCENE_INJECTION_PRE_GATE",
        2: "NON_ACCEPTANCE_MULTIPOSITION_SCENE_DIAGNOSTIC",
        3: "NON_ACCEPTANCE_SYNTHETIC_CAPTURE_PREFLIGHT",
    }[schema_version]
    if raw.get("scope") != expected_scope:
        raise ValueError("scene-condition scope does not match its schema")
    if raw.get("formal_acceptance") or raw.get("held_out_test_consumed"):
        raise ValueError("scene-condition pre-gate cannot assert formal acceptance")

    lighting = raw.get("lighting")
    occlusion = raw.get("occlusion")
    if not isinstance(lighting, Mapping) or set(lighting) != set(LIGHTING_LEVELS):
        raise ValueError("lighting levels must be dim, nominal, and bright")
    if not isinstance(occlusion, Mapping) or set(occlusion) != set(OCCLUSION_LEVELS):
        raise ValueError("occlusion levels must be none, partial, and heavy")
    for level in LIGHTING_LEVELS:
        entry = lighting[level]
        if not isinstance(entry, Mapping):
            raise ValueError(f"lighting {level} must be an object")
        _rgba(entry.get("ambient_rgba"), f"lighting.{level}.ambient_rgba")
        _rgba(entry.get("sun_diffuse_rgba"), f"lighting.{level}.sun_diffuse_rgba")
        _rgba(entry.get("sun_specular_rgba"), f"lighting.{level}.sun_specular_rgba")
    for level in OCCLUSION_LEVELS:
        entry = occlusion[level]
        if not isinstance(entry, Mapping) or not isinstance(entry.get("enabled"), bool):
            raise ValueError(f"occlusion {level} must declare boolean enabled")
        enabled = bool(entry["enabled"])
        if level == "none" and enabled:
            raise ValueError("none occlusion must be disabled")
        if level != "none":
            if not enabled or entry.get("visual_only") is not True:
                raise ValueError("partial/heavy occluders must be enabled and visual-only")
            _finite_vector(entry.get("pose_xyz_rpy"), 6, f"occlusion.{level}.pose")
            size = _finite_vector(entry.get("size_xyz_m"), 3, f"occlusion.{level}.size")
            if not all(item > 0.0 for item in size):
                raise ValueError("occluder dimensions must be positive")
            _rgba(entry.get("ambient_rgba"), f"occlusion.{level}.ambient_rgba")
            _rgba(entry.get("diffuse_rgba"), f"occlusion.{level}.diffuse_rgba")

    if schema_version >= 2:
        tracking = raw.get("occluder_tracking")
        if not isinstance(tracking, Mapping):
            raise ValueError("schema v2 requires occluder_tracking")
        if tracking.get("mode") != "anchor_relative_translation":
            raise ValueError("unsupported occluder tracking mode")
        anchor_target = _finite_vector(
            tracking.get("anchor_target_position_m"),
            3,
            "occluder_tracking.anchor_target_position_m",
        )
        anchor_occluder = _finite_vector(
            tracking.get("anchor_occluder_position_m"),
            3,
            "occluder_tracking.anchor_occluder_position_m",
        )
        scale = float(tracking.get("target_translation_scale", 0.0))
        maximum_offset = float(tracking.get("maximum_target_offset_m", 0.0))
        if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
            raise ValueError("target translation scale must be in (0, 1]")
        if not math.isfinite(maximum_offset) or maximum_offset <= 0.0:
            raise ValueError("maximum target offset must be positive")
        for level in ("partial", "heavy"):
            pose = _finite_vector(
                occlusion[level].get("pose_xyz_rpy"), 6, f"occlusion.{level}.pose"
            )
            if any(abs(pose[index] - anchor_occluder[index]) > 1e-12 for index in range(3)):
                raise ValueError("tracked occluder poses must use the anchor position")

    validation = raw.get("validation_scene")
    if not isinstance(validation, Mapping):
        raise ValueError("validation_scene must be an object")
    if int(validation.get("target_id", 0)) != 1:
        raise ValueError("scene pre-gate target_id is frozen to 1")
    if validation.get("target_model_name") != "strawberry_1":
        raise ValueError("scene pre-gate target model is frozen to strawberry_1")
    _finite_vector(validation.get("target_position_m"), 3, "target_position_m")
    if schema_version >= 2 and any(
        abs(float(validation["target_position_m"][index]) - anchor_target[index])
        > 1e-12
        for index in range(3)
    ):
        raise ValueError("schema v2 validation target must equal the tracking anchor")
    parked = validation.get("parked_models")
    if not isinstance(parked, Sequence) or len(parked) != 2:
        raise ValueError("validation scene must park exactly two non-target models")
    if {int(item["target_id"]) for item in parked} != {2, 3}:
        raise ValueError("validation scene parked IDs must be 2 and 3")
    if {str(item["model_name"]) for item in parked} != {
        "strawberry_2",
        "strawberry_3",
    }:
        raise ValueError("validation scene must park strawberry_2 and strawberry_3")
    for item in parked:
        _finite_vector(item.get("position_m"), 3, "parked position_m")
    radius = float(validation.get("fruit_radius_m", 0.0))
    frames = int(validation.get("frames_per_condition", 0))
    if not math.isfinite(radius) or radius <= 0.0 or frames <= 0:
        raise ValueError("fruit radius and frames_per_condition must be positive")
    if schema_version == 3:
        identifiers = raw.get("allowed_materialization_ids")
        if not isinstance(identifiers, Sequence) or isinstance(
            identifiers, (str, bytes)
        ):
            raise ValueError("schema v3 requires allowed_materialization_ids")
        normalized_ids = tuple(int(value) for value in identifiers)
        if normalized_ids != (20260601, 20260602):
            raise ValueError("synthetic materialization IDs differ from the frozen pair")
        if int(validation.get("seed", 0)) != normalized_ids[0]:
            raise ValueError("validation seed must equal the first materialization ID")
        if any(value in {20260710, 20260711, 20260712} for value in normalized_ids):
            raise ValueError("formal seed labels cannot be synthetic materialization IDs")

    thresholds = raw.get("pre_gate_thresholds")
    if not isinstance(thresholds, Mapping):
        raise ValueError("pre_gate_thresholds must be an object")
    for name, value in thresholds.items():
        number = float(value)
        if not math.isfinite(number) or number < 0.0:
            raise ValueError(f"threshold {name} must be non-negative and finite")
    return dict(raw)


def validate_benchmark_condition_mapping(
    config: Mapping[str, object],
    *,
    lighting_levels: Sequence[str],
    occlusion_levels: Sequence[str],
) -> list[dict[str, str]]:
    """Bind benchmark labels to the nine materialized condition identifiers."""

    normalized_lighting = tuple(str(value) for value in lighting_levels)
    normalized_occlusion = tuple(str(value) for value in occlusion_levels)
    if normalized_lighting != LIGHTING_LEVELS:
        raise ValueError("benchmark lighting order must be dim, nominal, bright")
    if normalized_occlusion != OCCLUSION_LEVELS:
        raise ValueError("benchmark occlusion order must be none, partial, heavy")
    if tuple(config["lighting"]) != LIGHTING_LEVELS:
        raise ValueError("scene config lighting order differs from the benchmark")
    if tuple(config["occlusion"]) != OCCLUSION_LEVELS:
        raise ValueError("scene config occlusion order differs from the benchmark")
    return [
        {
            "lighting": light,
            "occlusion": occlusion,
            "condition_id": f"{light}__{occlusion}",
        }
        for light in LIGHTING_LEVELS
        for occlusion in OCCLUSION_LEVELS
    ]


def _numbers(values: Sequence[float]) -> str:
    return " ".join(format(float(value), ".12g") for value in values)


def resolve_occluder_parameters(
    config: Mapping[str, object],
    occlusion_level: str,
    target_position_m: Sequence[float],
) -> dict[str, object]:
    """Return position-resolved visual occluder parameters."""

    target = _finite_vector(target_position_m, 3, "target_position_m")
    source = config["occlusion"][occlusion_level]
    resolved = dict(source)
    if not source["enabled"] or int(config["schema_version"]) == 1:
        return resolved
    tracking = config["occluder_tracking"]
    anchor_target = _finite_vector(
        tracking["anchor_target_position_m"], 3, "anchor_target_position_m"
    )
    anchor_occluder = _finite_vector(
        tracking["anchor_occluder_position_m"], 3, "anchor_occluder_position_m"
    )
    offset = math.dist(target, anchor_target)
    if offset > float(tracking["maximum_target_offset_m"]) + 1e-12:
        raise ValueError("target lies outside the frozen occluder tracking range")
    scale = float(tracking["target_translation_scale"])
    position = tuple(
        anchor_occluder[index] + scale * (target[index] - anchor_target[index])
        for index in range(3)
    )
    source_pose = _finite_vector(source["pose_xyz_rpy"], 6, "occluder pose")
    resolved["pose_xyz_rpy"] = [*position, *source_pose[3:]]
    resolved["tracking_target_position_m"] = list(target)
    resolved["tracking_target_offset_m"] = offset
    return resolved


def materialize_condition_world(
    *,
    base_world: str | Path,
    config_path: str | Path,
    lighting_level: str,
    occlusion_level: str,
    seed: int,
    output_world: str | Path,
    output_receipt: str | Path,
    target_position_m: Sequence[float] | None = None,
) -> dict[str, object]:
    config = load_scene_condition_config(config_path)
    if lighting_level not in LIGHTING_LEVELS or occlusion_level not in OCCLUSION_LEVELS:
        raise ValueError("unknown lighting or occlusion level")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("seed must be an integer")
    frozen_seed = int(config["validation_scene"]["seed"])
    allowed_seeds = (
        tuple(int(value) for value in config["allowed_materialization_ids"])
        if int(config["schema_version"]) == 3
        else (frozen_seed,)
    )
    if seed not in allowed_seeds:
        if int(config["schema_version"]) == 3:
            raise ValueError(
                "seed must match the frozen materialization contract "
                + ", ".join(str(value) for value in allowed_seeds)
            )
        raise ValueError(f"seed must match the frozen validation seed {frozen_seed}")
    configured_target = tuple(
        float(value) for value in config["validation_scene"]["target_position_m"]
    )
    requested_target = (
        _finite_vector(target_position_m, 3, "target_position_m")
        if target_position_m is not None
        else configured_target
    )
    if int(config["schema_version"]) == 1 and requested_target != configured_target:
        raise ValueError("schema v1 scene config does not allow target overrides")
    base_path = Path(base_world).resolve(strict=True)
    world_path = Path(output_world)
    receipt_path = Path(output_receipt)
    if world_path.exists() or receipt_path.exists():
        raise ValueError("refusing to overwrite scene-condition artifacts")

    tree = ET.parse(base_path)
    root = tree.getroot()
    world = root.find("./world")
    if world is None:
        raise ValueError("base SDF does not contain one world")
    scene = world.find("./scene")
    sun = world.find("./light[@name='sun']")
    if scene is None or sun is None:
        raise ValueError("base world lacks the frozen scene or sun light")
    lighting = config["lighting"][lighting_level]
    scene.find("ambient").text = _numbers(lighting["ambient_rgba"])
    sun.find("diffuse").text = _numbers(lighting["sun_diffuse_rgba"])
    sun.find("specular").text = _numbers(lighting["sun_specular_rgba"])

    for model in tuple(world.findall(f"./model[@name='{OCCLUDER_MODEL_NAME}']")):
        world.remove(model)
    occlusion = resolve_occluder_parameters(
        config, occlusion_level, requested_target
    )
    if occlusion["enabled"]:
        model = ET.SubElement(world, "model", {"name": OCCLUDER_MODEL_NAME})
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = _numbers(occlusion["pose_xyz_rpy"])
        link = ET.SubElement(model, "link", {"name": "occluder_link"})
        visual = ET.SubElement(link, "visual", {"name": "occluder_visual"})
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = _numbers(occlusion["size_xyz_m"])
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = _numbers(
            occlusion["ambient_rgba"]
        )
        ET.SubElement(material, "diffuse").text = _numbers(
            occlusion["diffuse_rgba"]
        )
        ET.SubElement(material, "specular").text = "0.02 0.02 0.02 1"

    world_path.parent.mkdir(parents=True, exist_ok=True)
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(world_path, encoding="utf-8", xml_declaration=True)
    validation_scene = dict(config["validation_scene"])
    validation_scene["target_position_m"] = list(requested_target)
    receipt = {
        "schema_version": int(config["schema_version"]),
        "scope": config["scope"],
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "lighting_level": lighting_level,
        "occlusion_level": occlusion_level,
        "seed": seed,
        "visual_only_occluder": bool(occlusion["enabled"]),
        "base_world": fingerprint(base_path),
        "condition_config": fingerprint(config_path),
        "materialized_world": fingerprint(world_path),
        "lighting_parameters": lighting,
        "occlusion_parameters": occlusion,
        "validation_scene": validation_scene,
        "occluder_tracking": config.get("occluder_tracking"),
        "materialization_id_role": (
            "synthetic_split_identifier_only"
            if int(config["schema_version"]) == 3
            else "frozen_validation_seed"
        ),
    }
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return receipt


def summarize_condition_matrix(
    probes: Sequence[Mapping[str, object]],
    config: Mapping[str, object],
) -> dict[str, object]:
    expected = {(light, occ) for light in LIGHTING_LEVELS for occ in OCCLUSION_LEVELS}
    by_condition: dict[tuple[str, str], Mapping[str, object]] = {}
    for probe in probes:
        key = (str(probe.get("lighting_level")), str(probe.get("occlusion_level")))
        if key not in expected or key in by_condition:
            raise ValueError(f"unexpected or duplicate condition {key}")
        by_condition[key] = probe
    thresholds = config["pre_gate_thresholds"]
    frames_required = int(config["validation_scene"]["frames_per_condition"])
    conditions_complete = set(by_condition) == expected and all(
        len(probe.get("frames", [])) == frames_required
        for probe in by_condition.values()
    )
    metrics = {}
    for key, probe in sorted(by_condition.items()):
        frames = probe.get("frames", [])
        luminance = [float(frame["luminance_mean_8bit"]) for frame in frames]
        coverage = [float(frame["blue_coverage_ratio"]) for frame in frames]
        metrics[f"{key[0]}__{key[1]}"] = {
            "frame_count": len(frames),
            "luminance_mean_8bit": mean(luminance) if luminance else None,
            "blue_coverage_ratio": mean(coverage) if coverage else None,
            "materialized_world_sha256": probe.get("materialized_world_sha256"),
        }

    lighting_checks = {}
    lighting_gap = float(thresholds["lighting_mean_gap_8bit_minimum"])
    for occ in OCCLUSION_LEVELS:
        values = [
            metrics[f"{light}__{occ}"]["luminance_mean_8bit"]
            for light in LIGHTING_LEVELS
        ]
        lighting_checks[occ] = bool(
            all(value is not None for value in values)
            and values[1] - values[0] >= lighting_gap
            and values[2] - values[1] >= lighting_gap
        )

    occlusion_checks = {}
    for light in LIGHTING_LEVELS:
        none = metrics[f"{light}__none"]["blue_coverage_ratio"]
        partial = metrics[f"{light}__partial"]["blue_coverage_ratio"]
        heavy = metrics[f"{light}__heavy"]["blue_coverage_ratio"]
        occlusion_checks[light] = bool(
            none is not None
            and partial is not None
            and heavy is not None
            and none <= float(thresholds["none_blue_coverage_maximum"])
            and partial >= float(thresholds["partial_blue_coverage_minimum"])
            and heavy >= float(thresholds["heavy_blue_coverage_minimum"])
            and heavy - partial
            >= float(thresholds["heavy_minus_partial_coverage_minimum"])
            and heavy <= float(thresholds["heavy_blue_coverage_maximum"])
        )
    world_hashes = {
        metric["materialized_world_sha256"] for metric in metrics.values()
    }
    unique_worlds = len(world_hashes) == len(expected) and None not in world_hashes
    passed = bool(
        conditions_complete
        and all(lighting_checks.values())
        and all(occlusion_checks.values())
        and unique_worlds
    )
    return {
        "schema_version": 1,
        "gate": "SCENE_CONDITION_INJECTION_PRE_GATE",
        "scope": config["scope"],
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "condition_count": len(by_condition),
        "conditions_complete": conditions_complete,
        "unique_materialized_worlds": unique_worlds,
        "lighting_checks": lighting_checks,
        "occlusion_checks": occlusion_checks,
        "thresholds": thresholds,
        "condition_metrics": metrics,
        "passed": passed,
    }
