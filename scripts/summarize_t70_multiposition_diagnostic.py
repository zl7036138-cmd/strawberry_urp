#!/usr/bin/env python3
"""Summarize the frozen T70-D2 five-position no-motion diagnostic."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from statistics import mean
import sys
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from validate_t70_multiposition_diagnostic import (  # noqa: E402
    load_diagnostic_manifest,
)
from strawberry_sim.scene_conditions import fingerprint  # noqa: E402


ERROR_MARKERS = (
    "traceback (most recent call last)",
    "exception was never retrieved",
    "process has died",
)


def classify_shadow_summary(summary: dict, required_frames: int = 60) -> str:
    """Classify the first missing stage without defining a performance gate."""

    ripe_frames = int(summary.get("frames_with_ripe_detection", 0))
    pose_frames = int(summary.get("frames_with_target_pose", 0))
    if ripe_frames == 0:
        return "DETECTION_ABSENT"
    if pose_frames < ripe_frames:
        return "LOCALIZATION_OR_PIPELINE_GAP"
    if ripe_frames <= required_frames and pose_frames <= required_frames:
        return "OBSERVATION_COMPLETE"
    raise ValueError("fixed-window summary exceeds its required frame count")


def _same_position(left: object, right: object) -> bool:
    try:
        first = tuple(float(value) for value in left)  # type: ignore[arg-type]
        second = tuple(float(value) for value in right)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    return len(first) == len(second) == 3 and all(
        math.isclose(a, b, abs_tol=1e-12) for a, b in zip(first, second)
    )


def _aggregate(records: Iterable[dict], key: str) -> dict[str, dict]:
    groups: dict[str, list[dict]] = {}
    for record in records:
        groups.setdefault(str(record[key]), []).append(record)
    result = {}
    for label, items in groups.items():
        total_frames = sum(int(item["fixed_shadow_frames"]) for item in items)
        ripe_frames = sum(int(item["ripe_detection_frames"]) for item in items)
        pose_frames = sum(int(item["target_pose_frames"]) for item in items)
        result[label] = {
            "scenario_count": len(items),
            "fixed_shadow_frames": total_frames,
            "ripe_detection_frames": ripe_frames,
            "target_pose_frames": pose_frames,
            "ripe_detection_frame_rate": (
                ripe_frames / total_frames if total_frames else None
            ),
            "target_pose_frame_rate": pose_frames / total_frames if total_frames else None,
            "classifications": dict(Counter(item["classification"] for item in items)),
        }
    return result


def summarize(
    output_dir: Path,
    manifest_path: Path,
    model_path: Path | None = None,
    base_domain_id: int | None = None,
) -> dict[str, object]:
    manifest = load_diagnostic_manifest(manifest_path, model_path)
    scenario_count = int(manifest["_scenario_count"])
    if base_domain_id is not None and not 0 <= base_domain_id <= 232 - scenario_count + 1:
        raise ValueError("fifteen diagnostic ROS domains must fit in [0, 232]")
    config_path = ROOT / manifest["condition_contract"][
        "scene_config_relative_path"
    ]
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config_hash = fingerprint(config_path)["sha256"]
    thresholds = config["pre_gate_thresholds"]
    measurement = manifest["measurement"]
    required_window = int(measurement["fixed_detection_frames"])
    required_probe = int(measurement["condition_probe_frames"])
    records = []
    blockers = []

    index = 0
    for position in manifest["positions"]:
        for condition in manifest["conditions"]:
            scenario_id = (
                f"multipos__{position['position_label']}__"
                f"{condition['condition_label']}"
            )
            directory = output_dir / scenario_id
            paths = {
                "receipt": directory / "receipt.json",
                "probe": directory / "condition_probe.json",
                "window": directory / "shadow_window.json",
                "launch_log": directory / "launch.log",
                "probe_log": directory / "condition_probe.log",
                "window_log": directory / "shadow_window.log",
            }
            errors = []
            receipt = probe = window = None
            try:
                receipt = json.loads(paths["receipt"].read_text(encoding="utf-8"))
                probe = json.loads(paths["probe"].read_text(encoding="utf-8"))
                window = json.loads(paths["window"].read_text(encoding="utf-8"))
                if receipt.get("schema_version") != 2:
                    errors.append("receipt does not use tracked-occluder schema v2")
                if receipt.get("lighting_level") != condition["lighting"]:
                    errors.append("receipt lighting mismatch")
                if receipt.get("occlusion_level") != condition["occlusion"]:
                    errors.append("receipt occlusion mismatch")
                if receipt.get("condition_config", {}).get("sha256") != config_hash:
                    errors.append("receipt config hash mismatch")
                if not _same_position(
                    receipt.get("validation_scene", {}).get("target_position_m"),
                    position["target_position_m"],
                ):
                    errors.append("receipt target position mismatch")
                if probe.get("receipt", {}).get("sha256") != fingerprint(
                    paths["receipt"]
                )["sha256"]:
                    errors.append("condition probe is not bound to its receipt")
                if probe.get("materialized_world_sha256") != receipt.get(
                    "materialized_world", {}
                ).get("sha256"):
                    errors.append("condition probe world hash mismatch")
                if len(probe.get("frames", [])) != required_probe:
                    errors.append("condition probe frame count mismatch")
                configured = {
                    int(item["target_id"]): item.get("position_m")
                    for item in probe.get("configured_models", [])
                }
                if not _same_position(configured.get(1), position["target_position_m"]):
                    errors.append("runtime target position was not acknowledged")
                if window.get("scenario_id") != scenario_id:
                    errors.append("fixed Shadow window scenario mismatch")
                if window.get("lighting") != condition["lighting"]:
                    errors.append("fixed Shadow window lighting mismatch")
                if window.get("occlusion") != condition["occlusion"]:
                    errors.append("fixed Shadow window occlusion mismatch")
                if int(window.get("required_frames", 0)) != required_window:
                    errors.append("fixed Shadow window contract mismatch")
                if len(window.get("frames", [])) != required_window:
                    errors.append("fixed Shadow window is incomplete")
                if window.get("window_boundary") != measurement["boundary"]:
                    errors.append("fixed Shadow window boundary mismatch")
                expected_domain = (
                    base_domain_id + index if base_domain_id is not None else None
                )
                if expected_domain is not None and int(
                    window.get("ros_domain_id", -1)
                ) != expected_domain:
                    errors.append("fixed Shadow window ROS domain mismatch")
            except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
                errors.append(f"missing or invalid runtime artifact: {error}")

            log_markers = []
            for name in ("launch_log", "probe_log", "window_log"):
                path = paths[name]
                if not path.exists():
                    errors.append(f"missing log: {path.name}")
                    continue
                text = path.read_text(encoding="utf-8", errors="replace").lower()
                log_markers.extend(marker for marker in ERROR_MARKERS if marker in text)
            if log_markers:
                errors.append(
                    "runtime log contains failure markers: "
                    + ", ".join(sorted(set(log_markers)))
                )

            frames = probe.get("frames", []) if isinstance(probe, dict) else []
            luminance = (
                mean(float(frame["luminance_mean_8bit"]) for frame in frames)
                if frames
                else None
            )
            coverage = (
                mean(float(frame["blue_coverage_ratio"]) for frame in frames)
                if frames
                else None
            )
            window_summary = (
                window.get("summary", {}) if isinstance(window, dict) else {}
            )
            fixed_frames = int(window_summary.get("frame_count", 0))
            ripe_frames = int(
                window_summary.get("frames_with_ripe_detection", 0)
            )
            pose_frames = int(window_summary.get("frames_with_target_pose", 0))
            try:
                classification = classify_shadow_summary(
                    window_summary, required_window
                )
            except (TypeError, ValueError):
                classification = "INVALID_RUNTIME_EVIDENCE"
                errors.append("fixed-window summary contains invalid counts")
            if errors:
                blockers.extend(f"{scenario_id}: {error}" for error in errors)
            records.append(
                {
                    "scenario_id": scenario_id,
                    "position_label": position["position_label"],
                    "source_position_label": position["source_position_label"],
                    "target_position_m": position["target_position_m"],
                    "condition_label": condition["condition_label"],
                    "purpose": condition["purpose"],
                    "lighting": condition["lighting"],
                    "occlusion": condition["occlusion"],
                    "ros_domain_id": (
                        base_domain_id + index if base_domain_id is not None else None
                    ),
                    "infrastructure_valid": not errors,
                    "errors": errors,
                    "luminance_mean_8bit": luminance,
                    "blue_coverage_ratio": coverage,
                    "fixed_shadow_frames": fixed_frames,
                    "ripe_detection_frames": ripe_frames,
                    "target_pose_frames": pose_frames,
                    "classification": classification,
                    "window_summary": window_summary,
                    "materialized_world_sha256": (
                        receipt.get("materialized_world", {}).get("sha256")
                        if isinstance(receipt, dict)
                        else None
                    ),
                    "artifacts": {
                        name: fingerprint(path) if path.exists() else None
                        for name, path in paths.items()
                    },
                }
            )
            index += 1

    by_key = {
        (record["position_label"], record["condition_label"]): record
        for record in records
    }
    position_checks = {}
    for position in manifest["positions"]:
        label = position["position_label"]
        baseline = by_key[(label, "nominal_none")]
        dim = by_key[(label, "dim_none")]
        heavy = by_key[(label, "nominal_heavy")]
        position_checks[label] = {
            "dim_is_darker_than_nominal": bool(
                baseline["luminance_mean_8bit"] is not None
                and dim["luminance_mean_8bit"] is not None
                and baseline["luminance_mean_8bit"]
                - dim["luminance_mean_8bit"]
                >= float(thresholds["lighting_mean_gap_8bit_minimum"])
            ),
            "none_has_no_blue_occluder": bool(
                baseline["blue_coverage_ratio"] is not None
                and dim["blue_coverage_ratio"] is not None
                and baseline["blue_coverage_ratio"]
                <= float(thresholds["none_blue_coverage_maximum"])
                and dim["blue_coverage_ratio"]
                <= float(thresholds["none_blue_coverage_maximum"])
            ),
            "heavy_is_visible_but_not_total": bool(
                heavy["blue_coverage_ratio"] is not None
                and heavy["blue_coverage_ratio"]
                >= float(thresholds["heavy_blue_coverage_minimum"])
                and heavy["blue_coverage_ratio"]
                <= float(thresholds["heavy_blue_coverage_maximum"])
            ),
        }
    distinct_worlds = {
        record["materialized_world_sha256"]
        for record in records
        if record["materialized_world_sha256"]
    }
    expected_worlds = int(measurement["expected_distinct_materialized_worlds"])
    world_count_valid = len(distinct_worlds) == expected_worlds
    if not world_count_valid:
        blockers.append(
            f"distinct materialized world count {len(distinct_worlds)} != {expected_worlds}"
        )
    fixed_frames_total = sum(record["fixed_shadow_frames"] for record in records)
    infrastructure_valid = all(record["infrastructure_valid"] for record in records)
    injection_valid = all(
        all(checks.values()) for checks in position_checks.values()
    )
    diagnostic_complete = bool(
        len(records) == scenario_count
        and infrastructure_valid
        and fixed_frames_total == scenario_count * required_window
        and world_count_valid
    )
    diagnostic_passed = bool(diagnostic_complete and injection_valid)
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "diagnostic_id": manifest["diagnostic_id"],
        "scope": manifest["scope"],
        "formal_acceptance": False,
        "may_close_t30_gate": False,
        "may_close_p2_gate": False,
        "may_close_p3_gate": False,
        "may_close_p4_gate": False,
        "held_out_test_consumed": False,
        "robot_motion_started": False,
        "independent_random_seed_count_claimed": 0,
        "seed_role": manifest["condition_contract"]["seed_role"],
        "shadow_model_role": manifest["shadow"]["model_role"],
        "manifest": fingerprint(manifest_path),
        "condition_config": fingerprint(config_path),
        "scenario_count": len(records),
        "position_count": len(manifest["positions"]),
        "condition_count": len(manifest["conditions"]),
        "fixed_shadow_window_frames": fixed_frames_total,
        "required_fixed_shadow_window_frames": scenario_count * required_window,
        "distinct_materialized_worlds": len(distinct_worlds),
        "expected_distinct_materialized_worlds": expected_worlds,
        "diagnostic_complete": diagnostic_complete,
        "condition_injection_valid": injection_valid,
        "diagnostic_passed": diagnostic_passed,
        "classification_counts": dict(Counter(record["classification"] for record in records)),
        "by_condition": _aggregate(records, "condition_label"),
        "by_position": _aggregate(records, "position_label"),
        "position_condition_checks": position_checks,
        "scenarios": records,
        "blockers": blockers,
        "interpretation": (
            "Five-position, one-seed, no-motion Shadow diagnostic only. A pass "
            "means evidence completeness and valid condition injection, not model "
            "accuracy or formal robustness acceptance."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--base-domain-id", type=int)
    options = parser.parse_args()
    summary_path = options.output_dir / "summary.json"
    if summary_path.exists():
        raise ValueError(f"refusing to overwrite summary: {summary_path}")
    result = summarize(
        options.output_dir,
        options.manifest.resolve(strict=True),
        options.model,
        options.base_domain_id,
    )
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "diagnostic_passed": result["diagnostic_passed"],
                "summary": str(summary_path),
            },
            sort_keys=True,
        )
    )
    return 0 if result["diagnostic_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
