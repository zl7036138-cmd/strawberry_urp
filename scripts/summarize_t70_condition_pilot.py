#!/usr/bin/env python3
"""Summarize the non-acceptance Oracle-control condition pilot."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from statistics import mean
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from validate_t70_condition_pilot import load_pilot_manifest  # noqa: E402
from strawberry_sim.scene_conditions import fingerprint  # noqa: E402


ERROR_MARKERS = (
    "traceback (most recent call last)",
    "exception was never retrieved",
    "process has died",
)


def _first_failure_stage(payload: dict) -> str | None:
    for transition in reversed(payload.get("final_status", {}).get("state_history", [])):
        if transition.get("current") == "FAILED":
            return transition.get("previous")
    return None


def summarize(
    output_dir: Path,
    manifest_path: Path,
    model_path: Path | None = None,
    base_domain_id: int | None = None,
) -> dict[str, object]:
    manifest = load_pilot_manifest(manifest_path, model_path)
    if base_domain_id is not None and not 0 <= base_domain_id <= 230:
        raise ValueError("three pilot ROS domains must fit in [0, 232]")
    config_path = ROOT / manifest["condition_contract"][
        "scene_config_relative_path"
    ]
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config_hash = fingerprint(config_path)["sha256"]
    thresholds = config["pre_gate_thresholds"]
    records = []
    blockers = []

    for index, condition in enumerate(manifest["pilot_conditions"]):
        scenario_id = condition["scenario_id"]
        directory = output_dir / scenario_id
        paths = {
            "receipt": directory / "receipt.json",
            "probe": directory / "condition_probe.json",
            "result": directory / "trial.json",
            "launch_log": directory / "launch.log",
            "probe_log": directory / "condition_probe.log",
            "client_log": directory / "client.log",
        }
        if int(manifest["schema_version"]) >= 2:
            paths["shadow_window"] = directory / "shadow_window.json"
            paths["shadow_window_log"] = directory / "shadow_window.log"
        errors = []
        try:
            receipt = json.loads(paths["receipt"].read_text(encoding="utf-8"))
            probe = json.loads(paths["probe"].read_text(encoding="utf-8"))
            payload = json.loads(paths["result"].read_text(encoding="utf-8"))
            if receipt.get("lighting_level") != condition["lighting"]:
                errors.append("receipt lighting mismatch")
            if receipt.get("occlusion_level") != condition["occlusion"]:
                errors.append("receipt occlusion mismatch")
            if receipt.get("condition_config", {}).get("sha256") != config_hash:
                errors.append("receipt config hash mismatch")
            if probe.get("receipt", {}).get("sha256") != fingerprint(
                paths["receipt"]
            )["sha256"]:
                errors.append("condition probe is not bound to its receipt")
            if probe.get("materialized_world_sha256") != receipt.get(
                "materialized_world", {}
            ).get("sha256"):
                errors.append("condition probe world hash mismatch")
            if len(probe.get("frames", [])) != 5:
                errors.append("condition probe did not collect five frames")
        except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError) as error:
            receipt, probe, payload = None, None, {"error": str(error)}
            errors.append(f"missing or invalid runtime artifact: {error}")

        log_markers = []
        log_names = ["launch_log", "probe_log", "client_log"]
        if int(manifest["schema_version"]) >= 2:
            log_names.append("shadow_window_log")
        for name in log_names:
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

        source_isolated = bool(payload.get("source_isolated", False))
        position_configured = bool(
            payload.get("position_configuration_applied", False)
        )
        parked_configured = bool(
            payload.get("parked_model_configuration_applied", False)
        )
        shadow_evidence = bool(payload.get("shadow_evidence_present", False))
        if not source_isolated:
            errors.append("Oracle/Shadow source isolation was not proven")
        if not position_configured or not parked_configured:
            errors.append("single-target scene setup was not acknowledged")
        if not shadow_evidence:
            errors.append("Shadow detection frames were not observed")

        shadow_window = None
        if int(manifest["schema_version"]) >= 2:
            try:
                shadow_window = json.loads(
                    paths["shadow_window"].read_text(encoding="utf-8")
                )
                required = int(
                    manifest["shadow_measurement"]["fixed_detection_frames"]
                )
                if shadow_window.get("scenario_id") != scenario_id:
                    errors.append("fixed Shadow window scenario mismatch")
                if shadow_window.get("lighting") != condition["lighting"]:
                    errors.append("fixed Shadow window lighting mismatch")
                if shadow_window.get("occlusion") != condition["occlusion"]:
                    errors.append("fixed Shadow window occlusion mismatch")
                if int(shadow_window.get("required_frames", 0)) != required:
                    errors.append("fixed Shadow window contract mismatch")
                if len(shadow_window.get("frames", [])) != required:
                    errors.append("fixed Shadow window is incomplete")
                if shadow_window.get("window_boundary") != manifest[
                    "shadow_measurement"
                ]["boundary"]:
                    errors.append("fixed Shadow window boundary mismatch")
            except (FileNotFoundError, json.JSONDecodeError, TypeError, ValueError) as error:
                errors.append(f"missing or invalid fixed Shadow window: {error}")

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
        infrastructure_valid = not errors
        if errors:
            blockers.extend(f"{scenario_id}: {error}" for error in errors)
        records.append(
            {
                "scenario_id": scenario_id,
                "purpose": condition["purpose"],
                "lighting": condition["lighting"],
                "occlusion": condition["occlusion"],
                "ros_domain_id": (
                    base_domain_id + index if base_domain_id is not None else None
                ),
                "infrastructure_valid": infrastructure_valid,
                "errors": errors,
                "luminance_mean_8bit": luminance,
                "blue_coverage_ratio": coverage,
                "oracle_motion_success": bool(payload.get("success", False)),
                "terminal_state": payload.get("final_status", {}).get("state"),
                "first_failure_stage": _first_failure_stage(payload),
                "failure_code": payload.get("failure_code"),
                "failure_message": payload.get(
                    "failure_message", payload.get("error", "")
                ),
                "shadow_detection_frames": int(
                    payload.get("shadow_detection_frames", 0)
                ),
                "shadow_detection_count": int(
                    payload.get("shadow_detection_count", 0)
                ),
                "shadow_ripe_detection_count": int(
                    payload.get("shadow_ripe_detection_count", 0)
                ),
                "shadow_target_observations": int(
                    payload.get("shadow_target_observations", 0)
                ),
                "fixed_shadow_window": (
                    shadow_window.get("summary")
                    if isinstance(shadow_window, dict)
                    else None
                ),
                "artifacts": {
                    name: fingerprint(path) if path.exists() else None
                    for name, path in paths.items()
                },
            }
        )

    by_id = {record["scenario_id"]: record for record in records}
    baseline = by_id["pilot_nominal_none"]
    dim = by_id["pilot_dim_none"]
    heavy = by_id["pilot_nominal_heavy"]
    checks = {
        "dim_is_darker_than_nominal": bool(
            dim["luminance_mean_8bit"] is not None
            and baseline["luminance_mean_8bit"] is not None
            and baseline["luminance_mean_8bit"] - dim["luminance_mean_8bit"]
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
    diagnostic_complete = all(record["infrastructure_valid"] for record in records)
    oracle_successes = sum(record["oracle_motion_success"] for record in records)
    pilot_passed = bool(
        diagnostic_complete
        and all(checks.values())
        and oracle_successes == len(records)
    )
    return {
        "schema_version": int(manifest["schema_version"]),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "diagnostic_id": manifest["diagnostic_id"],
        "scope": manifest["scope"],
        "formal_acceptance": False,
        "may_close_t30_gate": False,
        "may_close_p2_gate": False,
        "may_close_p3_gate": False,
        "may_close_p4_gate": False,
        "held_out_test_consumed": False,
        "control_source": "oracle",
        "shadow_model_role": manifest["shadow"]["model_role"],
        "manifest": fingerprint(manifest_path),
        "condition_config": fingerprint(config_path),
        "condition_mapping_count": len(manifest["_condition_mapping"]),
        "scenario_count": len(records),
        "diagnostic_complete": diagnostic_complete,
        "oracle_motion_successes": oracle_successes,
        "oracle_motion_success_rate": oracle_successes / len(records),
        "shadow_detection_frames": sum(
            record["shadow_detection_frames"] for record in records
        ),
        "shadow_detection_count": sum(
            record["shadow_detection_count"] for record in records
        ),
        "shadow_ripe_detection_count": sum(
            record["shadow_ripe_detection_count"] for record in records
        ),
        "shadow_target_observations": sum(
            record["shadow_target_observations"] for record in records
        ),
        "fixed_shadow_window_frames": sum(
            int((record["fixed_shadow_window"] or {}).get("frame_count", 0))
            for record in records
        ),
        "condition_checks": checks,
        "trials": records,
        "blockers": blockers,
        "pilot_passed": pilot_passed,
        "interpretation": (
            "Bounded Oracle-control, YOLO-shadow condition pilot only. It cannot "
            "replace the formal 135+30 benchmark or the sealed real-image test."
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
    print(json.dumps({"pilot_passed": result["pilot_passed"], "summary": str(summary_path)}))
    return 0 if result["pilot_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
