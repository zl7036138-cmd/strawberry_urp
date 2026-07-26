#!/usr/bin/env python3
"""Summarize the single no-motion P4 simulator-adaptation qualification."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))

from strawberry_benchmark.p4_qualification import (  # noqa: E402
    evaluate_qualification_records,
    generate_qualification_schedule,
    load_qualification_contract,
)


def fingerprint(path: Path) -> dict:
    return {
        "path": str(path.resolve().relative_to(ROOT.resolve())).replace("\\", "/"),
        "size_bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    options = parser.parse_args()
    manifest_path = options.manifest.resolve(strict=True)
    output_dir = options.output_dir.resolve(strict=True)
    summary_path = output_dir / "summary.json"
    if summary_path.exists():
        raise ValueError(f"refusing to overwrite summary: {summary_path}")
    contract = load_qualification_contract(manifest_path, ROOT)
    records = []
    scenarios = []
    for scenario in generate_qualification_schedule(contract):
        directory = output_dir / "trials" / f"{scenario.order_index:03d}"
        paths = {
            name: directory / filename
            for name, filename in {
                "condition_probe": "condition_probe.json",
                "scene_configuration": "scene_configuration.json",
                "shadow_window": "shadow_window.json",
                "launch_log": "launch.log",
                "shutdown_marker": "shutdown.ok",
            }.items()
        }
        complete_files = all(path.exists() for path in paths.values())
        condition = scene = window = {}
        parse_valid = False
        if complete_files:
            try:
                condition = json.loads(paths["condition_probe"].read_text(encoding="utf-8"))
                scene = json.loads(paths["scene_configuration"].read_text(encoding="utf-8"))
                window = json.loads(paths["shadow_window"].read_text(encoding="utf-8"))
                parse_valid = True
            except (json.JSONDecodeError, KeyError, TypeError):
                parse_valid = False
        window_summary = window.get("summary", {}) if parse_valid else {}
        frame_count = int(window_summary.get("frame_count", 0))
        scene_positions = {
            int(item.get("target_id", 0)): tuple(float(value) for value in item.get("position_m", []))
            for item in scene.get("configured_models", [])
            if len(item.get("position_m", [])) == 3
        } if parse_valid else {}
        position_valid = scenario.target_id in scene_positions and math.dist(
            scene_positions.get(scenario.target_id, (99.0, 99.0, 99.0)), scenario.position_m
        ) <= 0.005
        launch_text = paths["launch_log"].read_text(encoding="utf-8", errors="replace") if paths["launch_log"].exists() else ""
        infrastructure_valid = bool(
            parse_valid
            and len(condition.get("frames", [])) == 5
            and condition.get("lighting_level") == scenario.lighting
            and condition.get("occlusion_level") == scenario.occlusion
            and scene.get("maturity") == scenario.maturity
            and int(scene.get("target_id", 0)) == scenario.target_id
            and scene.get("robot_motion_started") is False
            and scene.get("manipulation_started") is False
            and scene.get("orchestrator_started") is False
            and int(scene.get("settled_camera_frames", 0)) >= 10
            and position_valid
            and window.get("scenario_id") == scenario.scenario_id
            and window.get("lighting") == scenario.lighting
            and window.get("occlusion") == scenario.occlusion
            and frame_count == 60
            and "pick_and_place_server" not in launch_text
            and "strawberry_orchestrator" not in launch_text
            and paths["shutdown_marker"].exists()
        )
        record = {
            "scenario_id": scenario.scenario_id,
            "order_index": scenario.order_index,
            "ros_domain_id": scenario.ros_domain_id,
            "maturity": scenario.maturity,
            "position_label": scenario.position_label,
            "condition_label": scenario.condition_label,
            "lighting": scenario.lighting,
            "occlusion": scenario.occlusion,
            "frame_count": frame_count,
            "ripe_frames": int(window_summary.get("frames_with_ripe_detection", 0)),
            "unripe_frames": int(window_summary.get("frames_with_unripe_detection", 0)),
            "target_pose_frames": int(window_summary.get("frames_with_target_pose", 0)),
            "infrastructure_valid": infrastructure_valid,
            "robot_motion_started": False if parse_valid and scene.get("robot_motion_started") is False else None,
        }
        records.append(record)
        scenarios.append({
            **record,
            "condition_probe": fingerprint(paths["condition_probe"]) if paths["condition_probe"].exists() else None,
            "scene_configuration": fingerprint(paths["scene_configuration"]) if paths["scene_configuration"].exists() else None,
            "shadow_window": fingerprint(paths["shadow_window"]) if paths["shadow_window"].exists() else None,
            "launch_log": fingerprint(paths["launch_log"]) if paths["launch_log"].exists() else None,
        })
    metrics = evaluate_qualification_records(contract, records)
    held_out = ROOT / "artifacts/perception/zenodo_6126677_held_out_test_receipt.json"
    claim_path = ROOT / contract["runtime"]["claim_relative_path"]
    result = {
        "schema_version": 1,
        "kind": "p4_sim_adapt_qualification_summary",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "qualification_id": contract["qualification_id"],
        "qualification_passed": metrics["qualification_passed"] and claim_path.exists() and not held_out.exists(),
        "metrics": metrics,
        "manifest": fingerprint(manifest_path),
        "claim": fingerprint(claim_path) if claim_path.exists() else None,
        "model": fingerprint(Path(contract["_resolved_model_path"])),
        "candidate_real_validation_status": "REJECTED_REGRESSION",
        "candidate_real_validation_macro_f1": contract["intervention"]["real_validation_macro_f1"],
        "formal_p3_baseline_remains_failed": True,
        "formal_p3_baseline_rerun": False,
        "held_out_test_receipt_exists": held_out.exists(),
        "held_out_real_test_consumed": False,
        "robot_motion_started": False,
        "scenarios": scenarios,
        "interpretation": "Simulator-only no-motion qualification; not real-image, physical-robot, sim-to-real, P3, or P4 acceptance.",
    }
    summary_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"qualification_passed": result["qualification_passed"], "summary": str(summary_path)}, sort_keys=True))
    return 0 if result["qualification_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
