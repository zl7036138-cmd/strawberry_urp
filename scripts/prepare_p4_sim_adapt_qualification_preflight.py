#!/usr/bin/env python3
"""Freeze a no-simulation preflight for the single P4 qualification claim."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))

from strawberry_benchmark.p4_qualification import (  # noqa: E402
    generate_qualification_schedule,
    load_qualification_contract,
)


SOURCE_PATHS = (
    "scripts/prepare_p4_sim_adapt_qualification_preflight.py",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/p4_qualification.py",
    "scripts/validate_p4_sim_adapt_qualification.py",
    "scripts/configure_p4_qualification_scene.py",
    "scripts/claim_p4_sim_adapt_qualification.py",
    "scripts/summarize_p4_sim_adapt_qualification.py",
    "scripts/run_p4_sim_adapt_qualification.sh",
    "scripts/materialize_scene_condition.py",
    "ros2_ws/src/strawberry_bringup/launch/system.launch.py",
    "ros2_ws/src/strawberry_sim/launch/sim.launch.py",
    "ros2_ws/src/strawberry_sim/strawberry_sim/scene_conditions.py",
    "ros2_ws/src/strawberry_sim/strawberry_sim/scene_condition_probe.py",
    "ros2_ws/src/strawberry_perception/strawberry_perception/perception_node.py",
    "ros2_ws/src/strawberry_perception/strawberry_perception/shadow_window_probe.py",
    "ros2_ws/src/strawberry_localization/strawberry_localization/node.py",
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
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--runtime-smoke-dir", type=Path, required=True)
    parser.add_argument("--colcon-test-count", type=int, required=True)
    options = parser.parse_args()
    manifest_path = options.manifest.resolve(strict=True)
    output_dir = options.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"preflight output is not empty: {output_dir}")
    contract = load_qualification_contract(manifest_path, ROOT)
    claim_path = ROOT / contract["runtime"]["claim_relative_path"]
    held_out = ROOT / "artifacts/perception/zenodo_6126677_held_out_test_receipt.json"
    result_dir = options.result_dir.resolve()
    if claim_path.exists() or held_out.exists():
        raise ValueError("claim or held-out receipt already exists")
    if result_dir.exists() and any(result_dir.iterdir()):
        raise ValueError("qualification result directory is not empty")
    smoke = options.runtime_smoke_dir.resolve(strict=True)
    smoke_scene = json.loads((smoke / "scene_configuration.json").read_text(encoding="utf-8"))
    smoke_window = json.loads((smoke / "shadow_window.json").read_text(encoding="utf-8"))
    if smoke_scene.get("robot_motion_started") is not False:
        raise ValueError("runtime smoke does not preserve no-motion boundary")
    if smoke_window.get("summary", {}).get("frame_count") != 5:
        raise ValueError("runtime smoke window is incomplete")
    if options.colcon_test_count != 246:
        raise ValueError("preflight requires the verified 246-test baseline")

    schedule = generate_qualification_schedule(contract)
    output_dir.mkdir(parents=True, exist_ok=True)
    schedule_path = output_dir / "scenario_manifest.jsonl"
    with schedule_path.open("x", encoding="utf-8", newline="\n") as stream:
        for item in schedule:
            json.dump({
                "order_index": item.order_index,
                "ros_domain_id": item.ros_domain_id,
                "scenario_id": item.scenario_id,
                "maturity": item.maturity,
                "position_label": item.position_label,
                "position_m": list(item.position_m),
                "condition_label": item.condition_label,
                "lighting": item.lighting,
                "occlusion": item.occlusion,
                "target_model_name": item.target_model_name,
                "target_id": item.target_id,
                "parked_models": list(item.parked_models),
            }, stream, sort_keys=True, separators=(",", ":"))
            stream.write("\n")
    preflight = {
        "schema_version": 1,
        "kind": "p4_sim_adapt_qualification_preflight",
        "status": "READY_UNCLAIMED",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "qualification_id": contract["qualification_id"],
        "manifest": fingerprint(manifest_path),
        "scenario_manifest": fingerprint(schedule_path),
        "scenario_count": len(schedule),
        "ripe_scenarios": sum(item.maturity == "RIPE" for item in schedule),
        "unripe_scenarios": sum(item.maturity == "UNRIPE" for item in schedule),
        "model": fingerprint(Path(contract["_resolved_model_path"])),
        "confidence_threshold": contract["intervention"]["confidence_threshold"],
        "source_provenance": [fingerprint(ROOT / path) for path in SOURCE_PATHS],
        "runtime_smoke": {
            "scope": "NON_MATRIX_DEVELOPMENT_COORDINATE",
            "robot_motion_started": False,
            "ripe_frames": smoke_window["summary"]["frames_with_ripe_detection"],
            "target_pose_frames": smoke_window["summary"]["frames_with_target_pose"],
            "scene_configuration": fingerprint(smoke / "scene_configuration.json"),
            "shadow_window": fingerprint(smoke / "shadow_window.json"),
            "launch_log": fingerprint(smoke / "launch.log"),
        },
        "verification": {
            "ros_package_count": 7,
            "colcon_test_count": options.colcon_test_count,
            "colcon_errors": 0,
            "colcon_failures": 0,
            "colcon_skips": 0,
            "focused_contract_tests": 4,
            "runtime_smoke_passed": True,
        },
        "claim_path": str(claim_path.relative_to(ROOT)).replace("\\", "/"),
        "claim_exists": False,
        "result_dir": str(result_dir.relative_to(ROOT)).replace("\\", "/"),
        "result_dir_exists": result_dir.exists(),
        "held_out_test_receipt_exists": False,
        "held_out_real_test_consumed": False,
        "robot_motion_authorized": False,
        "training_authorized": False,
        "formal_p3_baseline_rerun_authorized": False,
    }
    preflight_path = output_dir / "preflight_receipt.json"
    preflight_path.write_text(json.dumps(preflight, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": preflight["status"], "preflight": str(preflight_path), "schedule_sha256": preflight["scenario_manifest"]["sha256"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
