#!/usr/bin/env python3
"""Summarize the bounded, waived perception-control smoke trial."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0,
    str(REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_benchmark"),
)

from strawberry_benchmark.perception_control import (  # noqa: E402
    load_perception_control_waiver,
    sha256_file,
    verify_perception_control_waiver,
)


SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)

PROVENANCE_PATHS = (
    "config/p3_perception_control_waiver_v1.json",
    "docs/decisions/0026-accept-below-gate-model-for-simulation-control.md",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/perception_control.py",
    "ros2_ws/src/strawberry_bringup/launch/system.launch.py",
    "ros2_ws/src/strawberry_bringup/strawberry_bringup/orchestrator.py",
    "ros2_ws/src/strawberry_perception/strawberry_perception/perception_node.py",
    "ros2_ws/src/strawberry_localization/strawberry_localization/node.py",
    "ros2_ws/src/strawberry_manipulation/strawberry_manipulation/moveit_backend.py",
    "scripts/test_orchestrated_trial.py",
    "scripts/validate_p3_perception_control_waiver.py",
    "scripts/run_p3_perception_control_smoke.sh",
    "scripts/summarize_p3_perception_control_smoke.py",
)


def _fingerprint(path: Path) -> dict:
    content = path.read_bytes()
    return {
        "path": path.relative_to(REPOSITORY_ROOT).as_posix(),
        "size_bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def _optional_sha256(path: Path) -> str | None:
    return sha256_file(path) if path.exists() else None


def summarize(
    output_dir: Path,
    manifest_path: Path,
    *,
    model_path: Path | None = None,
    ros_domain_id: int | None = None,
) -> dict:
    waiver = load_perception_control_waiver(manifest_path)
    verified = verify_perception_control_waiver(
        waiver, REPOSITORY_ROOT, model_path=model_path
    )
    if ros_domain_id is not None and not 0 <= ros_domain_id <= 232:
        raise ValueError("ROS domain ID must be within [0, 232]")

    result_path = output_dir / "trial_01.json"
    launch_path = output_dir / "trial_01_launch.log"
    payload = (
        json.loads(result_path.read_text(encoding="utf-8"))
        if result_path.exists()
        else {"success": False, "error": "result JSON was not produced"}
    )
    launch_text = (
        launch_path.read_text(encoding="utf-8", errors="replace").lower()
        if launch_path.exists()
        else "launch log missing"
    )
    final_status = payload.get("final_status", {})
    shutdown_clean = launch_path.exists() and not any(
        marker in launch_text for marker in SHUTDOWN_ERROR_MARKERS
    )
    source_isolated = bool(payload.get("source_isolated", False))
    state_machine_complete = bool(payload.get("state_machine_complete", False))
    scene_configured = bool(payload.get("position_configuration_applied", False)) and bool(
        payload.get("parked_model_configuration_applied", False)
    )
    motion_success = bool(payload.get("success", False))
    smoke_passed = (
        motion_success
        and source_isolated
        and state_machine_complete
        and scene_configured
        and shutdown_clean
        and final_status.get("target_source") == "perception"
        and final_status.get("control_target_topic") == waiver.control_target_topic
        and final_status.get("control_target_id") == waiver.target.target_id
    )

    return {
        "schema_version": 1,
        "kind": "p3_perception_control_waiver_smoke",
        "waiver_id": waiver.waiver_id,
        "scope": waiver.scope,
        "engineering_waiver_active": True,
        "numeric_t30_gate_passed": False,
        "numeric_macro_f1": waiver.macro_f1,
        "required_macro_f1": waiver.required_macro_f1,
        "t30_engineering_status": waiver.engineering_status,
        "formal_acceptance": False,
        "may_close_formal_p3_gate": False,
        "formal_135_plus_30_matrix_consumed": False,
        "held_out_real_test_consumed": False,
        "held_out_test_receipt_exists": verified["held_out_receipt"].exists(),
        "control_source": "perception",
        "control_topic": waiver.control_target_topic,
        "oracle_provider_started": False,
        "model": {
            "path": str(verified["model"]),
            "size_bytes": verified["model"].stat().st_size,
            "sha256": sha256_file(verified["model"]),
            "confidence_threshold": waiver.confidence_threshold,
            "image_size": waiver.image_size,
        },
        "scenario_id": waiver.scenario_id,
        "ros_domain_id": ros_domain_id,
        "trial_count": 1,
        "successes": int(motion_success),
        "development_smoke_passed": smoke_passed,
        "perception_control_ready_for_bounded_trials": smoke_passed,
        "trial": {
            "success": motion_success,
            "source_isolated": source_isolated,
            "state_machine_complete": state_machine_complete,
            "scene_configured": scene_configured,
            "shutdown_clean": shutdown_clean,
            "terminal_state": final_status.get("state"),
            "outcome": final_status.get("outcome"),
            "control_target_id": final_status.get("control_target_id"),
            "planning_time_sec": payload.get("planning_time_sec"),
            "execution_time_sec": payload.get("execution_time_sec"),
            "failure_code": payload.get("failure_code"),
            "failure_message": payload.get("failure_message", payload.get("error", "")),
            "result_file": result_path.name,
            "launch_log": launch_path.name,
            "result_sha256": _optional_sha256(result_path),
            "launch_log_sha256": _optional_sha256(launch_path),
        },
        "interpretation": (
            "One bounded clear-scene engineering smoke under ADR 0026. A pass "
            "proves only that the waived detector can command the simulator "
            "pipeline; it does not satisfy the original T30 metric or formal P3."
        ),
        "remaining_before_formal_p3": [
            "freeze a separate perception-controlled trial matrix and acceptance contract",
            "run the required simple-scene repeated trials before robustness expansion",
            "retain the detector metric waiver in every report",
        ],
        "manifest": _fingerprint(manifest_path.resolve()),
        "provenance": [
            _fingerprint(REPOSITORY_ROOT / relative_path)
            for relative_path in PROVENANCE_PATHS
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--ros-domain-id", type=int)
    arguments = parser.parse_args()
    result = summarize(
        arguments.output_dir,
        arguments.manifest,
        model_path=arguments.model,
        ros_domain_id=arguments.ros_domain_id,
    )
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["development_smoke_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
