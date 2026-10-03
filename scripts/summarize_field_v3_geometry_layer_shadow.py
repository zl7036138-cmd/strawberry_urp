#!/usr/bin/env python3
"""Validate and summarize a no-motion field-v3 geometry-layer Shadow run."""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def summarize(run_directory: Path) -> dict[str, Any]:
    """Return a fail-closed summary for one fresh no-motion run."""

    run_directory = run_directory.resolve()
    sources = {
        "shadow_window": run_directory / "shadow_window.json",
        "target_pose_accuracy": run_directory / "target_pose_accuracy.json",
        "collision_scene_audit": run_directory / "collision_scene_audit.json",
    }
    documents = {key: _load(path) for key, path in sources.items()}
    shadow = documents["shadow_window"]
    accuracy = documents["target_pose_accuracy"]
    audit = documents["collision_scene_audit"]
    errors = [float(sample["error_mm"]) for sample in accuracy.get("samples", [])]
    sigmas = [
        float(sample["position_sigma_m"])
        for sample in accuracy.get("samples", [])
    ]
    summary = shadow.get("summary", {})
    readiness = shadow.get("readiness_gate", {})
    checks = {
        "window_completed": shadow.get("completed") is True,
        "sixty_measured_frames": summary.get("frame_count") == 60,
        "sixty_ripe_frames": summary.get("frames_with_ripe_detection") == 60,
        "sixty_target_pose_frames": summary.get("frames_with_target_pose") == 60,
        "readiness_satisfied": readiness.get("satisfied") is True,
        "accuracy_passed": accuracy.get("passed") is True,
        "sixty_accuracy_samples": len(errors) == 60,
        "target_one_only": accuracy.get("observed_target_ids") == [1],
        "no_accuracy_control_commands": accuracy.get("control_commands_sent") == 0,
        "handoff_passed": audit.get("handoff_passed") is True,
        "zero_joint_delta": float(audit.get("observed_joint_delta_rad", 1.0)) == 0.0,
        "no_control_interface": audit.get("control_interface_created") is False,
        "no_pick_action": audit.get("pick_action_called") is False,
        "no_trajectory_command": audit.get("trajectory_command_sent") is False,
        "no_gripper_command": audit.get("gripper_command_sent") is False,
        "pick_not_authorized": audit.get("pick_authorized") is False,
        "all_collisions_present": not audit.get("collision_scene", {}).get(
            "missing_ids", ["missing"]
        ),
        "no_violations": not accuracy.get("violations") and not audit.get(
            "violations"
        ),
    }
    result = {
        "schema_version": 1,
        "kind": "field_v3_geometry_layer_shadow_summary",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "measurements": {
            "frame_count": summary.get("frame_count"),
            "ripe_detection_frame_count": summary.get(
                "frames_with_ripe_detection"
            ),
            "target_pose_frame_count": summary.get("frames_with_target_pose"),
            "accuracy_sample_count": len(errors),
            "median_error_mm": statistics.median(errors) if errors else None,
            "maximum_error_mm": max(errors) if errors else None,
            "median_position_sigma_m": (
                statistics.median(sigmas) if sigmas else None
            ),
            "observed_joint_delta_rad": audit.get("observed_joint_delta_rad"),
        },
        "bindings": {
            key: {
                "path": path.relative_to(ROOT).as_posix(),
                "size_bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
            for key, path in sources.items()
        },
        "scope": {
            "fixed_scene": True,
            "fixed_target": True,
            "fixed_seed": True,
            "natural_field_occlusion_only": True,
            "synthetic_benchmark_occlusion_tested": False,
            "robot_motion": False,
            "formal_p3_or_p4_evidence": False,
            "runtime_promotion_authorized": False,
            "held_out_real_test_consumed": False,
        },
    }
    output = run_directory / "summary.json"
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n"
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-directory", type=Path, required=True)
    arguments = parser.parse_args()
    result = summarize(arguments.run_directory)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
