#!/usr/bin/env python3
"""Summarize Oracle-controlled, YOLO-shadow position diagnostics."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0,
    str(REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_benchmark"),
)

from strawberry_benchmark.diagnostics import (  # noqa: E402
    load_oracle_shadow_diagnostic_manifest,
    sha256_file,
    verify_shadow_model,
)


SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)


CONTACT_PATTERN = re.compile(
    r"found a contact between '([^']+)' .* and '([^']+)'"
)


def _file_sha256(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def _first_failure_stage(final_status: dict) -> str | None:
    for transition in reversed(final_status.get("state_history", [])):
        if transition.get("current") == "FAILED":
            return transition.get("previous")
    return None


def _diagnostic_failure_attribution(
    launch_text: str, *, motion_success: bool
) -> tuple[str | None, list[str]]:
    if motion_success:
        return None, []
    contacts = sorted(
        {
            name
            for match in CONTACT_PATTERN.finditer(launch_text)
            for name in match.groups()
            if name.startswith("strawberry_fruit_")
        }
    )
    non_target_contacts = [
        name for name in contacts if name != "strawberry_fruit_1"
    ]
    if non_target_contacts:
        return "NON_TARGET_FRUIT_COLLISION", non_target_contacts
    if "moveit ik failed at cartesian waypoint" in launch_text:
        return "CARTESIAN_IK_FAILURE", contacts
    if "cartesian endpoint missed its tolerance" in launch_text:
        return "CARTESIAN_ENDPOINT_TOLERANCE", contacts
    if "collision" in launch_text:
        return "OTHER_COLLISION", contacts
    return "UNCLASSIFIED_MOTION_FAILURE", contacts


def _requested_clearance_metrics(manifest, scenario) -> tuple[float | None, float | None]:
    if not manifest.parked_models:
        return None, None
    center_distance = min(
        math.dist(scenario.target_position_m, model.position_m)
        for model in manifest.parked_models
    )
    return center_distance, center_distance - 2.0 * manifest.fruit_collision_radius_m


def summarize(
    output_dir: Path,
    manifest_path: Path,
    *,
    model_path: Path | None = None,
    base_domain_id: int | None = None,
) -> dict:
    manifest = load_oracle_shadow_diagnostic_manifest(manifest_path)
    if base_domain_id is not None:
        if not 0 <= base_domain_id <= 232:
            raise ValueError("base_domain_id must be within [0, 232]")
        if base_domain_id + len(manifest.scenarios) - 1 > 232:
            raise ValueError("ROS domain range exceeds 232")
    verified_model = (
        verify_shadow_model(manifest, REPOSITORY_ROOT, model_path)
        if model_path is not None
        else None
    )

    records = []
    for index, scenario in enumerate(manifest.scenarios):
        result_path = output_dir / f"{scenario.scenario_id}.json"
        launch_path = output_dir / f"{scenario.scenario_id}_launch.log"
        payload = (
            json.loads(result_path.read_text(encoding="utf-8"))
            if result_path.exists()
            else {"error": "result JSON was not produced"}
        )
        launch_text = (
            launch_path.read_text(encoding="utf-8", errors="replace").lower()
            if launch_path.exists()
            else "launch log missing"
        )
        final_status = payload.get("final_status", {})
        no_client_error = "error" not in payload
        source_isolated = bool(payload.get("source_isolated", False))
        position_configured = bool(
            payload.get("position_configuration_applied", False)
        )
        parked_models_configured = bool(
            payload.get("parked_model_configuration_applied", False)
        )
        scene_setup_valid = (
            parked_models_configured if manifest.parked_models else True
        )
        shadow_evidence = bool(payload.get("shadow_evidence_present", False))
        shutdown_clean = not any(
            marker in launch_text for marker in SHUTDOWN_ERROR_MARKERS
        ) and launch_path.exists()
        infrastructure_valid = bool(
            result_path.exists()
            and no_client_error
            and source_isolated
            and position_configured
            and scene_setup_valid
            and shadow_evidence
            and shutdown_clean
        )
        shadow_frames = int(payload.get("shadow_detection_frames", 0))
        shadow_detections = int(payload.get("shadow_detection_count", 0))
        shadow_ripe = int(payload.get("shadow_ripe_detection_count", 0))
        shadow_targets = int(payload.get("shadow_target_observations", 0))
        motion_success = bool(payload.get("success", False))
        if not no_client_error or not position_configured or not scene_setup_valid:
            failure_attribution, collision_objects = "INFRASTRUCTURE_FAILURE", []
        else:
            failure_attribution, collision_objects = _diagnostic_failure_attribution(
                launch_text,
                motion_success=motion_success,
            )
        center_distance, surface_clearance = _requested_clearance_metrics(
            manifest, scenario
        )
        records.append(
            {
                "scenario_id": scenario.scenario_id,
                "position_label": scenario.position_label,
                "requested_target_position_m": list(scenario.target_position_m),
                "requested_min_non_target_center_distance_m": center_distance,
                "requested_min_surface_clearance_m": surface_clearance,
                "configured_truth_position_m": payload.get(
                    "configured_truth_position_m"
                ),
                "configured_control_position_m": payload.get(
                    "configured_control_position_m"
                ),
                "ros_domain_id": (
                    base_domain_id + index if base_domain_id is not None else None
                ),
                "infrastructure_valid": infrastructure_valid,
                "source_isolated": source_isolated,
                "position_configured": position_configured,
                "scene_setup_valid": scene_setup_valid,
                "parked_models_configured": parked_models_configured,
                "configured_parked_model_poses": payload.get(
                    "configured_parked_model_poses", []
                ),
                "pose_configuration_attempts": payload.get(
                    "pose_configuration_attempts", []
                ),
                "shadow_evidence_present": shadow_evidence,
                "shutdown_clean": shutdown_clean,
                "oracle_motion_success": motion_success,
                "state_machine_complete": bool(
                    payload.get("state_machine_complete", False)
                ),
                "terminal_state": final_status.get("state"),
                "outcome": final_status.get("outcome"),
                "first_failure_stage": _first_failure_stage(final_status),
                "failure_code": payload.get("failure_code"),
                "failure_message": payload.get(
                    "failure_message", payload.get("error", "")
                ),
                "diagnostic_failure_attribution": failure_attribution,
                "collision_objects": collision_objects,
                "planning_time_sec": payload.get("planning_time_sec"),
                "execution_time_sec": payload.get("execution_time_sec"),
                "shadow_detection_frames": shadow_frames,
                "shadow_detection_count": shadow_detections,
                "shadow_ripe_detection_count": shadow_ripe,
                "shadow_target_observations": shadow_targets,
                "shadow_has_ripe_detection": shadow_ripe > 0,
                "shadow_has_target_pose": shadow_targets > 0,
                "result_file": result_path.name,
                "launch_log": launch_path.name,
                "result_sha256": _file_sha256(result_path),
                "launch_log_sha256": _file_sha256(launch_path),
            }
        )

    scenario_count = len(records)
    oracle_successes = sum(record["oracle_motion_success"] for record in records)
    shadow_ripe_scenarios = sum(
        record["shadow_has_ripe_detection"] for record in records
    )
    shadow_target_scenarios = sum(
        record["shadow_has_target_pose"] for record in records
    )
    diagnostic_complete = all(record["infrastructure_valid"] for record in records)
    failure_attribution_counts = Counter(
        record["diagnostic_failure_attribution"]
        for record in records
        if record["diagnostic_failure_attribution"] is not None
    )
    return {
        "schema_version": 1,
        "diagnostic_id": manifest.diagnostic_id,
        "scope": manifest.scope,
        "formal_acceptance": False,
        "may_close_t30_gate": False,
        "may_close_p2_gate": False,
        "may_close_p3_gate": False,
        "may_close_p4_gate": False,
        "held_out_test_consumed": False,
        "control_source": "oracle",
        "control_topic": manifest.control_topic,
        "shadow_model_role": manifest.shadow_model_role,
        "shadow_model_path": str(verified_model) if verified_model else None,
        "shadow_model_sha256": (
            sha256_file(verified_model)
            if verified_model
            else manifest.shadow_model_sha256
        ),
        "shadow_confidence_threshold": manifest.confidence_threshold,
        "conditions": {
            "occlusion": manifest.occlusion,
            "lighting": manifest.lighting,
            "seed": manifest.seed,
            "formal_position_labels": False,
        },
        "scene_setup": {
            "mode": manifest.scene_mode,
            "fruit_collision_radius_m": manifest.fruit_collision_radius_m,
            "parked_models": [
                {
                    "model_name": model.model_name,
                    "target_id": model.target_id,
                    "position_m": list(model.position_m),
                }
                for model in manifest.parked_models
            ],
        },
        "scenario_count": scenario_count,
        "diagnostic_complete": diagnostic_complete,
        "oracle_motion_successes": oracle_successes,
        "oracle_motion_success_rate": oracle_successes / scenario_count,
        "shadow_ripe_scenario_count": shadow_ripe_scenarios,
        "shadow_ripe_scenario_rate": shadow_ripe_scenarios / scenario_count,
        "shadow_target_scenario_count": shadow_target_scenarios,
        "shadow_target_scenario_rate": shadow_target_scenarios / scenario_count,
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
        "failure_attribution_counts": dict(sorted(failure_attribution_counts.items())),
        "requested_surface_clearance_range_m": (
            {
                "minimum": min(
                    record["requested_min_surface_clearance_m"] for record in records
                ),
                "maximum": max(
                    record["requested_min_surface_clearance_m"] for record in records
                ),
            }
            if manifest.parked_models
            else None
        ),
        "interpretation": (
            "Bounded camera-clear position diagnostic only. Oracle is the sole "
            "motion source; YOLO is observational. Results cannot substitute for "
            "the sealed real-image test or the formal 135+30 robustness matrix."
        ),
        "formal_benchmark_blockers": [
            "T30 audited validation macro-F1 remains below 0.85",
            "perception-controlled motion remains disabled",
            "runtime occlusion and lighting scenario injection is not yet validated",
        ],
        "manifest_path": str(manifest_path.resolve()),
        "manifest_sha256": sha256_file(manifest_path),
        "trials": records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--base-domain-id", type=int)
    arguments = parser.parse_args()
    result = summarize(
        arguments.output_dir,
        arguments.manifest,
        model_path=arguments.model,
        base_domain_id=arguments.base_domain_id,
    )
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["diagnostic_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
