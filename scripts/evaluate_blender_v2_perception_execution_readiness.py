#!/usr/bin/env python3
"""Apply a geometry-aware execution interlock to one perceived fruit pose."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import yaml


def fingerprint(path: Path) -> dict[str, object]:
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def _finite_xyz(values: Sequence[object], label: str) -> tuple[float, ...]:
    if len(values) != 3:
        raise ValueError(f"{label} must contain three values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result


def evaluate_execution_readiness(
    *,
    handoff: Mapping[str, object],
    no_motion_gate: Mapping[str, object],
    scene: Mapping[str, object],
    geometry_gate: Mapping[str, object],
) -> dict[str, object]:
    if handoff.get("handoff_passed") is not True:
        raise ValueError("handoff input did not pass")
    if no_motion_gate.get("passed") is not True:
        raise ValueError("no-motion gate input did not pass")
    if geometry_gate.get("passed") is not True:
        raise ValueError("gripper geometry input did not pass")
    target_id = int(handoff.get("expected_target_id", 0))
    frozen = handoff.get("frozen_target_pose") or {}
    if int(frozen.get("target_id", 0)) != target_id:
        raise ValueError("frozen target identity changed")
    estimated = _finite_xyz(
        frozen.get("position_xyz_m", []), "estimated centre"
    )
    fruit_records = {
        int(record["target_id"]): record for record in scene.get("fruits", [])
    }
    if target_id not in fruit_records:
        raise ValueError("target is absent from the scene manifest")
    truth = _finite_xyz(
        fruit_records[target_id]["initial_pose_m"], "truth centre"
    )
    recommendation = geometry_gate.get("recommendation") or {}
    if recommendation.get("feasible") is not True:
        raise ValueError("selected gripper geometry is not feasible")
    tool_offset = float(recommendation.get("tool_center_offset_m"))
    close_width = float(recommendation.get("close_width_m_per_finger"))
    first_contact_width = float(
        recommendation.get("first_contact_width_m_per_finger")
    )
    axial_range = _finite_xyz(
        [
            recommendation.get("finger_axial_range_hand_z_m", [])[0],
            recommendation.get("finger_axial_range_hand_z_m", [])[1],
            tool_offset,
        ],
        "geometry axial bounds",
    )
    axial_min, axial_max, desired_axial = axial_range
    if not axial_min < desired_axial < axial_max:
        raise ValueError("desired fruit centre is outside the finger section")
    maximum_cross_jaw_error = first_contact_width - close_width
    if maximum_cross_jaw_error <= 0.0:
        raise ValueError("gripper geometry has no close overtravel")

    base_error = tuple(
        estimate - actual for estimate, actual in zip(estimated, truth)
    )
    euclidean_error = math.dist(estimated, truth)
    # The production grasp orientation is 180 degrees about hand X.  The hand
    # origin is the perceived centre plus the tool-centre offset in base Z.
    actual_center_in_hand = (
        truth[0] - estimated[0],
        estimated[1] - truth[1],
        estimated[2] + tool_offset - truth[2],
    )
    cross_jaw_error = abs(actual_center_in_hand[1])
    actual_axial = actual_center_in_hand[2]
    violations: list[str] = []
    if cross_jaw_error > maximum_cross_jaw_error:
        violations.append(
            "cross-jaw centre error exceeds the qualified bilateral-contact "
            "overtravel"
        )
    if not axial_min <= actual_axial <= axial_max:
        violations.append(
            "actual fruit centre lies outside the qualified finger axial section"
        )
    return {
        "schema_version": 1,
        "scope": "NON_ACCEPTANCE_BLENDER_V2_PERCEPTION_EXECUTION_READINESS",
        "formal_acceptance": False,
        "held_out_test_accessed": False,
        "truth_use": "SIMULATION_EXECUTION_SAFETY_VERIFICATION_ONLY",
        "target_id": target_id,
        "estimated_center_xyz_m": list(estimated),
        "truth_center_xyz_m": list(truth),
        "estimated_minus_truth_xyz_m": list(base_error),
        "euclidean_error_m": euclidean_error,
        "fruit_radius_m": float(scene["fruit_collision_radius_m"]),
        "grasp_geometry": {
            "tool_center_offset_m": tool_offset,
            "close_width_m_per_finger": close_width,
            "first_contact_width_m_per_finger": first_contact_width,
            "maximum_cross_jaw_error_m": maximum_cross_jaw_error,
            "finger_axial_range_hand_z_m": [axial_min, axial_max],
        },
        "actual_center_in_commanded_hand_xyz_m": list(
            actual_center_in_hand
        ),
        "observed_cross_jaw_error_m": cross_jaw_error,
        "observed_hand_axial_position_m": actual_axial,
        "execution_readiness_passed": not violations,
        "perception_execution_authorized": False,
        "pick_authorized": False,
        "violations": violations,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff", type=Path, required=True)
    parser.add_argument("--no-motion-gate", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--geometry-gate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    handoff = json.loads(args.handoff.read_text(encoding="utf-8"))
    no_motion_gate = json.loads(
        args.no_motion_gate.read_text(encoding="utf-8")
    )
    scene = yaml.safe_load(args.scene.read_text(encoding="utf-8"))
    geometry_gate = json.loads(
        args.geometry_gate.read_text(encoding="utf-8")
    )
    result = evaluate_execution_readiness(
        handoff=handoff,
        no_motion_gate=no_motion_gate,
        scene=scene,
        geometry_gate=geometry_gate,
    )
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["inputs"] = {
        "handoff": fingerprint(args.handoff),
        "no_motion_gate": fingerprint(args.no_motion_gate),
        "scene": fingerprint(args.scene),
        "geometry_gate": fingerprint(args.geometry_gate),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["execution_readiness_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
