#!/usr/bin/env python3
"""Run the frozen offline Blender-v2 gripper geometry sweep exactly once."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "strawberry_manipulation"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_manipulation.gripper_geometry import (  # noqa: E402
    GripperGeometry,
    evaluate_candidate,
    load_frozen_contract,
    read_binary_stl,
    select_recommendation,
    sha256,
)


def fingerprint(path: Path) -> dict[str, object]:
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--contract",
        type=Path,
        default=ROOT / "config" / "blender_v2_gripper_geometry_sweep_v1.json",
    )
    parser.add_argument("--repository-root", type=Path, default=ROOT)
    args = parser.parse_args()

    contract = load_frozen_contract(args.contract, args.repository_root)
    output_dir = (
        args.repository_root
        / str(contract["execution"]["output_directory"])
    ).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite {output_dir}")

    paths = {
        label: Path(value)
        for label, value in contract["_resolved_paths"].items()
    }
    hand_triangles = read_binary_stl(
        paths["upstream_hand_collision_mesh"]
    )
    finger_triangles = read_binary_stl(
        paths["upstream_finger_collision_mesh"]
    )
    geometry_record = contract["geometry"]
    geometry = GripperGeometry(
        fruit_radius_m=float(geometry_record["fruit_radius_m"]),
        finger_joint_z_m=float(geometry_record["finger_joint_z_m"]),
        contact_pad_z_from_finger_m=float(
            geometry_record["contact_pad_z_from_finger_m"]
        ),
        open_width_m=float(
            geometry_record["open_width_m_per_finger"]
        ),
        contact_margin_m=float(
            geometry_record["geometric_contact_margin_m"]
        ),
        pregrasp_offset_m=float(geometry_record["pregrasp_offset_m"]),
        wrist_housing_center_m=tuple(
            float(value)
            for value in geometry_record["wrist_housing_center_hand_m"]
        ),
        wrist_housing_size_m=tuple(
            float(value)
            for value in geometry_record["wrist_housing_size_m"]
        ),
    )
    thresholds = {
        key: float(value)
        for key, value in contract["thresholds"].items()
        if key != "minimum_feasible_candidates"
    }
    candidates = [
        evaluate_candidate(
            hand_triangles=hand_triangles,
            finger_triangles=finger_triangles,
            tool_center_offset_m=float(tool_offset),
            close_width_m=float(close_width),
            geometry=geometry,
            thresholds=thresholds,
        )
        for tool_offset in contract["candidate_grid"]["tool_center_offsets_m"]
        for close_width in contract["candidate_grid"][
            "close_widths_m_per_finger"
        ]
    ]
    recommendation = select_recommendation(candidates)
    current_parameters = contract["current_production_parameters"]
    current_result = next(
        candidate
        for candidate in candidates
        if (
            candidate["tool_center_offset_m"]
            == current_parameters["tool_center_offset_m"]
            and candidate["close_width_m_per_finger"]
            == current_parameters["close_width_m_per_finger"]
        )
    )
    feasible_count = sum(
        candidate["feasible"] is True for candidate in candidates
    )
    passed = (
        feasible_count
        >= int(contract["thresholds"]["minimum_feasible_candidates"])
        and recommendation is not None
    )
    summary = {
        "schema_version": 1,
        "gate": contract["gate_id"],
        "scope": contract["scope"],
        "passed": passed,
        "formal_acceptance": False,
        "formal_held_out_test_accessed": False,
        "robot_motion_started": False,
        "gripper_command_started": False,
        "simulated_fruit_motion_started": False,
        "attachment_started": False,
        "planning_started": False,
        "pick_action_started": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "contract": fingerprint(Path(contract["_contract_path"])),
        "resolved_inputs": {
            label: fingerprint(path) for label, path in paths.items()
        },
        "mesh_triangle_counts": {
            "hand": len(hand_triangles),
            "finger": len(finger_triangles),
        },
        "candidate_count": len(candidates),
        "feasible_candidate_count": feasible_count,
        "current_production_result": current_result,
        "recommendation": recommendation,
        "candidates": candidates,
    }
    output_dir.mkdir(parents=True, exist_ok=False)
    output_path = output_dir / "summary.json"
    output_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, sort_keys=True), flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
