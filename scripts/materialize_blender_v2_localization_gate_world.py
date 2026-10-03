#!/usr/bin/env python3
"""Materialize ADR-0039's camera-clear Blender-v2 localization world."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_localization"))

from strawberry_localization.v2_gate_contract import (  # noqa: E402
    load_contract,
    sha256,
)


def _set_include_pose(
    includes: dict[str, ET.Element],
    model_name: str,
    values: list[float],
) -> None:
    include = includes.get(model_name)
    if include is None:
        raise ValueError(f"base world is missing {model_name}")
    pose = include.find("pose")
    if pose is None:
        raise ValueError(f"{model_name} has no explicit pose")
    pose.text = " ".join(f"{float(value):.9f}" for value in values)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--output-world", type=Path, required=True)
    parser.add_argument("--output-receipt", type=Path, required=True)
    options = parser.parse_args()
    contract = load_contract(options.contract, ROOT)
    base_world = Path(
        contract["_resolved_paths"]["base_world"]
    ).resolve(strict=True)
    output_world = options.output_world.resolve()
    output_receipt = options.output_receipt.resolve()
    for output in (output_world, output_receipt):
        if output.exists():
            raise FileExistsError(f"refusing to overwrite {output}")

    tree = ET.parse(base_world)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("base SDF has no world element")
    includes = {
        str(include.findtext("name", "")).strip(): include
        for include in world.findall("include")
    }
    scene = contract["scene_materialization"]
    _set_include_pose(
        includes,
        str(scene["plant_model"]),
        list(scene["plant_park_pose_xyz_rpy"]),
    )
    _set_include_pose(
        includes,
        str(contract["target"]["model_name"]),
        list(scene["target_initial_pose_xyz_rpy"]),
    )
    for model_name, pose in scene["parked_fruit_poses_xyz_rpy"].items():
        _set_include_pose(includes, str(model_name), list(pose))

    output_world.parent.mkdir(parents=True, exist_ok=True)
    output_receipt.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_world, encoding="utf-8", xml_declaration=True)
    receipt = {
        "schema_version": 1,
        "kind": "blender_v2_localization_camera_clear_world_receipt",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "gate_id": contract["gate_id"],
        "scope": contract["scope"],
        "contract_path": str(Path(contract["_contract_path"]).resolve()),
        "contract_sha256": contract["_contract_sha256"],
        "base_world": {
            "path": str(base_world),
            "size_bytes": base_world.stat().st_size,
            "sha256": sha256(base_world),
        },
        "materialized_world": {
            "path": str(output_world),
            "size_bytes": output_world.stat().st_size,
            "sha256": sha256(output_world),
        },
        "target_model": contract["target"]["model_name"],
        "target_initial_pose_xyz_rpy": scene["target_initial_pose_xyz_rpy"],
        "plant_model": scene["plant_model"],
        "plant_park_pose_xyz_rpy": scene["plant_park_pose_xyz_rpy"],
        "plant_parked": True,
        "parked_fruit_poses_xyz_rpy": scene[
            "parked_fruit_poses_xyz_rpy"
        ],
        "non_target_fruits_parked": True,
        "camera_unchanged": True,
        "lighting_unchanged": True,
        "robot_unchanged": True,
        "robot_motion_started": False,
        "gripper_command_started": False,
        "attachment_started": False,
        "perception_model_started": False,
        "formal_acceptance": False,
        "formal_real_test_accessed": False,
    }
    output_receipt.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
