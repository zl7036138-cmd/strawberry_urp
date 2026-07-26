#!/usr/bin/env python3
"""Create a Blender-v2 world with exactly one fruit visible to the camera."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import xml.etree.ElementTree as ET

FRUITS = {"strawberry_1", "strawberry_2", "strawberry_3"}
def digest(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()
def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--base-world", type=Path, required=True)
    p.add_argument("--target", choices=sorted(FRUITS), required=True)
    p.add_argument("--output-world", type=Path, required=True)
    p.add_argument("--output-receipt", type=Path, required=True)
    o = p.parse_args()
    for output in (o.output_world, o.output_receipt):
        if output.exists(): raise ValueError(f"refusing to overwrite evidence: {output}")
    tree = ET.parse(o.base_world); world = tree.getroot().find("world")
    if world is None: raise ValueError("base SDF has no world")
    found = set()
    parked = []
    park_index = 0
    for include in list(world.findall("include")):
        name = (include.findtext("name") or "").strip()
        if name in FRUITS:
            found.add(name)
            if name != o.target:
                pose = include.find("pose")
                if pose is None or not (pose.text or "").strip():
                    raise ValueError(f"{name} has no explicit pose")
                values = [float(value) for value in pose.text.split()]
                if len(values) != 6:
                    raise ValueError(f"{name} pose is not xyz-rpy")
                values[:3] = [0.2 + 0.2 * park_index, -2.0, 1.0]
                pose.text = " ".join(f"{value:.9f}" for value in values)
                parked.append(name)
                park_index += 1
    if found != FRUITS: raise ValueError(f"base world fruit identities differ: {sorted(found)}")
    o.output_world.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  "); tree.write(o.output_world, encoding="utf-8", xml_declaration=True)
    receipt = {
        "schema_version": 1, "scope": "NON_ACCEPTANCE_NO_MOTION_BLENDER_V2_SINGLE_FRUIT_DIAGNOSTIC",
        "formal_acceptance": False, "held_out_test_consumed": False, "robot_motion_started": False,
        "target_model": o.target, "visible_models": [o.target],
        "retained_models": sorted(FRUITS), "parked_models": parked,
        "truth_streams_retained": True, "plant_retained": True,
        "camera_and_lighting_unchanged": True,
        "base_world": {"path": str(o.base_world.resolve()), "sha256": digest(o.base_world)},
        "materialized_world": {"path": str(o.output_world.resolve()), "sha256": digest(o.output_world)}}
    o.output_receipt.write_text(json.dumps(receipt, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    return 0
if __name__ == "__main__": raise SystemExit(main())
