"""Materialize every held-out generalized scene without consuming trial results."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Sequence

import yaml

from strawberry_sim.generalized_scene import generate_scene, materialize_world

from .generalized_acceptance import validate_matrix


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument("--base-scene", type=Path, required=True)
    parser.add_argument("--base-world", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    options = parser.parse_args(args)
    matrix = json.loads(options.matrix.read_text(encoding="utf-8"))
    scenarios = validate_matrix(matrix)
    base_scene = yaml.safe_load(options.base_scene.read_text(encoding="utf-8"))
    if options.output_dir.exists() and any(options.output_dir.iterdir()):
        raise FileExistsError("refusing to write into a non-empty formal matrix directory")
    options.output_dir.mkdir(parents=True, exist_ok=True)
    receipts = []
    for row in scenarios:
        scene = generate_scene(
            base_scene,
            seed=int(row["seed"]),
            profile=str(row["profile"]),
            plant_count=int(row["plant_count"]),
            occlusion=str(row["occlusion"]),
            position_band=str(row["position_band"]),
        )
        scene_path = options.output_dir / f"{row['scenario_id']}.yaml"
        world_path = options.output_dir / f"{row['scenario_id']}.sdf"
        scene_path.write_text(yaml.safe_dump(scene, sort_keys=False, allow_unicode=True), encoding="utf-8")
        materialize_world(options.base_world, scene).write(world_path, encoding="utf-8", xml_declaration=True)
        receipts.append(
            {
                "scenario_id": row["scenario_id"],
                "seed": row["seed"],
                "scene_sha256": _sha256(scene_path),
                "world_sha256": _sha256(world_path),
            }
        )
    manifest = {
        "schema_version": 1,
        "matrix_id": matrix["matrix_id"],
        "formal_held_out": True,
        "truth_for_runtime_control": False,
        "scenarios": receipts,
    }
    output = options.output_dir / "materialization_manifest.json"
    output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"scenario_count": len(receipts), "manifest": str(output)}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
