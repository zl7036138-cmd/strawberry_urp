#!/usr/bin/env python3
"""Apply ADR-0040's mechanical body-face winding repair."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_sim.obj_winding import (  # noqa: E402
    reverse_object_faces,
    winding_stats,
)


OBJECT_NAME = "strawberry_ripe_body_v5"
MESH_BINDINGS = {
    "ros2_ws/src/strawberry_sim/models/strawberry_ripe/meshes/"
    "strawberry_ripe_visual_v2.obj": {
        "size_bytes": 1377293,
        "sha256": "f8cbe53a61ee46b1de6ff1aa6f203291fe1b4f946084ed0617ca8f929dc5b347",
    },
    "ros2_ws/src/strawberry_sim/models/strawberry_unripe/meshes/"
    "strawberry_unripe_visual_v2.obj": {
        "size_bytes": 1377295,
        "sha256": "c8f213bed108c277bedbe910d3215bf73cf25780a90efd99e03cea9a33db0068",
    },
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _stats_json(stats) -> dict:
    return {
        "object_name": stats.object_name,
        "triangle_count": stats.triangle_count,
        "outward_triangles": stats.outward_triangles,
        "inward_triangles": stats.inward_triangles,
        "degenerate_triangles": stats.degenerate_triangles,
        "signed_sum": stats.signed_sum,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt", type=Path, required=True)
    options = parser.parse_args()
    receipt_path = options.receipt.resolve()
    if receipt_path.exists():
        raise FileExistsError(f"refusing to overwrite {receipt_path}")

    prepared = []
    try:
        for relative_path, binding in MESH_BINDINGS.items():
            mesh_path = (ROOT / relative_path).resolve(strict=True)
            if (
                mesh_path.stat().st_size != binding["size_bytes"]
                or sha256(mesh_path) != binding["sha256"]
            ):
                raise ValueError(f"pre-repair mesh binding changed: {relative_path}")
            before = winding_stats(mesh_path, OBJECT_NAME)
            if (
                before.triangle_count != 11796
                or before.outward_triangles != 0
                or before.inward_triangles != 11796
                or before.degenerate_triangles != 0
            ):
                raise ValueError(f"unexpected pre-repair winding: {relative_path}")
            temporary = mesh_path.with_name(f".{mesh_path.name}.winding-fixed")
            if temporary.exists():
                raise FileExistsError(f"temporary repair file exists: {temporary}")
            reversed_faces = reverse_object_faces(
                mesh_path,
                temporary,
                OBJECT_NAME,
            )
            after = winding_stats(temporary, OBJECT_NAME)
            if (
                reversed_faces != 11796
                or after.triangle_count != before.triangle_count
                or after.outward_triangles != 11796
                or after.inward_triangles != 0
                or after.degenerate_triangles != 0
            ):
                raise ValueError(f"post-repair winding failed: {relative_path}")
            prepared.append(
                {
                    "relative_path": relative_path,
                    "path": mesh_path,
                    "temporary": temporary,
                    "before": before,
                    "after": after,
                    "before_size_bytes": mesh_path.stat().st_size,
                    "before_sha256": sha256(mesh_path),
                    "after_size_bytes": temporary.stat().st_size,
                    "after_sha256": sha256(temporary),
                    "reversed_face_records": reversed_faces,
                }
            )

        for row in prepared:
            os.replace(row["temporary"], row["path"])
        decision = (
            ROOT
            / "docs/decisions/"
            "0040-record-v2-localization-failure-and-repair-mesh-winding.md"
        ).resolve(strict=True)
        receipt = {
            "schema_version": 1,
            "kind": "blender_v2_body_winding_repair_receipt",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "scope": "ADR_0040_BOUNDED_ASSET_DEFECT_CORRECTION",
            "decision": {
                "path": decision.relative_to(ROOT).as_posix(),
                "size_bytes": decision.stat().st_size,
                "sha256": sha256(decision),
            },
            "object_name": OBJECT_NAME,
            "vertex_coordinates_changed": False,
            "vertex_normals_changed": False,
            "material_assignments_changed": False,
            "collision_geometry_changed": False,
            "robot_motion_started": False,
            "formal_acceptance": False,
            "meshes": [
                {
                    "path": row["relative_path"],
                    "before": {
                        "size_bytes": row["before_size_bytes"],
                        "sha256": row["before_sha256"],
                        "winding": _stats_json(row["before"]),
                    },
                    "after": {
                        "size_bytes": row["after_size_bytes"],
                        "sha256": row["after_sha256"],
                        "winding": _stats_json(row["after"]),
                    },
                    "reversed_face_records": row["reversed_face_records"],
                }
                for row in prepared
            ],
        }
        receipt_path.parent.mkdir(parents=True, exist_ok=True)
        receipt_path.write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(receipt, indent=2, sort_keys=True))
        return 0
    finally:
        for row in prepared:
            temporary = row["temporary"]
            if temporary.exists():
                temporary.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
