#!/usr/bin/env python3
"""Validate and enumerate the frozen no-motion strawberry asset ablation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _check_binding(binding: dict) -> None:
    path = ROOT / str(binding["path"])
    if not path.is_file():
        raise ValueError(f"missing bound file: {binding['path']}")
    if path.stat().st_size != int(binding["size_bytes"]):
        raise ValueError(f"size mismatch: {binding['path']}")
    if hashlib.sha256(path.read_bytes()).hexdigest() != str(binding["sha256"]):
        raise ValueError(f"hash mismatch: {binding['path']}")


def load_manifest(path: Path, model: Path) -> tuple[dict, list[tuple]]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    required_false = (
        "formal_acceptance",
        "held_out_real_test_consumed",
        "formal_simulator_matrix_consumed",
        "robot_motion_started",
        "training_started",
    )
    if any(manifest.get(key) is not False for key in required_false):
        raise ValueError("asset ablation must remain non-formal, no-motion and no-training")
    if manifest.get("scope") != "NON_ACCEPTANCE_NO_MOTION_ASSET_ABLATION":
        raise ValueError("unexpected diagnostic scope")

    shadow = manifest["shadow"]
    if model.resolve() != (ROOT / shadow["model_relative_path"]).resolve():
        raise ValueError("runtime model differs from the frozen baseline")
    _check_binding(
        {
            "path": shadow["model_relative_path"],
            "size_bytes": shadow["model_size_bytes"],
            "sha256": shadow["model_sha256"],
        }
    )
    world = manifest["world"]
    _check_binding(
        {
            "path": world["base_relative_path"],
            "size_bytes": world["base_size_bytes"],
            "sha256": world["base_sha256"],
        }
    )

    variants = manifest["paired_design"]["variants"]
    if [item["id"] for item in variants] != ["A", "B", "C"]:
        raise ValueError("asset variants must be ordered A, B, C")
    for variant in variants:
        for key, value in variant.items():
            if key.endswith(("_model", "_texture", "_mesh", "_mtl")):
                _check_binding(value)

    maturities = manifest["paired_design"]["target_maturities"]
    positions = manifest["paired_design"]["positions"]
    rows = []
    for variant in variants:
        for maturity in maturities:
            for position in positions:
                scenario_id = (
                    f"asset_{variant['id'].lower()}_{maturity.lower()}_{position['label']}"
                )
                rows.append(
                    (
                        scenario_id,
                        variant["id"],
                        maturity,
                        position["label"],
                        *position["position_m"],
                    )
                )
    if len(rows) != int(manifest["measurement"]["scenario_count"]):
        raise ValueError("scenario count differs from the Cartesian design")
    if manifest["diagnostic_interpretation"].get("candidate_promotion_authorized") is not False:
        raise ValueError("asset ablation cannot authorize candidate promotion")
    if manifest["diagnostic_interpretation"].get("perception_control_authorized") is not False:
        raise ValueError("asset ablation cannot authorize perception control")
    return manifest, rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--emit-tsv", action="store_true")
    parser.add_argument("--emit-asset-revision", action="store_true")
    options = parser.parse_args()
    manifest, rows = load_manifest(options.manifest, options.model)
    if options.emit_asset_revision:
        print(str(manifest.get("asset_revision", "v1")))
    elif options.emit_tsv:
        for row in rows:
            print("\t".join(str(value) for value in row))
    else:
        print(
            json.dumps(
                {
                    "diagnostic_id": manifest["diagnostic_id"],
                    "scenario_count": len(rows),
                    "validated": True,
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
