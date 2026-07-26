#!/usr/bin/env python3
"""Materialize one no-motion A/B/C strawberry visual-asset world."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


VARIANT_MODELS_V1 = {
    "A": {
        "RIPE": "strawberry_ripe",
        "UNRIPE": "strawberry_unripe",
        "description": "legacy solid-color sphere and cylindrical calyx",
    },
    "B": {
        "RIPE": "strawberry_ripe_textured",
        "UNRIPE": "strawberry_unripe_textured",
        "description": "frozen sphere geometry with generated realistic albedo",
    },
    "C": {
        "RIPE": "strawberry_ripe_realistic",
        "UNRIPE": "strawberry_unripe_realistic",
        "description": "generated realistic albedo, shaped UV mesh, radial calyx and stem",
    },
}
VARIANT_MODELS_V2 = {
    "A": VARIANT_MODELS_V1["A"],
    "B": {
        "RIPE": "strawberry_ripe_textured_v2",
        "UNRIPE": "strawberry_unripe_textured_v2",
        "description": "frozen sphere geometry with camera-scale generated albedo v2",
    },
    "C": {
        "RIPE": "strawberry_ripe_realistic_v2",
        "UNRIPE": "strawberry_unripe_realistic_v2",
        "description": "camera-scale embedded albedo, shaped UV mesh, radial calyx and stem v2",
    },
}
VARIANT_MODELS_V3 = {
    "A": VARIANT_MODELS_V1["A"],
    "B": VARIANT_MODELS_V2["B"],
    "C": {
        "RIPE": "strawberry_ripe_realistic_v3",
        "UNRIPE": "strawberry_unripe_realistic_v3",
        "description": "mesh-local camera-scale albedo, shaped UV mesh, radial calyx and stem v3",
    },
}
VARIANT_MODELS_V4 = {
    "A": VARIANT_MODELS_V1["A"],
    "B": VARIANT_MODELS_V2["B"],
    "C": {
        "RIPE": "strawberry_ripe_realistic_v4",
        "UNRIPE": "strawberry_unripe_realistic_v4",
        "description": "camera-scale textured native sphere-cone body, radial calyx and stem v4",
    },
}
ASSET_REVISIONS = {
    "v1": VARIANT_MODELS_V1,
    "v2": VARIANT_MODELS_V2,
    "v3": VARIANT_MODELS_V3,
    "v4": VARIANT_MODELS_V4,
}
FRUIT_MATURITY = {"strawberry_1": "RIPE", "strawberry_2": "UNRIPE", "strawberry_3": "RIPE"}
TARGET_MODEL = {"RIPE": "strawberry_1", "UNRIPE": "strawberry_2"}
PARKED_POSITIONS = ((0.20, -2.00, 1.00), (0.40, -2.00, 1.00))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _fruit_includes(root: ET.Element) -> dict[str, ET.Element]:
    result: dict[str, ET.Element] = {}
    for include in root.findall("./world/include"):
        name = include.findtext("name", default="").strip()
        if name in FRUIT_MATURITY:
            result[name] = include
    if set(result) != set(FRUIT_MATURITY):
        raise ValueError("base world fruit includes differ from the frozen catalog")
    return result


def materialize(
    *,
    base_world: Path,
    models_root: Path,
    variant: str,
    asset_revision: str,
    target_maturity: str,
    target_position_m: tuple[float, float, float],
    output_world: Path,
    output_receipt: Path,
) -> dict:
    variant = variant.upper()
    asset_revision = asset_revision.lower()
    target_maturity = target_maturity.upper()
    if asset_revision not in ASSET_REVISIONS:
        raise ValueError(f"unsupported asset revision: {asset_revision}")
    variant_models = ASSET_REVISIONS[asset_revision]
    if variant not in variant_models:
        raise ValueError(f"unsupported asset variant: {variant}")
    if target_maturity not in TARGET_MODEL:
        raise ValueError(f"unsupported target maturity: {target_maturity}")
    if output_world.exists() or output_receipt.exists():
        raise ValueError("refusing to overwrite an asset-ablation world or receipt")

    tree = ET.parse(base_world)
    root = tree.getroot()
    includes = _fruit_includes(root)
    target_model = TARGET_MODEL[target_maturity]
    parked_index = 0
    model_bindings = {}
    for model_name, include in includes.items():
        maturity = FRUIT_MATURITY[model_name]
        asset_model = str(variant_models[variant][maturity])
        model_path = models_root / asset_model / "model.sdf"
        if not model_path.is_file():
            raise ValueError(f"asset model is missing: {model_path}")
        uri = include.find("uri")
        pose = include.find("pose")
        if uri is None or pose is None:
            raise ValueError(f"fruit include is incomplete: {model_name}")
        uri.text = f"model://{asset_model}"
        if model_name == target_model:
            position = target_position_m
        else:
            position = PARKED_POSITIONS[parked_index]
            parked_index += 1
        pose.text = f"{position[0]:.6f} {position[1]:.6f} {position[2]:.6f} 0 0 0"
        model_bindings[model_name] = {
            "maturity": maturity,
            "asset_model": asset_model,
            "model_sdf_sha256": _sha256(model_path),
            "position_m": list(position),
            "target": model_name == target_model,
        }

    ET.indent(tree, space="  ")
    output_world.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_world, encoding="utf-8", xml_declaration=True)
    receipt = {
        "schema_version": 1,
        "scope": "NON_ACCEPTANCE_NO_MOTION_ASSET_ABLATION",
        "formal_acceptance": False,
        "held_out_real_test_consumed": False,
        "formal_simulator_matrix_consumed": False,
        "robot_motion_started": False,
        "asset_revision": asset_revision,
        "variant": variant,
        "variant_description": variant_models[variant]["description"],
        "target_maturity": target_maturity,
        "target_model": target_model,
        "target_position_m": list(target_position_m),
        "base_world": {"path": str(base_world.resolve()), "sha256": _sha256(base_world)},
        "materialized_world": {
            "path": str(output_world.resolve()),
            "sha256": _sha256(output_world),
        },
        "model_bindings": model_bindings,
    }
    output_receipt.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-world", type=Path, required=True)
    parser.add_argument("--models-root", type=Path, required=True)
    parser.add_argument("--variant", choices=sorted(VARIANT_MODELS_V1), required=True)
    parser.add_argument("--asset-revision", choices=sorted(ASSET_REVISIONS), default="v1")
    parser.add_argument("--target-maturity", choices=sorted(TARGET_MODEL), required=True)
    parser.add_argument("--target-position-m", type=float, nargs=3, required=True)
    parser.add_argument("--output-world", type=Path, required=True)
    parser.add_argument("--output-receipt", type=Path, required=True)
    options = parser.parse_args()
    receipt = materialize(
        base_world=options.base_world,
        models_root=options.models_root,
        variant=options.variant,
        asset_revision=options.asset_revision,
        target_maturity=options.target_maturity,
        target_position_m=tuple(options.target_position_m),
        output_world=options.output_world,
        output_receipt=options.output_receipt,
    )
    print(receipt["materialized_world"]["sha256"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
