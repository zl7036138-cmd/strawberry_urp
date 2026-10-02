#!/usr/bin/env python3
"""Materialize a Gazebo world with the ADR 0087-E challenge obstacle.

The obstacle specification is deliberately shared verbatim with the
manipulation launch parameter: ``x y z size_x size_y size_z`` in the Panda
base/world frame.  This tool creates the Gazebo half of the physical-parity
contract; the action server installs the same hard box into MoveIt's live
PlanningScene.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


MODEL_NAME = "development_candidate_challenge_obstacle"


def parse_obstacle_spec(specification: str) -> tuple[float, float, float, float, float, float]:
    """Return a finite six-number base-frame obstacle specification."""

    fields = tuple(
        field for field in re.split(r"[\s,]+", str(specification).strip()) if field
    )
    if len(fields) != 6:
        raise ValueError(
            "obstacle specification must contain x y z size_x size_y size_z"
        )
    try:
        values = tuple(float(field) for field in fields)
    except ValueError as exc:
        raise ValueError("obstacle specification values must be numeric") from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError("obstacle specification values must be finite")
    if any(value <= 0.0 for value in values[3:]):
        raise ValueError("obstacle dimensions must be positive")
    return values  # type: ignore[return-value]


def sha256(path: Path) -> str:
    """Return the file digest without loading large SDF assets all at once."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def challenge_model(spec: tuple[float, float, float, float, float, float]) -> ET.Element:
    """Build a fixed collision and visual box with one stable model name."""

    x, y, z, size_x, size_y, size_z = spec
    model = ET.Element("model", {"name": MODEL_NAME})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x:.9f} {y:.9f} {z:.9f} 0 0 0"
    link = ET.SubElement(model, "link", {"name": "obstacle_link"})
    for kind in ("collision", "visual"):
        element = ET.SubElement(link, kind, {"name": f"obstacle_{kind}"})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = (
            f"{size_x:.9f} {size_y:.9f} {size_z:.9f}"
        )
        if kind == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = "0.75 0.15 0.05 1"
            ET.SubElement(material, "diffuse").text = "0.90 0.22 0.08 1"
    return model


def materialize_world(
    base_world: Path,
    output_world: Path,
    specification: str,
) -> dict[str, object]:
    """Copy ``base_world`` and append exactly one challenge obstacle model."""

    source = Path(base_world)
    output = Path(output_world)
    if not source.is_file():
        raise FileNotFoundError(source)
    if output.exists():
        raise FileExistsError(output)
    spec = parse_obstacle_spec(specification)
    root = ET.parse(source).getroot()
    world = root.find("./world")
    if world is None:
        raise ValueError("base SDF lacks a world element")
    duplicate = world.findall(f"./model[@name='{MODEL_NAME}']")
    if duplicate:
        raise ValueError(f"base world already contains {MODEL_NAME}")
    world.append(challenge_model(spec))
    ET.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    ET.ElementTree(root).write(
        temporary,
        encoding="utf-8",
        xml_declaration=True,
    )
    temporary.replace(output)
    return {
        "schema_version": 1,
        "scope": "ADR0087_E_DEVELOPMENT_ONLY_PHYSICAL_CHALLENGE",
        "source_world": str(source),
        "source_world_sha256": sha256(source),
        "output_world": str(output),
        "output_world_sha256": sha256(output),
        "model_name": MODEL_NAME,
        "obstacle_spec": list(spec),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-world", type=Path, required=True)
    parser.add_argument("--output-world", type=Path, required=True)
    parser.add_argument("--obstacle-spec", required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    options = parser.parse_args(argv)
    if options.receipt.exists():
        raise FileExistsError(options.receipt)
    receipt = materialize_world(
        options.base_world,
        options.output_world,
        options.obstacle_spec,
    )
    options.receipt.parent.mkdir(parents=True, exist_ok=True)
    options.receipt.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
