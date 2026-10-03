#!/usr/bin/env python3
"""Validate and expand the development-only generalized capture plan."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "strawberry_sim"))

from strawberry_sim.generalized_capture_core import (  # noqa: E402
    iter_capture_specs,
    load_capture_plan,
    sha256_file,
)


def validate_completed_subset(output_root: Path, specs: tuple[dict, ...]) -> int:
    """Verify every existing sample is complete, bound, and unmodified."""

    completed = 0
    for spec in specs:
        split = str(spec["split"])
        sample_id = str(spec["sample_id"])
        image = output_root / "images" / split / f"{sample_id}.png"
        label = output_root / "labels" / split / f"{sample_id}.txt"
        receipt = output_root / "receipts" / split / f"{sample_id}.json"
        present = [path.is_file() for path in (image, label, receipt)]
        if not any(present):
            continue
        if not all(present):
            raise ValueError(f"partial resume artifacts for {sample_id}")
        raw = json.loads(receipt.read_text(encoding="utf-8"))
        if raw.get("sample") != spec:
            raise ValueError(f"resume spec mismatch for {sample_id}")
        if (
            raw.get("formal_acceptance")
            or raw.get("formal_results_consumed")
            or raw.get("runtime_control_authorized")
            or raw.get("robot_motion_started")
        ):
            raise ValueError(f"resume boundary violation for {sample_id}")
        if raw.get("image", {}).get("sha256") != sha256_file(image):
            raise ValueError(f"resume image hash mismatch for {sample_id}")
        if raw.get("label", {}).get("sha256") != sha256_file(label):
            raise ValueError(f"resume label hash mismatch for {sample_id}")
        completed += 1
    return completed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--formal-matrix", type=Path, required=True)
    parser.add_argument("--emit-scenes-tsv", action="store_true")
    parser.add_argument("--validate-resume-root", type=Path)
    options = parser.parse_args()
    plan = load_capture_plan(options.plan, options.formal_matrix)
    specs = iter_capture_specs(plan)
    resumed = (
        validate_completed_subset(options.validate_resume_root.resolve(strict=True), specs)
        if options.validate_resume_root is not None
        else 0
    )
    if options.emit_scenes_tsv:
        for row in specs:
            print(
                "\t".join(
                    str(row[key])
                    for key in (
                        "sample_id",
                        "split",
                        "seed",
                        "profile",
                        "plant_count",
                        "position_band",
                        "occlusion",
                    )
                )
            )
    else:
        print(
            json.dumps(
                {
                    "capture_id": plan["capture_id"],
                    "scene_count": len(specs),
                    "split_counts": {
                        split: sum(row["split"] == split for row in specs)
                        for split in ("train", "validation", "qualification")
                    },
                    "formal_seed_overlap": 0,
                    "formal_results_consumed": False,
                    "validated_resume_samples": resumed,
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
