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
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--formal-matrix", type=Path, required=True)
    parser.add_argument("--emit-scenes-tsv", action="store_true")
    options = parser.parse_args()
    plan = load_capture_plan(options.plan, options.formal_matrix)
    specs = iter_capture_specs(plan)
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
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
