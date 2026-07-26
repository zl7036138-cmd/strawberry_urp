#!/usr/bin/env python3
"""Validate and query the frozen perception-control engineering waiver."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0,
    str(REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_benchmark"),
)

from strawberry_benchmark.perception_control import (  # noqa: E402
    load_perception_control_waiver,
    verify_perception_control_waiver,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--emit-runtime-tsv", action="store_true")
    parser.add_argument("--emit-target-tsv", action="store_true")
    parser.add_argument("--emit-park-tsv", action="store_true")
    arguments = parser.parse_args()
    emit_modes = sum(
        (
            arguments.emit_runtime_tsv,
            arguments.emit_target_tsv,
            arguments.emit_park_tsv,
        )
    )
    if emit_modes > 1:
        raise SystemExit("choose at most one TSV output mode")

    waiver = load_perception_control_waiver(arguments.manifest)
    verified = verify_perception_control_waiver(
        waiver, REPOSITORY_ROOT, model_path=arguments.model
    )
    if arguments.emit_runtime_tsv:
        print(
            "\t".join(
                (
                    str(verified["model"]),
                    format(waiver.confidence_threshold, ".12g"),
                    waiver.detections_topic,
                    waiver.target_pose_topic,
                    waiver.control_target_topic,
                )
            )
        )
    elif arguments.emit_target_tsv:
        print(
            "\t".join(
                (
                    waiver.scenario_id,
                    waiver.target.model_name,
                    str(waiver.target.target_id),
                    *(format(value, ".12g") for value in waiver.target.position_m),
                )
            )
        )
    elif arguments.emit_park_tsv:
        for model in waiver.parked_models:
            print(
                "\t".join(
                    (
                        model.model_name,
                        str(model.target_id),
                        *(format(value, ".12g") for value in model.position_m),
                    )
                )
            )
    else:
        print(
            f"validated {waiver.waiver_id}: {waiver.engineering_status}, "
            f"numeric macro-F1={waiver.macro_f1:.10f} below "
            f"{waiver.required_macro_f1:.2f}, held-out test sealed"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
