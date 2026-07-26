#!/usr/bin/env python3
"""Validate and query the frozen repeated perception-control development gate."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0,
    str(REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_benchmark"),
)

from strawberry_benchmark.perception_dev_gate import (  # noqa: E402
    load_perception_repeated_dev_gate,
    verify_perception_repeated_dev_gate,
)


def _print_target(scene) -> None:
    print(
        "\t".join(
            (
                scene.scenario_id,
                scene.target.model_name,
                str(scene.target.target_id),
                *(format(value, ".12g") for value in scene.target.position_m),
            )
        )
    )


def _print_parked(scene) -> None:
    for model in scene.parked_models:
        print(
            "\t".join(
                (
                    model.model_name,
                    str(model.target_id),
                    *(format(value, ".12g") for value in model.position_m),
                )
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--emit-runtime-tsv", action="store_true")
    parser.add_argument("--emit-positive-target-tsv", action="store_true")
    parser.add_argument("--emit-negative-target-tsv", action="store_true")
    parser.add_argument("--emit-positive-park-tsv", action="store_true")
    parser.add_argument("--emit-negative-park-tsv", action="store_true")
    arguments = parser.parse_args()
    modes = (
        arguments.emit_runtime_tsv,
        arguments.emit_positive_target_tsv,
        arguments.emit_negative_target_tsv,
        arguments.emit_positive_park_tsv,
        arguments.emit_negative_park_tsv,
    )
    if sum(modes) > 1:
        raise SystemExit("choose at most one TSV output mode")

    contract = load_perception_repeated_dev_gate(arguments.manifest)
    waiver, verified = verify_perception_repeated_dev_gate(
        contract, REPOSITORY_ROOT, model_path=arguments.model
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
                    str(contract.positive_trials),
                    str(contract.negative_trials),
                    str(contract.minimum_settled_detection_frames),
                    str(contract.base_ros_domain_id),
                )
            )
        )
    elif arguments.emit_positive_target_tsv:
        _print_target(contract.positive_scene)
    elif arguments.emit_negative_target_tsv:
        _print_target(contract.negative_scene)
    elif arguments.emit_positive_park_tsv:
        _print_parked(contract.positive_scene)
    elif arguments.emit_negative_park_tsv:
        _print_parked(contract.negative_scene)
    else:
        print(
            f"validated {contract.gate_id}: {contract.positive_trials} positive + "
            f"{contract.negative_trials} negative fresh worlds, non-formal, "
            f"model={waiver.model_sha256[:12]}..., test sealed"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
