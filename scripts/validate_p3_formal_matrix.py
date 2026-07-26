#!/usr/bin/env python3
"""Validate and emit the frozen formal P3 simulator matrix."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_benchmark.formal_matrix import (  # noqa: E402
    generate_formal_schedule,
    load_formal_matrix_contract,
    verify_formal_matrix_contract,
)


def _schedule_digest(schedule) -> str:
    lines = [
        json.dumps(item.to_dict(), sort_keys=True, separators=(",", ":"))
        for item in schedule
    ]
    return hashlib.sha256(("\n".join(lines) + "\n").encode("utf-8")).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--require-unclaimed", action="store_true")
    parser.add_argument("--emit-schedule-tsv", action="store_true")
    parser.add_argument("--emit-runtime-tsv", action="store_true")
    options = parser.parse_args()

    manifest_path = options.manifest.resolve(strict=True)
    contract = load_formal_matrix_contract(manifest_path)
    verified = verify_formal_matrix_contract(
        contract, ROOT, model_path=options.model
    )
    if options.require_unclaimed and verified["claim"].exists():
        raise SystemExit("formal matrix claim already exists")
    schedule = generate_formal_schedule(contract, verified["benchmark"])

    if options.emit_runtime_tsv:
        print(
            "\t".join(
                (
                    str(verified["model"]),
                    format(contract.confidence_threshold, ".12g"),
                    str(verified["scene_conditions"]),
                    str(verified["base_world"]),
                    str(contract.materialization_id),
                    str(contract.minimum_positive_detection_frames),
                    str(contract.minimum_negative_detection_frames),
                    format(contract.startup_timeout_sec, ".12g"),
                    format(contract.trial_timeout_sec, ".12g"),
                    format(contract.outer_timeout_sec, ".12g"),
                    format(contract.control_position_tolerance_m, ".12g"),
                    str(contract.maximum_infrastructure_attempts),
                    str(verified["claim"]),
                    str(verified["preflight_dir"]),
                    str(verified["default_result_dir"]),
                )
            )
        )
    elif options.emit_schedule_tsv:
        for item in schedule:
            fields = [
                str(item.order_index), str(item.ros_domain_id), item.trial_id,
                item.kind, item.lighting, item.occlusion, item.position.label,
                str(item.simulation_seed), item.expected_maturity,
                item.expected_outcome, item.target_model_name, str(item.target_id),
                *(format(value, ".12g") for value in item.position.position_m),
            ]
            for parked in item.parked_models:
                fields.extend(
                    [
                        parked.model_name, str(parked.target_id),
                        *(format(value, ".12g") for value in parked.position_m),
                    ]
                )
            print("\t".join(fields))
    else:
        positive = sum(item.kind == "POSITIVE" for item in schedule)
        negative = sum(item.kind == "NEGATIVE" for item in schedule)
        print(
            json.dumps(
                {
                    "matrix_id": contract.matrix_id,
                    "positive_trials": positive,
                    "negative_trials": negative,
                    "total_trials": len(schedule),
                    "schedule_sha256": _schedule_digest(schedule),
                    "ros_domain_range": [
                        min(item.ros_domain_id for item in schedule),
                        max(item.ros_domain_id for item in schedule),
                    ],
                    "gazebo_simulation_seeds": list(contract.simulation_seeds),
                    "simulation_seed_runtime_binding": "gz sim --seed",
                    "condition_materialization_id": contract.materialization_id,
                    "claim_exists": verified["claim"].exists(),
                    "held_out_test_receipt_exists": verified["held_out_receipt"].exists(),
                    "formal_matrix_consumed": verified["claim"].exists(),
                },
                sort_keys=True,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
