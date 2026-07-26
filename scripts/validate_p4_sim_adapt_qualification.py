#!/usr/bin/env python3
"""Validate and emit the frozen P4 simulator-adaptation qualification."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))

from strawberry_benchmark.p4_qualification import (  # noqa: E402
    generate_qualification_schedule,
    load_qualification_contract,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--emit-runtime-tsv", action="store_true")
    parser.add_argument("--emit-schedule-tsv", action="store_true")
    options = parser.parse_args()
    contract = load_qualification_contract(options.manifest.resolve(strict=True), ROOT)
    schedule = generate_qualification_schedule(contract)
    if options.emit_runtime_tsv:
        runtime = contract["runtime"]
        intervention = contract["intervention"]
        condition = contract["condition_contract"]
        measurement = contract["measurement"]
        print("\t".join((
            str(ROOT / intervention["model_relative_path"]),
            format(float(intervention["confidence_threshold"]), ".12g"),
            str(ROOT / condition["scene_config_relative_path"]),
            str(ROOT / condition["base_world_relative_path"]),
            str(runtime["condition_materialization_id"]),
            str(runtime["gazebo_simulation_seed"]),
            str(measurement["fixed_detection_frames"]),
            format(float(measurement["post_window_wait_sec"]), ".12g"),
            str(ROOT / runtime["claim_relative_path"]),
        )))
    elif options.emit_schedule_tsv:
        for item in schedule:
            parked = list(item.parked_models)
            print("\t".join((
                str(item.order_index), str(item.ros_domain_id), item.scenario_id,
                item.maturity, item.position_label, item.condition_label,
                item.lighting, item.occlusion, item.target_model_name,
                str(item.target_id), *(format(value, ".12g") for value in item.position_m),
                str(parked[0]["model_name"]), str(parked[0]["target_id"]),
                *(format(float(value), ".12g") for value in parked[0]["position_m"]),
                str(parked[1]["model_name"]), str(parked[1]["target_id"]),
                *(format(float(value), ".12g") for value in parked[1]["position_m"]),
            )))
    else:
        print(json.dumps({
            "qualification_id": contract["qualification_id"],
            "scenario_count": len(schedule),
            "ripe_scenarios": sum(item.maturity == "RIPE" for item in schedule),
            "unripe_scenarios": sum(item.maturity == "UNRIPE" for item in schedule),
            "ros_domain_range": [schedule[0].ros_domain_id, schedule[-1].ros_domain_id],
            "robot_motion_started": False,
            "held_out_test_consumed": False,
        }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
