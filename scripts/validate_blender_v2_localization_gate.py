#!/usr/bin/env python3
"""Validate ADR-0039's frozen Blender-v2 localization gate contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_localization"))

from strawberry_localization.v2_gate_contract import (  # noqa: E402
    load_contract,
    position_grid,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    options = parser.parse_args()
    contract = load_contract(options.contract, ROOT)
    positions = position_grid(contract)
    print(
        json.dumps(
            {
                "gate_id": contract["gate_id"],
                "scope": contract["scope"],
                "position_count": len(positions),
                "first_position_m": positions[0],
                "last_position_m": positions[-1],
                "fruit_radius_m": contract["target"]["fruit_radius_m"],
                "attempts_per_position": contract["runtime"][
                    "attempts_per_position"
                ],
                "median_error_mm_max": contract["thresholds"][
                    "median_error_mm_max"
                ],
                "p95_error_mm_max": contract["thresholds"][
                    "p95_error_mm_max"
                ],
                "formal_acceptance": False,
                "robot_motion_authorized": False,
                "simulated_fruit_pose_motion_authorized": True,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
