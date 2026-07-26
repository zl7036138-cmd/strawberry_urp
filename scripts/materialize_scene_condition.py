#!/usr/bin/env python3
"""Materialize one hash-bound Gazebo world for a lighting/occlusion condition."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_sim.scene_conditions import materialize_condition_world  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-world", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--lighting", required=True)
    parser.add_argument("--occlusion", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--output-world", type=Path, required=True)
    parser.add_argument("--output-receipt", type=Path, required=True)
    parser.add_argument("--target-position-m", type=float, nargs=3)
    options = parser.parse_args()
    receipt = materialize_condition_world(
        base_world=options.base_world,
        config_path=options.config,
        lighting_level=options.lighting,
        occlusion_level=options.occlusion,
        seed=options.seed,
        output_world=options.output_world,
        output_receipt=options.output_receipt,
        target_position_m=options.target_position_m,
    )
    print(
        f"materialized {receipt['lighting_level']}+{receipt['occlusion_level']} "
        f"world {receipt['materialized_world']['sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
