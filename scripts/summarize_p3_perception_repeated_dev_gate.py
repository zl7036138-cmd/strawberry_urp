#!/usr/bin/env python3
"""Apply the frozen repeated perception-control development gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0,
    str(REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_benchmark"),
)

from strawberry_benchmark.perception_dev_summary import (  # noqa: E402
    summarize_repeated_dev_gate,
)


PROVENANCE_PATHS = (
    "config/p3_perception_control_waiver_v1.json",
    "config/p3_perception_repeated_dev_gate_v1.json",
    "docs/decisions/0026-accept-below-gate-model-for-simulation-control.md",
    "docs/decisions/0027-freeze-perception-controlled-repeated-development-gate.md",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/perception_control.py",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/perception_dev_gate.py",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/perception_dev_summary.py",
    "ros2_ws/src/strawberry_bringup/launch/system.launch.py",
    "ros2_ws/src/strawberry_bringup/strawberry_bringup/orchestrator.py",
    "ros2_ws/src/strawberry_perception/strawberry_perception/perception_node.py",
    "ros2_ws/src/strawberry_localization/strawberry_localization/node.py",
    "ros2_ws/src/strawberry_manipulation/strawberry_manipulation/moveit_backend.py",
    "scripts/test_orchestrated_trial.py",
    "scripts/validate_p3_perception_repeated_dev_gate.py",
    "scripts/run_p3_perception_repeated_dev_gate.sh",
    "scripts/summarize_p3_perception_repeated_dev_gate.py",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--base-domain-id", type=int)
    arguments = parser.parse_args()
    result = summarize_repeated_dev_gate(
        arguments.output_dir,
        arguments.manifest,
        REPOSITORY_ROOT,
        model_path=arguments.model,
        base_domain_id=arguments.base_domain_id,
        provenance_paths=PROVENANCE_PATHS,
    )
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["development_gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
