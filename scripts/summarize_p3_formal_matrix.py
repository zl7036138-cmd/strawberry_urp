#!/usr/bin/env python3
"""Summarize and apply the frozen formal P3 simulator acceptance gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_benchmark"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_benchmark.formal_summary import summarize_formal_matrix  # noqa: E402


PROVENANCE_PATHS = (
    "config/p3_formal_matrix_v1.json",
    "docs/decisions/0029-freeze-formal-p3-simulator-matrix.md",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/formal_matrix.py",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/formal_summary.py",
    "ros2_ws/src/strawberry_sim/strawberry_sim/scene_conditions.py",
    "ros2_ws/src/strawberry_sim/launch/sim.launch.py",
    "ros2_ws/src/strawberry_bringup/launch/system.launch.py",
    "scripts/test_orchestrated_trial.py",
    "scripts/test_formal_p3_trial.py",
    "scripts/validate_p3_formal_matrix.py",
    "scripts/prepare_p3_formal_matrix_preflight.py",
    "scripts/claim_p3_formal_matrix.py",
    "scripts/classify_p3_formal_attempt.py",
    "scripts/run_p3_formal_matrix.sh",
    "scripts/summarize_p3_formal_matrix.py",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--preflight", type=Path, required=True)
    parser.add_argument("--model", type=Path)
    options = parser.parse_args()
    summary = summarize_formal_matrix(
        options.output_dir,
        options.manifest,
        options.preflight,
        ROOT,
        model_path=options.model,
        provenance_paths=PROVENANCE_PATHS,
    )
    path = options.output_dir / "summary.json"
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["formal_p3_simulator_gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
