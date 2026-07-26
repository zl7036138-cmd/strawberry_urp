#!/usr/bin/env python3
"""Create an immutable, no-simulation preflight receipt for formal P3."""

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
from strawberry_benchmark.perception_control import sha256_file  # noqa: E402


PROVENANCE_PATHS = (
    "docs/decisions/0026-accept-below-gate-model-for-simulation-control.md",
    "docs/decisions/0027-freeze-perception-controlled-repeated-development-gate.md",
    "docs/decisions/0028-accept-repeated-perception-development-gate-outcome.md",
    "docs/decisions/0029-freeze-formal-p3-simulator-matrix.md",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/formal_matrix.py",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/formal_summary.py",
    "ros2_ws/src/strawberry_benchmark/strawberry_benchmark/scenarios.py",
    "ros2_ws/src/strawberry_sim/strawberry_sim/scene_conditions.py",
    "ros2_ws/src/strawberry_sim/launch/sim.launch.py",
    "ros2_ws/src/strawberry_bringup/launch/system.launch.py",
    "scripts/materialize_scene_condition.py",
    "scripts/test_orchestrated_trial.py",
    "scripts/test_formal_p3_trial.py",
    "scripts/validate_p3_formal_matrix.py",
    "scripts/prepare_p3_formal_matrix_preflight.py",
    "scripts/claim_p3_formal_matrix.py",
    "scripts/classify_p3_formal_attempt.py",
    "scripts/run_p3_formal_matrix.sh",
    "scripts/summarize_p3_formal_matrix.py",
)


def _fingerprint(path: Path) -> dict:
    return {
        "path": path.resolve().relative_to(ROOT.resolve()).as_posix(),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--model", type=Path)
    options = parser.parse_args()
    manifest_path = options.manifest.resolve(strict=True)
    contract = load_formal_matrix_contract(manifest_path)
    verified = verify_formal_matrix_contract(contract, ROOT, model_path=options.model)
    if verified["claim"].exists():
        raise SystemExit("formal claim already exists; refusing a new preflight")
    output_dir = (
        options.output_dir.resolve()
        if options.output_dir
        else verified["preflight_dir"].resolve()
    )
    if output_dir.exists():
        raise SystemExit(f"preflight output directory already exists: {output_dir}")
    if ROOT.resolve() not in output_dir.parents:
        raise SystemExit("preflight output must remain inside the repository")

    schedule = generate_formal_schedule(contract, verified["benchmark"])
    rendered_lines = [
        json.dumps(item.to_dict(), sort_keys=True, separators=(",", ":"))
        for item in schedule
    ]
    rendered = "\n".join(rendered_lines) + "\n"
    schedule_digest = hashlib.sha256(rendered.encode("utf-8")).hexdigest()
    output_dir.mkdir(parents=True, exist_ok=False)
    schedule_path = output_dir / "scenario_manifest.jsonl"
    schedule_path.write_text(rendered, encoding="utf-8")

    receipt = {
        "schema_version": 1,
        "kind": "p3_formal_matrix_preflight",
        "matrix_id": contract.matrix_id,
        "status": "READY_UNCLAIMED",
        "formal_run_unlocked": True,
        "formal_run_started": False,
        "claim_consumed": False,
        "formal_matrix_consumed": False,
        "held_out_real_test_consumed": False,
        "held_out_test_receipt_exists": False,
        "engineering_waiver_active": True,
        "numeric_t30_gate_passed": False,
        "model": _fingerprint(verified["model"]),
        "confidence_threshold": contract.confidence_threshold,
        "trial_counts": {"positive": 135, "negative": 30, "total": 165},
        "schedule": {
            "path": schedule_path.resolve().relative_to(ROOT.resolve()).as_posix(),
            "size_bytes": len(rendered.encode("utf-8")),
            "sha256": schedule_digest,
            "method": "sha256_rank",
            "schedule_seed": contract.schedule_seed,
            "ros_domain_range": [
                min(item.ros_domain_id for item in schedule),
                max(item.ros_domain_id for item in schedule),
            ],
        },
        "randomization": {
            "simulation_seeds": list(contract.simulation_seeds),
            "runtime_binding": "gz sim --seed",
            "condition_materialization_id": contract.materialization_id,
            "materialization_id_is_random_seed": False,
        },
        "bindings": {
            "manifest": _fingerprint(manifest_path),
            **{
                name: _fingerprint(path)
                for name, path in verified.items()
                if name not in {"model", "held_out_receipt", "claim", "preflight_dir", "default_result_dir"}
            },
        },
        "provenance": [_fingerprint(ROOT / path) for path in PROVENANCE_PATHS],
        "claim_path": verified["claim"].resolve().relative_to(ROOT.resolve()).as_posix(),
        "default_result_dir": verified["default_result_dir"].resolve().relative_to(ROOT.resolve()).as_posix(),
        "interpretation": (
            "No-simulation preflight only. It freezes the formal simulator "
            "schedule under ADR-0026/0029 and does not itself consume a trial."
        ),
    }
    receipt_path = output_dir / "preflight_receipt.json"
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
