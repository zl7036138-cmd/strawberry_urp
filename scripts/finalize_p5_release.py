#!/usr/bin/env python3
"""Validate and write the final P5 release handoff."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

from verify_p5_release import verify


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "artifacts" / "p5" / "p5_final_release_handoff_v1.json"
)

EVIDENCE = {
    "release_manifest": "artifacts/p5/release_v1/evidence_manifest_v1.json",
    "release_metrics_json": "artifacts/p5/release_v1/final_metrics_v1.json",
    "release_metrics_csv": "artifacts/p5/release_v1/final_metrics_v1.csv",
    "release_summary": "artifacts/p5/release_v1/release_summary_v1.md",
    "final_video": "artifacts/p5/video/strawberry_urp_demo_v1.mp4",
    "final_video_receipt": (
        "artifacts/p5/video/strawberry_urp_demo_v1.receipt.json"
    ),
    "clean_reproduction": "artifacts/p5/p5_clean_reproduction_handoff_v1.json",
    "release_decision": "docs/decisions/0034-freeze-final-p5-release.md",
    "reproduction_guide": "docs/reproduction.md",
    "final_report": "docs/final-report.md",
    "report_outline": "docs/final-report-outline.md",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _binding(relative_path: str) -> dict[str, Any]:
    path = REPOSITORY_ROOT / relative_path
    if not path.is_file():
        raise ValueError(f"required final-release file is missing: {relative_path}")
    return {
        "path": relative_path,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def finalize(output: Path) -> dict[str, Any]:
    verification = verify(REPOSITORY_ROOT / "artifacts" / "p5" / "release_v1")
    if verification["status"] != "VERIFIED_FINAL_RELEASE":
        raise ValueError("P5 release verifier did not return its final status")
    metrics = json.loads(
        (REPOSITORY_ROOT / EVIDENCE["release_metrics_json"]).read_text(
            encoding="utf-8"
        )
    )
    if metrics["status"] != "FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4":
        raise ValueError("P5 metrics status differs")
    if metrics["stage_status"]["P5"] != "PASS":
        raise ValueError("P5 delivery is not complete")
    if metrics["stage_status"]["P3"] != "FAIL":
        raise ValueError("formal P3 failure was not preserved")
    if metrics["stage_status"]["P4"] != "FAIL":
        raise ValueError("P4 failure was not preserved")
    handoff = {
        "schema_version": 1,
        "kind": "p5_final_release_handoff",
        "status": "P5_COMPLETE_P6_DELIVERY_ONLY",
        "recorded_date": "2026-07-24",
        "decision": "ADR-0034",
        "release_id": "p5_release_v1",
        "verification": verification,
        "scientific_outcomes": {
            "t30_numeric_gate": "FAIL_ENGINEERING_WAIVER",
            "formal_p3": "FAIL",
            "formal_p3_positive_successes": 39,
            "formal_p3_positive_trials": 135,
            "p4_qualification": "FAIL",
            "p4_heavy_target_pose_frames": 0,
            "p4_heavy_frames": 300,
        },
        "delivery_outcomes": {
            "clean_reproduction": "PASS",
            "clean_colcon_tests": 246,
            "clean_smoke_behaviors": "10/10",
            "final_video": "PASS",
            "final_video_duration_sec": metrics["release_video"]["duration_sec"],
            "final_video_codec": metrics["release_video"]["codec"],
            "final_video_dimensions": [
                metrics["release_video"]["width"],
                metrics["release_video"]["height"],
            ],
        },
        "bindings": {
            key: _binding(path) for key, path in EVIDENCE.items()
        },
        "safety": {
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_repeated": False,
            "physical_robot_claim": False,
            "sim_to_real_claim": False,
        },
        "p6_scope": [
            "proofreading and material corrections",
            "archive packaging and delivery",
            "reproduction-defect fixes only",
        ],
        "prohibited_after_freeze": [
            "new features",
            "model retraining or threshold changes",
            "formal P3 rerun",
            "another P4 intervention",
            "held-out real-test access without a new approved research phase",
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(handoff, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return handoff


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    result = finalize(arguments.output.resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
