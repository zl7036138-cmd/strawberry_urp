#!/usr/bin/env python3
"""Verify hashes and semantic safety constraints of the P5 release package."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PACKAGE = REPOSITORY_ROOT / "artifacts" / "p5" / "release_v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _safe_path(relative_path: str) -> Path:
    candidate = (REPOSITORY_ROOT / relative_path).resolve()
    if REPOSITORY_ROOT.resolve() not in candidate.parents:
        raise ValueError(f"unsafe binding path: {relative_path}")
    return candidate


def _verify_binding(binding: dict[str, Any]) -> None:
    path = _safe_path(str(binding["path"]))
    if not path.is_file():
        raise ValueError(f"bound file is missing: {binding['path']}")
    if path.stat().st_size != binding["size_bytes"]:
        raise ValueError(f"size differs: {binding['path']}")
    if _sha256(path) != binding["sha256"]:
        raise ValueError(f"SHA-256 differs: {binding['path']}")


def verify(package_dir: Path) -> dict[str, Any]:
    manifest_path = package_dir.resolve() / "evidence_manifest_v1.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("kind") != "p5_release_evidence_manifest":
        raise ValueError("unexpected P5 manifest kind")
    if manifest.get("status") != "FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4":
        raise ValueError("P5 release is not frozen")
    for binding in (*manifest["inputs"], *manifest["outputs"]):
        _verify_binding(binding)
    _verify_binding(manifest["generator"])

    metrics_path = package_dir.resolve() / "final_metrics_v1.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    if metrics["stage_status"]["P3"] != "FAIL":
        raise ValueError("formal P3 failure was not preserved")
    if metrics["stage_status"]["P4"] != "FAIL":
        raise ValueError("P4 qualification failure was not preserved")
    if metrics["stage_status"]["P5"] != "PASS":
        raise ValueError("P5 release-delivery stage did not pass")
    reproduction = metrics["release_reproduction"]
    if not reproduction["clean_build_passed"]:
        raise ValueError("clean P5 build was not preserved")
    if reproduction["clean_build_tests"] != 246:
        raise ValueError("clean P5 test count differs")
    if any(
        reproduction[key] != 0
        for key in ("clean_build_errors", "clean_build_failures", "clean_build_skips")
    ):
        raise ValueError("clean P5 build contains a non-green test result")
    if not reproduction["clean_reproduction_passed"]:
        raise ValueError("clean P5 smoke result was not preserved")
    if reproduction["clean_behavior_passes"] != 10:
        raise ValueError("clean P5 behaviour numerator differs")
    if (
        reproduction["reference_absolute_difference"]
        > reproduction["maximum_allowed_difference"]
    ):
        raise ValueError("clean P5 result differs too much from its reference")
    video = metrics["release_video"]
    if not video["passed"] or video["formal_evidence"]:
        raise ValueError("final video scope or result differs")
    if not 240.0 <= video["duration_sec"] <= 360.0:
        raise ValueError("final video duration is outside 4-6 minutes")
    video_path = _safe_path(video["path"])
    if _sha256(video_path) != video["sha256"]:
        raise ValueError("final video hash differs")
    if video["codec"] != "h264" or video["width"] != 1280 or video["height"] != 720:
        raise ValueError("final video media contract differs")
    safety = metrics["safety_and_scope"]
    forbidden_true = (
        "held_out_real_test_consumed",
        "held_out_test_receipt_exists",
        "physical_robot_evidence",
        "sim_to_real_claim",
        "formal_p3_rerun_authorized",
        "post_intervention_motion_matrix_authorized",
    )
    if any(safety[key] for key in forbidden_true):
        raise ValueError("a P5 safety/scope boundary was unexpectedly enabled")
    if metrics["formal_p3"]["positive_trials"] != 135:
        raise ValueError("formal P3 positive denominator differs")
    if metrics["formal_p3"]["positive_successes"] != 39:
        raise ValueError("formal P3 success numerator differs")
    if metrics["formal_p3"]["negative_trials"] != 30:
        raise ValueError("formal P3 negative denominator differs")
    if metrics["p4_qualification"]["heavy_target_pose_rate"] != 0.0:
        raise ValueError("P4 heavy target-pose failure differs")
    return {
        "schema_version": 1,
        "kind": "p5_release_verification",
        "status": "VERIFIED_FINAL_RELEASE",
        "input_binding_count": len(manifest["inputs"]),
        "output_binding_count": len(manifest["outputs"]),
        "formal_p3_preserved": True,
        "p4_failure_preserved": True,
        "clean_reproduction_preserved": True,
        "final_video_preserved": True,
        "held_out_test_sealed": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, default=DEFAULT_PACKAGE)
    arguments = parser.parse_args()
    result = verify(arguments.package_dir)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
