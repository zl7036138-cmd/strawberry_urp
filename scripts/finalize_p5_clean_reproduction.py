#!/usr/bin/env python3
"""Validate and freeze the clean P5 reproduction evidence handoff."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "artifacts" / "p5" / "p5_clean_reproduction_handoff_v1.json"
)

EVIDENCE = {
    "decision": "docs/decisions/0033-accept-clean-p5-release-reproduction.md",
    "clean_build": "results/p5/clean_build_v1/summary.json",
    "clean_smoke": "results/p5/release_smoke_clean_v1/summary.json",
    "reference_smoke": "results/p5/release_smoke_reference_v1/summary.json",
    "clean_environment": "results/p5/release_smoke_clean_v1/environment.json",
    "smoke_contract": "config/p5_release_smoke_v1.json",
    "smoke_runner": "scripts/run_p5_release_smoke.sh",
    "smoke_summarizer": "scripts/summarize_p5_release_smoke.py",
    "bootstrap": "scripts/bootstrap_ubuntu_2404.sh",
    "build_and_test": "scripts/build_and_test.sh",
    "wheel_lock": "requirements/perception-linux-cp312-wheelhouse.txt",
    "wheel_cache_script": "scripts/cache_perception_wheels.ps1",
    "wheelhouse_manifest": ".cache/wheels/wheelhouse-manifest.json",
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
        raise ValueError(f"required evidence is missing: {relative_path}")
    return {
        "path": relative_path,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _read(relative_path: str) -> dict[str, Any]:
    path = REPOSITORY_ROOT / relative_path
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{relative_path} must contain a JSON object")
    return value


def _validate(build: dict[str, Any], clean: dict[str, Any]) -> None:
    if build.get("kind") != "p5_clean_build_receipt":
        raise ValueError("unexpected clean-build receipt kind")
    if build.get("status") != "PASS":
        raise ValueError("clean build did not pass")
    if build["environment"]["wsl_distribution"] != "Ubuntu-24.04-URP-Repro":
        raise ValueError("clean build was not captured in the reproduction distro")
    if build["environment"]["os_version_id"] != "24.04":
        raise ValueError("clean build did not use Ubuntu 24.04")
    if build["environment"]["ros_distro"] != "jazzy":
        raise ValueError("clean build did not use ROS 2 Jazzy")
    if build["build"]["package_count"] != 7:
        raise ValueError("clean build package count differs")
    tests = build["tests"]
    if (
        tests["tests"] != 246
        or tests["errors"] != 0
        or tests["failures"] != 0
        or tests["skipped"] != 0
    ):
        raise ValueError("clean build test receipt differs from the acceptance gate")

    if clean.get("kind") != "p5_release_smoke":
        raise ValueError("unexpected clean-smoke receipt kind")
    if clean.get("environment_role") != "clean":
        raise ValueError("release smoke was not marked as clean")
    required_true = (
        "smoke_passed",
        "clean_reproduction_passed",
        "infrastructure_passed",
        "shutdown_passed",
    )
    if not all(clean.get(key) is True for key in required_true):
        raise ValueError("clean release smoke did not pass every subgate")
    if clean["positive_successes"] != 5 or clean["positive_trials"] != 5:
        raise ValueError("clean positive smoke result differs")
    if clean["negative_no_picks"] != 5 or clean["negative_trials"] != 5:
        raise ValueError("clean negative smoke result differs")
    if clean["negative_false_picks"] != 0:
        raise ValueError("clean smoke contains a false pick")
    comparison = clean["reference_comparison"]
    if not comparison["passed"] or comparison["absolute_difference"] > 0.05:
        raise ValueError("clean smoke differs too much from the reference")

    forbidden_true = (
        "held_out_real_test_consumed",
        "formal_p3_rerun",
        "p4_intervention_used",
        "physical_robot_evidence",
        "sim_to_real_claim",
    )
    if any(build["scope"][key] for key in forbidden_true):
        raise ValueError("clean build enabled a forbidden scope claim")
    if any(clean["safety"][key] for key in forbidden_true):
        raise ValueError("clean smoke enabled a forbidden scope claim")


def finalize(output: Path) -> dict[str, Any]:
    build = _read(EVIDENCE["clean_build"])
    clean = _read(EVIDENCE["clean_smoke"])
    _validate(build, clean)
    handoff = {
        "schema_version": 1,
        "kind": "p5_clean_reproduction_handoff",
        "status": "CLEAN_REPRODUCTION_ACCEPTED_VIDEO_PENDING",
        "recorded_date": "2026-07-24",
        "decision": "ADR-0033",
        "environment": {
            "wsl_distribution": build["environment"]["wsl_distribution"],
            "os_pretty_name": build["environment"]["os_pretty_name"],
            "ros_distro": build["environment"]["ros_distro"],
            "cuda_device": build["environment"]["cuda_device"],
        },
        "build": {
            "package_count": build["build"]["package_count"],
            "tests": build["tests"]["tests"],
            "errors": build["tests"]["errors"],
            "failures": build["tests"]["failures"],
            "skipped": build["tests"]["skipped"],
            "passed": True,
        },
        "release_smoke": {
            "trials": clean["trial_count"],
            "behavior_passes": clean["behavior_passes"],
            "behavior_pass_rate": clean["behavior_pass_rate"],
            "positive_successes": clean["positive_successes"],
            "positive_trials": clean["positive_trials"],
            "negative_no_picks": clean["negative_no_picks"],
            "negative_trials": clean["negative_trials"],
            "negative_false_picks": clean["negative_false_picks"],
            "reference_absolute_difference": clean["reference_comparison"][
                "absolute_difference"
            ],
            "maximum_allowed_difference": clean["reference_comparison"][
                "maximum_difference"
            ],
            "passed": True,
        },
        "bindings": {
            key: _binding(relative_path)
            for key, relative_path in EVIDENCE.items()
        },
        "safety": {
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_used": False,
            "formal_p3_result_changed": False,
            "p4_result_changed": False,
            "physical_robot_claim": False,
            "sim_to_real_claim": False,
        },
        "remaining": [
            "capture and assemble the headed demonstration recording",
            "verify and bind the video artifact",
            "freeze the final P5 release manifest",
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
