#!/usr/bin/env python3
"""Validate and query the isolated ten-trial P5 release-smoke contract."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws" / "src" / "strawberry_benchmark"))

from strawberry_benchmark.perception_dev_gate import (  # noqa: E402
    load_perception_repeated_dev_gate,
    verify_perception_repeated_dev_gate,
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be an object")
    return value


def load_contract(path: Path) -> dict[str, Any]:
    root = _mapping(json.loads(path.read_text(encoding="utf-8")), "P5 smoke")
    if root.get("schema_version") != 1:
        raise ValueError("P5 smoke schema_version must be 1")
    if root.get("smoke_id") != "p5_release_smoke_v1":
        raise ValueError("unexpected P5 smoke ID")
    if root.get("scope") != "NON_FORMAL_RELEASE_REPRODUCTION_SMOKE":
        raise ValueError("P5 smoke scope must remain non-formal")
    for key in (
        "formal_acceptance",
        "formal_p3_rerun",
        "p4_intervention_used",
        "held_out_real_test_consumed",
    ):
        if root.get(key) is not False:
            raise ValueError(f"{key} must remain false")
    execution = _mapping(root.get("execution"), "execution")
    acceptance = _mapping(root.get("acceptance"), "acceptance")
    if execution.get("fresh_world_per_trial") is not True:
        raise ValueError("P5 smoke requires a fresh world per trial")
    if execution.get("headless") is not True:
        raise ValueError("P5 smoke must remain headless")
    if (execution.get("positive_trials"), execution.get("negative_trials")) != (5, 5):
        raise ValueError("P5 smoke must retain five positive and five negative trials")
    base_domain = execution.get("base_ros_domain_id")
    if not isinstance(base_domain, int) or not 0 <= base_domain <= 223:
        raise ValueError("base ROS domain must leave room for ten trials")
    expected = {
        "minimum_positive_success_rate": 0.8,
        "minimum_negative_no_pick_rate": 1.0,
        "maximum_negative_false_picks": 0,
        "maximum_negative_control_attempts": 0,
        "planning_time_p95_limit_sec": 5.0,
        "maximum_reference_difference": 0.05,
    }
    for key, value in expected.items():
        actual = acceptance.get(key)
        if isinstance(value, float):
            if not isinstance(actual, (int, float)) or not math.isclose(actual, value):
                raise ValueError(f"{key} differs from {value}")
        elif actual != value:
            raise ValueError(f"{key} differs from {value}")
    if acceptance.get("require_all_infrastructure_valid") is not True:
        raise ValueError("all infrastructure must be valid")
    if acceptance.get("require_all_shutdown_clean") is not True:
        raise ValueError("all shutdowns must be clean")
    binding = _mapping(root.get("source_contract_binding"), "source binding")
    relative = Path(str(binding.get("relative_path", "")))
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("source contract path must be repository-relative")
    source_path = (REPOSITORY_ROOT / relative).resolve()
    if not source_path.is_file() or REPOSITORY_ROOT.resolve() not in source_path.parents:
        raise ValueError("bound source contract is missing or unsafe")
    if sha256_file(source_path) != binding.get("sha256"):
        raise ValueError("bound source contract digest differs")
    source = load_perception_repeated_dev_gate(source_path)
    waiver, verified = verify_perception_repeated_dev_gate(source, REPOSITORY_ROOT)
    return {
        "raw": root,
        "source_path": source_path,
        "source": source,
        "waiver": waiver,
        "verified": verified,
    }


def _target(scene: Any) -> str:
    return "\t".join(
        (
            scene.scenario_id,
            scene.target.model_name,
            str(scene.target.target_id),
            *(format(value, ".12g") for value in scene.target.position_m),
        )
    )


def _parked(scene: Any) -> str:
    return "\n".join(
        "\t".join(
            (
                item.model_name,
                str(item.target_id),
                *(format(value, ".12g") for value in item.position_m),
            )
        )
        for item in scene.parked_models
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--emit-runtime-tsv", action="store_true")
    parser.add_argument("--emit-positive-target-tsv", action="store_true")
    parser.add_argument("--emit-negative-target-tsv", action="store_true")
    parser.add_argument("--emit-positive-park-tsv", action="store_true")
    parser.add_argument("--emit-negative-park-tsv", action="store_true")
    arguments = parser.parse_args()
    modes = (
        arguments.emit_runtime_tsv,
        arguments.emit_positive_target_tsv,
        arguments.emit_negative_target_tsv,
        arguments.emit_positive_park_tsv,
        arguments.emit_negative_park_tsv,
    )
    if sum(modes) > 1:
        raise SystemExit("choose at most one TSV output mode")
    contract = load_contract(arguments.config)
    root = contract["raw"]
    source = contract["source"]
    waiver = contract["waiver"]
    verified = contract["verified"]
    if arguments.emit_runtime_tsv:
        execution = root["execution"]
        acceptance = root["acceptance"]
        print(
            "\t".join(
                (
                    str(verified["model"]),
                    format(waiver.confidence_threshold, ".12g"),
                    waiver.detections_topic,
                    waiver.target_pose_topic,
                    waiver.control_target_topic,
                    str(execution["positive_trials"]),
                    str(execution["negative_trials"]),
                    str(source.minimum_settled_detection_frames),
                    str(execution["base_ros_domain_id"]),
                    format(acceptance["maximum_reference_difference"], ".12g"),
                )
            )
        )
    elif arguments.emit_positive_target_tsv:
        print(_target(source.positive_scene))
    elif arguments.emit_negative_target_tsv:
        print(_target(source.negative_scene))
    elif arguments.emit_positive_park_tsv:
        print(_parked(source.positive_scene))
    elif arguments.emit_negative_park_tsv:
        print(_parked(source.negative_scene))
    else:
        print(
            f"validated {root['smoke_id']}: 5 positive + 5 negative fresh worlds, "
            f"non-formal, model={waiver.model_sha256[:12]}..., held-out test sealed"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
