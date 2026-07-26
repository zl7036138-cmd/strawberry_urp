#!/usr/bin/env python3
"""Verify and summarize the nine-condition scene-injection pre-gate."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from strawberry_sim.scene_conditions import (  # noqa: E402
    LIGHTING_LEVELS,
    OCCLUSION_LEVELS,
    fingerprint,
    load_scene_condition_config,
    summarize_condition_matrix,
)


ERROR_MARKERS = (
    "traceback (most recent call last)",
    "exception was never retrieved",
    "process has died",
)


def summarize(output_dir: Path, config_path: Path) -> dict[str, object]:
    config = load_scene_condition_config(config_path)
    config_record = fingerprint(config_path)
    condition_records = []
    probes = []
    blockers = []
    expected_count = len(LIGHTING_LEVELS) * len(OCCLUSION_LEVELS)

    for light in LIGHTING_LEVELS:
        for occlusion in OCCLUSION_LEVELS:
            condition_id = f"{light}__{occlusion}"
            directory = output_dir / condition_id
            receipt_path = directory / "receipt.json"
            probe_path = directory / "probe.json"
            launch_path = directory / "launch.log"
            probe_log_path = directory / "probe.log"
            errors = []
            try:
                receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
                probe = json.loads(probe_path.read_text(encoding="utf-8"))
                if receipt.get("lighting_level") != light:
                    errors.append("receipt lighting level mismatch")
                if receipt.get("occlusion_level") != occlusion:
                    errors.append("receipt occlusion level mismatch")
                if receipt.get("condition_config", {}).get("sha256") != (
                    config_record["sha256"]
                ):
                    errors.append("receipt is not bound to the frozen config")
                if receipt.get("formal_acceptance") is not False:
                    errors.append("receipt crossed the non-acceptance boundary")
                if receipt.get("held_out_test_consumed") is not False:
                    errors.append("receipt claims held-out test consumption")
                if probe.get("lighting_level") != light:
                    errors.append("probe lighting level mismatch")
                if probe.get("occlusion_level") != occlusion:
                    errors.append("probe occlusion level mismatch")
                if probe.get("receipt", {}).get("sha256") != fingerprint(
                    receipt_path
                )["sha256"]:
                    errors.append("probe is not bound to its receipt")
                if probe.get("materialized_world_sha256") != receipt.get(
                    "materialized_world", {}
                ).get("sha256"):
                    errors.append("probe world hash differs from its receipt")
            except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError) as error:
                receipt = None
                probe = None
                errors.append(f"missing or invalid runtime artifact: {error}")

            log_markers = []
            for log_path in (launch_path, probe_log_path):
                if not log_path.exists():
                    errors.append(f"missing log: {log_path.name}")
                    continue
                text = log_path.read_text(encoding="utf-8", errors="replace").lower()
                log_markers.extend(
                    marker for marker in ERROR_MARKERS if marker in text
                )
            if log_markers:
                errors.append(
                    "runtime log contains failure markers: "
                    + ", ".join(sorted(set(log_markers)))
                )

            valid = not errors
            if valid:
                probes.append(probe)
            else:
                blockers.extend(f"{condition_id}: {error}" for error in errors)
            condition_records.append(
                {
                    "condition_id": condition_id,
                    "lighting_level": light,
                    "occlusion_level": occlusion,
                    "infrastructure_valid": valid,
                    "errors": errors,
                    "receipt": (
                        fingerprint(receipt_path) if receipt_path.exists() else None
                    ),
                    "probe": fingerprint(probe_path) if probe_path.exists() else None,
                    "launch_log": (
                        fingerprint(launch_path) if launch_path.exists() else None
                    ),
                    "probe_log": (
                        fingerprint(probe_log_path)
                        if probe_log_path.exists()
                        else None
                    ),
                }
            )

    if len(probes) == expected_count:
        matrix = summarize_condition_matrix(probes, config)
        if not matrix["passed"]:
            blockers.append("runtime matrix did not meet the frozen injection checks")
    else:
        matrix = {
            "gate": "SCENE_CONDITION_INJECTION_PRE_GATE",
            "passed": False,
            "error": "not all nine conditions produced valid bound artifacts",
        }

    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "gate": "SCENE_CONDITION_INJECTION_PRE_GATE",
        "scope": config["scope"],
        "formal_acceptance": False,
        "may_close_t30_gate": False,
        "may_close_p2_gate": False,
        "may_close_p3_gate": False,
        "may_close_p4_gate": False,
        "held_out_test_consumed": False,
        "condition_config": config_record,
        "expected_condition_count": expected_count,
        "valid_condition_count": len(probes),
        "conditions": condition_records,
        "matrix": matrix,
        "blockers": blockers,
        "passed": not blockers and bool(matrix["passed"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    options = parser.parse_args()
    result = summarize(options.output_dir, options.config)
    summary_path = options.summary or options.output_dir / "summary.json"
    if summary_path.exists():
        raise ValueError(f"refusing to overwrite summary: {summary_path}")
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"passed": result["passed"], "summary": str(summary_path)}))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
