#!/usr/bin/env python3
"""Summarize isolated oracle pick trials and apply the P2 manipulation gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


SHUTDOWN_ERROR_MARKERS = (
    "exception was never retrieved",
    "failed to terminate",
    "traceback (most recent call last)",
    "process has died",
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-trials", type=int, required=True)
    return parser


def _unexpected_contact_events(diagnostics: dict) -> list[dict]:
    events = diagnostics.get("fruit_contacts", {}).get(
        "first_pair_events", []
    )
    unexpected = []
    for event in events:
        pair = event.get("pair", [])
        stage = str(event.get("stage", "UNKNOWN"))
        finger_contact = any(
            "panda_leftfinger" in item or "panda_rightfinger" in item
            for item in pair
        )
        released_bin_contact = (
            any("collection_bin" in item for item in pair)
            and stage in {"VERIFY", "DONE"}
        )
        if not finger_contact and not released_bin_contact:
            unexpected.append({"pair": pair, "stage": stage})
    return unexpected


def summarize(output_dir: Path, expected_trials: int) -> dict:
    if expected_trials <= 0:
        raise ValueError("expected_trials must be positive")
    records = []
    for index in range(1, expected_trials + 1):
        label = f"trial_{index:02d}"
        result_path = output_dir / f"{label}.json"
        launch_path = output_dir / f"{label}_launch.log"
        if result_path.exists():
            payload = json.loads(result_path.read_text(encoding="utf-8"))
        else:
            payload = {
                "success": False,
                "error": "result JSON was not produced",
            }

        diagnostics = payload.get("diagnostics", {})
        contacts = diagnostics.get("contacts", {})
        raw_dual_contact = all(
            contacts.get(side, {}).get("raw_contacts", 0) > 0
            for side in ("left", "right")
        )
        launch_text = (
            launch_path.read_text(encoding="utf-8", errors="replace").lower()
            if launch_path.exists()
            else "launch log missing"
        )
        shutdown_clean = not any(
            marker in launch_text for marker in SHUTDOWN_ERROR_MARKERS
        )
        action_success = bool(payload.get("success", False))
        planning_time = payload.get("planning_time_sec")
        records.append(
            {
                "trial_id": label,
                "action_success": action_success,
                "failure_code": payload.get("failure_code"),
                "message": payload.get(
                    "message", payload.get("error", "")
                ),
                "planning_time_sec": planning_time,
                "planning_time_within_5_sec": (
                    planning_time is not None
                    and float(planning_time) <= 5.0
                ),
                "raw_dual_contact": raw_dual_contact,
                "unexpected_fruit_contact_events": (
                    _unexpected_contact_events(diagnostics)
                ),
                "shutdown_clean": shutdown_clean,
                "result_file": result_path.name,
                "launch_log": launch_path.name,
            }
        )

    successes = sum(record["action_success"] for record in records)
    success_rate = successes / expected_trials
    gate_passed = (
        success_rate >= 0.90
        and all(record["shutdown_clean"] for record in records)
        and all(
            not record["unexpected_fruit_contact_events"]
            for record in records
        )
        and all(
            record["raw_dual_contact"]
            and record["planning_time_within_5_sec"]
            for record in records
            if record["action_success"]
        )
    )
    return {
        "schema_version": 1,
        "trial_count": expected_trials,
        "successes": successes,
        "success_rate": success_rate,
        "required_success_rate": 0.90,
        "gate_passed": gate_passed,
        "trials": records,
    }


def main() -> int:
    arguments = _parser().parse_args()
    summary = summarize(arguments.output_dir, arguments.expected_trials)
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
