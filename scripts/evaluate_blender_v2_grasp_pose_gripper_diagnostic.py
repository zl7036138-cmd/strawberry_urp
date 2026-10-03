#!/usr/bin/env python3
"""Evaluate one isolated grasp-pose gripper diagnostic under current policy."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MANIPULATION_ROOT = ROOT / "ros2_ws" / "src" / "strawberry_manipulation"
if str(MANIPULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(MANIPULATION_ROOT))

from strawberry_manipulation.moveit_backend import (  # noqa: E402
    gripper_result_allows_command,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def evaluate(payload: dict[str, object]) -> list[str]:
    violations: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            violations.append(message)

    require(
        payload.get("scope")
        == "NON_ACCEPTANCE_BLENDER_V2_GRASP_POSE_GRIPPER_DIAGNOSTIC",
        "unexpected diagnostic scope",
    )
    require(payload.get("errors") == [], "runtime errors were recorded")
    for boundary in (
        "pick_action_started",
        "attachment_command_started",
        "place_motion_started",
    ):
        require(payload.get(boundary) is False, f"{boundary} crossed")
    require(
        payload.get("observation_pose_control_close_command", {}).get(
            "measured_position_validation_passed"
        )
        is True,
        "free-space control close did not pass",
    )
    close = payload.get("close_command") or {}
    observed = close.get("observed_position_m")
    close_allowed = False
    if isinstance(observed, (int, float)):
        close_allowed = gripper_result_allows_command(
            target_position_m=float(
                close.get("target_position_m_per_finger", 0.0)
            ),
            observed_position_m=float(observed),
            open_position_m=float(payload.get("open_width_m_per_finger", 0.0)),
            closed_position_m=float(
                payload.get("closed_width_m_per_finger", 0.0)
            ),
            stalled=close.get("stalled") is True,
            reached_goal=close.get("reached_goal") is True,
        )
    require(close.get("status_succeeded") is True, "close action status failed")
    require(close_allowed, "close failed current measured-position policy")
    require(
        float(payload.get("measured_close_travel_m", 0.0)) >= 0.002,
        "close travel was below 2 mm",
    )
    require(
        all((payload.get("raw_target_contact_seen") or {}).get(side) is True
            for side in ("left", "right")),
        "raw bilateral target contact was not observed",
    )
    require(
        all(
            (payload.get("processed_target_contact_seen") or {}).get(side)
            is True
            for side in ("left", "right")
        ),
        "processed bilateral target contact was not observed",
    )
    require(
        payload.get("non_target_contact_seen") is False,
        "non-target contact occurred",
    )
    require(
        float(payload.get("fruit_displacement_during_close_m", 1.0)) <= 0.005,
        "fruit displacement exceeded 5 mm",
    )
    require(
        (payload.get("reopen_command") or {}).get(
            "measured_position_validation_passed"
        )
        is True,
        "reopen did not pass",
    )
    require(
        payload.get("safe_recovery_complete") is True,
        "safe recovery did not complete",
    )
    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()
    if options.output.exists():
        raise FileExistsError(f"refusing to overwrite {options.output}")
    payload = json.loads(options.input.read_text(encoding="utf-8"))
    violations = evaluate(payload)
    receipt = {
        "schema_version": 1,
        "scope": "NON_ACCEPTANCE_BLENDER_V2_GRASP_POSE_GRIPPER_VALIDATION",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "input": {
            "path": str(options.input),
            "size_bytes": options.input.stat().st_size,
            "sha256": _sha256(options.input),
        },
        "policy": {
            "controller_goal_tolerance_m": 0.003,
            "minimum_stalled_close_travel_m": 0.002,
            "minimum_measured_close_travel_m": 0.002,
            "maximum_fruit_displacement_m": 0.005,
            "require_raw_bilateral_target_contact": True,
            "require_processed_bilateral_target_contact": True,
            "require_safe_recovery": True,
        },
        "source_diagnostic_passed": payload.get("diagnostic_passed"),
        "passed": not violations,
        "violations": violations,
    }
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(receipt, sort_keys=True))
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
