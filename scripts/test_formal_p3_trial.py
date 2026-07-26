#!/usr/bin/env python3
"""Enrich the proven orchestrated trial client with formal-matrix metadata."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def _fingerprint(path: Path) -> dict:
    content = path.read_bytes()
    return {
        "path": path.resolve().relative_to(ROOT.resolve()).as_posix(),
        "size_bytes": len(content),
        "sha256": hashlib.sha256(content).hexdigest(),
    }


def _load_base_client():
    path = ROOT / "scripts/test_orchestrated_trial.py"
    spec = importlib.util.spec_from_file_location("formal_base_trial_client", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load the base orchestrated trial client")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--formal-kind", choices=("POSITIVE", "NEGATIVE"), required=True)
    parser.add_argument("--formal-lighting", required=True)
    parser.add_argument("--formal-occlusion", required=True)
    parser.add_argument("--formal-position", required=True)
    parser.add_argument("--formal-simulation-seed", type=int, required=True)
    parser.add_argument("--formal-order-index", type=int, required=True)
    parser.add_argument("--condition-receipt", type=Path, required=True)
    parser.add_argument("--condition-probe", type=Path, required=True)
    formal, remaining = parser.parse_known_args()
    if formal.formal_order_index <= 0 or formal.formal_simulation_seed < 0:
        raise SystemExit("formal order index and simulation seed are invalid")
    receipt_path = formal.condition_receipt.resolve(strict=True)
    probe_path = formal.condition_probe.resolve(strict=True)
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    probe = json.loads(probe_path.read_text(encoding="utf-8"))
    if receipt.get("lighting_level") != formal.formal_lighting:
        raise SystemExit("condition receipt lighting differs from the formal scenario")
    if receipt.get("occlusion_level") != formal.formal_occlusion:
        raise SystemExit("condition receipt occlusion differs from the formal scenario")
    if probe.get("lighting_level") != formal.formal_lighting:
        raise SystemExit("condition probe lighting differs from the formal scenario")
    if probe.get("occlusion_level") != formal.formal_occlusion:
        raise SystemExit("condition probe occlusion differs from the formal scenario")

    base = _load_base_client()
    original_emit = base._emit

    def enriched_emit(payload: dict, output: Path) -> None:
        statuses = payload.get("status_messages", [])
        final_status = payload.get("final_status") or {}
        history = final_status.get("state_history", [])
        visited = set(payload.get("visited_states", []))
        for status in statuses:
            state = status.get("state")
            if state:
                visited.add(state)
            for transition in status.get("state_history", []):
                visited.update(
                    value for value in (transition.get("previous"), transition.get("current"))
                    if value
                )
        for transition in history:
            visited.update(
                value for value in (transition.get("previous"), transition.get("current"))
                if value
            )
        behavior_started = bool(final_status) or any(
            state not in {None, "IDLE"} for state in visited
        )
        pick_attempted = final_status.get("control_target_id") is not None or bool(
            visited.intersection({"PLAN", "APPROACH", "GRASP", "RETREAT", "PLACE", "VERIFY"})
        )
        fruit_picked = bool(
            visited.intersection({"RETREAT", "PLACE", "VERIFY"})
            or final_status.get("outcome") == "SUCCESS"
        )
        truth = payload.get("configured_truth_position_m")
        control = payload.get("configured_control_position_m")
        localization_error_mm = None
        if truth is not None and control is not None:
            localization_error_mm = 1000.0 * sum(
                (float(left) - float(right)) ** 2
                for left, right in zip(truth, control)
            ) ** 0.5
        payload.update(
            {
                "formal_schema_version": 1,
                "formal_kind": formal.formal_kind,
                "formal_lighting": formal.formal_lighting,
                "formal_occlusion": formal.formal_occlusion,
                "formal_position": formal.formal_position,
                "formal_simulation_seed": formal.formal_simulation_seed,
                "formal_order_index": formal.formal_order_index,
                "trial_service_accepted": behavior_started,
                "behavior_started": behavior_started,
                "pick_attempted": pick_attempted,
                "fruit_picked": fruit_picked,
                "localization_error_mm": localization_error_mm,
                "condition_receipt": _fingerprint(receipt_path),
                "condition_probe": _fingerprint(probe_path),
            }
        )
        original_emit(payload, output)

    base._emit = enriched_emit
    sys.argv = [sys.argv[0], *remaining]
    return int(base.main())


if __name__ == "__main__":
    raise SystemExit(main())
