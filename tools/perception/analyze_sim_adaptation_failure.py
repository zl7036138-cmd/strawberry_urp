#!/usr/bin/env python3
"""Analyze the rejected ADR-0021 candidate without changing its decision."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from materialize_sim_adaptation_training import (  # noqa: E402
    DEFAULT_CONTRACT,
    _display,
    _load_object,
    _resolve,
    _sha256,
)
from workflow import (  # noqa: E402
    _record_from_json,
    evaluate_detection_metrics,
    select_validation_threshold,
    threshold_grid,
    write_json_exclusive,
)


DEFAULT_SELECTION = Path(
    "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_synthetic_heldout_v1/summary.json"
)
DEFAULT_REAL_DIR = Path(
    "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_real_nonregression_v1"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_failure_analysis_v1.json"
)


def analyze(contract_path: Path, selection_path: Path, real_dir: Path, output_path: Path) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    selection_path = _resolve(selection_path).resolve()
    real_dir = _resolve(real_dir).resolve()
    output_path = _resolve(output_path).resolve()
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite failure analysis: {output_path}")
    contract = _load_object(contract_path)
    selection = _load_object(selection_path)
    real_summary_path = real_dir / "summary.json"
    predictions_path = real_dir / "predictions.json"
    real_summary = _load_object(real_summary_path)
    predictions = _load_object(predictions_path)
    if selection.get("synthetic_gate_passed") is not True:
        raise ValueError("synthetic gate was not passed")
    if real_summary.get("real_nonregression_passed") is not False:
        raise ValueError("real non-regression screen did not fail")
    if real_summary.get("threshold_search_performed") is not False:
        raise ValueError("real screen was retuned")
    if predictions.get("threshold_search_performed") is not False:
        raise ValueError("real predictions claim a threshold search")
    if real_summary.get("formal_real_test_accessed") is not False or predictions.get("formal_real_test_accessed") is not False:
        raise ValueError("formal real-test boundary was crossed")
    if real_summary.get("predictions", {}).get("sha256") != _sha256(predictions_path):
        raise ValueError("real prediction binding mismatch")
    records_value = predictions.get("records")
    if not isinstance(records_value, list) or len(records_value) != 116:
        raise ValueError("real prediction record count changed")
    records = tuple(_record_from_json(item) for item in records_value)

    evaluation = contract["selection_and_evaluation"]
    thresholds = threshold_grid(
        float(evaluation["threshold_grid"]["start"]),
        float(evaluation["threshold_grid"]["stop"]),
        float(evaluation["threshold_grid"]["step"]),
    )
    counterfactual = select_validation_threshold(
        records,
        thresholds,
        iou_threshold=float(evaluation["metric_iou"]),
        class_ids=(0, 1),
    )
    curve = [
        evaluate_detection_metrics(
            records,
            confidence_threshold=value,
            iou_threshold=float(evaluation["metric_iou"]),
            class_ids=(0, 1),
        )
        for value in thresholds
    ]
    frozen_metrics = real_summary["metrics"]
    best_metrics = counterfactual["validation_metrics"]
    baseline = evaluation["promotion"]["audited_real_validation_baseline"]
    minimum = evaluation["promotion"]["audited_real_validation_minimum"]

    def compact(metrics: dict[str, Any]) -> dict[str, float]:
        return {
            "confidence_threshold": float(metrics["confidence_threshold"]),
            "macro_f1": float(metrics["macro_f1"]),
            "ripe_f1": float(metrics["per_class"]["0"]["f1"]),
            "unripe_f1": float(metrics["per_class"]["1"]["f1"]),
        }

    frozen = compact(frozen_metrics)
    diagnostic_best = compact(best_metrics)
    diagnostic_best_passes = {
        "macro_f1": diagnostic_best["macro_f1"] >= float(minimum["macro_f1"]),
        "ripe_f1": diagnostic_best["ripe_f1"] >= float(minimum["ripe_f1"]),
        "unripe_f1": diagnostic_best["unripe_f1"] >= float(minimum["unripe_f1"]),
    }
    analysis_path = Path(__file__).resolve()
    result = {
        "schema_version": 1,
        "kind": "simulator_adaptation_failed_candidate_analysis",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "variant": contract["variant"],
        "bindings": {
            "contract": {"path": _display(contract_path), "size_bytes": contract_path.stat().st_size, "sha256": _sha256(contract_path)},
            "synthetic_selection": {"path": _display(selection_path), "size_bytes": selection_path.stat().st_size, "sha256": _sha256(selection_path)},
            "real_summary": {"path": _display(real_summary_path), "size_bytes": real_summary_path.stat().st_size, "sha256": _sha256(real_summary_path)},
            "real_predictions": {"path": _display(predictions_path), "size_bytes": predictions_path.stat().st_size, "sha256": _sha256(predictions_path)},
            "analyzer": {"path": _display(analysis_path), "size_bytes": analysis_path.stat().st_size, "sha256": _sha256(analysis_path)},
        },
        "frozen_decision": {
            "synthetic_gate_passed": True,
            "real_nonregression_passed": False,
            "candidate_promoted": False,
            "promotion_terminated_at": "audited_real_validation_nonregression",
            "remaining_promotion_screens_run": False,
            "remaining_promotion_screens": ["D2 multiposition Shadow", "simulator ripe/unripe negative screen"],
            "retry_authorized": False,
            "threshold_relaxation_authorized": False,
        },
        "frozen_real_result": frozen,
        "audited_baseline": baseline,
        "required_minimum": minimum,
        "delta_from_baseline_at_frozen_threshold": {
            "macro_f1": frozen["macro_f1"] - float(baseline["macro_f1"]),
            "ripe_f1": frozen["ripe_f1"] - float(baseline["ripe_f1"]),
            "unripe_f1": frozen["unripe_f1"] - float(baseline["unripe_f1"]),
        },
        "diagnostic_counterfactual_only": {
            "decision_effect": "none",
            "retuning_authorized": False,
            "purpose": "separate threshold-transfer failure from checkpoint capability after rejection",
            "best_real_validation_result": diagnostic_best,
            "would_pass_individual_minimums": diagnostic_best_passes,
            "would_pass_all_minimums": all(diagnostic_best_passes.values()),
            "threshold_curve": [compact(item) for item in curve],
        },
        "interpretation": {
            "threshold_transfer_penalty_macro_f1": diagnostic_best["macro_f1"] - frozen["macro_f1"],
            "checkpoint_below_real_minimum_even_after_counterfactual_retuning": not all(diagnostic_best_passes.values()),
            "conclusion": "The candidate is rejected. The offline sweep is diagnostic only and cannot change the frozen threshold, rerun training, or promote the model.",
        },
        "formal_real_test_accessed": False,
        "formal_simulator_matrix_started": False,
        "perception_control_authorized": False,
        "robot_motion_authorized": False,
        "formal_acceptance": False,
    }
    write_json_exclusive(output_path, result)
    print(
        json.dumps(
            {
                "event": "sim_adaptation_failure_analysis_complete",
                "frozen_macro_f1": frozen["macro_f1"],
                "diagnostic_best_threshold": diagnostic_best["confidence_threshold"],
                "diagnostic_best_macro_f1": diagnostic_best["macro_f1"],
                "diagnostic_best_passes_all": all(diagnostic_best_passes.values()),
                "candidate_promoted": False,
            },
            indent=2,
        )
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--selection", type=Path, default=DEFAULT_SELECTION)
    parser.add_argument("--real-dir", type=Path, default=DEFAULT_REAL_DIR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    analyze(arguments.contract, arguments.selection, arguments.real_dir, arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
