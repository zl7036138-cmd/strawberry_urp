#!/usr/bin/env python3
"""Apply the synthetic-selected ADR-0021 candidate to audited real validation."""

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

from materialize_audited_validation import verify_audited_derivative  # noqa: E402
from materialize_sim_adaptation_training import (  # noqa: E402
    DEFAULT_CONTRACT,
    _display,
    _load_object,
    _resolve,
    _sha256,
)
from run_inference import _run_model  # noqa: E402
from workflow import (  # noqa: E402
    DatasetSample,
    evaluate_detection_metrics,
    record_to_json,
    summarize_speed,
    write_json_exclusive,
)


DEFAULT_SELECTION = Path(
    "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_synthetic_heldout_v1/summary.json"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_real_nonregression_v1"
)


def evaluate(contract_path: Path, selection_path: Path, output_dir: Path) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    contract = _load_object(contract_path)
    safety = contract["safety"]
    test_receipt = _resolve(Path(str(safety["formal_real_test_receipt_must_remain_absent"]))).resolve()
    if test_receipt.exists():
        raise FileExistsError("formal real-test receipt exists")
    selection_path = _resolve(selection_path).resolve()
    selection = _load_object(selection_path)
    if selection.get("kind") != "simulator_adaptation_synthetic_selection_decision":
        raise ValueError("unexpected synthetic selection artifact")
    if selection.get("variant") != contract["variant"] or selection.get("synthetic_gate_passed") is not True:
        raise ValueError("synthetic selection did not pass")
    if selection.get("selection_source") != "synthetic_heldout_only":
        raise ValueError("checkpoint was not selected only on synthetic held-out data")
    for key in ("audited_real_validation_accessed", "formal_real_test_accessed", "formal_simulator_matrix_started"):
        if selection.get(key) is not False:
            raise ValueError(f"synthetic selection boundary changed: {key}")

    selected = selection.get("selected_checkpoint")
    if not isinstance(selected, dict):
        raise ValueError("synthetic selection has no checkpoint")
    threshold = float(selected["selected_threshold"])
    recorded_threshold = float(selected["validation_metrics"]["confidence_threshold"])
    if threshold != recorded_threshold or not 0.0 <= threshold <= 1.0:
        raise ValueError("synthetic-selected threshold is inconsistent")
    weight_path = _resolve(Path(str(selected["weight_path"]))).resolve()
    if not weight_path.is_file() or _sha256(weight_path) != selected["weight_sha256"]:
        raise ValueError("synthetic-selected checkpoint changed")
    candidate_path = selection_path.parent / str(selected["artifact"])
    if not candidate_path.is_file() or _sha256(candidate_path) != selected["artifact_sha256"]:
        raise ValueError("synthetic-selected prediction artifact changed")

    claim_path = _resolve(Path(str(contract["training"]["claim"]))).resolve()
    claim = _load_object(claim_path)
    if claim.get("status") != "completed" or claim.get("purpose") != f"simulator_adaptation_training:{contract['variant']}":
        raise ValueError("simulator-adaptation claim is not complete")
    checkpoint_inventory = claim.get("details", {}).get("checkpoint_inventory", [])
    match = [item for item in checkpoint_inventory if item.get("path") == selected["weight_path"]]
    if len(match) != 1 or match[0].get("sha256") != selected["weight_sha256"]:
        raise ValueError("selected checkpoint is outside the completed claim")

    audit_binding = contract["audited_real_validation"]["manifest"]
    audit_path = _resolve(Path(str(audit_binding["path"]))).resolve()
    if (
        not audit_path.is_file()
        or audit_path.stat().st_size != int(audit_binding["size_bytes"])
        or _sha256(audit_path) != audit_binding["sha256"]
    ):
        raise ValueError("audited real-validation manifest changed")
    verification = verify_audited_derivative(audit_path)
    if verification["canonical_validation_derivative_sha256"] != audit_binding["canonical_sha256"]:
        raise ValueError("audited real-validation canonical digest changed")
    audit = _load_object(audit_path)
    entries = audit.get("entries")
    if not isinstance(entries, list) or len(entries) != int(contract["audited_real_validation"]["image_count"]):
        raise ValueError("audited real-validation sample count changed")
    audit_root = audit_path.parent
    real_root = _resolve(Path("data/processed/zenodo_6126677")).resolve()
    samples: list[DatasetSample] = []
    for item in sorted(entries, key=lambda row: str(row["image"])):
        image_path = real_root / str(item["image"])
        label_path = audit_root / str(item["audited_label"])
        if (
            not image_path.is_file()
            or image_path.stat().st_size != int(item["source_image_size_bytes"])
            or _sha256(image_path) != item["source_image_sha256"]
            or not label_path.is_file()
            or label_path.stat().st_size != int(item["audited_label_size_bytes"])
            or _sha256(label_path) != item["audited_label_sha256"]
        ):
            raise ValueError(f"audited real-validation source changed: {item['image']}")
        samples.append(DatasetSample(image_key=str(item["image"]), image_path=image_path, label_path=label_path))

    evaluation = contract["selection_and_evaluation"]
    if evaluation.get("real_validation_rule") != "evaluate_selected_checkpoint_once_at_synthetic_selected_threshold_no_retuning":
        raise ValueError("real non-regression rule changed")
    output_dir = _resolve(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite real non-regression output: {output_dir}")
    output_dir.mkdir(parents=True)
    records = _run_model(
        weight_path,
        tuple(samples),
        (0, 1),
        raw_confidence=float(evaluation["raw_confidence"]),
        nms_iou=float(evaluation["nms_iou"]),
        imgsz=int(evaluation["imgsz"]),
        batch_size=int(evaluation["batch"]),
        device=str(evaluation["device"]),
    )
    metrics = evaluate_detection_metrics(
        records,
        confidence_threshold=threshold,
        iou_threshold=float(evaluation["metric_iou"]),
        class_ids=(0, 1),
    )
    minimum = evaluation["promotion"]["audited_real_validation_minimum"]
    checks = {
        "macro_f1": {"required": float(minimum["macro_f1"]), "actual": float(metrics["macro_f1"])},
        "ripe_f1": {"required": float(minimum["ripe_f1"]), "actual": float(metrics["per_class"]["0"]["f1"])},
        "unripe_f1": {"required": float(minimum["unripe_f1"]), "actual": float(metrics["per_class"]["1"]["f1"])},
    }
    for check in checks.values():
        check["passed"] = check["actual"] >= check["required"]
    passed = all(bool(item["passed"]) for item in checks.values())
    predictions = {
        "schema_version": 1,
        "kind": "simulator_adaptation_audited_real_validation_predictions",
        "variant": contract["variant"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "selection": {"path": _display(selection_path), "size_bytes": selection_path.stat().st_size, "sha256": _sha256(selection_path)},
        "checkpoint": {"path": _display(weight_path), "size_bytes": weight_path.stat().st_size, "sha256": _sha256(weight_path)},
        "confidence_threshold_source": "synthetic_heldout_selection",
        "confidence_threshold": threshold,
        "metrics": metrics,
        "speed": summarize_speed(records),
        "records": [record_to_json(record) for record in records],
        "threshold_search_performed": False,
        "formal_real_test_accessed": False,
    }
    prediction_path = output_dir / "predictions.json"
    write_json_exclusive(prediction_path, predictions)
    summary = {
        "schema_version": 1,
        "kind": "simulator_adaptation_real_nonregression_decision",
        "variant": contract["variant"],
        "selected_checkpoint": selected["checkpoint_name"],
        "selected_weight": selected["weight_path"],
        "selected_weight_sha256": selected["weight_sha256"],
        "confidence_threshold": threshold,
        "confidence_threshold_source": "synthetic_heldout_selection",
        "threshold_search_performed": False,
        "sample_count": len(samples),
        "metrics": metrics,
        "nonregression_checks": checks,
        "real_nonregression_passed": passed,
        "predictions": {"path": prediction_path.relative_to(output_dir).as_posix(), "sha256": _sha256(prediction_path)},
        "formal_real_test_accessed": False,
        "formal_simulator_matrix_started": False,
        "perception_control_authorized": False,
        "robot_motion_authorized": False,
        "formal_acceptance": False,
    }
    write_json_exclusive(output_dir / "summary.json", summary)
    if test_receipt.exists():
        raise RuntimeError("formal real-test receipt appeared during real validation")
    print(json.dumps({"event": "real_nonregression_complete", "threshold": threshold, "macro_f1": metrics["macro_f1"], "ripe_f1": metrics["per_class"]["0"]["f1"], "unripe_f1": metrics["per_class"]["1"]["f1"], "passed": passed}, indent=2), flush=True)
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--selection", type=Path, default=DEFAULT_SELECTION)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    result = evaluate(arguments.contract, arguments.selection, arguments.output)
    return 0 if result["real_nonregression_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
