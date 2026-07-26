#!/usr/bin/env python3
"""Recompute validation metrics from frozen predictions and audited labels."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from materialize_audited_validation import verify_audited_derivative  # noqa: E402
from workflow import (  # noqa: E402
    ImageDetections,
    evaluate_detection_metrics,
    load_prediction_bundle,
    load_yolo_ground_truth,
    select_validation_threshold,
    threshold_grid,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DERIVATIVE = Path(
    "data/processed/zenodo_6126677_val_audit_v1/audit_manifest.json"
)
DEFAULT_BUNDLE = Path("artifacts/perception/yolo11s_validation_predictions.json")
DEFAULT_THRESHOLD = Path("artifacts/perception/yolo11s_frozen_threshold.json")
DEFAULT_EXPERIMENTS = Path("tools/perception/experiments.json")
DEFAULT_OUTPUT = Path(
    "artifacts/perception/label_audit/yolo11s_val_threshold_031_audited_metrics_v1.json"
)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _metric_delta(current: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    return {
        "macro_f1": float(current["macro_f1"]) - float(baseline["macro_f1"]),
        "per_class_f1": {
            class_id: float(current["per_class"][class_id]["f1"])
            - float(baseline["per_class"][class_id]["f1"])
            for class_id in ("0", "1")
        },
    }


def _assert_metric_reproduction(
    reproduced: dict[str, Any], frozen: dict[str, Any]
) -> None:
    for field in ("confidence_threshold", "iou_threshold", "macro_f1"):
        if abs(float(reproduced[field]) - float(frozen[field])) > 1e-12:
            raise ValueError(f"current metric contract does not reproduce frozen {field}")
    for class_id in ("0", "1"):
        for field in ("support", "predictions", "tp", "fp", "fn"):
            if int(reproduced["per_class"][class_id][field]) != int(
                frozen["per_class"][class_id][field]
            ):
                raise ValueError(
                    f"current metric contract does not reproduce class {class_id} {field}"
                )
        for field in ("precision", "recall", "f1"):
            if abs(
                float(reproduced["per_class"][class_id][field])
                - float(frozen["per_class"][class_id][field])
            ) > 1e-12:
                raise ValueError(
                    f"current metric contract does not reproduce class {class_id} {field}"
                )


def evaluate(
    derivative_manifest_path: Path,
    prediction_bundle_path: Path,
    threshold_path: Path,
    experiments_path: Path,
    output_path: Path,
) -> dict[str, Any]:
    derivative_manifest_path = _resolve(derivative_manifest_path).resolve()
    prediction_bundle_path = _resolve(prediction_bundle_path).resolve()
    threshold_path = _resolve(threshold_path).resolve()
    experiments_path = _resolve(experiments_path).resolve()
    output_path = _resolve(output_path).resolve()
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite audited metric report: {output_path}")

    derivative_verification = verify_audited_derivative(derivative_manifest_path)
    derivative = _load_object(derivative_manifest_path)
    threshold = _load_object(threshold_path)
    experiments = _load_object(experiments_path)
    bundle, source_records = load_prediction_bundle(
        prediction_bundle_path, expected_split="val"
    )

    bindings = threshold.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("threshold document has no bindings")
    if bindings.get("validation_bundle_sha256") != _sha256(prediction_bundle_path):
        raise ValueError("prediction bundle differs from the frozen threshold binding")
    historical_experiments_sha256 = str(bindings.get("experiments_sha256", ""))
    current_experiments_sha256 = _sha256(experiments_path)
    if threshold.get("kind") != "frozen_threshold":
        raise ValueError("unsupported threshold document")
    class_ids = tuple(int(value) for value in threshold.get("class_ids", []))
    if class_ids != (0, 1):
        raise ValueError("threshold class contract must be exactly 0,1")
    iou_threshold = float(threshold["iou_threshold"])
    original_threshold = float(threshold["confidence_threshold"])

    metric_policy = experiments.get("metric", {})
    if metric_policy.get("name") != threshold.get("metric_name"):
        raise ValueError("current metric name differs from the frozen threshold")
    if abs(float(metric_policy.get("iou_threshold")) - iou_threshold) > 1e-12:
        raise ValueError("current IoU threshold differs from the frozen threshold")
    if tuple(int(value) for value in metric_policy.get("class_ids", [])) != class_ids:
        raise ValueError("current metric classes differ from the frozen threshold")
    grid = metric_policy.get("threshold_grid", {})
    candidates = threshold_grid(
        float(grid["start"]), float(grid["stop"]), float(grid["step"])
    )
    frozen_selection = threshold.get("selection", {})
    if len(candidates) != int(frozen_selection.get("candidate_count", -1)):
        raise ValueError("current threshold grid size differs from the frozen selection")

    entries = derivative["entries"]
    entry_by_image = {str(item["image"]): item for item in entries}
    record_by_image = {record.image: record for record in source_records}
    if set(entry_by_image) != set(record_by_image):
        missing = sorted(set(entry_by_image) - set(record_by_image))
        extra = sorted(set(record_by_image) - set(entry_by_image))
        raise ValueError(f"prediction/derivative membership mismatch; missing={missing}, extra={extra}")

    reproduced_original_selection = select_validation_threshold(
        source_records, candidates, iou_threshold, class_ids
    )
    if abs(
        float(reproduced_original_selection["selected_threshold"])
        - float(frozen_selection.get("selected_threshold"))
    ) > 1e-12:
        raise ValueError("current threshold grid does not reproduce the frozen selection")

    derivative_root = derivative_manifest_path.parent
    audited_records = []
    for image in sorted(entry_by_image):
        source = record_by_image[image]
        label_path = derivative_root / str(entry_by_image[image]["audited_label"])
        audited_records.append(
            ImageDetections(
                image=image,
                ground_truth=load_yolo_ground_truth(label_path, class_ids),
                predictions=source.predictions,
                inference_ms=source.inference_ms,
                total_ms=source.total_ms,
            )
        )

    original_metrics = frozen_selection.get("validation_metrics")
    if not isinstance(original_metrics, dict):
        raise ValueError("threshold document has no original validation metrics")
    _assert_metric_reproduction(
        reproduced_original_selection["validation_metrics"], original_metrics
    )
    audited_at_original = evaluate_detection_metrics(
        audited_records, original_threshold, iou_threshold, class_ids
    )
    audited_selection = select_validation_threshold(
        audited_records, candidates, iou_threshold, class_ids
    )
    selected_metrics = audited_selection["validation_metrics"]
    gate = float(metric_policy["macro_f1_min"])
    gate_passed = float(selected_metrics["macro_f1"]) >= gate

    report = {
        "schema_version": 1,
        "kind": "audited_validation_label_only_diagnostic",
        "scope": "same frozen predictions; only human-approved validation ground truth changed",
        "bindings": {
            "audited_derivative_manifest": {
                "path": str(derivative_manifest_path),
                "sha256": _sha256(derivative_manifest_path),
                "canonical_validation_derivative_sha256": derivative_verification[
                    "canonical_validation_derivative_sha256"
                ],
            },
            "prediction_bundle": {
                "path": str(prediction_bundle_path),
                "sha256": _sha256(prediction_bundle_path),
            },
            "original_frozen_threshold": {
                "path": str(threshold_path),
                "sha256": _sha256(threshold_path),
            },
            "experiments": {
                "path": str(experiments_path),
                "sha256": current_experiments_sha256,
                "historical_frozen_sha256": historical_experiments_sha256,
                "whole_file_hash_match": (
                    current_experiments_sha256 == historical_experiments_sha256
                ),
                "metric_contract_reproduces_frozen_selection": True,
            },
        },
        "image_count": len(audited_records),
        "added_ground_truth_instances": derivative["added_instance_counts"],
        "original_validation_metrics": original_metrics,
        "audited_metrics_at_original_threshold": audited_at_original,
        "delta_at_original_threshold": _metric_delta(audited_at_original, original_metrics),
        "audited_threshold_selection": audited_selection,
        "delta_at_audited_threshold_vs_original_baseline": _metric_delta(
            selected_metrics, original_metrics
        ),
        "validation_gate": {
            "metric": "macro_f1",
            "required": gate,
            "actual": float(selected_metrics["macro_f1"]),
            "passed": gate_passed,
        },
        "formal_test_authorized": False,
        "training_authorized": False,
        "test_split_accessed": False,
        "limitations": [
            "The audit was model-screened, so this is not an independent test result.",
            "Eight reviewer disagreements remain conservatively unmodified and can keep this metric as a lower bound.",
            "The audited derivative is not yet promoted into the frozen formal T30 contract.",
        ],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--derivative-manifest", type=Path, default=DEFAULT_DERIVATIVE)
    parser.add_argument("--prediction-bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--threshold", type=Path, default=DEFAULT_THRESHOLD)
    parser.add_argument("--experiments", type=Path, default=DEFAULT_EXPERIMENTS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    result = evaluate(
        arguments.derivative_manifest,
        arguments.prediction_bundle,
        arguments.threshold,
        arguments.experiments,
        arguments.output,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
