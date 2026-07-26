#!/usr/bin/env python3
"""Attribute audited-validation errors for the selected checkpoint."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SWEEP = Path(
    "artifacts/perception/label_audit/yolo11s_audited_checkpoint_sweep_v1/summary.json"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/label_audit/yolo11s_audited_checkpoint_error_analysis_v1.json"
)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _bbox(item: dict[str, Any]) -> tuple[float, float, float, float]:
    value = item.get("bbox_xyxy_normalized")
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("detection box must contain four coordinates")
    box = tuple(float(coordinate) for coordinate in value)
    if not all(math.isfinite(coordinate) for coordinate in box):
        raise ValueError("detection box must be finite")
    if not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1):
        raise ValueError("detection box must have normalized positive area")
    return box


def _iou(left: Sequence[float], right: Sequence[float]) -> float:
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (left[2] - left[0]) * (left[3] - left[1])
    right_area = (right[2] - right[0]) * (right[3] - right[1])
    return intersection / (left_area + right_area - intersection)


def classify_false_negative(
    truth: dict[str, Any], predictions: Sequence[dict[str, Any]], threshold: float
) -> str:
    truth_class = int(truth["class_id"])
    truth_box = _bbox(truth)
    wrong_class_iou = max(
        (
            _iou(truth_box, _bbox(item))
            for item in predictions
            if int(item["class_id"]) != truth_class
            and float(item["confidence"]) >= threshold
        ),
        default=0.0,
    )
    if wrong_class_iou >= 0.5:
        return "WRONG_CLASS"
    low_confidence_iou = max(
        (
            _iou(truth_box, _bbox(item))
            for item in predictions
            if int(item["class_id"]) == truth_class
            and float(item["confidence"]) < threshold
        ),
        default=0.0,
    )
    if low_confidence_iou >= 0.5:
        return "LOW_CONFIDENCE"
    localization_iou = max(
        (
            _iou(truth_box, _bbox(item))
            for item in predictions
            if int(item["class_id"]) == truth_class
            and float(item["confidence"]) >= threshold
        ),
        default=0.0,
    )
    if localization_iou >= 0.1:
        return "LOCALIZATION"
    return "NO_DETECTION"


def _greedy_unmatched(
    truths: Sequence[dict[str, Any]],
    predictions: Sequence[dict[str, Any]],
    class_id: int,
    threshold: float,
) -> tuple[list[int], list[int]]:
    truth_indices = [index for index, item in enumerate(truths) if int(item["class_id"]) == class_id]
    prediction_indices = sorted(
        (
            index
            for index, item in enumerate(predictions)
            if int(item["class_id"]) == class_id and float(item["confidence"]) >= threshold
        ),
        key=lambda index: -float(predictions[index]["confidence"]),
    )
    unmatched_truths = set(truth_indices)
    unmatched_predictions = []
    for prediction_index in prediction_indices:
        prediction_box = _bbox(predictions[prediction_index])
        options = [
            (_iou(prediction_box, _bbox(truths[truth_index])), truth_index)
            for truth_index in unmatched_truths
        ]
        best_iou, best_truth = max(options, default=(0.0, -1))
        if best_iou >= 0.5:
            unmatched_truths.remove(best_truth)
        else:
            unmatched_predictions.append(prediction_index)
    return sorted(unmatched_truths), unmatched_predictions


def analyze(sweep_summary_path: Path, output_path: Path) -> dict[str, Any]:
    sweep_summary_path = _resolve(sweep_summary_path).resolve()
    output_path = _resolve(output_path).resolve()
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite error analysis: {output_path}")
    sweep = _load(sweep_summary_path)
    if sweep.get("kind") != "audited_checkpoint_sweep_summary":
        raise ValueError("unexpected checkpoint sweep summary")
    selected = sweep["selected_candidate"]
    artifact_path = sweep_summary_path.parent / str(selected["artifact"])
    if _sha256(artifact_path) != selected["artifact_sha256"]:
        raise ValueError("selected checkpoint artifact hash mismatch")
    artifact = _load(artifact_path)
    threshold = float(selected["selected_threshold"])
    records = artifact.get("records")
    if not isinstance(records, list) or len(records) != 116:
        raise ValueError("selected checkpoint artifact must contain 116 records")

    fn_counts = {"0": Counter(), "1": Counter()}
    fp_counts = {"0": Counter(), "1": Counter()}
    fn_events = []
    for record in records:
        truths = record["ground_truth"]
        predictions = record["predictions"]
        for class_id in (0, 1):
            unmatched_truths, unmatched_predictions = _greedy_unmatched(
                truths, predictions, class_id, threshold
            )
            for truth_index in unmatched_truths:
                category = classify_false_negative(
                    truths[truth_index], predictions, threshold
                )
                fn_counts[str(class_id)][category] += 1
                fn_events.append(
                    {
                        "image": record["image"],
                        "class_id": class_id,
                        "category": category,
                        "bbox_xyxy_normalized": truths[truth_index][
                            "bbox_xyxy_normalized"
                        ],
                    }
                )
            for prediction_index in unmatched_predictions:
                prediction = predictions[prediction_index]
                prediction_box = _bbox(prediction)
                other_iou = max(
                    (
                        _iou(prediction_box, _bbox(truth))
                        for truth in truths
                        if int(truth["class_id"]) != class_id
                    ),
                    default=0.0,
                )
                same_iou = max(
                    (
                        _iou(prediction_box, _bbox(truth))
                        for truth in truths
                        if int(truth["class_id"]) == class_id
                    ),
                    default=0.0,
                )
                category = (
                    "WRONG_CLASS"
                    if other_iou >= 0.5
                    else "LOCALIZATION"
                    if same_iou >= 0.1
                    else "BACKGROUND_OR_UNRESOLVED_LABEL"
                )
                fp_counts[str(class_id)][category] += 1

    official = selected["validation_metrics"]["per_class"]
    for class_id in ("0", "1"):
        if sum(fn_counts[class_id].values()) != int(official[class_id]["fn"]):
            raise ValueError(f"false-negative attribution mismatch for class {class_id}")
        if sum(fp_counts[class_id].values()) != int(official[class_id]["fp"]):
            raise ValueError(f"false-positive attribution mismatch for class {class_id}")
    result = {
        "schema_version": 1,
        "kind": "audited_checkpoint_error_attribution",
        "selected_checkpoint_id": sweep["selected_checkpoint_id"],
        "confidence_threshold": threshold,
        "bindings": {
            "sweep_summary": {
                "path": str(sweep_summary_path),
                "sha256": _sha256(sweep_summary_path),
            },
            "selected_artifact": {
                "path": str(artifact_path),
                "sha256": _sha256(artifact_path),
            },
        },
        "false_negative_attribution": {
            class_id: dict(sorted(counts.items())) for class_id, counts in fn_counts.items()
        },
        "false_positive_attribution": {
            class_id: dict(sorted(counts.items())) for class_id, counts in fp_counts.items()
        },
        "false_negative_events": fn_events,
        "test_split_accessed": False,
        "training_authorized": False,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep-summary", type=Path, default=DEFAULT_SWEEP)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    print(json.dumps(analyze(arguments.sweep_summary, arguments.output), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
