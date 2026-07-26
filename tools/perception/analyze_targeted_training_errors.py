#!/usr/bin/env python3
"""Attribute errors for ADR-0009's selected audited-validation checkpoint."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_audited_checkpoint_errors import (  # noqa: E402
    _bbox,
    _greedy_unmatched,
    _iou,
    classify_false_negative,
)
from materialize_unripe_exposure_training import _load_object, _resolve, _sha256  # noqa: E402


DEFAULT_SUMMARY = Path(
    "artifacts/perception/optimization/"
    "yolo11s_640_unripe_x2_opt2_audited_validation_v1/summary.json"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/optimization/"
    "yolo11s_640_unripe_x2_opt2_audited_validation_v1/error_analysis.json"
)


def analyze(summary_path: Path, output_path: Path) -> dict[str, Any]:
    summary_path = _resolve(summary_path).resolve()
    output_path = _resolve(output_path).resolve()
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite targeted error analysis: {output_path}")
    summary = _load_object(summary_path)
    if summary.get("kind") != "targeted_training_audited_validation_decision":
        raise ValueError("unexpected targeted validation summary kind")
    if summary.get("test_split_accessed") is not False:
        raise ValueError("targeted validation did not preserve the test seal")
    selected = summary["selected_checkpoint"]
    artifact_path = summary_path.parent / str(selected["artifact"])
    if _sha256(artifact_path) != selected["artifact_sha256"]:
        raise ValueError("selected targeted checkpoint artifact changed")
    artifact = _load_object(artifact_path)
    records = artifact.get("records")
    if not isinstance(records, list) or len(records) != 116:
        raise ValueError("targeted checkpoint artifact must contain 116 records")
    threshold = float(selected["selected_threshold"])

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
                truth = truths[truth_index]
                category = classify_false_negative(truth, predictions, threshold)
                fn_counts[str(class_id)][category] += 1
                fn_events.append(
                    {
                        "image": record["image"],
                        "class_id": class_id,
                        "category": category,
                        "bbox_xyxy_normalized": truth["bbox_xyxy_normalized"],
                    }
                )
            for prediction_index in unmatched_predictions:
                prediction_box = _bbox(predictions[prediction_index])
                other_iou = max(
                    (_iou(prediction_box, _bbox(truth)) for truth in truths if int(truth["class_id"]) != class_id),
                    default=0.0,
                )
                same_iou = max(
                    (_iou(prediction_box, _bbox(truth)) for truth in truths if int(truth["class_id"]) == class_id),
                    default=0.0,
                )
                category = "WRONG_CLASS" if other_iou >= 0.5 else "LOCALIZATION" if same_iou >= 0.1 else "BACKGROUND_OR_UNRESOLVED_LABEL"
                fp_counts[str(class_id)][category] += 1

    official = selected["validation_metrics"]["per_class"]
    for class_id in ("0", "1"):
        if sum(fn_counts[class_id].values()) != int(official[class_id]["fn"]):
            raise ValueError(f"targeted false-negative attribution mismatch for class {class_id}")
        if sum(fp_counts[class_id].values()) != int(official[class_id]["fp"]):
            raise ValueError(f"targeted false-positive attribution mismatch for class {class_id}")
    result = {
        "schema_version": 1,
        "kind": "targeted_training_error_attribution",
        "variant": summary["variant"],
        "selected_checkpoint": selected["checkpoint_name"],
        "confidence_threshold": threshold,
        "bindings": {
            "summary": {"path": str(summary_path), "size_bytes": summary_path.stat().st_size, "sha256": _sha256(summary_path)},
            "selected_artifact": {"path": str(artifact_path), "size_bytes": artifact_path.stat().st_size, "sha256": _sha256(artifact_path)},
            "analyzer": {"path": str(Path(__file__).resolve()), "size_bytes": Path(__file__).stat().st_size, "sha256": _sha256(Path(__file__))},
        },
        "false_negative_attribution": {key: dict(sorted(value.items())) for key, value in fn_counts.items()},
        "false_positive_attribution": {key: dict(sorted(value.items())) for key, value in fp_counts.items()},
        "false_negative_events": fn_events,
        "test_split_accessed": False,
        "formal_test_authorized": False,
    }
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    result = analyze(arguments.summary, arguments.output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
