#!/usr/bin/env python3
"""Reproducible error taxonomy for a frozen validation threshold.

This tool is deliberately validation-only.  It refuses prediction bundles whose
``split`` is not exactly ``val`` and never traverses train/test image or label
directories.  The core analysis uses only the Python standard library; Pillow is
needed only when ``--render-top`` is requested.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUNDLE = Path("artifacts/perception/yolo11s_validation_predictions.json")
DEFAULT_THRESHOLD = Path("artifacts/perception/yolo11s_frozen_threshold.json")
DEFAULT_OUTPUT_DIR = Path(
    "artifacts/perception/diagnostics/yolo11s_val_threshold_031"
)
CLASS_NAMES = {0: "ripe", 1: "unripe"}


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def _bbox(item: dict[str, Any]) -> tuple[float, float, float, float]:
    values = item.get("bbox_xyxy_normalized")
    if not isinstance(values, list) or len(values) != 4:
        raise ValueError("bbox_xyxy_normalized must contain four values")
    result = tuple(float(value) for value in values)
    x1, y1, x2, y2 = result
    if not all(math.isfinite(value) for value in result):
        raise ValueError("bbox values must be finite")
    if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
        raise ValueError(f"invalid normalized bbox: {result}")
    return result


def _iou(
    left: Sequence[float], right: Sequence[float]
) -> float:
    x1 = max(left[0], right[0])
    y1 = max(left[1], right[1])
    x2 = min(left[2], right[2])
    y2 = min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (left[2] - left[0]) * (left[3] - left[1])
    right_area = (right[2] - right[0]) * (right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union else 0.0


def _linear_percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _distribution(values: Iterable[float]) -> dict[str, float | int | None]:
    materialized = list(values)
    if not materialized:
        return {
            "count": 0,
            "min": None,
            "p10": None,
            "p25": None,
            "median": None,
            "p75": None,
            "p90": None,
            "p95": None,
            "max": None,
            "mean": None,
        }
    return {
        "count": len(materialized),
        "min": min(materialized),
        "p10": _linear_percentile(materialized, 0.10),
        "p25": _linear_percentile(materialized, 0.25),
        "median": statistics.median(materialized),
        "p75": _linear_percentile(materialized, 0.75),
        "p90": _linear_percentile(materialized, 0.90),
        "p95": _linear_percentile(materialized, 0.95),
        "max": max(materialized),
        "mean": statistics.fmean(materialized),
    }


def _xywh_to_xyxy(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError("YOLO label needs x_center, y_center, width, height")
    x, y, width, height = values
    return (
        max(0.0, x - width / 2.0),
        max(0.0, y - height / 2.0),
        min(1.0, x + width / 2.0),
        min(1.0, y + height / 2.0),
    )


def _load_label(path: Path) -> list[tuple[int, tuple[float, float, float, float]]]:
    result = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        stripped = line.strip()
        if not stripped:
            continue
        fields = stripped.split()
        if len(fields) != 5:
            raise ValueError(f"invalid YOLO label at {path}:{line_number}")
        class_id = int(fields[0])
        if class_id not in CLASS_NAMES:
            raise ValueError(f"unexpected class {class_id} at {path}:{line_number}")
        result.append((class_id, _xywh_to_xyxy(tuple(map(float, fields[1:])))))
    return result


def _same_boxes(
    left: Sequence[tuple[int, Sequence[float]]],
    right: Sequence[tuple[int, Sequence[float]]],
    tolerance: float = 1e-10,
) -> bool:
    if len(left) != len(right):
        return False
    for (left_class, left_box), (right_class, right_box) in zip(left, right):
        if left_class != right_class:
            return False
        if any(abs(a - b) > tolerance for a, b in zip(left_box, right_box)):
            return False
    return True


def _best_overlap(
    box: Sequence[float],
    candidates: Iterable[tuple[int, Sequence[float]]],
) -> tuple[float, int | None]:
    scored = [(_iou(box, candidate_box), index) for index, candidate_box in candidates]
    return max(scored, default=(0.0, None))


def _event_row(
    *,
    image: str,
    event_type: str,
    reason: str,
    class_id: int,
    confidence: float | None,
    bbox: Sequence[float],
    related_class_id: int | None,
    related_iou: float,
    cross_class_incidence: bool,
) -> dict[str, Any]:
    return {
        "image": image,
        "event_type": event_type,
        "reason": reason,
        "class_id": class_id,
        "class_name": CLASS_NAMES[class_id],
        "confidence": confidence,
        "bbox_xyxy_normalized": list(bbox),
        "related_class_id": related_class_id,
        "related_class_name": (
            CLASS_NAMES[related_class_id] if related_class_id is not None else None
        ),
        "related_iou": related_iou,
        "cross_class_incidence": cross_class_incidence,
    }


def _analyze_record(
    record: dict[str, Any],
    confidence_threshold: float,
    iou_threshold: float,
    localization_iou_floor: float,
) -> dict[str, Any]:
    image = str(record["image"])
    truths = [
        {"class_id": int(item["class_id"]), "bbox": _bbox(item)}
        for item in record["ground_truth"]
    ]
    predictions = [
        {
            "class_id": int(item["class_id"]),
            "confidence": float(item["confidence"]),
            "bbox": _bbox(item),
            "index": index,
        }
        for index, item in enumerate(record["predictions"])
    ]
    active = [item for item in predictions if item["confidence"] >= confidence_threshold]

    matched_prediction_to_truth: dict[int, tuple[int, float]] = {}
    matched_truth_to_prediction: dict[int, tuple[int, float]] = {}
    for class_id in sorted(CLASS_NAMES):
        truth_indices = [
            index for index, item in enumerate(truths) if item["class_id"] == class_id
        ]
        unmatched = set(truth_indices)
        class_predictions = sorted(
            (item for item in active if item["class_id"] == class_id),
            key=lambda item: (
                -item["confidence"],
                item["class_id"],
                item["bbox"],
            ),
        )
        for prediction in class_predictions:
            candidates = [
                (_iou(prediction["bbox"], truths[index]["bbox"]), index)
                for index in unmatched
            ]
            best_iou, best_index = max(candidates, default=(0.0, -1))
            if best_iou >= iou_threshold:
                matched_prediction_to_truth[prediction["index"]] = (
                    best_index,
                    best_iou,
                )
                matched_truth_to_prediction[best_index] = (
                    prediction["index"],
                    best_iou,
                )
                unmatched.remove(best_index)

    events: list[dict[str, Any]] = []
    prediction_status: dict[int, str] = {}
    for prediction in active:
        prediction_index = prediction["index"]
        if prediction_index in matched_prediction_to_truth:
            truth_index, match_iou = matched_prediction_to_truth[prediction_index]
            events.append(
                _event_row(
                    image=image,
                    event_type="tp",
                    reason="same_class_match",
                    class_id=prediction["class_id"],
                    confidence=prediction["confidence"],
                    bbox=prediction["bbox"],
                    related_class_id=truths[truth_index]["class_id"],
                    related_iou=match_iou,
                    cross_class_incidence=False,
                )
            )
            prediction_status[prediction_index] = "tp"
            continue

        same_class = [
            (index, item["bbox"])
            for index, item in enumerate(truths)
            if item["class_id"] == prediction["class_id"]
        ]
        other_class = [
            (index, item["bbox"])
            for index, item in enumerate(truths)
            if item["class_id"] != prediction["class_id"]
        ]
        all_truth = [(index, item["bbox"]) for index, item in enumerate(truths)]
        best_same_iou, best_same_index = _best_overlap(prediction["bbox"], same_class)
        best_cross_iou, best_cross_index = _best_overlap(prediction["bbox"], other_class)
        best_any_iou, best_any_index = _best_overlap(prediction["bbox"], all_truth)
        duplicate = (
            best_same_index is not None
            and best_same_iou >= iou_threshold
            and best_same_index in matched_truth_to_prediction
        )
        cross_class = best_cross_index is not None and best_cross_iou >= iou_threshold

        if duplicate:
            reason = "duplicate_same_class"
            related_index = best_same_index
            related_iou = best_same_iou
        elif cross_class:
            reason = "cross_class_confusion"
            related_index = best_cross_index
            related_iou = best_cross_iou
        elif best_any_iou >= localization_iou_floor:
            reason = "localization_or_partial_overlap"
            related_index = best_any_index
            related_iou = best_any_iou
        else:
            reason = "background_or_missing_label_candidate"
            related_index = best_any_index
            related_iou = best_any_iou
        events.append(
            _event_row(
                image=image,
                event_type="fp",
                reason=reason,
                class_id=prediction["class_id"],
                confidence=prediction["confidence"],
                bbox=prediction["bbox"],
                related_class_id=(
                    truths[related_index]["class_id"]
                    if related_index is not None
                    else None
                ),
                related_iou=related_iou,
                cross_class_incidence=cross_class,
            )
        )
        prediction_status[prediction_index] = f"fp:{reason}"

    for truth_index, truth in enumerate(truths):
        if truth_index in matched_truth_to_prediction:
            continue
        same_active = [
            (item["index"], item["bbox"])
            for item in active
            if item["class_id"] == truth["class_id"]
        ]
        cross_active = [
            (item["index"], item["bbox"])
            for item in active
            if item["class_id"] != truth["class_id"]
        ]
        same_below = [
            (item["index"], item["bbox"])
            for item in predictions
            if item["class_id"] == truth["class_id"]
            and item["confidence"] < confidence_threshold
        ]
        best_same_iou, best_same_prediction = _best_overlap(truth["bbox"], same_active)
        best_cross_iou, best_cross_prediction = _best_overlap(truth["bbox"], cross_active)
        best_below_iou, best_below_prediction = _best_overlap(truth["bbox"], same_below)
        assignment_conflict = (
            best_same_prediction is not None
            and best_same_iou >= iou_threshold
            and best_same_prediction in matched_prediction_to_truth
        )
        cross_class = best_cross_prediction is not None and best_cross_iou >= iou_threshold
        below_threshold = (
            best_below_prediction is not None and best_below_iou >= iou_threshold
        )

        related_prediction: dict[str, Any] | None = None
        if assignment_conflict:
            reason = "same_class_assignment_conflict"
            related_prediction = predictions[best_same_prediction]
            related_iou = best_same_iou
        elif cross_class:
            reason = "cross_class_confusion"
            related_prediction = predictions[best_cross_prediction]
            related_iou = best_cross_iou
        elif below_threshold:
            reason = "correct_class_below_threshold"
            related_prediction = predictions[best_below_prediction]
            related_iou = best_below_iou
        elif best_same_iou >= localization_iou_floor:
            reason = "localization_miss"
            related_prediction = predictions[best_same_prediction]
            related_iou = best_same_iou
        else:
            reason = "pure_detection_miss"
            related_iou = max(best_same_iou, best_cross_iou, best_below_iou)
        events.append(
            _event_row(
                image=image,
                event_type="fn",
                reason=reason,
                class_id=truth["class_id"],
                confidence=(
                    related_prediction["confidence"]
                    if related_prediction is not None
                    else None
                ),
                bbox=truth["bbox"],
                related_class_id=(
                    related_prediction["class_id"]
                    if related_prediction is not None
                    else None
                ),
                related_iou=related_iou,
                cross_class_incidence=cross_class,
            )
        )

    return {
        "image": image,
        "truths": truths,
        "predictions": predictions,
        "events": events,
        "prediction_status": prediction_status,
    }


def _counter_dict(counter: Counter[Any]) -> dict[str, int]:
    return {str(key): value for key, value in sorted(counter.items(), key=lambda x: str(x[0]))}


def _write_csv(path: Path, rows: Sequence[dict[str, Any]], fields: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            cooked = dict(row)
            if isinstance(cooked.get("bbox_xyxy_normalized"), list):
                cooked["bbox_xyxy_normalized"] = json.dumps(
                    cooked["bbox_xyxy_normalized"], separators=(",", ":")
                )
            writer.writerow({field: cooked.get(field) for field in fields})


def _render_overlays(
    analyses: Sequence[dict[str, Any]],
    dataset_root: Path,
    output_dir: Path,
) -> None:
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:  # pragma: no cover - optional diagnostic feature
        raise RuntimeError("Pillow is required only for --render-top") from exc

    output_dir.mkdir(parents=True, exist_ok=True)
    reason_colors = {
        "duplicate_same_class": "magenta",
        "cross_class_confusion": "orange",
        "localization_or_partial_overlap": "yellow",
        "background_or_missing_label_candidate": "red",
    }
    font = ImageFont.load_default()
    for analysis in analyses:
        source = dataset_root / analysis["image"]
        with Image.open(source) as opened:
            image = opened.convert("RGB")
        max_dimension = 1800
        if max(image.size) > max_dimension:
            scale = max_dimension / max(image.size)
            image = image.resize(
                (round(image.width * scale), round(image.height * scale)),
                Image.Resampling.LANCZOS,
            )
        draw = ImageDraw.Draw(image)
        width, height = image.size
        line_width = max(2, round(max(width, height) / 500))

        for truth in analysis["truths"]:
            x1, y1, x2, y2 = truth["bbox"]
            box = (x1 * width, y1 * height, x2 * width, y2 * height)
            draw.rectangle(box, outline="lime", width=line_width)
            draw.text(
                (box[0] + 2, box[1] + 2),
                f"GT {CLASS_NAMES[truth['class_id']]}",
                fill="lime",
                font=font,
                stroke_width=2,
                stroke_fill="black",
            )
        for prediction in analysis["predictions"]:
            status = analysis["prediction_status"].get(prediction["index"])
            if status is None:
                continue
            reason = status.split(":", 1)[1] if status.startswith("fp:") else None
            color = "deepskyblue" if status == "tp" else reason_colors[reason]
            x1, y1, x2, y2 = prediction["bbox"]
            box = (x1 * width, y1 * height, x2 * width, y2 * height)
            draw.rectangle(box, outline=color, width=line_width)
            draw.text(
                (box[0] + 2, max(0, box[1] - 14)),
                f"P {CLASS_NAMES[prediction['class_id']]} {prediction['confidence']:.2f} {status}",
                fill=color,
                font=font,
                stroke_width=2,
                stroke_fill="black",
            )
        destination = output_dir / Path(analysis["image"]).name
        image.save(destination, format="JPEG", quality=90, optimize=True)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--threshold", type=Path, default=DEFAULT_THRESHOLD)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--localization-iou-floor", type=float, default=0.10)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--render-top", type=int, default=0)
    args = parser.parse_args(argv)

    bundle_path = _resolve(args.bundle)
    threshold_path = _resolve(args.threshold)
    output_dir = _resolve(args.output_dir)
    bundle = _load_json(bundle_path)
    threshold_document = _load_json(threshold_path)

    if bundle.get("kind") != "prediction_bundle" or bundle.get("split") != "val":
        raise ValueError("this tool accepts only a prediction_bundle with split='val'")
    if bundle.get("role") != "primary":
        raise ValueError("validation error analysis requires the primary model bundle")
    if threshold_document.get("kind") != "frozen_threshold":
        raise ValueError("threshold input is not a frozen_threshold artifact")
    if bundle.get("variant") != threshold_document.get("variant"):
        raise ValueError("bundle and threshold variants differ")
    bundle_sha256 = _sha256(bundle_path)
    bound_bundle_sha256 = threshold_document["bindings"]["validation_bundle_sha256"]
    if bundle_sha256 != bound_bundle_sha256:
        raise ValueError("validation bundle hash does not match frozen threshold binding")

    confidence_threshold = float(threshold_document["confidence_threshold"])
    iou_threshold = float(threshold_document["iou_threshold"])
    if not 0.0 < args.localization_iou_floor < iou_threshold:
        raise ValueError("localization IoU floor must lie in (0, frozen IoU threshold)")
    if args.top < 1 or args.render_top < 0:
        raise ValueError("--top must be positive and --render-top non-negative")

    dataset_yaml = _resolve(Path(threshold_document["bindings"]["dataset_yaml"]))
    dataset_root = dataset_yaml.parent
    split_manifest_path = _resolve(
        Path(threshold_document["bindings"]["split_manifest"])
    )
    if _sha256(split_manifest_path) != threshold_document["bindings"][
        "split_manifest_sha256"
    ]:
        raise ValueError("split manifest hash does not match frozen threshold binding")
    split_manifest = _load_json(split_manifest_path)
    # Intentionally select only validation entries; no test labels/images are opened.
    validation_entries = split_manifest["splits"]["val"]
    expected_images = {str(item["output_image"]) for item in validation_entries}
    records = bundle.get("records")
    if not isinstance(records, list):
        raise ValueError("prediction bundle records must be a list")
    record_images = {str(item["image"]) for item in records}
    if len(record_images) != len(records) or record_images != expected_images:
        raise ValueError("validation record image set differs from split manifest")

    label_comparisons = 0
    for record in records:
        image_key = str(record["image"])
        image_path = dataset_root / image_key
        label_path = dataset_root / image_key.replace("images/val/", "labels/val/")
        label_path = label_path.with_suffix(".txt")
        if not image_path.is_file() or not label_path.is_file():
            raise FileNotFoundError(f"missing validation sample files for {image_key}")
        label_truth = _load_label(label_path)
        bundle_truth = [
            (int(item["class_id"]), _bbox(item)) for item in record["ground_truth"]
        ]
        if not _same_boxes(label_truth, bundle_truth):
            raise ValueError(f"bundle ground truth differs from label: {image_key}")
        label_comparisons += 1

    analyses = [
        _analyze_record(
            record,
            confidence_threshold,
            iou_threshold,
            args.localization_iou_floor,
        )
        for record in records
    ]
    events = [event for item in analyses for event in item["events"]]
    metric_events = [event for event in events if event["event_type"] in {"tp", "fp", "fn"}]

    metric_counts: dict[str, dict[str, Any]] = {}
    for class_id in CLASS_NAMES:
        class_events = [event for event in metric_events if event["class_id"] == class_id]
        tp = sum(event["event_type"] == "tp" for event in class_events)
        fp = sum(event["event_type"] == "fp" for event in class_events)
        fn = sum(event["event_type"] == "fn" for event in class_events)
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        f1 = 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else 0.0
        metric_counts[str(class_id)] = {
            "class_name": CLASS_NAMES[class_id],
            "support": tp + fn,
            "predictions": tp + fp,
            "tp": tp,
            "fp": fp,
            "fn": fn,
            "precision": precision,
            "recall": recall,
            "f1": f1,
        }
    frozen_counts = threshold_document["selection"]["validation_metrics"]["per_class"]
    for class_id in CLASS_NAMES:
        for field in ("support", "predictions", "tp", "fp", "fn"):
            if metric_counts[str(class_id)][field] != frozen_counts[str(class_id)][field]:
                raise AssertionError(
                    f"recomputed class {class_id} {field} differs from frozen artifact"
                )

    fp_events = [event for event in events if event["event_type"] == "fp"]
    fn_events = [event for event in events if event["event_type"] == "fn"]
    tp_events = [event for event in events if event["event_type"] == "tp"]
    fp_reason_counts = Counter(event["reason"] for event in fp_events)
    fn_reason_counts = Counter(event["reason"] for event in fn_events)
    fp_reasons_by_class = {
        str(class_id): _counter_dict(
            Counter(
                event["reason"]
                for event in fp_events
                if event["class_id"] == class_id
            )
        )
        for class_id in CLASS_NAMES
    }
    fn_reasons_by_class = {
        str(class_id): _counter_dict(
            Counter(
                event["reason"]
                for event in fn_events
                if event["class_id"] == class_id
            )
        )
        for class_id in CLASS_NAMES
    }
    fp_cross = [event for event in fp_events if event["cross_class_incidence"]]
    fn_cross = [event for event in fn_events if event["cross_class_incidence"]]
    cross_matrix = Counter(
        f"gt_{event['related_class_id']}_as_pred_{event['class_id']}"
        for event in fp_cross
    )

    confidence_distributions: dict[str, Any] = {
        "all_raw_predictions": {},
        "above_threshold_predictions": {},
        "true_positives": {},
        "false_positives": {},
        "false_positives_by_reason": {},
        "false_negatives_related_prediction_by_reason": {},
    }
    for class_id in CLASS_NAMES:
        raw = [
            item["confidence"]
            for analysis in analyses
            for item in analysis["predictions"]
            if item["class_id"] == class_id
        ]
        active = [value for value in raw if value >= confidence_threshold]
        confidence_distributions["all_raw_predictions"][str(class_id)] = _distribution(raw)
        confidence_distributions["above_threshold_predictions"][str(class_id)] = _distribution(active)
        confidence_distributions["true_positives"][str(class_id)] = _distribution(
            event["confidence"] for event in tp_events if event["class_id"] == class_id
        )
        confidence_distributions["false_positives"][str(class_id)] = _distribution(
            event["confidence"] for event in fp_events if event["class_id"] == class_id
        )
    for reason in sorted(fp_reason_counts):
        confidence_distributions["false_positives_by_reason"][reason] = _distribution(
            event["confidence"] for event in fp_events if event["reason"] == reason
        )
    for reason in sorted(fn_reason_counts):
        confidence_distributions["false_negatives_related_prediction_by_reason"][
            reason
        ] = _distribution(
            event["confidence"]
            for event in fn_events
            if event["reason"] == reason and event["confidence"] is not None
        )

    image_rows = []
    for analysis in analyses:
        image_events = analysis["events"]
        fps = [event for event in image_events if event["event_type"] == "fp"]
        fns = [event for event in image_events if event["event_type"] == "fn"]
        tps = [event for event in image_events if event["event_type"] == "tp"]
        cross = sum(event["cross_class_incidence"] for event in fps)
        duplicates = sum(event["reason"] == "duplicate_same_class" for event in fps)
        missing_label_candidates = sum(
            event["reason"] == "background_or_missing_label_candidate"
            for event in fps
        )
        high_confidence_missing_label_candidates = sum(
            event["reason"] == "background_or_missing_label_candidate"
            and event["confidence"] >= 0.50
            for event in fps
        )
        row = {
            "image": analysis["image"],
            "tp": len(tps),
            "fp": len(fps),
            "fn": len(fns),
            "cross_class_fp": cross,
            "duplicate_fp": duplicates,
            "background_or_missing_label_fp": missing_label_candidates,
            "high_confidence_missing_label_candidate": high_confidence_missing_label_candidates,
            "max_fp_confidence": max(
                (event["confidence"] for event in fps), default=None
            ),
            "error_score": (
                4 * cross
                + 3 * high_confidence_missing_label_candidates
                + 2 * len(fns)
                + len(fps)
                + duplicates
            ),
        }
        image_rows.append(row)
    image_rows.sort(
        key=lambda row: (
            -row["error_score"],
            -row["fn"],
            -row["fp"],
            row["image"],
        )
    )

    label_candidates = sorted(
        (
            event
            for event in fp_events
            if event["reason"] == "background_or_missing_label_candidate"
            and event["confidence"] >= 0.50
        ),
        key=lambda event: (-event["confidence"], event["image"]),
    )
    report = {
        "schema_version": 1,
        "kind": "validation_error_analysis",
        "scope": {
            "split": "val",
            "image_count": len(records),
            "label_files_compared": label_comparisons,
            "test_split_accessed": False,
        },
        "bindings": {
            "prediction_bundle": bundle_path.relative_to(REPOSITORY_ROOT).as_posix(),
            "prediction_bundle_sha256": bundle_sha256,
            "frozen_threshold": threshold_path.relative_to(REPOSITORY_ROOT).as_posix(),
            "frozen_threshold_sha256": _sha256(threshold_path),
            "split_manifest": split_manifest_path.relative_to(REPOSITORY_ROOT).as_posix(),
            "split_manifest_sha256": _sha256(split_manifest_path),
            "canonical_val_sha256": threshold_document["bindings"][
                "canonical_split_sha256"
            ]["val"],
        },
        "parameters": {
            "confidence_threshold": confidence_threshold,
            "iou_threshold": iou_threshold,
            "localization_iou_floor": args.localization_iou_floor,
            "high_confidence_label_candidate_floor": 0.50,
        },
        "metric_reproduction": {
            "per_class": metric_counts,
            "macro_f1": statistics.fmean(
                item["f1"] for item in metric_counts.values()
            ),
            "matches_frozen_artifact": True,
        },
        "false_positive_taxonomy": {
            "exclusive_reason_counts": _counter_dict(fp_reason_counts),
            "exclusive_reason_counts_by_class": fp_reasons_by_class,
            "cross_class_incidence_count": len(fp_cross),
            "cross_class_matrix": _counter_dict(cross_matrix),
            "note": (
                "Exclusive reasons prioritize duplicate, then cross-class, then "
                "partial localization, then background/missing-label candidate. "
                "Cross-class incidence is also counted independently."
            ),
        },
        "false_negative_taxonomy": {
            "exclusive_reason_counts": _counter_dict(fn_reason_counts),
            "exclusive_reason_counts_by_class": fn_reasons_by_class,
            "cross_class_incidence_count": len(fn_cross),
            "note": (
                "Exclusive reasons prioritize assignment conflict, cross-class, "
                "correct-class below threshold, localization miss, then pure miss."
            ),
        },
        "confidence_distributions": confidence_distributions,
        "label_incompleteness_screen": {
            "method": (
                "FP box has IoU < localization floor against every annotation "
                "and confidence >= 0.50; this flags candidates, not confirmed omissions."
            ),
            "candidate_count": len(label_candidates),
            "candidate_image_count": len({event["image"] for event in label_candidates}),
            "candidates": label_candidates,
        },
        "top_error_images": image_rows[: args.top],
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "summary.json"
    events_path = output_dir / "events.csv"
    images_path = output_dir / "images.csv"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_csv(
        events_path,
        events,
        (
            "image",
            "event_type",
            "reason",
            "class_id",
            "class_name",
            "confidence",
            "bbox_xyxy_normalized",
            "related_class_id",
            "related_class_name",
            "related_iou",
            "cross_class_incidence",
        ),
    )
    _write_csv(
        images_path,
        image_rows,
        (
            "image",
            "tp",
            "fp",
            "fn",
            "cross_class_fp",
            "duplicate_fp",
            "background_or_missing_label_fp",
            "high_confidence_missing_label_candidate",
            "max_fp_confidence",
            "error_score",
        ),
    )

    if args.render_top:
        by_image = {analysis["image"]: analysis for analysis in analyses}
        render_images = [
            by_image[row["image"]] for row in image_rows[: args.render_top]
        ]
        # Always include high-confidence missing-label candidates in the visual audit.
        for image_key in sorted({event["image"] for event in label_candidates}):
            if by_image[image_key] not in render_images:
                render_images.append(by_image[image_key])
        _render_overlays(render_images, dataset_root, output_dir / "overlays")

    print(
        json.dumps(
            {
                "summary": report_path.relative_to(REPOSITORY_ROOT).as_posix(),
                "events": events_path.relative_to(REPOSITORY_ROOT).as_posix(),
                "images": images_path.relative_to(REPOSITORY_ROOT).as_posix(),
                "per_class": metric_counts,
                "fp_reasons": _counter_dict(fp_reason_counts),
                "fn_reasons": _counter_dict(fn_reason_counts),
                "label_candidates": len(label_candidates),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
