"""Threshold-specific development evaluation for generalized detection."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(frozen=True)
class DetectionBox:
    image_id: str
    class_id: int
    xyxy: tuple[float, float, float, float]
    confidence: float = 1.0


def box_iou(
    left: Sequence[float], right: Sequence[float]
) -> float:
    if len(left) != 4 or len(right) != 4:
        raise ValueError("boxes must contain x1, y1, x2, y2")
    lx1, ly1, lx2, ly2 = (float(value) for value in left)
    rx1, ry1, rx2, ry2 = (float(value) for value in right)
    values = (lx1, ly1, lx2, ly2, rx1, ry1, rx2, ry2)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("box coordinates must be finite")
    if lx2 <= lx1 or ly2 <= ly1 or rx2 <= rx1 or ry2 <= ry1:
        raise ValueError("boxes must have positive area")
    intersection = max(0.0, min(lx2, rx2) - max(lx1, rx1)) * max(
        0.0, min(ly2, ry2) - max(ly1, ry1)
    )
    union = (lx2 - lx1) * (ly2 - ly1) + (rx2 - rx1) * (ry2 - ry1) - intersection
    return intersection / union if union > 0.0 else 0.0


def _metrics(tp: int, fp: int, fn: int) -> dict[str, float | int]:
    precision = tp / (tp + fp) if tp + fp else 1.0
    recall = tp / (tp + fn) if tp + fn else 1.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": precision,
        "recall": recall,
        "f1": f1,
    }


def evaluate_threshold(
    truth: Sequence[DetectionBox],
    predictions: Sequence[DetectionBox],
    *,
    threshold: float,
    iou_threshold: float = 0.5,
) -> dict[str, object]:
    if not 0.0 <= threshold <= 1.0 or not 0.0 < iou_threshold <= 1.0:
        raise ValueError("confidence and IoU thresholds are invalid")
    image_ids = sorted({row.image_id for row in truth} | {row.image_id for row in predictions})
    per_class = {}
    totals = [0, 0, 0]
    for class_id, class_name in ((0, "ripe"), (1, "unripe")):
        tp = fp = fn = 0
        for image_id in image_ids:
            expected = [
                row for row in truth if row.image_id == image_id and row.class_id == class_id
            ]
            observed = sorted(
                (
                    row
                    for row in predictions
                    if row.image_id == image_id
                    and row.class_id == class_id
                    and row.confidence >= threshold
                ),
                key=lambda row: (-row.confidence, row.xyxy),
            )
            unmatched = set(range(len(expected)))
            for prediction in observed:
                candidates = sorted(
                    (
                        (box_iou(prediction.xyxy, expected[index].xyxy), index)
                        for index in unmatched
                    ),
                    reverse=True,
                )
                if candidates and candidates[0][0] >= iou_threshold:
                    unmatched.remove(candidates[0][1])
                    tp += 1
                else:
                    fp += 1
            fn += len(unmatched)
        per_class[class_name] = _metrics(tp, fp, fn)
        totals[0] += tp
        totals[1] += fp
        totals[2] += fn
    return {
        "threshold": threshold,
        "iou_threshold": iou_threshold,
        "ripe": per_class["ripe"],
        "unripe": per_class["unripe"],
        "overall": _metrics(*totals),
    }


def select_validation_threshold(
    truth: Sequence[DetectionBox],
    predictions: Sequence[DetectionBox],
    *,
    iou_threshold: float = 0.5,
    minimum_ripe_precision: float = 0.95,
    minimum_ripe_recall: float = 0.90,
) -> dict[str, object]:
    confidence_values = sorted(
        {
            min(1.0, max(0.0, float(row.confidence)))
            for row in predictions
        }
    )
    thresholds = {0.0, 1.0}
    for value in confidence_values:
        thresholds.add(value)
        thresholds.add(math.nextafter(value, 1.0))
    rows = [
        evaluate_threshold(
            truth,
            predictions,
            threshold=value,
            iou_threshold=iou_threshold,
        )
        for value in sorted(thresholds)
    ]
    eligible = [
        row
        for row in rows
        if float(row["ripe"]["precision"]) >= minimum_ripe_precision
        and float(row["ripe"]["recall"]) >= minimum_ripe_recall
    ]
    pool = eligible or rows
    selected = max(
        pool,
        key=lambda row: (
            float(row["ripe"]["f1"]),
            float(row["overall"]["f1"]),
            float(row["threshold"]),
        ),
    )
    return {
        "gate_pass": bool(eligible),
        "requirements": {
            "minimum_ripe_precision": minimum_ripe_precision,
            "minimum_ripe_recall": minimum_ripe_recall,
            "iou_threshold": iou_threshold,
        },
        "selected": selected,
        "evaluated_threshold_count": len(rows),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _truth_rows(labels_dir: Path, images: Sequence[Path]) -> list[DetectionBox]:
    rows = []
    for image in images:
        label_path = labels_dir / f"{image.stem}.txt"
        if not label_path.is_file():
            raise ValueError(f"missing label file: {label_path}")
        for line in label_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            fields = line.split()
            if len(fields) != 5:
                raise ValueError(f"invalid label row in {label_path}")
            class_id = int(fields[0])
            center_x, center_y, width, height = (float(value) for value in fields[1:])
            rows.append(
                DetectionBox(
                    image_id=image.stem,
                    class_id=class_id,
                    xyxy=(
                        center_x - width / 2.0,
                        center_y - height / 2.0,
                        center_x + width / 2.0,
                        center_y + height / 2.0,
                    ),
                )
            )
    return rows


def _prediction_rows(
    results: Iterable[object], image_ids: Sequence[str] | None = None
) -> list[DetectionBox]:
    rows = []
    result_rows = list(results)
    if image_ids is not None and len(result_rows) != len(image_ids):
        raise ValueError("prediction result count does not match source image count")
    for index, result in enumerate(result_rows):
        height, width = result.orig_shape
        # Ultralytics renames an in-memory list source to image0.jpg, image1.jpg,
        # ... . Preserve the caller's ordered source identities so detections are
        # matched against the corresponding ground-truth file.
        image_id = image_ids[index] if image_ids is not None else Path(str(result.path)).stem
        boxes = result.boxes
        for xyxy, confidence, class_id in zip(
            boxes.xyxy.cpu().tolist(),
            boxes.conf.cpu().tolist(),
            boxes.cls.cpu().tolist(),
        ):
            rows.append(
                DetectionBox(
                    image_id=image_id,
                    class_id=int(class_id),
                    xyxy=(
                        float(xyxy[0]) / width,
                        float(xyxy[1]) / height,
                        float(xyxy[2]) / width,
                        float(xyxy[3]) / height,
                    ),
                    confidence=float(confidence),
                )
            )
    return rows


def main() -> int:  # pragma: no cover - exercised as a GPU development tool
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--split", choices=("validation", "qualification"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threshold", type=float)
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--nms-iou", type=float, default=0.7)
    parser.add_argument("--device", default="0")
    options = parser.parse_args()
    if options.split == "qualification" and options.threshold is None:
        raise ValueError("qualification requires a threshold frozen on validation")
    if options.imgsz <= 0 or not 0.0 < options.nms_iou < 1.0:
        raise ValueError("inference image size or NMS IoU is invalid")
    model_path = options.model.resolve(strict=True)
    dataset_root = options.dataset_root.resolve(strict=True)
    output_path = options.output.resolve()
    if output_path.exists():
        raise ValueError(f"refusing to overwrite evaluation: {output_path}")
    summary_path = dataset_root / "capture_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if summary.get("formal_acceptance") or summary.get("formal_results_consumed"):
        raise ValueError("dataset summary crosses the development boundary")
    images = sorted((dataset_root / "images" / options.split).glob("*.png"))
    if len(images) != 24:
        raise ValueError("evaluation split must contain exactly 24 images")
    truth = _truth_rows(dataset_root / "labels" / options.split, images)
    from ultralytics import YOLO

    predictions = _prediction_rows(
        YOLO(str(model_path)).predict(
            source=[str(path) for path in images],
            imgsz=options.imgsz,
            conf=0.001,
            iou=options.nms_iou,
            device=options.device,
            verbose=False,
            stream=False,
        ),
        [path.stem for path in images],
    )
    if options.threshold is None:
        evaluation = select_validation_threshold(
            truth, predictions, iou_threshold=options.iou_threshold
        )
    else:
        selected = evaluate_threshold(
            truth,
            predictions,
            threshold=options.threshold,
            iou_threshold=options.iou_threshold,
        )
        evaluation = {
            "gate_pass": (
                float(selected["ripe"]["precision"]) >= 0.95
                and float(selected["ripe"]["recall"]) >= 0.90
            ),
            "requirements": {
                "minimum_ripe_precision": 0.95,
                "minimum_ripe_recall": 0.90,
                "iou_threshold": options.iou_threshold,
            },
            "selected": selected,
            "evaluated_threshold_count": 1,
        }
    result = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "GENERALIZED_DEVELOPMENT_DETECTOR_EVALUATION",
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "split": options.split,
        "qualification_opened": options.split == "qualification",
        "image_count": len(images),
        "truth_count": len(truth),
        "prediction_count_at_0_001": len(predictions),
        "inference": {
            "imgsz": options.imgsz,
            "nms_iou": options.nms_iou,
            "minimum_prediction_confidence": 0.001,
        },
        "model": {"path": str(model_path), "sha256": _sha256(model_path)},
        "dataset_summary": {"path": str(summary_path), "sha256": _sha256(summary_path)},
        "evaluation": evaluation,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
