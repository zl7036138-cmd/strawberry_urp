"""Offline scoring for fixed-base camera observability diagnostics.

This module consumes detector/localizer receipts together with simulation truth.
It is development and evaluation code only; runtime perception and control must
never import it.
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence


def _finite_nonnegative(value, label: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{label} must be finite and non-negative")
    return result


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    rank = (len(ordered) - 1) * float(percentile)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def score_observability(
    truth_rows: Sequence[Mapping[str, object]],
    detections: Sequence[Mapping[str, object]],
    *,
    maximum_localization_error_m: float = 0.03,
) -> tuple[list[dict[str, object]], dict[str, int | float | None]]:
    """Classify every truth fruit and summarize detector/localizer coverage."""

    error_limit = _finite_nonnegative(
        maximum_localization_error_m, "maximum localization error"
    )
    if error_limit <= 0.0:
        raise ValueError("maximum localization error must be positive")
    normalized_detections = [dict(row) for row in detections]
    truth_by_id: dict[int, dict[str, object]] = {}
    for raw in truth_rows:
        row = dict(raw)
        target_id = int(row["target_id"])
        maturity = int(row["maturity"])
        if target_id <= 0 or target_id in truth_by_id:
            raise ValueError("truth target IDs must be positive and unique")
        if maturity not in {0, 1, 2}:
            raise ValueError("truth maturity is invalid")
        truth_by_id[target_id] = row

    scored = []
    localized_errors = []
    for target_id in sorted(truth_by_id):
        row = truth_by_id[target_id]
        maturity = int(row["maturity"])
        containing = tuple(int(index) for index in row["containing_detection_indices"])
        if any(index < 0 or index >= len(normalized_detections) for index in containing):
            raise ValueError("truth row references an invalid detection index")
        correct_class = [
            index
            for index in containing
            if int(normalized_detections[index]["maturity"]) == maturity
        ]
        localized = []
        for index in correct_class:
            detection = normalized_detections[index]
            if detection.get("localization_status") != "ACCEPTED":
                continue
            error = _finite_nonnegative(
                detection["nearest_truth_error_m"], "localization error"
            )
            if (
                int(detection["nearest_truth_target_id"]) == target_id
                and error <= error_limit
            ):
                localized.append((error, index))
        in_image = bool(row["projected_center_in_image"])
        depth_visible = bool(row.get("depth_visible", in_image))
        if not in_image:
            status = "OUT_OF_IMAGE"
        elif not depth_visible:
            status = "DEPTH_OCCLUDED"
        elif not containing:
            status = "NO_DETECTION"
        elif not correct_class:
            status = "WRONG_MATURITY"
        elif not localized:
            status = "NO_ACCEPTED_LOCALIZATION"
        else:
            status = "LOCALIZED"
        best = min(localized, default=None)
        if best is not None:
            localized_errors.append(best[0])
        scored.append(
            row
            | {
                "correct_maturity_detection_indices": correct_class,
                "observability_status": status,
                "best_localization_error_m": None if best is None else best[0],
                "best_localization_detection_index": None if best is None else best[1],
            }
        )

    accepted_ripe = [
        row
        for row in normalized_detections
        if int(row["maturity"]) == 1 and row.get("localization_status") == "ACCEPTED"
    ]
    true_ripe_predictions = 0
    for detection in accepted_ripe:
        target_id = int(detection["nearest_truth_target_id"])
        truth = truth_by_id.get(target_id)
        error = _finite_nonnegative(
            detection["nearest_truth_error_m"], "localization error"
        )
        if truth is not None and int(truth["maturity"]) == 1 and error <= error_limit:
            true_ripe_predictions += 1

    ripe = [row for row in scored if int(row["maturity"]) == 1]
    visible_ripe = [
        row
        for row in ripe
        if bool(row["projected_center_in_image"])
        and bool(row.get("depth_visible", True))
    ]
    localized_ripe = [
        row for row in ripe if row["observability_status"] == "LOCALIZED"
    ]
    metrics: dict[str, int | float | None] = {
        "truth_count": len(scored),
        "ripe_truth_count": len(ripe),
        "visible_ripe_truth_count": len(visible_ripe),
        "truth_center_in_image_count": sum(
            bool(row["projected_center_in_image"]) for row in scored
        ),
        "truth_with_detection_count": sum(
            bool(row["containing_detection_indices"]) for row in scored
        ),
        "localized_truth_count": sum(
            row["observability_status"] == "LOCALIZED" for row in scored
        ),
        "localized_ripe_truth_count": len(localized_ripe),
        "ripe_localization_recall": (
            len(localized_ripe) / len(ripe) if ripe else 1.0
        ),
        "visible_ripe_localization_recall": (
            len(localized_ripe) / len(visible_ripe) if visible_ripe else 1.0
        ),
        "accepted_ripe_prediction_count": len(accepted_ripe),
        "true_ripe_prediction_count": true_ripe_predictions,
        "ripe_prediction_precision": (
            true_ripe_predictions / len(accepted_ripe) if accepted_ripe else 1.0
        ),
        "localization_error_p95_m": _percentile(localized_errors, 0.95),
        "maximum_localization_error_m": error_limit,
    }
    return scored, metrics


def aggregate_observability_receipts(
    receipts: Sequence[Mapping[str, object]],
    *,
    maximum_localization_error_m: float = 0.03,
) -> dict[str, object]:
    """Re-score and aggregate immutable per-scene RGB-D diagnostics."""

    if not receipts:
        raise ValueError("at least one observability receipt is required")
    status_counts: dict[str, int] = {}
    truth_count = 0
    ripe_truth_count = 0
    visible_ripe_truth_count = 0
    localized_truth_count = 0
    localized_ripe_truth_count = 0
    accepted_ripe_prediction_count = 0
    true_ripe_prediction_count = 0
    all_ripe_localized_scene_count = 0
    errors = []
    scene_rows = []
    for index, receipt_raw in enumerate(receipts):
        receipt = dict(receipt_raw)
        if (
            int(receipt.get("schema_version", 0)) != 3
            or receipt.get("kind") != "generalized_rgbd_frame_diagnostic"
            or receipt.get("runtime_truth_use") is not False
            or int(receipt.get("commands_published", -1)) != 0
        ):
            raise ValueError("observability receipt contract is invalid")
        scored, metrics = score_observability(
            receipt["truth_projection_scoring"],
            receipt["detections"],
            maximum_localization_error_m=maximum_localization_error_m,
        )
        for row in scored:
            status = str(row["observability_status"])
            status_counts[status] = status_counts.get(status, 0) + 1
            if row["best_localization_error_m"] is not None:
                errors.append(float(row["best_localization_error_m"]))
        truth_count += int(metrics["truth_count"])
        ripe_truth_count += int(metrics["ripe_truth_count"])
        visible_ripe_truth_count += int(metrics["visible_ripe_truth_count"])
        localized_truth_count += int(metrics["localized_truth_count"])
        localized_ripe_truth_count += int(metrics["localized_ripe_truth_count"])
        accepted_ripe_prediction_count += int(
            metrics["accepted_ripe_prediction_count"]
        )
        true_ripe_prediction_count += int(metrics["true_ripe_prediction_count"])
        visible_ripe_count = int(metrics["visible_ripe_truth_count"])
        ripe_complete = visible_ripe_count > 0 and int(
            metrics["localized_ripe_truth_count"]
        ) == visible_ripe_count
        all_ripe_localized_scene_count += int(ripe_complete)
        scene_rows.append(
            {
                "receipt_index": index,
                "ripe_truth_count": int(metrics["ripe_truth_count"]),
                "visible_ripe_truth_count": int(
                    metrics["visible_ripe_truth_count"]
                ),
                "localized_ripe_truth_count": int(
                    metrics["localized_ripe_truth_count"]
                ),
                "ripe_localization_recall": float(
                    metrics["ripe_localization_recall"]
                ),
                "visible_ripe_localization_recall": float(
                    metrics["visible_ripe_localization_recall"]
                ),
                "all_ripe_localized": ripe_complete,
            }
        )
    return {
        "scene_count": len(receipts),
        "truth_count": truth_count,
        "ripe_truth_count": ripe_truth_count,
        "visible_ripe_truth_count": visible_ripe_truth_count,
        "localized_truth_count": localized_truth_count,
        "localized_ripe_truth_count": localized_ripe_truth_count,
        "ripe_localization_recall": (
            localized_ripe_truth_count / ripe_truth_count
            if ripe_truth_count
            else 1.0
        ),
        "visible_ripe_localization_recall": (
            localized_ripe_truth_count / visible_ripe_truth_count
            if visible_ripe_truth_count
            else 1.0
        ),
        "accepted_ripe_prediction_count": accepted_ripe_prediction_count,
        "true_ripe_prediction_count": true_ripe_prediction_count,
        "ripe_prediction_precision": (
            true_ripe_prediction_count / accepted_ripe_prediction_count
            if accepted_ripe_prediction_count
            else 1.0
        ),
        "all_ripe_localized_scene_count": all_ripe_localized_scene_count,
        "all_ripe_localized_scene_rate": (
            all_ripe_localized_scene_count / len(receipts)
        ),
        "localization_error_p95_m": _percentile(errors, 0.95),
        "maximum_localization_error_m": float(maximum_localization_error_m),
        "observability_status_counts": dict(sorted(status_counts.items())),
        "scenes": scene_rows,
    }
