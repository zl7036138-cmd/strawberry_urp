#!/usr/bin/env python3
"""Rebaseline T30 from immutable audited-validation evidence.

This diagnostic is deliberately validation-only.  It reuses the selected
audited checkpoint predictions, verifies their bindings, tests whether
per-class confidence calibration can close the gate, and summarizes the
geometry of the remaining false negatives.  It never starts inference or
training and refuses any record outside ``images/val``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Iterable, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from analyze_audited_checkpoint_errors import (  # noqa: E402
    _bbox,
    _greedy_unmatched,
    classify_false_negative,
)
from workflow import (  # noqa: E402
    ImageDetections,
    _record_from_json,
    evaluate_detection_metrics,
    select_validation_threshold,
    threshold_grid,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SWEEP = Path(
    "artifacts/perception/label_audit/"
    "yolo11s_audited_checkpoint_sweep_v1/summary.json"
)
DEFAULT_ERROR_ANALYSIS = Path(
    "artifacts/perception/label_audit/"
    "yolo11s_audited_checkpoint_error_analysis_v1.json"
)
DEFAULT_AUDITED_METRICS = Path(
    "artifacts/perception/label_audit/"
    "yolo11s_val_threshold_031_audited_metrics_v1.json"
)
DEFAULT_TARGETED_SUMMARY = Path(
    "artifacts/perception/optimization/"
    "yolo11s_640_unripe_x2_opt2_audited_validation_v1/summary.json"
)
DEFAULT_SIM_ADAPT_SUMMARY = Path(
    "artifacts/perception/optimization/"
    "yolo11s_640_sim_adapt_v1_real_nonregression_v1/summary.json"
)
DEFAULT_DATASET_ROOT = Path("data/processed/zenodo_6126677")
DEFAULT_OUTPUT = Path(
    "artifacts/perception/rebaseline/t30_validation_rebaseline_v1.json"
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


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _binding(path: Path) -> dict[str, Any]:
    try:
        display_path = path.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        display_path = str(path)
    return {
        "path": display_path,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _require_kind(value: Mapping[str, Any], expected: str, path: Path) -> None:
    if value.get("kind") != expected:
        raise ValueError(f"unexpected kind in {path}: {value.get('kind')!r}")


def _selected_artifact(sweep_path: Path, sweep: Mapping[str, Any]) -> Path:
    selected = sweep.get("selected_candidate")
    if not isinstance(selected, dict):
        raise ValueError("checkpoint sweep has no selected candidate")
    artifact = sweep_path.parent / str(selected.get("artifact", ""))
    if not artifact.is_file():
        raise FileNotFoundError(artifact)
    if _sha256(artifact) != selected.get("artifact_sha256"):
        raise ValueError("selected checkpoint artifact hash mismatch")
    return artifact


def _class_only_records(
    records: Sequence[ImageDetections], class_id: int
) -> tuple[ImageDetections, ...]:
    return tuple(
        ImageDetections(
            image=record.image,
            ground_truth=tuple(
                item for item in record.ground_truth if item.class_id == class_id
            ),
            predictions=tuple(
                item for item in record.predictions if item.class_id == class_id
            ),
            inference_ms=record.inference_ms,
            total_ms=record.total_ms,
        )
        for record in records
    )


def _per_class_calibration(
    records: Sequence[ImageDetections], candidates: Iterable[float]
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    selected_f1 = []
    for class_id in CLASS_NAMES:
        selection = select_validation_threshold(
            _class_only_records(records, class_id),
            candidates,
            iou_threshold=0.5,
            class_ids=(class_id,),
        )
        metrics = selection["validation_metrics"]
        class_metrics = metrics["per_class"][str(class_id)]
        selected_f1.append(float(class_metrics["f1"]))
        result[str(class_id)] = {
            "class_name": CLASS_NAMES[class_id],
            "selected_threshold": float(selection["selected_threshold"]),
            "metrics": class_metrics,
        }
    result["combined_macro_f1"] = statistics.fmean(selected_f1)
    result["selection_rule"] = (
        "independently maximize each class F1 on the frozen 0.05..0.95/0.01 grid"
    )
    result["decision_effect"] = "diagnostic_only"
    return result


def _minimum_ideal_repairs(
    *, tp: int, fp: int, fn: int, required_f1: float
) -> dict[str, int | None]:
    """Return idealized one-axis repairs; real interventions can add errors."""

    recoveries = None
    for recovered in range(fn + 1):
        candidate_tp = tp + recovered
        candidate_fn = fn - recovered
        f1 = 2 * candidate_tp / (2 * candidate_tp + fp + candidate_fn)
        if f1 >= required_f1:
            recoveries = recovered
            break
    removals = None
    for removed in range(fp + 1):
        candidate_fp = fp - removed
        f1 = 2 * tp / (2 * tp + candidate_fp + fn)
        if f1 >= required_f1:
            removals = removed
            break
    return {
        "false_negatives_recovered_with_no_new_fp": recoveries,
        "false_positives_removed_with_no_new_fn": removals,
    }


def _distribution(values: Sequence[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "median": None, "mean": None, "min": None, "max": None}
    return {
        "count": len(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "min": min(values),
        "max": max(values),
    }


def _geometry_summary(
    artifact: Mapping[str, Any], dataset_root: Path, threshold: float, imgsz: int
) -> dict[str, Any]:
    try:
        from PIL import Image
    except ImportError as error:  # pragma: no cover - environment dependency
        raise RuntimeError("Pillow is required for target-geometry analysis") from error

    raw_records = artifact.get("records")
    if not isinstance(raw_records, list) or len(raw_records) != 116:
        raise ValueError("selected artifact must contain 116 validation records")
    rows: list[dict[str, Any]] = []
    for raw in raw_records:
        image_key = str(raw.get("image", "")).replace("\\", "/")
        if not image_key.startswith("images/val/") or "/test/" in image_key:
            raise ValueError(f"record escapes validation scope: {image_key}")
        image_path = (dataset_root / image_key).resolve()
        try:
            image_path.relative_to(dataset_root.resolve())
        except ValueError as error:
            raise ValueError(f"validation image escapes dataset root: {image_key}") from error
        if not image_path.is_file():
            raise FileNotFoundError(image_path)
        with Image.open(image_path) as image:
            width, height = image.size
        scale = imgsz / max(width, height)
        truths = raw.get("ground_truth")
        predictions = raw.get("predictions")
        if not isinstance(truths, list) or not isinstance(predictions, list):
            raise ValueError("selected artifact record is malformed")
        for class_id in CLASS_NAMES:
            truth_indices = [
                index
                for index, truth in enumerate(truths)
                if int(truth["class_id"]) == class_id
            ]
            unmatched, _ = _greedy_unmatched(
                truths, predictions, class_id, threshold
            )
            unmatched_set = set(unmatched)
            for index in truth_indices:
                truth = truths[index]
                box = _bbox(truth)
                box_width = (box[2] - box[0]) * width * scale
                box_height = (box[3] - box[1]) * height * scale
                status = (
                    "TP"
                    if index not in unmatched_set
                    else classify_false_negative(truth, predictions, threshold)
                )
                rows.append(
                    {
                        "class_id": class_id,
                        "status": status,
                        "model_input_min_side_px": min(box_width, box_height),
                        "model_input_area_px2": box_width * box_height,
                        "image_truth_count": len(truths),
                    }
                )

    result: dict[str, Any] = {}
    for class_id, class_name in CLASS_NAMES.items():
        class_rows = [row for row in rows if row["class_id"] == class_id]
        groups: dict[str, Any] = {}
        for status in ("TP", "LOW_CONFIDENCE", "NO_DETECTION", "WRONG_CLASS", "LOCALIZATION"):
            group = [row for row in class_rows if row["status"] == status]
            if not group:
                continue
            min_sides = [float(row["model_input_min_side_px"]) for row in group]
            groups[status] = {
                "count": len(group),
                "min_side_px": _distribution(min_sides),
                "under_32_px_count": sum(value < 32.0 for value in min_sides),
                "image_truth_count_median": statistics.median(
                    int(row["image_truth_count"]) for row in group
                ),
            }
        false_negative_rows = [row for row in class_rows if row["status"] != "TP"]
        result[str(class_id)] = {
            "class_name": class_name,
            "groups": groups,
            "false_negative_count": len(false_negative_rows),
            "false_negative_under_32_px_count": sum(
                float(row["model_input_min_side_px"]) < 32.0
                for row in false_negative_rows
            ),
        }
    result["imgsz"] = imgsz
    result["interpretation"] = (
        "Model-input box dimensions use the standard letterbox scale imgsz/max(width,height); "
        "they diagnose target scale but do not measure occlusion or visual ambiguity."
    )
    return result


def rebaseline(
    *,
    sweep_path: Path,
    error_analysis_path: Path,
    audited_metrics_path: Path,
    targeted_summary_path: Path,
    sim_adapt_summary_path: Path,
    dataset_root: Path,
    output_path: Path,
) -> dict[str, Any]:
    inputs = [
        sweep_path,
        error_analysis_path,
        audited_metrics_path,
        targeted_summary_path,
        sim_adapt_summary_path,
    ]
    for path in inputs:
        if not path.is_file():
            raise FileNotFoundError(path)
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite rebaseline report: {output_path}")

    sweep = _load_object(sweep_path)
    errors = _load_object(error_analysis_path)
    audited = _load_object(audited_metrics_path)
    targeted = _load_object(targeted_summary_path)
    sim_adapt = _load_object(sim_adapt_summary_path)
    _require_kind(sweep, "audited_checkpoint_sweep_summary", sweep_path)
    _require_kind(errors, "audited_checkpoint_error_attribution", error_analysis_path)
    _require_kind(audited, "audited_validation_label_only_diagnostic", audited_metrics_path)
    _require_kind(targeted, "targeted_training_audited_validation_decision", targeted_summary_path)
    _require_kind(sim_adapt, "simulator_adaptation_real_nonregression_decision", sim_adapt_summary_path)

    artifact_path = _selected_artifact(sweep_path, sweep)
    artifact = _load_object(artifact_path)
    raw_records = artifact.get("records")
    if not isinstance(raw_records, list):
        raise ValueError("selected checkpoint artifact has no records")
    records = tuple(_record_from_json(item) for item in raw_records)
    if len(records) != 116 or any(
        not record.image.replace("\\", "/").startswith("images/val/")
        for record in records
    ):
        raise ValueError("rebaseline accepts exactly 116 validation records")

    selected = sweep["selected_candidate"]
    threshold = float(selected["selected_threshold"])
    shared_metrics = evaluate_detection_metrics(records, threshold, 0.5, (0, 1))
    if shared_metrics != selected["validation_metrics"]:
        raise ValueError("selected checkpoint metrics cannot be reproduced")
    if errors.get("selected_checkpoint_id") != sweep.get("selected_checkpoint_id"):
        raise ValueError("error analysis selected checkpoint mismatch")
    if not math.isclose(float(errors.get("confidence_threshold")), threshold, abs_tol=1e-12):
        raise ValueError("error analysis threshold mismatch")

    candidates = threshold_grid(0.05, 0.95, 0.01)
    calibration = _per_class_calibration(records, candidates)
    gate = float(sweep["validation_gate"]["required"])
    ripe = shared_metrics["per_class"]["0"]
    unripe = shared_metrics["per_class"]["1"]
    required_unripe_f1 = 2.0 * gate - float(ripe["f1"])
    geometry = _geometry_summary(artifact, dataset_root, threshold, imgsz=640)
    unripe_geometry = geometry["1"]
    unripe_fn = int(unripe_geometry["false_negative_count"])
    unripe_small = int(unripe_geometry["false_negative_under_32_px_count"])

    result = {
        "schema_version": 1,
        "kind": "t30_validation_rebaseline_diagnostic",
        "scope": {
            "selection_split": "audited_validation_only",
            "validation_image_count": len(records),
            "held_out_test_accessed": False,
            "training_started": False,
            "simulator_experiment_started": False,
            "robot_motion_started": False,
            "formal_acceptance": False,
        },
        "bindings": {
            "checkpoint_sweep": _binding(sweep_path),
            "selected_checkpoint_artifact": _binding(artifact_path),
            "error_analysis": _binding(error_analysis_path),
            "audited_label_metrics": _binding(audited_metrics_path),
            "unripe_exposure_outcome": _binding(targeted_summary_path),
            "sim_adaptation_real_outcome": _binding(sim_adapt_summary_path),
        },
        "selected_baseline": {
            "checkpoint_id": sweep["selected_checkpoint_id"],
            "confidence_threshold": threshold,
            "metrics": shared_metrics,
            "gate": {"required": gate, "passed": False},
        },
        "class_specific_threshold_counterfactual": {
            **calibration,
            "gate_required": gate,
            "gate_passed": float(calibration["combined_macro_f1"]) >= gate,
            "shortfall": gate - float(calibration["combined_macro_f1"]),
        },
        "idealized_gate_gap": {
            "required_unripe_f1_if_ripe_unchanged": required_unripe_f1,
            "current_unripe_f1": float(unripe["f1"]),
            "repairs": _minimum_ideal_repairs(
                tp=int(unripe["tp"]),
                fp=int(unripe["fp"]),
                fn=int(unripe["fn"]),
                required_f1=required_unripe_f1,
            ),
            "interpretation": (
                "These are optimistic arithmetic bounds, not expected gains: a real "
                "intervention can trade false negatives against false positives."
            ),
        },
        "error_attribution": {
            "false_negatives": errors["false_negative_attribution"],
            "false_positives": errors["false_positive_attribution"],
        },
        "target_geometry": geometry,
        "ruled_out_primary_remedies": [
            {
                "remedy": "shared_or_per_class_threshold_tuning",
                "evidence": (
                    f"per-class optimum macro-F1 is {calibration['combined_macro_f1']:.6f}, "
                    f"still {gate - float(calibration['combined_macro_f1']):.6f} below gate"
                ),
            },
            {
                "remedy": "existing_checkpoint_reselection",
                "evidence": "all 26 frozen checkpoints were swept; baseline__best remained selected",
            },
            {
                "remedy": "simple_unripe_image_duplication",
                "evidence": (
                    f"audited macro-F1 {float(targeted['selected_checkpoint']['validation_metrics']['macro_f1']):.6f}; "
                    "candidate was rejected"
                ),
            },
            {
                "remedy": "current_simulator_mixed_finetune",
                "evidence": (
                    f"real-validation macro-F1 {float(sim_adapt['metrics']['macro_f1']):.6f}; "
                    "candidate was rejected"
                ),
            },
            {
                "remedy": "input_resolution_as_first_intervention",
                "evidence": (
                    f"only {unripe_small}/{unripe_fn} unripe false negatives have a model-input "
                    "minimum side below 32 px; low-confidence misses have target scale similar to true positives"
                ),
            },
        ],
        "decision": {
            "status": "REBASELINED_T30_REMAINS_NOT_ACCEPTED",
            "primary_remaining_uncertainty": (
                "training-label completeness and maturity-class consistency"
            ),
            "next_single_intervention": "bounded_train_split_label_audit",
            "rationale": [
                "The validation audit added 13 confirmed boxes and materially changed the measured macro-F1.",
                "The 501-image training split has not received the same annotation-quality audit.",
                "Calibration, old-checkpoint selection, naive unripe duplication, and the completed synthetic intervention do not close the gate.",
                "A read-only training-label audit can test the remaining data-quality hypothesis before authorizing another training claim.",
            ],
            "authorization": {
                "training_label_audit": True,
                "new_training": False,
                "held_out_test": False,
                "perception_control": False,
                "formal_simulator_matrix": False,
                "new_t70_feature_work": False,
            },
        },
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", type=Path, default=DEFAULT_SWEEP)
    parser.add_argument("--error-analysis", type=Path, default=DEFAULT_ERROR_ANALYSIS)
    parser.add_argument("--audited-metrics", type=Path, default=DEFAULT_AUDITED_METRICS)
    parser.add_argument("--targeted-summary", type=Path, default=DEFAULT_TARGETED_SUMMARY)
    parser.add_argument("--sim-adapt-summary", type=Path, default=DEFAULT_SIM_ADAPT_SUMMARY)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    result = rebaseline(
        sweep_path=_resolve(args.sweep).resolve(),
        error_analysis_path=_resolve(args.error_analysis).resolve(),
        audited_metrics_path=_resolve(args.audited_metrics).resolve(),
        targeted_summary_path=_resolve(args.targeted_summary).resolve(),
        sim_adapt_summary_path=_resolve(args.sim_adapt_summary).resolve(),
        dataset_root=_resolve(args.dataset_root).resolve(),
        output_path=_resolve(args.output).resolve(),
    )
    print(
        json.dumps(
            {
                "output": str(_resolve(args.output)),
                "macro_f1": result["selected_baseline"]["metrics"]["macro_f1"],
                "per_class_macro_f1": result[
                    "class_specific_threshold_counterfactual"
                ]["combined_macro_f1"],
                "next_single_intervention": result["decision"][
                    "next_single_intervention"
                ],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
