#!/usr/bin/env python3
"""Decide whether the single Opt1 candidate may consume the held-out test.

This command is validation-only. It compares the frozen validation metrics
against the policy pinned in ``experiments.json`` and writes one immutable
promotion artifact. It never opens the held-out test receipt or test samples.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path, PurePosixPath
from typing import Mapping, Sequence

from workflow import (
    FORMAL_NMS_IOU,
    load_prediction_bundle,
    load_validation_inference_contract,
    optimization_promotion_artifact_path,
    read_json,
    select_validation_threshold,
    sha256_file,
    threshold_grid,
    validate_formal_validation_bundle,
    validate_validation_records_against_split,
    write_json_exclusive,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXPERIMENTS = Path(__file__).resolve().with_name("experiments.json")
DEFAULT_BASELINE_DECISION = (
    REPOSITORY_ROOT
    / "artifacts/perception/optimization/yolo11s_640_opt1_decision.json"
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-freeze", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--experiments", type=Path, default=DEFAULT_EXPERIMENTS)
    parser.add_argument(
        "--baseline-decision", type=Path, default=DEFAULT_BASELINE_DECISION
    )
    return parser.parse_args(argv)


def _safe_repository_path(value: object) -> Path:
    relative = PurePosixPath(str(value).replace("\\", "/"))
    if (
        relative.is_absolute()
        or not relative.parts
        or ".." in relative.parts
        or relative.parts[0].endswith(":")
    ):
        raise ValueError(f"artifact path must be repository-relative: {value}")
    root = REPOSITORY_ROOT.resolve()
    resolved = root.joinpath(*relative.parts).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"artifact path escapes the repository: {value}") from error
    return resolved


def _repository_relative(path: Path) -> str:
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(REPOSITORY_ROOT.resolve())
    except ValueError as error:
        raise ValueError(f"artifact is outside the repository: {path}") from error
    return relative.as_posix()


def _load_flat_yaml(path: Path) -> Mapping[str, str]:
    """Read the flat Ultralytics configs without adding a YAML dependency."""

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            raise ValueError(f"{path}:{line_number}: expected a flat YAML key")
        key, value = line.split(":", 1)
        key = key.strip()
        if not key or key in values:
            raise ValueError(f"{path}:{line_number}: duplicate or empty YAML key")
        values[key] = value.strip()
    return values


def _validate_single_intervention(
    spec: Mapping[str, object], policy: Mapping[str, object]
) -> Mapping[str, object]:
    variants = spec.get("variants")
    if not isinstance(variants, dict):
        raise ValueError("experiment matrix has no variants")
    baseline_name = policy.get("baseline_variant")
    candidate_name = policy.get("candidate_variant")
    baseline = variants.get(baseline_name)
    candidate = variants.get(candidate_name)
    if not isinstance(baseline, dict) or not isinstance(candidate, dict):
        raise ValueError("optimization policy references an unknown variant")

    baseline_train_path = _safe_repository_path(baseline["training_config"])
    candidate_train_path = _safe_repository_path(candidate["training_config"])
    baseline_train = dict(_load_flat_yaml(baseline_train_path))
    candidate_train = dict(_load_flat_yaml(candidate_train_path))
    intervention = policy.get("intervention")
    if not isinstance(intervention, dict) or intervention.get("parameter") != "cls_pw":
        raise ValueError("Opt1 policy must pin cls_pw as its single intervention")
    baseline_default = float(intervention.get("baseline_value"))
    candidate_value = float(intervention.get("candidate_value"))
    if "cls_pw" in baseline_train or baseline_default != 0.0:
        raise ValueError("baseline must use the pinned Ultralytics cls_pw=0.0 default")
    if float(candidate_train.pop("cls_pw", "nan")) != candidate_value:
        raise ValueError("candidate training config does not pin the Opt1 cls_pw")
    expected_train = {
        **baseline_train,
        "name": "yolo11s_640_cls_pw05_opt1",
    }
    if candidate_train != expected_train:
        raise ValueError("candidate changes training settings beyond name and cls_pw")

    baseline_eval_path = _safe_repository_path(baseline["validation_config"])
    candidate_eval_path = _safe_repository_path(candidate["validation_config"])
    baseline_eval = dict(_load_flat_yaml(baseline_eval_path))
    candidate_eval = dict(_load_flat_yaml(candidate_eval_path))
    expected_eval = {
        **baseline_eval,
        "model": str(candidate["trained_weight"]),
        "name": "yolo11s_640_cls_pw05_opt1_validation_raw",
    }
    if candidate_eval != expected_eval:
        raise ValueError("candidate changes validation settings beyond model and name")
    validation_contract = load_validation_inference_contract(candidate_eval_path)
    if validation_contract["model"] != candidate.get("trained_weight"):
        raise ValueError("candidate validation config model does not match its weight")
    if not float(validation_contract["nms_iou"]) == FORMAL_NMS_IOU:
        raise ValueError(f"formal validation NMS IoU is fixed at {FORMAL_NMS_IOU}")
    return {
        "training_config": _repository_relative(candidate_train_path),
        "training_config_sha256": sha256_file(candidate_train_path),
        "validation_config": _repository_relative(candidate_eval_path),
        "validation_config_sha256": sha256_file(candidate_eval_path),
        "validation_task": validation_contract["task"],
        "validation_mode": validation_contract["mode"],
        "validation_split": validation_contract["split"],
        "validation_imgsz": validation_contract["imgsz"],
        "validation_batch": validation_contract["batch"],
        "validation_device": validation_contract["device"],
        "validation_raw_confidence": validation_contract["raw_confidence"],
        "validation_nms_iou": validation_contract["nms_iou"],
    }


def _verify_baseline_decision(decision: Mapping[str, object]) -> Mapping[str, object]:
    if decision.get("schema_version") != 1 or decision.get("kind") != (
        "validation_only_optimization_decision"
    ):
        raise ValueError("baseline optimization decision has the wrong schema or kind")
    scope = decision.get("scope")
    provenance = decision.get("baseline_provenance")
    if not isinstance(scope, dict) or not isinstance(provenance, dict):
        raise ValueError("baseline optimization decision is incomplete")
    if scope.get("selection_split") != "val" or scope.get("held_out_test_accessed") is not False:
        raise ValueError("baseline decision is not validation-only")
    for name, artifact in provenance.items():
        if name == "validation_metrics" or not isinstance(artifact, dict):
            continue
        path_value = artifact.get("path")
        digest = artifact.get("sha256")
        if path_value is None or digest is None:
            continue
        path = _safe_repository_path(path_value)
        if not path.is_file():
            raise FileNotFoundError(path)
        if sha256_file(path) != digest:
            raise ValueError(f"historical baseline {name} digest does not match")

    freeze_artifact = provenance.get("frozen_validation_threshold")
    metrics = provenance.get("validation_metrics")
    if not isinstance(freeze_artifact, dict) or not isinstance(metrics, dict):
        raise ValueError("baseline decision has no frozen validation metrics")
    baseline_freeze = read_json(_safe_repository_path(freeze_artifact["path"]))
    selection = baseline_freeze.get("selection")
    if not isinstance(selection, dict):
        raise ValueError("historical baseline freeze has no selection")
    frozen_metrics = selection.get("validation_metrics")
    if not isinstance(frozen_metrics, dict):
        raise ValueError("historical baseline freeze has no validation metrics")
    per_class = frozen_metrics.get("per_class")
    if not isinstance(per_class, dict):
        raise ValueError("historical baseline freeze has no per-class metrics")
    expected = {
        "macro_f1": frozen_metrics.get("macro_f1"),
        "ripe_f1": per_class.get("0", {}).get("f1"),
        "unripe_f1": per_class.get("1", {}).get("f1"),
    }
    for key, value in expected.items():
        if metrics.get(key) != value:
            raise ValueError(f"historical baseline {key} does not match its freeze")
    return metrics


def _candidate_metrics(freeze: Mapping[str, object]) -> Mapping[str, float]:
    selection = freeze.get("selection")
    if not isinstance(selection, dict):
        raise ValueError("candidate freeze has no validation selection")
    metrics = selection.get("validation_metrics")
    if not isinstance(metrics, dict):
        raise ValueError("candidate freeze has no validation metrics")
    per_class = metrics.get("per_class")
    if not isinstance(per_class, dict):
        raise ValueError("candidate freeze has no per-class metrics")
    try:
        return {
            "macro_f1": float(metrics["macro_f1"]),
            "ripe_f1": float(per_class["0"]["f1"]),
            "unripe_f1": float(per_class["1"]["f1"]),
        }
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("candidate freeze has malformed two-class metrics") from error


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.experiments.resolve() != DEFAULT_EXPERIMENTS.resolve():
        raise ValueError("promotion requires the repository's pinned experiments.json")
    if args.baseline_decision.resolve() != DEFAULT_BASELINE_DECISION.resolve():
        raise ValueError("promotion requires the pinned historical baseline decision")
    if args.output.exists():
        raise FileExistsError(args.output)
    spec = read_json(args.experiments)
    policy = spec.get("optimization_policy")
    formal_test = spec.get("formal_test")
    if not isinstance(policy, dict) or not isinstance(formal_test, dict):
        raise ValueError("experiment has no optimization or formal-test policy")
    expected_output = optimization_promotion_artifact_path(spec, REPOSITORY_ROOT)
    if expected_output is None or args.output.resolve() != expected_output:
        raise ValueError(f"promotion output is fixed by the contract: {expected_output}")
    baseline_name = str(policy.get("baseline_variant", ""))
    candidate_name = str(policy.get("candidate_variant", ""))
    if formal_test.get("variant") != candidate_name:
        raise ValueError("formal test is not reserved for the Opt1 candidate")
    config_bindings = _validate_single_intervention(spec, policy)

    baseline_decision = read_json(args.baseline_decision)
    baseline_metrics = _verify_baseline_decision(baseline_decision)
    scope = baseline_decision.get("scope")
    assert isinstance(scope, dict)
    if scope.get("baseline_variant") != baseline_name or scope.get(
        "candidate_variant"
    ) != candidate_name:
        raise ValueError("baseline decision variants do not match current policy")

    candidate_freeze = read_json(args.candidate_freeze)
    if candidate_freeze.get("schema_version") != 1 or candidate_freeze.get("kind") != (
        "frozen_threshold"
    ):
        raise ValueError("candidate input is not a frozen validation threshold")
    if candidate_freeze.get("variant") != candidate_name:
        raise ValueError("candidate freeze variant does not match Opt1")
    bindings = candidate_freeze.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("candidate freeze has no provenance bindings")
    experiments_sha256 = sha256_file(args.experiments)
    if bindings.get("experiments_sha256") != experiments_sha256:
        raise ValueError("candidate freeze belongs to a different experiment contract")
    if bindings.get("weights_source") != "expected_trained_weight":
        raise ValueError("candidate freeze does not use the formal trained weight")
    validation_bundle_value = bindings.get("validation_bundle")
    validation_bundle_sha256 = bindings.get("validation_bundle_sha256")
    validation_bundle = _safe_repository_path(validation_bundle_value)
    if sha256_file(validation_bundle) != validation_bundle_sha256:
        raise ValueError("candidate validation-bundle digest does not match")
    for name, expected in config_bindings.items():
        if bindings.get(name) != expected:
            raise ValueError(f"candidate freeze {name} binding does not match")

    variants = spec.get("variants")
    if not isinstance(variants, dict) or not isinstance(variants.get(candidate_name), dict):
        raise ValueError("candidate variant is missing from the experiment")
    candidate_variant = variants[candidate_name]
    trained_weight = _safe_repository_path(candidate_variant["trained_weight"])
    if sha256_file(trained_weight) != bindings.get("weights_sha256"):
        raise ValueError("candidate freeze weight digest does not match current weight")
    bundle, records = load_prediction_bundle(validation_bundle, expected_split="val")
    validate_formal_validation_bundle(
        bundle,
        variant=candidate_name,
        expected_bindings=bindings,
        expected_imgsz=int(formal_test["imgsz"]),
    )
    metric = spec.get("metric")
    split_contract = bindings.get("split_contract")
    if not isinstance(metric, dict) or not isinstance(split_contract, dict):
        raise ValueError("candidate freeze has no metric or split contract")
    counts = split_contract.get("counts")
    if not isinstance(counts, dict):
        raise ValueError("candidate freeze split contract has no counts")
    class_ids = tuple(int(value) for value in metric["class_ids"])
    validate_validation_records_against_split(
        records,
        _safe_repository_path(spec["split_manifest"]),
        class_ids=class_ids,
        expected_count=int(counts["val"]),
    )
    grid = metric.get("threshold_grid")
    if not isinstance(grid, dict):
        raise ValueError("experiment has no threshold grid")
    reproduced_selection = select_validation_threshold(
        records,
        threshold_grid(float(grid["start"]), float(grid["stop"]), float(grid["step"])),
        iou_threshold=float(metric["iou_threshold"]),
        class_ids=class_ids,
    )
    if candidate_freeze.get("selection") != reproduced_selection:
        raise ValueError("candidate freeze selection cannot be reproduced")
    if sha256_file(validation_bundle) != validation_bundle_sha256:
        raise ValueError("candidate validation bundle changed during promotion")

    actual = _candidate_metrics(candidate_freeze)
    criteria = policy.get("validation_promotion")
    if not isinstance(criteria, dict):
        raise ValueError("optimization policy has no validation promotion criteria")
    threshold_map = {
        "unripe_f1": float(criteria["unripe_f1_min"]),
        "macro_f1": float(criteria["macro_f1_min"]),
        "ripe_f1": float(criteria["ripe_f1_min"]),
    }
    checks = {
        metric: {
            "operator": ">=",
            "required": required,
            "actual": actual[metric],
            "passed": actual[metric] >= required,
        }
        for metric, required in threshold_map.items()
    }
    promotion_passed = all(bool(check["passed"]) for check in checks.values())
    final_acceptance = policy.get("formal_acceptance")
    metric_contract = spec.get("metric")
    if not isinstance(final_acceptance, dict) or not isinstance(metric_contract, dict):
        raise ValueError("formal acceptance policy is incomplete")
    if float(final_acceptance.get("macro_f1_min")) != float(
        metric_contract.get("macro_f1_min")
    ):
        raise ValueError("optimization and formal macro-F1 gates disagree")
    authorization_required = float(final_acceptance["macro_f1_min"])
    authorization_passed = (
        promotion_passed and actual["macro_f1"] >= authorization_required
    )

    output = {
        "schema_version": 1,
        "kind": "validation_promotion_decision",
        "created_utc": utc_now(),
        "status": "completed",
        "scope": {
            "baseline_variant": baseline_name,
            "candidate_variant": candidate_name,
            "selection_split": "val",
            "held_out_test_accessed": False,
        },
        "bindings": {
            "experiments": _repository_relative(args.experiments),
            "experiments_sha256": experiments_sha256,
            "baseline_decision": _repository_relative(args.baseline_decision),
            "baseline_decision_sha256": sha256_file(args.baseline_decision),
            "candidate_frozen_threshold": _repository_relative(args.candidate_freeze),
            "candidate_frozen_threshold_sha256": sha256_file(args.candidate_freeze),
            "candidate_weights_sha256": bindings.get("weights_sha256"),
            "candidate_validation_bundle": validation_bundle_value,
            "candidate_validation_bundle_sha256": validation_bundle_sha256,
            "canonical_dataset_sha256": bindings.get("canonical_dataset_sha256"),
            **config_bindings,
        },
        "baseline_validation_metrics": {
            "macro_f1": baseline_metrics["macro_f1"],
            "ripe_f1": baseline_metrics["ripe_f1"],
            "unripe_f1": baseline_metrics["unripe_f1"],
        },
        "candidate_validation_metrics": actual,
        "candidate_minus_baseline": {
            key: actual[key] - float(baseline_metrics[key]) for key in actual
        },
        "checks": checks,
        "all_validation_conditions_passed": promotion_passed,
        "promote_candidate": promotion_passed,
        "validation_authorization_gate": {
            "metric": "macro_f1",
            "operator": ">=",
            "required": authorization_required,
            "actual": actual["macro_f1"],
            "passed": actual["macro_f1"] >= authorization_required,
        },
        "authorize_held_out_test": authorization_passed,
        "formal_test_gate": {
            "metric": "macro_f1",
            "operator": ">=",
            "required": float(final_acceptance["macro_f1_min"]),
            "evaluated": False,
        },
    }
    write_json_exclusive(args.output, output)
    print(json.dumps(output, indent=2, ensure_ascii=False, sort_keys=True))
    # 0: candidate promoted and test authorized; 2: candidate promoted but the
    # validation macro-F1 is below the formal gate; 3: Opt1 promotion failed.
    if authorization_passed:
        return 0
    return 2 if promotion_passed else 3


if __name__ == "__main__":
    raise SystemExit(main())
