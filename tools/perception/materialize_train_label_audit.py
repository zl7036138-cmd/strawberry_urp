#!/usr/bin/env python3
"""Materialize the ADR-0025 consensus-corrected train/audited-val derivative."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import tempfile
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = Path("tools/perception/train_label_audit_v1_contract.json")

from materialize_audited_validation import verify_audited_derivative  # noqa: E402
from materialize_unripe_exposure_training import (  # noqa: E402
    _canonical_rows,
    _canonical_sha256,
    _count_classes,
    _display,
    _link_or_copy,
    _load_object,
    _resolve,
    _safe_split_path,
    _sha256,
    _verify_binding,
)


def _validate_contract(contract_path: Path) -> tuple[dict[str, Any], dict[str, Path]]:
    contract_path = _resolve(contract_path).resolve()
    contract = _load_object(contract_path)
    if contract.get("kind") != "training_label_audit_training_contract":
        raise ValueError("unexpected training-label-audit contract kind")
    if contract.get("status") != "AUTHORIZED":
        raise ValueError("training-label-audit contract is not authorized")
    formal_test = contract.get("formal_test", {})
    if formal_test.get("access_authorized") is not False:
        raise ValueError("contract does not preserve the formal-test seal")
    receipt_path = _resolve(Path(str(formal_test["receipt_must_remain_absent"]))).resolve()
    if receipt_path.exists():
        raise FileExistsError("held-out-test receipt exists")

    authorization_path = _resolve(Path(str(contract["authorization_receipt"]))).resolve()
    authorization = _load_object(authorization_path)
    if authorization.get("kind") != "training_label_audit_user_authorization":
        raise ValueError("unexpected training-label authorization kind")
    if (
        authorization.get("status") != "AUTHORIZED"
        or authorization.get("materialization_authorized") is not True
        or authorization.get("new_training_authorized") is not True
        or authorization.get("formal_test_authorized") is not False
        or authorization.get("test_split_access_authorized") is not False
    ):
        raise ValueError("training-label authorization is incomplete")
    contract_binding = authorization.get("contract", {})
    if (
        contract_binding.get("path") != _display(contract_path)
        or contract_binding.get("size_bytes") != contract_path.stat().st_size
        or contract_binding.get("sha256") != _sha256(contract_path)
    ):
        raise ValueError("authorization contract binding mismatch")

    decision_path = _verify_binding(authorization["decision_record"], label="ADR-0025")
    config_path = _verify_binding(authorization["training_config"], label="training config")
    resolution_path = _verify_binding(contract["review_resolution"], label="review resolution")
    upstream_path = _verify_binding(contract["upstream_handoff"], label="T30 rebaseline")
    split_path = _verify_binding(contract["source_split_manifest"], label="source split manifest")
    audited_path = _verify_binding(contract["audited_validation_manifest"], label="audited validation manifest")
    audited = verify_audited_derivative(audited_path)
    if audited["canonical_validation_derivative_sha256"] != contract["audited_validation_manifest"]["canonical_sha256"]:
        raise ValueError("audited validation canonical binding mismatch")
    if _display(decision_path) != contract["decision_record"]:
        raise ValueError("decision-record path mismatch")
    if _display(config_path) != contract["training"]["config"]:
        raise ValueError("training-config path mismatch")
    return contract, {
        "authorization": authorization_path,
        "decision": decision_path,
        "config": config_path,
        "resolution": resolution_path,
        "upstream": upstream_path,
        "split_manifest": split_path,
        "audited_manifest": audited_path,
    }


def _xywh_to_xyxy(values: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, width, height = (float(value) for value in values)
    return (x - width / 2, y - height / 2, x + width / 2, y + height / 2)


def _xyxy_to_xywh(values: Sequence[float]) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = (float(value) for value in values)
    return ((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)


def _iou(left: Sequence[float], right: Sequence[float]) -> float:
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (left[2] - left[0]) * (left[3] - left[1])
    right_area = (right[2] - right[0]) * (right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0


def _parse_label(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not raw.strip():
            continue
        fields = raw.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: expected five YOLO fields")
        values = [float(value) for value in fields]
        if not values[0].is_integer() or int(values[0]) not in (0, 1):
            raise ValueError(f"{path}:{line_number}: class outside v1 contract")
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path}:{line_number}: non-finite value")
        coordinates = values[1:]
        if not all(0.0 <= value <= 1.0 for value in coordinates):
            raise ValueError(f"{path}:{line_number}: coordinate outside [0,1]")
        if coordinates[2] <= 0 or coordinates[3] <= 0:
            raise ValueError(f"{path}:{line_number}: non-positive box")
        rows.append(
            {
                "class_id": int(values[0]),
                "coordinates": coordinates,
                "source_line": raw.strip(),
            }
        )
    return rows


def _decision_class(decision: str) -> int:
    if decision.endswith("_RIPE"):
        return 0
    if decision.endswith("_UNRIPE"):
        return 1
    raise ValueError(f"decision has no target class: {decision}")


def apply_confirmed_changes(
    source_label: Path,
    changes: Sequence[Mapping[str, Any]],
) -> tuple[str, list[dict[str, Any]]]:
    rows = _parse_label(source_label)
    applied = []
    relabeled_indexes: set[int] = set()
    for change in sorted(changes, key=lambda item: str(item["candidate_id"])):
        decision = str(change["final_decision"])
        target_class = _decision_class(decision)
        candidate_box = tuple(float(value) for value in change["bbox_xyxy_normalized"])
        if decision.startswith("CONFIRMED_RELABEL_"):
            source_class = 1 - target_class
            candidates = [
                (_iou(candidate_box, _xywh_to_xyxy(row["coordinates"])), index)
                for index, row in enumerate(rows)
                if row["class_id"] == source_class and index not in relabeled_indexes
            ]
            if not candidates:
                raise ValueError(f"no opposite-class label for {change['candidate_id']}")
            overlap, index = max(candidates)
            if overlap < 0.50:
                raise ValueError(f"relabel match below frozen IoU for {change['candidate_id']}")
            before = rows[index]["source_line"]
            rows[index]["class_id"] = target_class
            rows[index]["source_line"] = f"{target_class} " + " ".join(
                f"{value:.6f}" for value in rows[index]["coordinates"]
            )
            relabeled_indexes.add(index)
            applied.append(
                {
                    "candidate_id": change["candidate_id"],
                    "operation": "relabel",
                    "target_class_id": target_class,
                    "matched_iou": overlap,
                    "source_line_index": index,
                    "before": before,
                    "after": rows[index]["source_line"],
                }
            )
        elif decision.startswith("CONFIRMED_ADD_"):
            coordinates = _xyxy_to_xywh(candidate_box)
            if not all(0.0 <= value <= 1.0 for value in coordinates) or coordinates[2] <= 0 or coordinates[3] <= 0:
                raise ValueError(f"invalid addition box for {change['candidate_id']}")
            line = f"{target_class} " + " ".join(f"{value:.6f}" for value in coordinates)
            rows.append(
                {
                    "class_id": target_class,
                    "coordinates": list(coordinates),
                    "source_line": line,
                }
            )
            applied.append(
                {
                    "candidate_id": change["candidate_id"],
                    "operation": "add",
                    "target_class_id": target_class,
                    "after": line,
                }
            )
        else:
            raise ValueError(f"non-change decision reached materializer: {decision}")
    return "\n".join(row["source_line"] for row in rows) + "\n", applied


def _load_confirmed_changes(
    resolution_path: Path,
    candidate_manifest_path: Path,
) -> list[dict[str, Any]]:
    resolution = _load_object(resolution_path)
    if (
        resolution.get("kind") != "training_label_audit_resolution"
        or resolution.get("status") != "REVIEW_FINALIZED_LABELS_UNCHANGED"
        or resolution.get("labels_modified") is not False
        or resolution.get("training_started") is not False
        or resolution.get("held_out_test_accessed") is not False
    ):
        raise ValueError("review resolution is outside the frozen pre-materialization state")
    manifest_binding = resolution.get("bindings", {}).get("candidate_manifest", {})
    if (
        manifest_binding.get("path") != _display(candidate_manifest_path)
        or manifest_binding.get("size_bytes") != candidate_manifest_path.stat().st_size
        or manifest_binding.get("sha256") != _sha256(candidate_manifest_path)
    ):
        raise ValueError("resolution candidate-manifest binding mismatch")
    manifest = _load_object(candidate_manifest_path)
    candidates = {item["candidate_id"]: item for item in manifest["candidates"]}
    changes = []
    for row in resolution.get("resolutions", []):
        decision = str(row["final_decision"])
        if not decision.startswith("CONFIRMED_"):
            continue
        candidate = candidates.get(row["candidate_id"])
        if candidate is None:
            raise ValueError(f"resolved candidate missing from manifest: {row['candidate_id']}")
        for key in ("reason", "image", "label", "bbox_xyxy_normalized"):
            if row[key] != candidate[key]:
                raise ValueError(f"resolved candidate field changed: {row['candidate_id']} {key}")
        changes.append({**row, "source_label_sha256": candidate["label_sha256"]})
    if len(changes) != 13:
        raise ValueError("ADR-0025 requires exactly 13 consensus label changes")
    if sum(item["final_decision"].startswith("CONFIRMED_RELABEL_") for item in changes) != 5:
        raise ValueError("ADR-0025 requires exactly five relabels")
    if sum(item["final_decision"].startswith("CONFIRMED_ADD_") for item in changes) != 8:
        raise ValueError("ADR-0025 requires exactly eight additions")
    return changes


def materialize(contract_path: Path) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    contract, paths = _validate_contract(contract_path)
    derivative = contract["dataset_derivative"]
    output_dir = _resolve(Path(str(derivative["output_dir"]))).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite training derivative: {output_dir}")
    if derivative.get("test_split_present") is not False:
        raise ValueError("training derivative contract must omit test")

    resolution = _load_object(paths["resolution"])
    candidate_manifest_path = _resolve(
        Path(str(resolution["bindings"]["candidate_manifest"]["path"]))
    ).resolve()
    changes = _load_confirmed_changes(paths["resolution"], candidate_manifest_path)
    changes_by_label: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for change in changes:
        changes_by_label[str(change["label"])].append(change)

    split_manifest = _load_object(paths["split_manifest"])
    if split_manifest.get("content_binding", {}).get("canonical_split_sha256", {}).get("train") != contract["source_split_manifest"]["canonical_train_sha256"]:
        raise ValueError("source train canonical digest mismatch")
    train_items = split_manifest.get("splits", {}).get("train")
    if not isinstance(train_items, list) or len(train_items) != int(derivative["train_image_count"]):
        raise ValueError("source training count mismatch")
    source_root = paths["split_manifest"].parent

    audited_manifest = _load_object(paths["audited_manifest"])
    val_items = audited_manifest.get("entries")
    if not isinstance(val_items, list) or len(val_items) != int(derivative["validation_image_count"]):
        raise ValueError("audited validation count mismatch")
    audited_root = paths["audited_manifest"].parent

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".train-audit-staging-", dir=output_dir.parent) as temporary:
        staging = Path(temporary)
        for split in ("train", "val"):
            (staging / "images" / split).mkdir(parents=True)
            (staging / "labels" / split).mkdir(parents=True)

        entries: list[dict[str, Any]] = []
        original_counts: Counter[int] = Counter()
        corrected_counts: Counter[int] = Counter()
        image_modes: Counter[str] = Counter()
        applied_changes = []
        seen_change_labels = set()
        for source_item in sorted(train_items, key=lambda item: str(item["output_image"])):
            image_part = _safe_split_path(str(source_item["output_image"]), "images", "train")
            label_part = _safe_split_path(str(source_item["output_label"]), "labels", "train")
            if image_part.stem != label_part.stem:
                raise ValueError(f"train image/label mismatch: {image_part}")
            source_image = source_root / image_part
            source_label = source_root / label_part
            for source, hash_key, size_key in (
                (source_image, "output_image_sha256", "output_image_size_bytes"),
                (source_label, "output_label_sha256", "output_label_size_bytes"),
            ):
                if not source.is_file() or _sha256(source) != source_item.get(hash_key) or source.stat().st_size != source_item.get(size_key):
                    raise ValueError(f"registered train file changed: {source}")
            before_counts = _count_classes(source_label)
            original_counts.update(before_counts)
            derived_image = staging / image_part
            derived_label = staging / label_part
            mode = _link_or_copy(source_image, derived_image)
            image_modes[mode] += 1
            label_key = label_part.as_posix()
            label_changes = changes_by_label.get(label_key, [])
            if label_changes:
                for change in label_changes:
                    if change["source_label_sha256"] != _sha256(source_label):
                        raise ValueError(f"candidate source label changed: {change['candidate_id']}")
                text, applied = apply_confirmed_changes(source_label, label_changes)
                derived_label.write_text(text, encoding="utf-8")
                for item in applied:
                    applied_changes.append({"label": label_key, **item})
                seen_change_labels.add(label_key)
            else:
                shutil.copy2(source_label, derived_label)
            after_counts = _count_classes(derived_label)
            corrected_counts.update(after_counts)
            entries.append(
                {
                    "role": "train_label_audited",
                    "source_image": image_part.as_posix(),
                    "source_image_sha256": _sha256(source_image),
                    "source_label": label_part.as_posix(),
                    "source_label_sha256": _sha256(source_label),
                    "derived_image": image_part.as_posix(),
                    "derived_image_sha256": _sha256(derived_image),
                    "derived_image_size_bytes": derived_image.stat().st_size,
                    "derived_label": label_part.as_posix(),
                    "derived_label_sha256": _sha256(derived_label),
                    "derived_label_size_bytes": derived_label.stat().st_size,
                    "class_counts": {"0": after_counts[0], "1": after_counts[1]},
                    "image_materialization": mode,
                    "label_changed": bool(label_changes),
                }
            )
        if seen_change_labels != set(changes_by_label):
            raise ValueError("one or more approved label changes were not materialized")
        if len(applied_changes) != len(changes):
            raise ValueError("applied change count mismatch")
        if {"0": original_counts[0], "1": original_counts[1]} != derivative["original_train_class_counts"]:
            raise ValueError("original training class totals changed")
        if {"0": corrected_counts[0], "1": corrected_counts[1]} != derivative["corrected_train_class_counts"]:
            raise ValueError("corrected training class totals differ from contract")

        for item in sorted(val_items, key=lambda row: str(row["image"])):
            image_part = _safe_split_path(str(item["image"]), "images", "val")
            label_part = _safe_split_path(str(item["audited_label"]), "labels", "val")
            source_image = source_root / image_part
            source_label = audited_root / label_part
            if not source_image.is_file() or _sha256(source_image) != item["source_image_sha256"] or source_image.stat().st_size != item["source_image_size_bytes"]:
                raise ValueError(f"registered validation image changed: {source_image}")
            if not source_label.is_file() or _sha256(source_label) != item["audited_label_sha256"] or source_label.stat().st_size != item["audited_label_size_bytes"]:
                raise ValueError(f"audited validation label changed: {source_label}")
            counts = _count_classes(source_label)
            derived_image = staging / image_part
            derived_label = staging / label_part
            mode = _link_or_copy(source_image, derived_image)
            image_modes[mode] += 1
            shutil.copy2(source_label, derived_label)
            entries.append(
                {
                    "role": "audited_validation",
                    "source_image": image_part.as_posix(),
                    "source_image_sha256": _sha256(source_image),
                    "source_label": label_part.as_posix(),
                    "source_label_sha256": _sha256(source_label),
                    "derived_image": image_part.as_posix(),
                    "derived_image_sha256": _sha256(derived_image),
                    "derived_image_size_bytes": derived_image.stat().st_size,
                    "derived_label": label_part.as_posix(),
                    "derived_label_sha256": _sha256(derived_label),
                    "derived_label_size_bytes": derived_label.stat().st_size,
                    "class_counts": {"0": counts[0], "1": counts[1]},
                    "image_materialization": mode,
                    "label_changed": False,
                }
            )

        dataset_yaml = staging / "dataset.yaml"
        dataset_yaml.write_text(
            "# ADR-0025 consensus-corrected train labels; no test split.\n"
            "train: images/train\n"
            "val: images/val\n"
            "names:\n"
            "  0: ripe\n"
            "  1: unripe\n",
            encoding="utf-8",
        )
        canonical = _canonical_sha256(_canonical_rows(entries))
        manifest = {
            "schema_version": 1,
            "kind": "training_label_audit_derivative",
            "variant": contract["variant"],
            "bindings": {
                "contract": {"path": _display(contract_path), "size_bytes": contract_path.stat().st_size, "sha256": _sha256(contract_path)},
                "authorization": {"path": _display(paths["authorization"]), "size_bytes": paths["authorization"].stat().st_size, "sha256": _sha256(paths["authorization"])},
                "decision_record": {"path": _display(paths["decision"]), "size_bytes": paths["decision"].stat().st_size, "sha256": _sha256(paths["decision"])},
                "review_resolution": {"path": _display(paths["resolution"]), "size_bytes": paths["resolution"].stat().st_size, "sha256": _sha256(paths["resolution"])},
                "source_split_manifest": {"path": _display(paths["split_manifest"]), "size_bytes": paths["split_manifest"].stat().st_size, "sha256": _sha256(paths["split_manifest"])},
                "audited_validation_manifest": {"path": _display(paths["audited_manifest"]), "size_bytes": paths["audited_manifest"].stat().st_size, "sha256": _sha256(paths["audited_manifest"])},
                "materializer": {"path": _display(Path(__file__)), "size_bytes": Path(__file__).stat().st_size, "sha256": _sha256(Path(__file__))},
            },
            "dataset_yaml": {"path": "dataset.yaml", "size_bytes": dataset_yaml.stat().st_size, "sha256": _sha256(dataset_yaml)},
            "counts": {
                "train_images": len(train_items),
                "audited_validation_images": len(val_items),
                "approved_label_changes": len(applied_changes),
                "relabels": sum(item["operation"] == "relabel" for item in applied_changes),
                "additions": sum(item["operation"] == "add" for item in applied_changes),
                "original_train_classes": {"0": original_counts[0], "1": original_counts[1]},
                "corrected_train_classes": {"0": corrected_counts[0], "1": corrected_counts[1]},
            },
            "applied_changes": sorted(applied_changes, key=lambda item: item["candidate_id"]),
            "image_materialization_counts": dict(sorted(image_modes.items())),
            "entries": entries,
            "content_binding": {
                "algorithm": "sha256",
                "canonicalization": "json-sort-keys-utf8-v1",
                "canonical_training_derivative_sha256": canonical,
            },
            "original_files_modified": False,
            "test_split_present": False,
            "test_split_accessed": False,
            "training_authorized": True,
        }
        (staging / "training_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        staging.replace(output_dir)
    return manifest


def verify_training_derivative(
    manifest_path: Path,
    contract_path: Path = DEFAULT_CONTRACT,
) -> dict[str, Any]:
    manifest_path = _resolve(manifest_path).resolve()
    contract_path = _resolve(contract_path).resolve()
    contract, _ = _validate_contract(contract_path)
    manifest = _load_object(manifest_path)
    if manifest.get("kind") != "training_label_audit_derivative":
        raise ValueError("unexpected training-label derivative kind")
    if manifest.get("variant") != contract.get("variant"):
        raise ValueError("training-label derivative variant mismatch")
    if manifest.get("test_split_present") is not False or manifest.get("test_split_accessed") is not False:
        raise ValueError("training-label derivative does not preserve the test seal")
    if (manifest_path.parent / "images" / "test").exists() or (manifest_path.parent / "labels" / "test").exists():
        raise ValueError("training-label derivative unexpectedly contains test files")
    binding = manifest.get("bindings", {}).get("contract", {})
    if binding.get("sha256") != _sha256(contract_path) or binding.get("size_bytes") != contract_path.stat().st_size:
        raise ValueError("training-label derivative contract binding mismatch")
    dataset_yaml = manifest_path.parent / str(manifest["dataset_yaml"]["path"])
    if not dataset_yaml.is_file() or _sha256(dataset_yaml) != manifest["dataset_yaml"]["sha256"] or dataset_yaml.stat().st_size != manifest["dataset_yaml"]["size_bytes"]:
        raise ValueError("training-label derivative dataset YAML changed")
    if "test:" in dataset_yaml.read_text(encoding="utf-8"):
        raise ValueError("training-label derivative YAML contains a test key")
    entries = manifest.get("entries")
    expected = int(contract["dataset_derivative"]["train_image_count"]) + int(contract["dataset_derivative"]["validation_image_count"])
    if not isinstance(entries, list) or len(entries) != expected:
        raise ValueError("training-label derivative entry count mismatch")
    for item in entries:
        for key, hash_key, size_key in (
            ("derived_image", "derived_image_sha256", "derived_image_size_bytes"),
            ("derived_label", "derived_label_sha256", "derived_label_size_bytes"),
        ):
            relative = PurePosixPath(str(item[key]))
            if relative.is_absolute() or ".." in relative.parts or "test" in relative.parts:
                raise ValueError(f"unsafe derivative path: {relative}")
            path = manifest_path.parent / relative
            if not path.is_file() or _sha256(path) != item[hash_key] or path.stat().st_size != item[size_key]:
                raise ValueError(f"training-label derivative content changed: {path}")
        _count_classes(manifest_path.parent / str(item["derived_label"]))
    canonical = _canonical_sha256(_canonical_rows(entries))
    if canonical != manifest.get("content_binding", {}).get("canonical_training_derivative_sha256"):
        raise ValueError("training-label derivative canonical digest mismatch")
    counts = manifest.get("counts", {})
    if counts.get("approved_label_changes") != 13 or counts.get("relabels") != 5 or counts.get("additions") != 8:
        raise ValueError("training-label derivative change counts mismatch")
    return {
        "valid": True,
        "variant": manifest["variant"],
        "entry_count": len(entries),
        "approved_label_changes": 13,
        "canonical_training_derivative_sha256": canonical,
        "test_split_accessed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--verify-only", action="store_true")
    arguments = parser.parse_args(argv)
    contract_path = _resolve(arguments.contract).resolve()
    contract = _load_object(contract_path)
    manifest_path = _resolve(Path(str(contract["dataset_derivative"]["manifest"]))).resolve()
    if not arguments.verify_only:
        materialize(contract_path)
    result = verify_training_derivative(manifest_path, contract_path)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
