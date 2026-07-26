#!/usr/bin/env python3
"""Materialize a hash-bound, validation-only audited-label derivative."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import tempfile
from typing import Any, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESOLUTION = Path(
    "artifacts/perception/label_audit/yolo11s_val_threshold_031_resolution_v1.json"
)
DEFAULT_APPROVAL = Path(
    "artifacts/perception/label_audit/yolo11s_val_threshold_031_box_approval_v1.json"
)
DEFAULT_SPLIT_MANIFEST = Path("data/processed/zenodo_6126677/split_manifest.json")
DEFAULT_OUTPUT = Path("data/processed/zenodo_6126677_val_audit_v1")
CLASS_IDS = {"ripe": 0, "unripe": 1}


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _display_path(path: Path) -> str:
    try:
        return path.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: object) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _validate_bbox(value: object) -> tuple[float, float, float, float]:
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("approved box must contain four coordinates")
    bbox = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in bbox):
        raise ValueError("approved box coordinates must be finite")
    x1, y1, x2, y2 = bbox
    if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
        raise ValueError("approved box must have positive normalized area")
    return bbox


def _xyxy_to_yolo(bbox: Sequence[float]) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = bbox
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1)


def _yolo_to_xyxy(values: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, width, height = values
    return _validate_bbox(
        [
            max(0.0, x - width / 2.0),
            max(0.0, y - height / 2.0),
            min(1.0, x + width / 2.0),
            min(1.0, y + height / 2.0),
        ]
    )


def _iou(left: Sequence[float], right: Sequence[float]) -> float:
    x1 = max(left[0], right[0])
    y1 = max(left[1], right[1])
    x2 = min(left[2], right[2])
    y2 = min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (left[2] - left[0]) * (left[3] - left[1])
    right_area = (right[2] - right[0]) * (right[3] - right[1])
    return intersection / (left_area + right_area - intersection)


def _parse_label(path: Path) -> list[tuple[int, tuple[float, float, float, float]]]:
    result = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not raw.strip():
            continue
        fields = raw.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: expected five YOLO fields")
        try:
            class_value = float(fields[0])
            values = tuple(float(item) for item in fields[1:])
        except ValueError as error:
            raise ValueError(f"{path}:{line_number}: invalid numeric field") from error
        if not class_value.is_integer() or int(class_value) not in (0, 1):
            raise ValueError(f"{path}:{line_number}: class outside v1 contract")
        if not all(math.isfinite(item) and 0.0 <= item <= 1.0 for item in values):
            raise ValueError(f"{path}:{line_number}: invalid normalized coordinate")
        result.append((int(class_value), _yolo_to_xyxy(values)))
    return result


def _safe_val_path(relative: str, kind: str) -> PurePosixPath:
    value = PurePosixPath(relative)
    expected_parent = PurePosixPath(kind, "val")
    expected_suffixes = {".jpg", ".jpeg", ".png"} if kind == "images" else {".txt"}
    if value.parent != expected_parent or value.suffix.lower() not in expected_suffixes:
        raise ValueError(f"unsafe validation {kind} path: {relative}")
    return value


def _validated_inputs(
    resolution_path: Path, approval_path: Path, split_manifest_path: Path
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], list[dict[str, Any]]]:
    resolution = _load_object(resolution_path)
    approval = _load_object(approval_path)
    split_manifest = _load_object(split_manifest_path)
    if resolution.get("kind") != "validation_label_audit_resolution":
        raise ValueError("unexpected resolution kind")
    if resolution.get("status") != "READY_FOR_MANUAL_BOX_ANNOTATION":
        raise ValueError("resolution is not ready for box approval")
    if resolution.get("labels_modified") is not False or resolution.get("test_split_accessed") is not False:
        raise ValueError("resolution provenance is not validation-only and unmodified")
    worklist = resolution.get("manual_annotation_worklist")
    if not isinstance(worklist, list) or not worklist:
        raise ValueError("resolution contains no manual annotation worklist")

    if approval.get("kind") != "validation_label_audit_human_box_approval":
        raise ValueError("unexpected approval kind")
    binding = approval.get("resolution_receipt")
    if not isinstance(binding, dict):
        raise ValueError("approval has no resolution binding")
    if binding.get("sha256") != _sha256(resolution_path) or binding.get("size_bytes") != resolution_path.stat().st_size:
        raise ValueError("approval resolution binding mismatch")
    if approval.get("approval_mode") != "APPROVE_PROPOSED_BOX_AS_FINAL":
        raise ValueError("approval does not authorize the proposed boxes as final")
    if approval.get("original_labels_modified") is not False or approval.get("test_split_accessed") is not False:
        raise ValueError("approval provenance is not safe")

    approved_ids = approval.get("approved_candidate_ids")
    expected_ids = [str(item.get("candidate_id")) for item in worklist]
    if not isinstance(approved_ids, list) or set(approved_ids) != set(expected_ids):
        raise ValueError("approval must cover exactly the resolution worklist")
    if len(approved_ids) != len(set(approved_ids)) or approval.get("approved_count") != len(worklist):
        raise ValueError("approval contains duplicate IDs or an incorrect count")

    val_items = split_manifest.get("splits", {}).get("val")
    if not isinstance(val_items, list) or not val_items:
        raise ValueError("original manifest has no validation split")
    return resolution, approval, split_manifest, val_items


def _canonical_rows(entries: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "image": item["image"],
            "source_image_sha256": item["source_image_sha256"],
            "source_label_sha256": item["source_label_sha256"],
            "audited_label_sha256": item["audited_label_sha256"],
            "class_counts": item["class_counts"],
            "additions": item["additions"],
        }
        for item in sorted(entries, key=lambda value: str(value["image"]))
    ]


def materialize(
    resolution_path: Path,
    approval_path: Path,
    split_manifest_path: Path,
    output_dir: Path,
) -> dict[str, Any]:
    resolution_path = _resolve(resolution_path).resolve()
    approval_path = _resolve(approval_path).resolve()
    split_manifest_path = _resolve(split_manifest_path).resolve()
    output_dir = _resolve(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite audited derivative: {output_dir}")

    resolution, approval, split_manifest, val_items = _validated_inputs(
        resolution_path, approval_path, split_manifest_path
    )
    source_root = split_manifest_path.parent
    additions_by_label: dict[str, list[dict[str, Any]]] = {}
    for item in resolution["manual_annotation_worklist"]:
        candidate_id = str(item["candidate_id"])
        image = str(item["image"])
        label = str(item["label"])
        image_path = _safe_val_path(image, "images")
        label_path = _safe_val_path(label, "labels")
        if image_path.stem != label_path.stem:
            raise ValueError(f"image/label mismatch for {candidate_id}")
        class_name = str(item["class_name"])
        if class_name not in CLASS_IDS:
            raise ValueError(f"unsupported class for {candidate_id}")
        additions_by_label.setdefault(label, []).append(
            {
                "candidate_id": candidate_id,
                "class_id": CLASS_IDS[class_name],
                "class_name": class_name,
                "bbox_xyxy_normalized": list(
                    _validate_bbox(item["candidate_box_for_review_only"])
                ),
            }
        )

    manifest_labels = {str(item.get("output_label")) for item in val_items}
    if not set(additions_by_label).issubset(manifest_labels):
        raise ValueError("approved worklist refers outside the registered validation labels")

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".val-audit-staging-", dir=output_dir.parent) as temp:
        staging = Path(temp)
        (staging / "labels" / "val").mkdir(parents=True)
        entries: list[dict[str, Any]] = []
        total_counts: Counter[int] = Counter()
        added_counts: Counter[int] = Counter()
        for source_item in sorted(val_items, key=lambda value: str(value["output_image"])):
            image_relative = str(source_item["output_image"])
            label_relative = str(source_item["output_label"])
            image_part = _safe_val_path(image_relative, "images")
            label_part = _safe_val_path(label_relative, "labels")
            if image_part.stem != label_part.stem:
                raise ValueError(f"manifest pair mismatch: {image_relative}")
            source_image = source_root / image_part
            source_label = source_root / label_part
            for path, hash_key, size_key in (
                (source_image, "output_image_sha256", "output_image_size_bytes"),
                (source_label, "output_label_sha256", "output_label_size_bytes"),
            ):
                if not path.is_file():
                    raise ValueError(f"missing registered validation file: {path}")
                if _sha256(path) != source_item.get(hash_key) or path.stat().st_size != source_item.get(size_key):
                    raise ValueError(f"registered validation file changed: {path}")

            existing = _parse_label(source_label)
            additions = additions_by_label.get(label_relative, [])
            for addition in additions:
                approved_box = addition["bbox_xyxy_normalized"]
                maximum_iou = max(
                    (_iou(approved_box, existing_box) for _, existing_box in existing),
                    default=0.0,
                )
                if maximum_iou >= 0.5:
                    raise ValueError(
                        f"approved candidate overlaps an existing label at IoU {maximum_iou:.6f}: "
                        f"{addition['candidate_id']}"
                    )

            original_text = source_label.read_text(encoding="utf-8-sig")
            audited_text = original_text
            if additions:
                if audited_text and not audited_text.endswith("\n"):
                    audited_text += "\n"
                for addition in additions:
                    yolo = _xyxy_to_yolo(addition["bbox_xyxy_normalized"])
                    audited_text += (
                        f"{addition['class_id']} "
                        + " ".join(f"{value:.10f}" for value in yolo)
                        + "\n"
                    )
                    added_counts[addition["class_id"]] += 1
            audited_label = staging / label_part
            audited_label.write_text(audited_text, encoding="utf-8")
            audited_records = _parse_label(audited_label)
            counts = Counter(class_id for class_id, _ in audited_records)
            total_counts.update(counts)
            entries.append(
                {
                    "image": image_relative,
                    "source_image_sha256": _sha256(source_image),
                    "source_image_size_bytes": source_image.stat().st_size,
                    "source_label": label_relative,
                    "source_label_sha256": _sha256(source_label),
                    "source_label_size_bytes": source_label.stat().st_size,
                    "audited_label": label_relative,
                    "audited_label_sha256": _sha256(audited_label),
                    "audited_label_size_bytes": audited_label.stat().st_size,
                    "class_counts": {"0": counts[0], "1": counts[1]},
                    "additions": additions,
                }
            )

        if len(entries) != len(val_items):
            raise ValueError("not every validation label was materialized")
        canonical_digest = _canonical_sha256(_canonical_rows(entries))
        manifest = {
            "schema_version": 1,
            "kind": "validation_label_audit_derivative",
            "dataset_id": str(split_manifest.get("dataset_id", "")),
            "scope": "validation labels only; source validation images remain immutable",
            "source_split_manifest": {
                "path": _display_path(split_manifest_path),
                "sha256": _sha256(split_manifest_path),
                "canonical_val_sha256": split_manifest["content_binding"][
                    "canonical_split_sha256"
                ]["val"],
            },
            "resolution_receipt": {
                "path": _display_path(resolution_path),
                "sha256": _sha256(resolution_path),
            },
            "human_box_approval": {
                "path": _display_path(approval_path),
                "sha256": _sha256(approval_path),
            },
            "validation_sample_count": len(entries),
            "modified_label_count": len(additions_by_label),
            "added_instance_counts": {"0": added_counts[0], "1": added_counts[1]},
            "audited_class_instance_counts": {"0": total_counts[0], "1": total_counts[1]},
            "entries": entries,
            "content_binding": {
                "algorithm": "sha256",
                "canonicalization": "json-sort-keys-utf8-v1",
                "canonical_validation_derivative_sha256": canonical_digest,
            },
            "original_labels_modified": False,
            "test_split_accessed": False,
            "training_authorized": False,
        }
        (staging / "audit_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        staging.replace(output_dir)
    return manifest


def verify_audited_derivative(manifest_path: Path) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    manifest = _load_object(manifest_path)
    if manifest.get("kind") != "validation_label_audit_derivative":
        raise ValueError("unsupported audited derivative manifest")
    if manifest.get("test_split_accessed") is not False:
        raise ValueError("audited derivative does not preserve the test seal")
    entries = manifest.get("entries")
    if not isinstance(entries, list) or len(entries) != manifest.get("validation_sample_count"):
        raise ValueError("audited derivative entry count mismatch")
    root = manifest_path.parent
    for item in entries:
        if not isinstance(item, dict):
            raise ValueError("audited derivative entry must be an object")
        label_part = _safe_val_path(str(item["audited_label"]), "labels")
        label_path = root / label_part
        if not label_path.is_file():
            raise ValueError(f"missing audited label: {label_path}")
        if _sha256(label_path) != item.get("audited_label_sha256") or label_path.stat().st_size != item.get("audited_label_size_bytes"):
            raise ValueError(f"audited label content mismatch: {label_path}")
        _parse_label(label_path)
    actual = _canonical_sha256(_canonical_rows(entries))
    expected = manifest.get("content_binding", {}).get(
        "canonical_validation_derivative_sha256"
    )
    if actual != expected:
        raise ValueError("audited derivative canonical digest mismatch")
    return {
        "valid": True,
        "validation_sample_count": len(entries),
        "canonical_validation_derivative_sha256": actual,
        "test_split_accessed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resolution", type=Path, default=DEFAULT_RESOLUTION)
    parser.add_argument("--approval", type=Path, default=DEFAULT_APPROVAL)
    parser.add_argument("--split-manifest", type=Path, default=DEFAULT_SPLIT_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    result = materialize(
        arguments.resolution,
        arguments.approval,
        arguments.split_manifest,
        arguments.output_dir,
    )
    verification = verify_audited_derivative(_resolve(arguments.output_dir) / "audit_manifest.json")
    print(json.dumps({"manifest": result, "verification": verification}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
