#!/usr/bin/env python3
"""Freeze duplicate-image and missing-annotation exclusions before splitting."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Iterable, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = (
    REPOSITORY_ROOT
    / "data"
    / "raw"
    / "zenodo_6126677"
    / "extracted"
    / "strawberries"
)
DEFAULT_GROUPS = (
    REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677_groups.csv"
)
DEFAULT_GROUP_AUDIT = (
    REPOSITORY_ROOT / "artifacts" / "data" / "zenodo_6126677_group_audit.json"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677_exclusions.csv"
)
DEFAULT_AUDIT = (
    REPOSITORY_ROOT / "artifacts" / "data" / "zenodo_6126677_curation_audit.json"
)
V1_CLASS_IDS = frozenset({0, 1})


class UnionFind:
    def __init__(self, values: Iterable[str]):
        self.parent = {value: value for value in values}

    def find(self, value: str) -> str:
        if self.parent[value] != value:
            self.parent[value] = self.find(self.parent[value])
        return self.parent[value]

    def union(self, left: str, right: str) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_groups(path: Path) -> Mapping[str, str]:
    groups: dict[str, str] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not {"image", "group_id"}.issubset(
            reader.fieldnames
        ):
            raise ValueError("groups CSV must have image and group_id columns")
        for row in reader:
            image = row["image"].strip().replace("\\", "/")
            group_id = row["group_id"].strip()
            if not image or not group_id or image in groups:
                raise ValueError(f"invalid or duplicate groups row for {image!r}")
            groups[image] = group_id
    return groups


def _read_v1_boxes(path: Path) -> list[Mapping[str, object]]:
    boxes: list[Mapping[str, object]] = []
    with path.open("r", encoding="utf-8-sig") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            fields = raw_line.strip().split()
            if not fields:
                continue
            if len(fields) != 5:
                raise ValueError(f"{path}:{line_number}: expected five fields")
            values = [float(field) for field in fields]
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"{path}:{line_number}: non-finite field")
            class_id = int(values[0])
            if values[0] != class_id or class_id < 0:
                raise ValueError(f"{path}:{line_number}: invalid class id")
            if class_id not in V1_CLASS_IDS:
                continue
            x, y, width, height = values[1:]
            boxes.append(
                {
                    "line": line_number,
                    "class_id": class_id,
                    "box": (x, y, width, height),
                    "positive_area": width > 0.0 and height > 0.0,
                }
            )
    return boxes


def _corners(box: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, width, height = box
    return (
        x - width / 2.0,
        y - height / 2.0,
        x + width / 2.0,
        y + height / 2.0,
    )


def _iou(left: Sequence[float], right: Sequence[float]) -> float:
    lx1, ly1, lx2, ly2 = _corners(left)
    rx1, ry1, rx2, ry2 = _corners(right)
    intersection = max(0.0, min(lx2, rx2) - max(lx1, rx1)) * max(
        0.0, min(ly2, ry2) - max(ly1, ry1)
    )
    left_area = max(0.0, lx2 - lx1) * max(0.0, ly2 - ly1)
    right_area = max(0.0, rx2 - rx1) * max(0.0, ry2 - ry1)
    union = left_area + right_area - intersection
    return 0.0 if union <= 0.0 else intersection / union


def _compare_labels(
    left: Sequence[Mapping[str, object]],
    right: Sequence[Mapping[str, object]],
    iou_min: float,
) -> Mapping[str, object]:
    candidates = sorted(
        (
            (_iou(a["box"], b["box"]), left_index, right_index)
            for left_index, a in enumerate(left)
            for right_index, b in enumerate(right)
        ),
        reverse=True,
    )
    used_left: set[int] = set()
    used_right: set[int] = set()
    matches = []
    for overlap, left_index, right_index in candidates:
        if overlap < iou_min:
            break
        if left_index in used_left or right_index in used_right:
            continue
        used_left.add(left_index)
        used_right.add(right_index)
        matches.append(
            {
                "left_line": left[left_index]["line"],
                "right_line": right[right_index]["line"],
                "left_class_id": left[left_index]["class_id"],
                "right_class_id": right[right_index]["class_id"],
                "iou": overlap,
                "same_class": (
                    left[left_index]["class_id"] == right[right_index]["class_id"]
                ),
            }
        )
    unmatched_left = [
        left[index]["line"] for index in range(len(left)) if index not in used_left
    ]
    unmatched_right = [
        right[index]["line"] for index in range(len(right)) if index not in used_right
    ]
    class_conflicts = [item for item in matches if not item["same_class"]]
    return {
        "matched_count": len(matches),
        "same_class_match_count": len(matches) - len(class_conflicts),
        "class_conflict_count": len(class_conflicts),
        "unmatched_left_lines": unmatched_left,
        "unmatched_right_lines": unmatched_right,
        "requires_review": bool(
            class_conflicts or unmatched_left or unmatched_right
        ),
        "matches": matches,
    }


def _write_exclusions(
    path: Path,
    rows: Sequence[tuple[str, str, str]],
) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("image", "reason", "representative"))
        writer.writerows(rows)
    return _sha256(path)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--groups-csv", type=Path, default=DEFAULT_GROUPS)
    parser.add_argument("--group-audit", type=Path, default=DEFAULT_GROUP_AUDIT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--audit-json", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--duplicate-good-min", type=int, default=1000)
    parser.add_argument("--duplicate-inlier-ratio-min", type=float, default=0.99)
    parser.add_argument("--label-iou-min", type=float, default=0.5)
    parser.add_argument("--expected-images", type=int, default=813)
    parser.add_argument("--expected-duplicate-components", type=int, default=65)
    parser.add_argument("--expected-empty-labels", type=int, default=11)
    parser.add_argument("--expected-zero-area-v1", type=int, default=1)
    parser.add_argument("--expected-class-conflict-components", type=int, default=5)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    for path in (args.source, args.groups_csv, args.group_audit):
        if not path.exists():
            raise FileNotFoundError(path)
    if (args.output.exists() or args.audit_json.exists()) and not args.force:
        raise FileExistsError("curation output already exists; pass --force after review")
    if args.duplicate_good_min < 4 or not 0.0 <= args.duplicate_inlier_ratio_min <= 1.0:
        raise ValueError("invalid duplicate thresholds")
    if not 0.0 < args.label_iou_min <= 1.0:
        raise ValueError("--label-iou-min must be in (0, 1]")

    group_audit = json.loads(args.group_audit.read_text(encoding="utf-8"))
    groups = _read_groups(args.groups_csv)
    metadata = {item["path"]: item for item in group_audit["input"]["images"]}
    if set(metadata) != set(groups):
        raise ValueError("group audit and groups CSV do not cover the same images")
    if len(metadata) != args.expected_images:
        raise ValueError(
            f"expected {args.expected_images} images, found {len(metadata)}"
        )

    duplicate_pairs = []
    union_find = UnionFind(metadata)
    for pair in group_audit["pairs"]:
        if (
            pair["homography_valid"]
            and pair["good_matches"] >= args.duplicate_good_min
            and pair["inlier_ratio"] >= args.duplicate_inlier_ratio_min
        ):
            left = pair["image_a"]
            right = pair["image_b"]
            left_meta = metadata[left]
            right_meta = metadata[right]
            if (left_meta["width"], left_meta["height"]) != (
                right_meta["width"],
                right_meta["height"],
            ):
                continue
            if groups[left] != groups[right]:
                raise ValueError(f"near-identical pair crosses scene groups: {left}, {right}")
            union_find.union(left, right)
            duplicate_pairs.append(pair)
    duplicate_members: dict[str, list[str]] = defaultdict(list)
    involved = {
        path
        for pair in duplicate_pairs
        for path in (pair["image_a"], pair["image_b"])
    }
    for path in sorted(involved, key=str.casefold):
        duplicate_members[union_find.find(path)].append(path)
    components = sorted(
        (sorted(members, key=str.casefold) for members in duplicate_members.values()),
        key=lambda members: members[0].casefold(),
    )
    if len(components) != args.expected_duplicate_components:
        raise ValueError(
            f"expected {args.expected_duplicate_components} duplicate components, "
            f"found {len(components)}"
        )

    exclusion_rows: list[tuple[str, str, str]] = []
    component_reports = []
    for members in components:
        representative = max(
            members,
            key=lambda path: (
                (args.source / path).stat().st_size,
                metadata[path]["sha256"],
                path,
            ),
        )
        excluded = [path for path in members if path != representative]
        # The released archive currently has two encodings per component.  The
        # generic component logic remains deterministic if a third appears.
        label_comparisons = []
        representative_boxes = _read_v1_boxes(
            (args.source / representative).with_suffix(".txt")
        )
        for path in excluded:
            boxes = _read_v1_boxes((args.source / path).with_suffix(".txt"))
            label_comparisons.append(
                {
                    "image": path,
                    "image_bytes": (args.source / path).stat().st_size,
                    "label_sha256": _sha256(
                        (args.source / path).with_suffix(".txt")
                    ),
                    "comparison_to_representative": _compare_labels(
                        boxes, representative_boxes, args.label_iou_min
                    ),
                }
            )
        has_class_conflict = any(
            item["comparison_to_representative"]["class_conflict_count"] > 0
            for item in label_comparisons
        )
        if has_class_conflict:
            # Maturity is the task label, so contradictory class supervision is
            # not resolved by guessing which source annotation is newer.  Drop
            # the complete near-identical component before any split is frozen.
            for path in members:
                exclusion_rows.append((path, "maturity_class_conflict", ""))
        else:
            for path in excluded:
                exclusion_rows.append(
                    (path, "near_identical_jpeg_reencode", representative)
                )
        component_reports.append(
            {
                "group_id": groups[representative],
                "members": members,
                "representative": representative,
                "representative_image_bytes": (
                    args.source / representative
                ).stat().st_size,
                "representative_label_sha256": _sha256(
                    (args.source / representative).with_suffix(".txt")
                ),
                "representative_v1_class_counts": dict(
                    sorted(
                        Counter(
                            int(box["class_id"]) for box in representative_boxes
                        ).items()
                    )
                ),
                "representative_retained": not has_class_conflict,
                "excluded": label_comparisons,
            }
        )

    empty_labels = []
    zero_area = []
    for image_path in sorted(metadata, key=str.casefold):
        label_path = (args.source / image_path).with_suffix(".txt")
        if not label_path.read_text(encoding="utf-8-sig").strip():
            empty_labels.append(image_path)
        for box in _read_v1_boxes(label_path):
            if not box["positive_area"]:
                zero_area.append(
                    {
                        "image": image_path,
                        "label": label_path.relative_to(args.source).as_posix(),
                        "line": box["line"],
                        "class_id": box["class_id"],
                        "box": list(box["box"]),
                        "action": "drop_zero_area_v1_box_during_materialization",
                    }
                )
    if len(empty_labels) != args.expected_empty_labels:
        raise ValueError(
            f"expected {args.expected_empty_labels} empty labels, found {len(empty_labels)}"
        )
    if len(zero_area) != args.expected_zero_area_v1:
        raise ValueError(
            f"expected {args.expected_zero_area_v1} zero-area v1 boxes, "
            f"found {len(zero_area)}"
        )
    excluded_images = {row[0] for row in exclusion_rows}
    overlap = sorted(excluded_images.intersection(empty_labels))
    if overlap:
        raise ValueError(f"empty-label exclusions overlap duplicates: {overlap}")
    for image_path in empty_labels:
        exclusion_rows.append((image_path, "missing_annotation", ""))
    exclusion_rows.sort(key=lambda row: row[0].casefold())

    review_components = [
        component
        for component in component_reports
        if any(
            item["comparison_to_representative"]["requires_review"]
            for item in component["excluded"]
        )
    ]
    class_conflict_components = [
        component
        for component in component_reports
        if not component["representative_retained"]
    ]
    if len(class_conflict_components) != args.expected_class_conflict_components:
        raise ValueError(
            f"expected {args.expected_class_conflict_components} maturity-conflict "
            f"components, found {len(class_conflict_components)}"
        )
    temporary_output = args.output.with_name(args.output.name + ".part")
    temporary_audit = args.audit_json.with_name(args.audit_json.name + ".part")
    for path in (temporary_output, temporary_audit):
        if path.exists():
            path.unlink()
    try:
        exclusions_sha256 = _write_exclusions(temporary_output, exclusion_rows)
        audit = {
            "schema_version": 1,
            "dataset_id": "zenodo_6126677",
            "policy": {
                "name": "deduplicate_reencoded_images_and_exclude_missing_annotations",
                "duplicate_good_matches_min": args.duplicate_good_min,
                "duplicate_inlier_ratio_min": args.duplicate_inlier_ratio_min,
                "representative_rule": (
                    "largest encoded byte size, then source image SHA-256, then path"
                ),
                "label_iou_min": args.label_iou_min,
                "duplicate_label_policy": (
                    "retain the selected representative's own labels for "
                    "non-conflicting components; exclude both encodings when a "
                    "matched fruit has contradictory maturity; never union boxes"
                ),
                "empty_label_policy": "exclude because visual review found unlabeled fruit",
                "zero_area_v1_policy": "drop during label materialization",
            },
            "bindings": {
                "groups_csv": str(args.groups_csv.resolve()),
                "groups_csv_sha256": _sha256(args.groups_csv),
                "group_audit": str(args.group_audit.resolve()),
                "group_audit_sha256": _sha256(args.group_audit),
            },
            "summary": {
                "source_image_count": len(metadata),
                "duplicate_component_count": len(components),
                "duplicate_redundant_copy_count": sum(
                    len(component) - 1 for component in components
                ),
                "duplicate_components_requiring_label_review": len(
                    review_components
                ),
                "maturity_class_conflict_component_count": len(
                    class_conflict_components
                ),
                "maturity_class_conflict_image_exclusion_count": sum(
                    len(component["members"])
                    for component in class_conflict_components
                ),
                "empty_label_exclusion_count": len(empty_labels),
                "zero_area_v1_box_drop_count": len(zero_area),
                "total_excluded_image_count": len(exclusion_rows),
                "retained_image_count": len(metadata) - len(exclusion_rows),
            },
            "duplicate_components": component_reports,
            "duplicate_components_requiring_label_review": [
                component["members"] for component in review_components
            ],
            "maturity_class_conflict_exclusions": [
                component["members"] for component in class_conflict_components
            ],
            "empty_label_exclusions": empty_labels,
            "zero_area_v1_boxes": zero_area,
            "output": {
                "exclusions_csv": str(args.output.resolve()),
                "exclusions_csv_sha256": exclusions_sha256,
            },
        }
        args.audit_json.parent.mkdir(parents=True, exist_ok=True)
        temporary_audit.write_text(
            json.dumps(audit, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary_audit, args.audit_json)
        os.replace(temporary_output, args.output)
    finally:
        for path in (temporary_output, temporary_audit):
            if path.exists():
                path.unlink()
    print(
        json.dumps(
            {
                "source_images": len(metadata),
                "duplicate_components": len(components),
                "label_review_components": len(review_components),
                "empty_labels_excluded": len(empty_labels),
                "total_images_excluded": len(exclusion_rows),
                "retained_images": len(metadata) - len(exclusion_rows),
                "exclusions_csv": str(args.output),
                "exclusions_csv_sha256": exclusions_sha256,
                "audit_json": str(args.audit_json),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
