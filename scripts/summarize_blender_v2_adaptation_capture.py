#!/usr/bin/env python3
"""Verify and summarize the frozen ADR-0037 Blender-v2 capture."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from validate_blender_v2_adaptation_capture import (  # noqa: E402
    iter_groups,
    load_capture_manifest,
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def png_size(path: Path) -> tuple[int, int]:
    header = path.read_bytes()[:24]
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"not a PNG file: {path}")
    return struct.unpack(">II", header[16:24])


def _one_yolo_label(path: Path, expected_class: int) -> None:
    rows = [
        row.split()
        for row in path.read_text(encoding="utf-8-sig").splitlines()
        if row.strip()
    ]
    if len(rows) != 1 or len(rows[0]) != 5:
        raise ValueError(f"{path} must contain one YOLO label")
    class_value = float(rows[0][0])
    values = [float(value) for value in rows[0][1:]]
    if (
        not class_value.is_integer()
        or int(class_value) != expected_class
        or not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in values)
        or values[2] <= 0.0
        or values[3] <= 0.0
    ):
        raise ValueError(f"{path} contains an invalid frozen label")


def canonical_dataset_digest(records: list[dict]) -> str:
    fields = (
        "sample_id",
        "split",
        "maturity",
        "class_id",
        "condition_id",
        "position_id",
        "target_position_m",
        "target_orientation_xyzw",
        "image_sha256",
        "label_sha256",
        "source_image_sha256",
    )
    canonical = [
        {field: row[field] for field in fields}
        for row in sorted(records, key=lambda item: item["sample_id"])
    ]
    payload = json.dumps(
        canonical,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    options = parser.parse_args()
    output_root = options.output_root.resolve()
    manifest_path = options.manifest.resolve(strict=True)
    manifest = load_capture_manifest(manifest_path)
    receipt_path = output_root / "capture_receipt.json"
    inventory_path = output_root / "inventory.jsonl"
    dataset_path = output_root / "dataset.yaml"
    for path in (receipt_path, inventory_path, dataset_path):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite {path}")

    expected_groups = {
        row["group_id"]: row for row in iter_groups(manifest)
    }
    group_paths = sorted((output_root / "groups").glob("*.json"))
    if {path.stem for path in group_paths} != set(expected_groups):
        raise ValueError("captured group IDs differ from the manifest")

    inventory: list[dict] = []
    encoded_hashes: dict[str, set[str]] = {"train": set(), "heldout": set()}
    source_hashes: dict[str, set[str]] = {"train": set(), "heldout": set()}
    split_counts: Counter[str] = Counter()
    class_counts: Counter[str] = Counter()
    condition_counts: Counter[str] = Counter()
    for group_path in group_paths:
        group = json.loads(group_path.read_text(encoding="utf-8"))
        expected = expected_groups[group_path.stem]
        split = str(expected["split"])
        expected_samples = len(manifest["splits"][split]["positions"])
        if (
            group.get("capture_id") != manifest["capture_id"]
            or group.get("group_id") != group_path.stem
            or group.get("split") != split
            or group.get("maturity") != expected["maturity"]
            or group.get("condition_id") != expected["condition_id"]
            or int(group.get("sample_count", -1)) != expected_samples
            or group.get("formal_acceptance")
            or group.get("held_out_test_consumed")
            or group.get("training_started")
            or group.get("robot_motion_started")
        ):
            raise ValueError(f"invalid capture group: {group_path}")
        samples = group.get("samples", [])
        if not isinstance(samples, list) or len(samples) != expected_samples:
            raise ValueError(f"invalid sample list: {group_path}")
        for sample in samples:
            image_path = Path(str(sample["image"]["path"])).resolve(strict=True)
            label_path = Path(str(sample["label"]["path"])).resolve(strict=True)
            if output_root not in image_path.parents or output_root not in label_path.parents:
                raise ValueError("capture artifact escapes the output root")
            image_hash = sha256(image_path)
            label_hash = sha256(label_path)
            if (
                image_hash != sample["image"]["sha256"]
                or label_hash != sample["label"]["sha256"]
                or png_size(image_path) != (640, 480)
            ):
                raise ValueError(f"capture artifact binding failed: {image_path}")
            class_id = int(sample["class_id"])
            _one_yolo_label(label_path, class_id)
            source_hash = str(sample["source_image_sha256"])
            if image_hash in encoded_hashes[split]:
                raise ValueError(f"duplicate encoded image inside {split}")
            if source_hash in source_hashes[split]:
                raise ValueError(f"duplicate source image inside {split}")
            encoded_hashes[split].add(image_hash)
            source_hashes[split].add(source_hash)
            split_counts[split] += 1
            class_counts[f"{split}__{sample['maturity'].lower()}"] += 1
            condition_counts[f"{split}__{sample['condition_id']}"] += 1
            inventory.append(
                {
                    "sample_id": str(sample["sample_id"]),
                    "split": split,
                    "maturity": str(sample["maturity"]),
                    "class_id": class_id,
                    "condition_id": str(sample["condition_id"]),
                    "position_id": str(sample["position_id"]),
                    "target_position_m": sample["target_position_m"],
                    "target_orientation_xyzw": sample[
                        "target_orientation_xyzw"
                    ],
                    "image": image_path.relative_to(output_root).as_posix(),
                    "image_sha256": image_hash,
                    "label": label_path.relative_to(output_root).as_posix(),
                    "label_sha256": label_hash,
                    "source_image_sha256": source_hash,
                    "group": group_path.relative_to(output_root).as_posix(),
                    "group_sha256": sha256(group_path),
                }
            )

    if split_counts != Counter({"train": 72, "heldout": 24}):
        raise ValueError("capture split counts changed")
    if class_counts != Counter(
        {
            "train__ripe": 36,
            "train__unripe": 36,
            "heldout__ripe": 12,
            "heldout__unripe": 12,
        }
    ):
        raise ValueError("capture class counts changed")
    encoded_overlap = encoded_hashes["train"] & encoded_hashes["heldout"]
    source_overlap = source_hashes["train"] & source_hashes["heldout"]
    if encoded_overlap or source_overlap:
        raise ValueError("train and held-out image hashes overlap")

    inventory_text = "".join(
        json.dumps(row, sort_keys=True) + "\n"
        for row in sorted(inventory, key=lambda item: item["sample_id"])
    )
    inventory_path.write_text(inventory_text, encoding="utf-8")
    dataset_path.write_text(
        "path: .\n"
        "train: images/train\n"
        "val: images/heldout\n"
        "names:\n"
        "  0: ripe\n"
        "  1: unripe\n",
        encoding="utf-8",
    )
    receipt = {
        "schema_version": 1,
        "kind": "blender_v2_adaptation_capture_receipt",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "capture_id": manifest["capture_id"],
        "scope": manifest["scope"],
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "training_started": False,
        "robot_motion_started": False,
        "sample_count": len(inventory),
        "split_counts": dict(sorted(split_counts.items())),
        "class_counts": dict(sorted(class_counts.items())),
        "condition_counts": dict(sorted(condition_counts.items())),
        "group_count": len(group_paths),
        "hash_partition_checks": {
            "within_split_encoded_hashes_unique": True,
            "within_split_source_hashes_unique": True,
            "cross_split_encoded_image_hash_overlap": 0,
            "cross_split_source_image_hash_overlap": 0,
        },
        "canonical_synthetic_dataset_sha256": canonical_dataset_digest(
            inventory
        ),
        "manifest": {
            "path": manifest_path.relative_to(ROOT).as_posix(),
            "size_bytes": manifest_path.stat().st_size,
            "sha256": sha256(manifest_path),
        },
        "inventory": {
            "path": inventory_path.relative_to(ROOT).as_posix(),
            "size_bytes": inventory_path.stat().st_size,
            "sha256": sha256(inventory_path),
        },
        "dataset_yaml": {
            "path": dataset_path.relative_to(ROOT).as_posix(),
            "size_bytes": dataset_path.stat().st_size,
            "sha256": sha256(dataset_path),
        },
        "capture_preflight_passed": True,
    }
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
