#!/usr/bin/env python3
"""Materialize ADR-0037's real-plus-Blender-v2 training derivative."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REAL = Path("data/processed/zenodo_6126677_train_audit_v1")
DEFAULT_SYNTHETIC = Path("data/processed/blender_v2_adaptation_capture_v1")
DEFAULT_OUTPUT = Path("data/processed/yolo11s_640_blender_v2_research_v1")


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def _display(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _link_or_copy(source: Path, destination: Path) -> str:
    try:
        os.link(source, destination)
        return "hardlink"
    except OSError:
        shutil.copy2(source, destination)
        return "copy"


def _class_counts(path: Path) -> Counter[int]:
    counts: Counter[int] = Counter()
    for line_number, raw in enumerate(
        path.read_text(encoding="utf-8-sig").splitlines(), 1
    ):
        if not raw.strip():
            continue
        fields = raw.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: invalid YOLO label")
        class_value = float(fields[0])
        coordinates = [float(value) for value in fields[1:]]
        if (
            not class_value.is_integer()
            or int(class_value) not in (0, 1)
            or not all(0.0 <= value <= 1.0 for value in coordinates)
            or coordinates[2] <= 0.0
            or coordinates[3] <= 0.0
        ):
            raise ValueError(f"{path}:{line_number}: invalid label value")
        counts[int(class_value)] += 1
    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-source", type=Path, default=DEFAULT_REAL)
    parser.add_argument("--synthetic-source", type=Path, default=DEFAULT_SYNTHETIC)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    options = parser.parse_args()
    real_root = _resolve(options.real_source).resolve(strict=True)
    synthetic_root = _resolve(options.synthetic_source).resolve(strict=True)
    output_root = _resolve(options.output).resolve()
    if output_root.exists():
        raise FileExistsError(f"refusing to overwrite {output_root}")

    real_manifest_path = real_root / "training_manifest.json"
    real_manifest = json.loads(real_manifest_path.read_text(encoding="utf-8"))
    if (
        real_manifest.get("kind") != "training_label_audit_derivative"
        or real_manifest.get("test_split_present") is not False
        or int(real_manifest.get("counts", {}).get("train_images", 0)) != 501
    ):
        raise ValueError("real training derivative is not the frozen ADR-0025 source")
    capture_receipt_path = synthetic_root / "capture_receipt.json"
    capture = json.loads(capture_receipt_path.read_text(encoding="utf-8"))
    if (
        capture.get("kind") != "blender_v2_adaptation_capture_receipt"
        or capture.get("capture_preflight_passed") is not True
        or capture.get("training_started") is not False
        or capture.get("held_out_test_consumed") is not False
        or capture.get("split_counts") != {"heldout": 24, "train": 72}
    ):
        raise ValueError("synthetic capture is not the frozen ADR-0037 source")

    real_images = sorted((real_root / "images" / "train").glob("*"))
    real_labels = sorted((real_root / "labels" / "train").glob("*.txt"))
    synthetic_train_images = sorted(
        (synthetic_root / "images" / "train").glob("*.png")
    )
    synthetic_train_labels = sorted(
        (synthetic_root / "labels" / "train").glob("*.txt")
    )
    synthetic_val_images = sorted(
        (synthetic_root / "images" / "heldout").glob("*.png")
    )
    synthetic_val_labels = sorted(
        (synthetic_root / "labels" / "heldout").glob("*.txt")
    )
    expected_lengths = (501, 501, 72, 72, 24, 24)
    actual_lengths = tuple(
        len(rows)
        for rows in (
            real_images,
            real_labels,
            synthetic_train_images,
            synthetic_train_labels,
            synthetic_val_images,
            synthetic_val_labels,
        )
    )
    if actual_lengths != expected_lengths:
        raise ValueError(
            f"source file counts changed: {actual_lengths} != {expected_lengths}"
        )
    for image_paths, label_paths in (
        (real_images, real_labels),
        (synthetic_train_images, synthetic_train_labels),
        (synthetic_val_images, synthetic_val_labels),
    ):
        if {path.stem for path in image_paths} != {
            path.stem for path in label_paths
        }:
            raise ValueError("source image and label stems differ")

    for split in ("train", "val"):
        (output_root / "images" / split).mkdir(parents=True)
        (output_root / "labels" / split).mkdir(parents=True)
    entries = []
    materialization_counts: Counter[str] = Counter()
    total_classes: dict[str, Counter[int]] = {
        "train": Counter(),
        "val": Counter(),
    }

    def materialize(
        image_paths: list[Path],
        label_paths: list[Path],
        *,
        split: str,
        role: str,
    ) -> None:
        labels = {path.stem: path for path in label_paths}
        for image_path in image_paths:
            label_path = labels[image_path.stem]
            destination_image = output_root / "images" / split / image_path.name
            destination_label = output_root / "labels" / split / label_path.name
            image_method = _link_or_copy(image_path, destination_image)
            label_method = _link_or_copy(label_path, destination_label)
            materialization_counts[image_method] += 1
            materialization_counts[label_method] += 1
            classes = _class_counts(destination_label)
            total_classes[split].update(classes)
            entries.append(
                {
                    "role": role,
                    "split": split,
                    "source_image": _display(image_path),
                    "source_image_sha256": sha256(image_path),
                    "source_label": _display(label_path),
                    "source_label_sha256": sha256(label_path),
                    "derived_image": destination_image.relative_to(
                        output_root
                    ).as_posix(),
                    "derived_image_sha256": sha256(destination_image),
                    "derived_label": destination_label.relative_to(
                        output_root
                    ).as_posix(),
                    "derived_label_sha256": sha256(destination_label),
                    "class_counts": {
                        str(key): value for key, value in sorted(classes.items())
                    },
                }
            )

    materialize(
        real_images,
        real_labels,
        split="train",
        role="consensus_corrected_real_train",
    )
    materialize(
        synthetic_train_images,
        synthetic_train_labels,
        split="train",
        role="blender_v2_synthetic_train",
    )
    materialize(
        synthetic_val_images,
        synthetic_val_labels,
        split="val",
        role="blender_v2_synthetic_heldout",
    )

    dataset_path = output_root / "dataset.yaml"
    dataset_path.write_text(
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: ripe\n"
        "  1: unripe\n",
        encoding="utf-8",
    )
    canonical_rows = [
        {
            key: row[key]
            for key in (
                "role",
                "split",
                "source_image_sha256",
                "source_label_sha256",
                "derived_image",
                "derived_image_sha256",
                "derived_label",
                "derived_label_sha256",
                "class_counts",
            )
        }
        for row in sorted(entries, key=lambda item: item["derived_image"])
    ]
    canonical_sha = hashlib.sha256(
        json.dumps(
            canonical_rows,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    result = {
        "schema_version": 1,
        "kind": "blender_v2_research_training_derivative",
        "variant": "yolo11s_640_blender_v2_research_v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "test_split_present": False,
        "training_started": False,
        "counts": {
            "train_images": 573,
            "real_train_images": 501,
            "synthetic_train_images": 72,
            "synthetic_validation_images": 24,
            "train_classes": {
                str(key): value
                for key, value in sorted(total_classes["train"].items())
            },
            "validation_classes": {
                str(key): value
                for key, value in sorted(total_classes["val"].items())
            },
        },
        "bindings": {
            "real_training_manifest": {
                "path": _display(real_manifest_path),
                "size_bytes": real_manifest_path.stat().st_size,
                "sha256": sha256(real_manifest_path),
            },
            "synthetic_capture_receipt": {
                "path": _display(capture_receipt_path),
                "size_bytes": capture_receipt_path.stat().st_size,
                "sha256": sha256(capture_receipt_path),
                "canonical_synthetic_dataset_sha256": capture[
                    "canonical_synthetic_dataset_sha256"
                ],
            },
        },
        "dataset_yaml": {
            "path": "dataset.yaml",
            "size_bytes": dataset_path.stat().st_size,
            "sha256": sha256(dataset_path),
        },
        "canonical_training_derivative_sha256": canonical_sha,
        "materialization_counts": dict(sorted(materialization_counts.items())),
        "entries": entries,
    }
    manifest_path = output_root / "training_manifest.json"
    manifest_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "output": _display(output_root),
                "train_images": 573,
                "validation_images": 24,
                "train_classes": result["counts"]["train_classes"],
                "validation_classes": result["counts"]["validation_classes"],
                "canonical_sha256": canonical_sha,
                "training_started": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
