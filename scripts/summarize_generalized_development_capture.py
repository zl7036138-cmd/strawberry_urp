#!/usr/bin/env python3
"""Verify a complete randomized development dataset and write its receipts."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "strawberry_sim"))

from strawberry_sim.generalized_capture_core import (  # noqa: E402
    iter_capture_specs,
    load_capture_plan,
    sha256_file,
)


def _read_label(path: Path) -> list[tuple[int, tuple[float, ...]]]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"invalid YOLO label row in {path}")
        class_id = int(fields[0])
        box = tuple(float(value) for value in fields[1:])
        if class_id not in {0, 1} or not all(0.0 < value <= 1.0 for value in box):
            raise ValueError(f"invalid YOLO values in {path}")
        rows.append((class_id, box))
    if not rows:
        raise ValueError(f"empty generalized label: {path}")
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--formal-matrix", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    options = parser.parse_args()
    plan_path = options.plan.resolve(strict=True)
    formal_path = options.formal_matrix.resolve(strict=True)
    output_root = options.output_root.resolve(strict=True)
    plan = load_capture_plan(plan_path, formal_path)
    specs = iter_capture_specs(plan)
    inventory = []
    class_counts: Counter[tuple[str, int]] = Counter()
    source_hashes: dict[str, set[str]] = {
        split: set() for split in ("train", "validation", "qualification")
    }
    for spec in specs:
        split = str(spec["split"])
        sample_id = str(spec["sample_id"])
        image_path = output_root / "images" / split / f"{sample_id}.png"
        label_path = output_root / "labels" / split / f"{sample_id}.txt"
        receipt_path = output_root / "receipts" / split / f"{sample_id}.json"
        for path in (image_path, label_path, receipt_path):
            if not path.is_file():
                raise ValueError(f"missing capture artifact: {path}")
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        if receipt.get("sample") != spec:
            raise ValueError(f"capture spec drift in {receipt_path}")
        if (
            receipt.get("formal_acceptance")
            or receipt.get("formal_results_consumed")
            or receipt.get("runtime_control_authorized")
            or receipt.get("robot_motion_started")
        ):
            raise ValueError(f"capture boundary violation in {receipt_path}")
        if receipt["image"]["sha256"] != sha256_file(image_path):
            raise ValueError(f"image hash mismatch in {receipt_path}")
        if receipt["label"]["sha256"] != sha256_file(label_path):
            raise ValueError(f"label hash mismatch in {receipt_path}")
        labels = _read_label(label_path)
        if len(labels) != len(receipt.get("visible_labels", [])):
            raise ValueError(f"label count mismatch in {receipt_path}")
        for class_id, _box in labels:
            class_counts[(split, class_id)] += 1
        source_hash = str(receipt["source_color_sha256"])
        if source_hash in source_hashes[split]:
            raise ValueError(f"duplicate source frame in {split}: {source_hash}")
        source_hashes[split].add(source_hash)
        inventory.append(
            {
                "sample": spec,
                "image_sha256": sha256_file(image_path),
                "label_sha256": sha256_file(label_path),
                "source_color_sha256": source_hash,
                "label_count": len(labels),
                "receipt_sha256": sha256_file(receipt_path),
            }
        )
    for left_index, left in enumerate(source_hashes):
        for right in tuple(source_hashes)[left_index + 1 :]:
            if source_hashes[left] & source_hashes[right]:
                raise ValueError(f"source image overlap between {left} and {right}")
    for split in source_hashes:
        if not class_counts[(split, 0)] or not class_counts[(split, 1)]:
            raise ValueError(f"{split} does not contain both maturity classes")

    dataset_path = output_root / "dataset.yaml"
    qualification_path = output_root / "qualification.yaml"
    inventory_path = output_root / "inventory.jsonl"
    summary_path = output_root / "capture_summary.json"
    for path in (dataset_path, qualification_path, inventory_path, summary_path):
        if path.exists():
            raise ValueError(f"refusing to overwrite dataset artifact: {path}")
    dataset_path.write_text(
        "path: .\ntrain: images/train\nval: images/validation\nnames:\n  0: ripe\n  1: unripe\n",
        encoding="utf-8",
    )
    qualification_path.write_text(
        "path: .\ntrain: images/qualification\nval: images/qualification\nnames:\n  0: ripe\n  1: unripe\n",
        encoding="utf-8",
    )
    with inventory_path.open("x", encoding="utf-8", newline="\n") as stream:
        for row in inventory:
            stream.write(json.dumps(row, separators=(",", ":"), sort_keys=True) + "\n")
    summary = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "capture_id": plan["capture_id"],
        "formal_acceptance": False,
        "formal_results_consumed": False,
        "runtime_control_authorized": False,
        "training_unlocked": True,
        "qualification_split_allowed_during_training": False,
        "scene_count": len(inventory),
        "split_counts": dict(Counter(row["sample"]["split"] for row in inventory)),
        "class_counts": {
            f"{split}__class_{class_id}": count
            for (split, class_id), count in sorted(class_counts.items())
        },
        "plan_sha256": sha256_file(plan_path),
        "formal_matrix_sha256": sha256_file(formal_path),
        "dataset_yaml_sha256": sha256_file(dataset_path),
        "qualification_yaml_sha256": sha256_file(qualification_path),
        "inventory_sha256": sha256_file(inventory_path),
        "interpretation": (
            "Development dataset preflight only. Training and independent "
            "qualification remain separate; formal 30-seed behavior was not run."
        ),
    }
    with summary_path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
