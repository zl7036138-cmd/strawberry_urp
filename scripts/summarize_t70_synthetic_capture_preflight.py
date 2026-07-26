#!/usr/bin/env python3
"""Verify the ADR 0019 synthetic capture and emit its locked preflight receipt."""

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
sys.path.insert(0, str(ROOT / "ros2_ws/src/strawberry_sim"))

from validate_t70_synthetic_capture_preflight import (  # noqa: E402
    FORMAL_SEED_LABELS,
    iter_groups,
    load_capture_manifest,
)
from strawberry_sim.scene_conditions import fingerprint  # noqa: E402
from strawberry_sim.synthetic_capture_core import parse_yolo_label  # noqa: E402


ERROR_MARKERS = (
    "traceback (most recent call last)",
    "exception was never retrieved",
    "process has died",
)
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def png_size(path: Path) -> tuple[int, int]:
    content = path.read_bytes()[:24]
    if len(content) != 24 or content[:8] != PNG_SIGNATURE or content[12:16] != b"IHDR":
        raise ValueError(f"invalid PNG header: {path}")
    return struct.unpack(">II", content[16:24])


def validate_hash_partitions(records: list[dict]) -> dict[str, object]:
    by_split: dict[str, list[dict]] = {"train": [], "heldout": []}
    for record in records:
        split = str(record["split"])
        if split not in by_split:
            raise ValueError(f"unexpected synthetic split {split}")
        by_split[split].append(record)
    image_sets = {}
    source_sets = {}
    for split, items in by_split.items():
        image_hashes = [str(item["image"]["sha256"]) for item in items]
        source_hashes = [str(item["source_image_sha256"]) for item in items]
        if len(set(image_hashes)) != len(image_hashes):
            raise ValueError(f"duplicate encoded image hash inside {split}")
        if len(set(source_hashes)) != len(source_hashes):
            raise ValueError(f"duplicate source image hash inside {split}")
        image_sets[split] = set(image_hashes)
        source_sets[split] = set(source_hashes)
    image_overlap = image_sets["train"] & image_sets["heldout"]
    source_overlap = source_sets["train"] & source_sets["heldout"]
    if image_overlap or source_overlap:
        raise ValueError("synthetic train and heldout image hashes overlap")
    return {
        "within_split_encoded_image_hashes_unique": True,
        "within_split_source_image_hashes_unique": True,
        "cross_split_encoded_image_hash_overlap": 0,
        "cross_split_source_image_hash_overlap": 0,
    }


def canonical_dataset_digest(records: list[dict]) -> str:
    canonical = [
        {
            "sample_id": item["sample_id"],
            "split": item["split"],
            "maturity": item["maturity"],
            "class_id": item["class_id"],
            "condition_id": item["condition_id"],
            "position_id": item["position_id"],
            "target_position_m": item["target_position_m"],
            "image_sha256": item["image"]["sha256"],
            "label_sha256": item["label"]["sha256"],
        }
        for item in sorted(records, key=lambda row: str(row["sample_id"]))
    ]
    payload = json.dumps(canonical, separators=(",", ":"), sort_keys=True).encode(
        "utf-8"
    )
    return hashlib.sha256(payload).hexdigest()


def _same_vector(left: object, right: object) -> bool:
    try:
        first = tuple(float(value) for value in left)  # type: ignore[arg-type]
        second = tuple(float(value) for value in right)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    return len(first) == len(second) == 3 and all(
        math.isclose(a, b, abs_tol=1e-12) for a, b in zip(first, second)
    )


def _inside(path: Path, root: Path) -> bool:
    resolved = path.resolve(strict=True)
    return resolved == root or root in resolved.parents


def _verify_fingerprint(record: dict, root: Path | None = None) -> Path:
    path = Path(str(record["path"])).resolve(strict=True)
    if root is not None and not _inside(path, root):
        raise ValueError(f"artifact escapes output root: {path}")
    actual = fingerprint(path)
    if int(actual["size_bytes"]) != int(record["size_bytes"]):
        raise ValueError(f"artifact size mismatch: {path}")
    if actual["sha256"] != record["sha256"]:
        raise ValueError(f"artifact hash mismatch: {path}")
    return path


def _d2_source_hashes(intervention: dict) -> set[str]:
    summary_path = (ROOT / intervention["trigger_evidence"]["path"]).resolve(
        strict=True
    )
    trigger = intervention["trigger_evidence"]
    if summary_path.stat().st_size != int(trigger["size_bytes"]):
        raise ValueError("D2 trigger summary size mismatch")
    if fingerprint(summary_path)["sha256"] != trigger["sha256"]:
        raise ValueError("D2 trigger summary hash mismatch")
    hashes = set()
    for probe_path in sorted(summary_path.parent.glob("*/condition_probe.json")):
        probe = json.loads(probe_path.read_text(encoding="utf-8"))
        hashes.update(str(frame["image_sha256"]) for frame in probe.get("frames", []))
    if not hashes:
        raise ValueError("D2 source image hashes are unavailable")
    return hashes


def summarize(
    output_root: Path,
    manifest_path: Path,
    runner_path: Path,
    base_domain_id: int | None = None,
) -> dict[str, object]:
    output_root = output_root.resolve(strict=True)
    manifest = load_capture_manifest(manifest_path)
    groups = list(iter_groups(manifest))
    if base_domain_id is not None and not 0 <= base_domain_id <= 232 - len(groups) + 1:
        raise ValueError("synthetic capture ROS domains must fit in [0, 232]")
    runner_path = runner_path.resolve(strict=True)
    intervention_path = Path(manifest["_resolved_paths"]["intervention"])
    intervention = json.loads(intervention_path.read_text(encoding="utf-8"))
    d2_hashes = _d2_source_hashes(intervention)
    manifest_hash = fingerprint(manifest_path)["sha256"]
    scene_config_hash = manifest["contracts"]["scene_condition_sha256"]
    expected_size = (
        int(manifest["capture"]["image_width"]),
        int(manifest["capture"]["image_height"]),
    )
    records = []
    blockers = []
    world_hashes = set()

    for index, expected in enumerate(groups):
        group_id = expected["group_id"]
        group_path = output_root / "groups" / f"{group_id}.json"
        world_dir = output_root / "worlds" / group_id
        receipt_path = world_dir / "receipt.json"
        log_paths = {
            "launch": output_root / "logs" / f"{group_id}.launch.log",
            "capture": output_root / "logs" / f"{group_id}.capture.log",
        }
        errors = []
        group = receipt = None
        try:
            group = json.loads(group_path.read_text(encoding="utf-8"))
            receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
            if group.get("group_id") != group_id:
                errors.append("group ID mismatch")
            for key in (
                "split",
                "maturity",
                "class_id",
                "target_id",
                "target_model_name",
                "condition_id",
                "lighting",
                "occlusion",
                "materialization_id",
            ):
                if group.get(key) != expected[key]:
                    errors.append(f"group {key} mismatch")
            if group.get("formal_acceptance") or group.get("held_out_test_consumed"):
                errors.append("group violates non-acceptance/test boundary")
            if group.get("training_started") or group.get("robot_motion_started"):
                errors.append("group claims training or robot motion")
            if group.get("manifest", {}).get("sha256") != manifest_hash:
                errors.append("group manifest hash mismatch")
            if group.get("scene_condition_config", {}).get("sha256") != scene_config_hash:
                errors.append("group scene-condition hash mismatch")
            if group.get("condition_receipt", {}).get("sha256") != fingerprint(
                receipt_path
            )["sha256"]:
                errors.append("group is not bound to its condition receipt")
            if int(receipt.get("seed", 0)) != int(expected["materialization_id"]):
                errors.append("receipt materialization ID mismatch")
            if int(receipt.get("seed", 0)) in FORMAL_SEED_LABELS:
                errors.append("formal seed label appears in a capture receipt")
            if receipt.get("materialization_id_role") != "synthetic_split_identifier_only":
                errors.append("receipt materialization role mismatch")
            if receipt.get("lighting_level") != expected["lighting"]:
                errors.append("receipt lighting mismatch")
            if receipt.get("occlusion_level") != expected["occlusion"]:
                errors.append("receipt occlusion mismatch")
            world_hash = receipt.get("materialized_world", {}).get("sha256")
            if group.get("materialized_world_sha256") != world_hash:
                errors.append("group materialized world mismatch")
            if world_hash:
                world_hashes.add(str(world_hash))
            expected_positions = manifest["splits"][expected["split"]]["positions"]
            samples = group.get("samples", [])
            if len(samples) != len(expected_positions):
                errors.append("group sample count mismatch")
            by_position = {
                str(sample["position_id"]): sample for sample in samples
            }
            if set(by_position) != {
                str(item["position_id"]) for item in expected_positions
            }:
                errors.append("group position membership mismatch")
            for position_entry in expected_positions:
                position_id = str(position_entry["position_id"])
                sample = by_position.get(position_id)
                if sample is None:
                    continue
                expected_sample_id = f"{group_id}__{position_id}"
                if sample.get("sample_id") != expected_sample_id:
                    errors.append(f"{position_id}: sample ID mismatch")
                if not _same_vector(
                    sample.get("target_position_m"), position_entry["position_m"]
                ):
                    errors.append(f"{position_id}: target position mismatch")
                for key in (
                    "split",
                    "maturity",
                    "class_id",
                    "target_id",
                    "target_model_name",
                    "condition_id",
                    "lighting",
                    "occlusion",
                    "materialization_id",
                ):
                    if sample.get(key) != expected[key]:
                        errors.append(f"{position_id}: sample {key} mismatch")
                try:
                    image_path = _verify_fingerprint(sample["image"], output_root)
                    label_path = _verify_fingerprint(sample["label"], output_root)
                    if png_size(image_path) != expected_size:
                        errors.append(f"{position_id}: PNG dimensions mismatch")
                    parsed_class, parsed_box = parse_yolo_label(
                        label_path.read_text(encoding="utf-8")
                    )
                    if parsed_class != int(expected["class_id"]):
                        errors.append(f"{position_id}: label class mismatch")
                    recorded_box = tuple(float(value) for value in sample["yolo_xywh"])
                    if any(
                        not math.isclose(actual, recorded, abs_tol=5e-10)
                        for actual, recorded in zip(parsed_box, recorded_box)
                    ):
                        errors.append(f"{position_id}: label geometry mismatch")
                except (FileNotFoundError, KeyError, TypeError, ValueError) as error:
                    errors.append(f"{position_id}: invalid sample artifact: {error}")
                records.append(sample)
        except (FileNotFoundError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            errors.append(f"missing or invalid group artifact: {error}")

        for name, log_path in log_paths.items():
            if not log_path.exists():
                errors.append(f"missing {name} log")
                continue
            text = log_path.read_text(encoding="utf-8", errors="replace").lower()
            markers = sorted(marker for marker in ERROR_MARKERS if marker in text)
            if markers:
                errors.append(f"{name} log failure markers: {', '.join(markers)}")
        if base_domain_id is not None and isinstance(group, dict):
            if int(group.get("ros_domain_id", -1)) != base_domain_id + index:
                errors.append("group ROS domain mismatch")
        blockers.extend(f"{group_id}: {error}" for error in errors)

    expected_total = int(manifest["capture"]["expected_total_images"])
    if len(records) != expected_total:
        blockers.append(f"sample count {len(records)} != {expected_total}")
    hash_checks = {}
    try:
        hash_checks = validate_hash_partitions(records)
    except ValueError as error:
        blockers.append(str(error))
    d2_overlap = {
        str(record["source_image_sha256"]) for record in records
    } & d2_hashes
    if d2_overlap:
        blockers.append(f"D2 source image overlap: {len(d2_overlap)}")

    split_counts = Counter(str(record["split"]) for record in records)
    class_counts = Counter(
        (str(record["split"]), str(record["maturity"])) for record in records
    )
    condition_counts = Counter(
        (str(record["split"]), str(record["condition_id"])) for record in records
    )
    expected_split_counts = {
        "train": int(manifest["capture"]["expected_train_images"]),
        "heldout": int(manifest["capture"]["expected_heldout_images"]),
    }
    if dict(split_counts) != expected_split_counts:
        blockers.append("synthetic split counts differ from the manifest")
    expected_class_counts = {
        ("train", "RIPE"): 108,
        ("train", "UNRIPE"): 108,
        ("heldout", "RIPE"): 36,
        ("heldout", "UNRIPE"): 36,
    }
    if dict(class_counts) != expected_class_counts:
        blockers.append("synthetic class balance differs from the manifest")
    if any(
        condition_counts[(split, condition["condition_id"])]
        != (24 if split == "train" else 8)
        for split in ("train", "heldout")
        for condition in manifest["conditions"]
    ):
        blockers.append("synthetic condition balance differs from the manifest")
    if len(world_hashes) != 15:
        blockers.append(f"distinct materialized world count {len(world_hashes)} != 15")

    dataset_yaml_path = output_root / "dataset.yaml"
    inventory_path = output_root / "inventory.jsonl"
    for output in (dataset_yaml_path, inventory_path):
        if output.exists():
            raise ValueError(f"refusing to overwrite preflight artifact: {output}")
    dataset_yaml_path.write_text(
        "path: .\ntrain: images/train\nval: images/heldout\nnames:\n  0: ripe\n  1: unripe\n",
        encoding="utf-8",
    )
    with inventory_path.open("x", encoding="utf-8", newline="\n") as stream:
        for record in sorted(records, key=lambda row: str(row["sample_id"])):
            stream.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")

    passed = not blockers
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "capture_id": manifest["capture_id"],
        "scope": manifest["scope"],
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "training_started": False,
        "robot_motion_started": False,
        "perception_control_authorized": False,
        "formal_matrix_authorized": False,
        "synthetic_capture_preflight_passed": passed,
        "training_preflight_satisfied": passed,
        "training_unlocked": False,
        "training_unlock_requires": (
            "a separate hash-frozen hyperparameter manifest and one-time claim"
        ),
        "manifest": fingerprint(manifest_path),
        "intervention": fingerprint(intervention_path),
        "runner": fingerprint(runner_path),
        "group_count": len(groups),
        "sample_count": len(records),
        "split_counts": dict(split_counts),
        "class_counts": {
            f"{split}__{maturity.lower()}": count
            for (split, maturity), count in sorted(class_counts.items())
        },
        "condition_counts": {
            f"{split}__{condition}": count
            for (split, condition), count in sorted(condition_counts.items())
        },
        "distinct_materialized_worlds": len(world_hashes),
        "hash_partition_checks": hash_checks,
        "d2_source_image_hash_count": len(d2_hashes),
        "d2_source_image_overlap": len(d2_overlap),
        "formal_seed_labels": sorted(FORMAL_SEED_LABELS),
        "formal_seed_labels_used": [],
        "canonical_synthetic_dataset_sha256": canonical_dataset_digest(records),
        "dataset_yaml": fingerprint(dataset_yaml_path),
        "inventory": fingerprint(inventory_path),
        "blockers": blockers,
        "interpretation": (
            "Synthetic capture isolation preflight only. No model was trained; "
            "a pass does not authorize perception control, formal testing, or "
            "stage-gate acceptance."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--base-domain-id", type=int)
    options = parser.parse_args()
    receipt_path = options.output_root / "preflight_receipt.json"
    if receipt_path.exists():
        raise ValueError(f"refusing to overwrite preflight receipt: {receipt_path}")
    result = summarize(
        options.output_root,
        options.manifest.resolve(strict=True),
        options.runner,
        options.base_domain_id,
    )
    receipt_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "preflight_passed": result["synthetic_capture_preflight_passed"],
                "receipt": str(receipt_path),
                "samples": result["sample_count"],
                "training_unlocked": False,
            },
            sort_keys=True,
        )
    )
    return 0 if result["synthetic_capture_preflight_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
