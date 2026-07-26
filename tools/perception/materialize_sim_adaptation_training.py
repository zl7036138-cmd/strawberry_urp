#!/usr/bin/env python3
"""Materialize and verify ADR-0021's mixed real/synthetic training data."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = Path("tools/perception/sim_adaptation_v1_contract.json")


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _display(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPOSITORY_ROOT.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


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


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        value = json.loads(raw)
        if not isinstance(value, dict):
            raise ValueError(f"{path}:{line_number}: expected JSON object")
        rows.append(value)
    return rows


def _verify_binding(binding: Mapping[str, Any], *, label: str) -> Path:
    path = _resolve(Path(str(binding.get("path", "")))).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"missing {label}: {path}")
    if path.stat().st_size != int(binding.get("size_bytes", -1)):
        raise ValueError(f"{label} size binding mismatch")
    if _sha256(path) != binding.get("sha256"):
        raise ValueError(f"{label} hash binding mismatch")
    return path


def _count_classes(path: Path) -> Counter[int]:
    counts: Counter[int] = Counter()
    for line_number, raw in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not raw.strip():
            continue
        fields = raw.split()
        if len(fields) != 5:
            raise ValueError(f"{path}:{line_number}: expected five YOLO fields")
        try:
            class_value = float(fields[0])
            coordinates = [float(value) for value in fields[1:]]
        except ValueError as error:
            raise ValueError(f"{path}:{line_number}: invalid numeric field") from error
        if not class_value.is_integer() or int(class_value) not in (0, 1):
            raise ValueError(f"{path}:{line_number}: class outside v1 contract")
        if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in coordinates):
            raise ValueError(f"{path}:{line_number}: invalid normalized coordinate")
        if coordinates[2] <= 0.0 or coordinates[3] <= 0.0:
            raise ValueError(f"{path}:{line_number}: non-positive box size")
        counts[int(class_value)] += 1
    return counts


def _safe_registered_path(value: str, kind: str, split: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if path.parent != PurePosixPath(kind, split):
        raise ValueError(f"unsafe registered {split} {kind} path: {value}")
    allowed = {".jpg", ".jpeg", ".png"} if kind == "images" else {".txt"}
    if path.suffix.lower() not in allowed:
        raise ValueError(f"unsupported registered {kind} suffix: {value}")
    return path


def _link_or_copy(source: Path, destination: Path) -> str:
    try:
        os.link(source, destination)
        return "hardlink"
    except OSError:
        shutil.copy2(source, destination)
        return "copy"


def _canonical_rows(entries: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    keys = (
        "role",
        "source_image",
        "source_image_sha256",
        "source_label",
        "source_label_sha256",
        "derived_image",
        "derived_image_sha256",
        "derived_label",
        "derived_label_sha256",
        "class_counts",
    )
    return [
        {key: item[key] for key in keys}
        for item in sorted(entries, key=lambda row: (str(row["role"]), str(row["derived_image"])))
    ]


def _validate_contract(contract_path: Path) -> tuple[dict[str, Any], dict[str, Path]]:
    contract_path = _resolve(contract_path).resolve()
    contract = _load_object(contract_path)
    if contract.get("kind") != "simulator_adaptation_training_contract":
        raise ValueError("unexpected simulator-adaptation contract kind")
    if contract.get("status") != "FROZEN_TRAINING_AUTHORIZED":
        raise ValueError("simulator-adaptation training is not frozen and authorized")
    if int(contract.get("training", {}).get("maximum_claims", 0)) != 1:
        raise ValueError("contract must permit exactly one claim")
    if contract.get("training", {}).get("retry_authorized") is not False:
        raise ValueError("contract must forbid retries")
    safety = contract.get("safety", {})
    for key in (
        "formal_real_test_access_authorized",
        "perception_control_authorized",
        "robot_motion_authorized",
        "formal_simulator_matrix_authorized",
        "sim_to_real_claim_authorized",
    ):
        if safety.get(key) is not False:
            raise ValueError(f"unsafe authorization in contract: {key}")
    forbidden_receipt = _resolve(Path(str(safety["formal_real_test_receipt_must_remain_absent"])))
    if forbidden_receipt.exists():
        raise ValueError("formal real-test receipt exists; simulator adaptation must fail closed")

    authorization_path = _resolve(Path(str(contract["authorization_receipt"]))).resolve()
    authorization = _load_object(authorization_path)
    if authorization.get("kind") != "simulator_adaptation_training_user_authorization":
        raise ValueError("unexpected simulator-adaptation authorization kind")
    if authorization.get("status") != "AUTHORIZED" or authorization.get("new_training_authorized") is not True:
        raise ValueError("simulator-adaptation training lacks user authorization")
    if int(authorization.get("maximum_claims", 0)) != 1 or authorization.get("retry_or_parameter_search_authorized") is not False:
        raise ValueError("authorization does not enforce one attempt")
    for key in (
        "formal_real_test_authorized",
        "formal_simulator_matrix_authorized",
        "perception_control_authorized",
        "robot_motion_authorized",
    ):
        if authorization.get(key) is not False:
            raise ValueError(f"unsafe user authorization: {key}")
    binding = authorization.get("contract", {})
    if (
        binding.get("path") != _display(contract_path)
        or binding.get("size_bytes") != contract_path.stat().st_size
        or binding.get("sha256") != _sha256(contract_path)
    ):
        raise ValueError("authorization contract binding mismatch")
    decision = _verify_binding(authorization["decision_record"], label="ADR-0021")
    config = _verify_binding(authorization["training_config"], label="sim-adaptation training config")
    if _display(decision) != contract.get("decision_record"):
        raise ValueError("decision path does not match contract")
    if _display(config) != contract.get("training", {}).get("config"):
        raise ValueError("training-config path does not match contract")

    upstream = contract["upstream_preflight"]
    handoff = _verify_binding(upstream["handoff"], label="synthetic preflight handoff")
    receipt_path = _verify_binding(upstream["receipt"], label="synthetic preflight receipt")
    inventory = _verify_binding(upstream["inventory"], label="synthetic inventory")
    receipt = _load_object(receipt_path)
    if receipt.get("synthetic_capture_preflight_passed") is not True:
        raise ValueError("synthetic capture preflight did not pass")
    if receipt.get("training_started") is not False or receipt.get("held_out_test_consumed") is not False:
        raise ValueError("synthetic capture receipt crosses the frozen boundary")
    if receipt.get("canonical_synthetic_dataset_sha256") != upstream["receipt"]["canonical_synthetic_dataset_sha256"]:
        raise ValueError("synthetic canonical digest mismatch")
    if receipt.get("sample_count") != 288 or receipt.get("split_counts") != {"heldout": 72, "train": 216}:
        raise ValueError("synthetic capture count mismatch")
    checks = receipt.get("hash_partition_checks", {})
    if (
        checks.get("within_split_encoded_image_hashes_unique") is not True
        or checks.get("within_split_source_image_hashes_unique") is not True
        or checks.get("cross_split_encoded_image_hash_overlap") != 0
        or checks.get("cross_split_source_image_hash_overlap") != 0
        or receipt.get("d2_source_image_overlap") != 0
        or receipt.get("formal_seed_labels_used") != []
    ):
        raise ValueError("synthetic preflight isolation checks changed")

    split_manifest = _verify_binding(contract["real_training_source"]["split_manifest"], label="real split manifest")
    split_value = _load_object(split_manifest)
    canonical_train = split_value.get("content_binding", {}).get("canonical_split_sha256", {}).get("train")
    if canonical_train != contract["real_training_source"]["split_manifest"]["canonical_train_sha256"]:
        raise ValueError("real training canonical digest mismatch")

    audit_manifest = _verify_binding(contract["audited_real_validation"]["manifest"], label="audited validation manifest")
    audit_value = _load_object(audit_manifest)
    canonical_audit = audit_value.get("content_binding", {}).get("canonical_validation_derivative_sha256")
    if canonical_audit != contract["audited_real_validation"]["manifest"]["canonical_sha256"]:
        raise ValueError("audited validation canonical digest mismatch")
    if len(audit_value.get("entries", [])) != int(contract["audited_real_validation"]["image_count"]):
        raise ValueError("audited validation image count mismatch")
    for key in ("training_access", "checkpoint_selection_access", "threshold_selection_access"):
        if contract["audited_real_validation"].get(key) is not False:
            raise ValueError(f"audited validation cannot be used for {key}")

    base_weight = _verify_binding(contract["base_weight"], label="baseline__best checkpoint")
    return contract, {
        "authorization": authorization_path,
        "decision": decision,
        "config": config,
        "handoff": handoff,
        "receipt": receipt_path,
        "inventory": inventory,
        "split_manifest": split_manifest,
        "audit_manifest": audit_manifest,
        "base_weight": base_weight,
    }


def _copy_registered(
    *,
    source_image: Path,
    source_label: Path,
    destination_root: Path,
    destination_split: str,
    destination_stem_prefix: str,
    role: str,
    source_image_display: str,
    source_label_display: str,
    counts: Counter[int],
) -> dict[str, Any]:
    image_name = f"{destination_stem_prefix}{source_image.name}"
    label_name = f"{destination_stem_prefix}{source_label.name}"
    derived_image_part = PurePosixPath("images", destination_split, image_name)
    derived_label_part = PurePosixPath("labels", destination_split, label_name)
    derived_image = destination_root / derived_image_part
    derived_label = destination_root / derived_label_part
    mode = _link_or_copy(source_image, derived_image)
    shutil.copy2(source_label, derived_label)
    return {
        "role": role,
        "source_image": source_image_display,
        "source_image_sha256": _sha256(source_image),
        "source_image_size_bytes": source_image.stat().st_size,
        "source_label": source_label_display,
        "source_label_sha256": _sha256(source_label),
        "source_label_size_bytes": source_label.stat().st_size,
        "derived_image": derived_image_part.as_posix(),
        "derived_image_sha256": _sha256(derived_image),
        "derived_image_size_bytes": derived_image.stat().st_size,
        "derived_label": derived_label_part.as_posix(),
        "derived_label_sha256": _sha256(derived_label),
        "derived_label_size_bytes": derived_label.stat().st_size,
        "class_counts": {"0": counts[0], "1": counts[1]},
        "image_materialization": mode,
    }


def materialize(contract_path: Path) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    contract, paths = _validate_contract(contract_path)
    derivative = contract["dataset_derivative"]
    output_dir = _resolve(Path(str(derivative["output_dir"]))).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite simulator-adaptation derivative: {output_dir}")
    if derivative.get("test_split_present") is not False:
        raise ValueError("simulator-adaptation derivative must omit test")

    split_manifest = _load_object(paths["split_manifest"])
    real_rows = split_manifest.get("splits", {}).get("train")
    if not isinstance(real_rows, list) or len(real_rows) != int(contract["real_training_source"]["image_count"]):
        raise ValueError("real training image count mismatch")
    real_root = paths["split_manifest"].parent

    synthetic_rows = _load_jsonl(paths["inventory"])
    if len(synthetic_rows) != 288:
        raise ValueError("synthetic inventory row count mismatch")
    synthetic_root = paths["inventory"].parent

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".sim-adapt-v1-staging-", dir=output_dir.parent) as temporary:
        staging = Path(temporary) / output_dir.name
        for split in ("train", "val"):
            (staging / "images" / split).mkdir(parents=True)
            (staging / "labels" / split).mkdir(parents=True)

        entries: list[dict[str, Any]] = []
        train_counts: Counter[int] = Counter()
        validation_counts: Counter[int] = Counter()
        source_counts: Counter[str] = Counter()

        for item in sorted(real_rows, key=lambda row: str(row["output_image"])):
            image_part = _safe_registered_path(str(item["output_image"]), "images", "train")
            label_part = _safe_registered_path(str(item["output_label"]), "labels", "train")
            if image_part.stem != label_part.stem:
                raise ValueError(f"real image/label mismatch: {image_part}")
            source_image = real_root / image_part
            source_label = real_root / label_part
            bindings = (
                (source_image, "output_image_sha256", "output_image_size_bytes"),
                (source_label, "output_label_sha256", "output_label_size_bytes"),
            )
            for source, hash_key, size_key in bindings:
                if not source.is_file() or _sha256(source) != item.get(hash_key) or source.stat().st_size != item.get(size_key):
                    raise ValueError(f"registered real training file changed: {source}")
            counts = _count_classes(source_label)
            registered = item.get("v1_class_counts", {})
            if {"0": counts[0], "1": counts[1]} != {"0": int(registered.get("0", 0)), "1": int(registered.get("1", 0))}:
                raise ValueError(f"registered real class counts changed: {source_label}")
            train_counts.update(counts)
            source_counts["real"] += 1
            entries.append(
                _copy_registered(
                    source_image=source_image,
                    source_label=source_label,
                    destination_root=staging,
                    destination_split="train",
                    destination_stem_prefix="real__",
                    role="real_train",
                    source_image_display=image_part.as_posix(),
                    source_label_display=label_part.as_posix(),
                    counts=counts,
                )
            )

        encoded_hashes: dict[str, set[str]] = {"train": set(), "heldout": set()}
        source_hashes: dict[str, set[str]] = {"train": set(), "heldout": set()}
        for item in sorted(synthetic_rows, key=lambda row: str(row["sample_id"])):
            split = str(item.get("split"))
            if split not in ("train", "heldout"):
                raise ValueError(f"unexpected synthetic split: {split}")
            sample_id = str(item.get("sample_id", ""))
            if not sample_id.startswith(f"syn__{split}__"):
                raise ValueError(f"synthetic sample ID does not bind split: {sample_id}")
            source_image = synthetic_root / "images" / split / f"{sample_id}.png"
            source_label = synthetic_root / "labels" / split / f"{sample_id}.txt"
            image_binding = item.get("image", {})
            label_binding = item.get("label", {})
            if (
                not source_image.is_file()
                or source_image.stat().st_size != int(image_binding.get("size_bytes", -1))
                or _sha256(source_image) != image_binding.get("sha256")
                or not source_label.is_file()
                or source_label.stat().st_size != int(label_binding.get("size_bytes", -1))
                or _sha256(source_label) != label_binding.get("sha256")
            ):
                raise ValueError(f"registered synthetic source changed: {sample_id}")
            counts = _count_classes(source_label)
            class_id = int(item.get("class_id", -1))
            expected_counts = Counter({class_id: 1})
            if counts != expected_counts or class_id not in (0, 1):
                raise ValueError(f"synthetic truth label changed: {sample_id}")
            image_hash = str(image_binding["sha256"])
            source_hash = str(item.get("source_image_sha256", ""))
            if image_hash in encoded_hashes[split] or source_hash in source_hashes[split]:
                raise ValueError(f"duplicate synthetic hash in {split}: {sample_id}")
            encoded_hashes[split].add(image_hash)
            source_hashes[split].add(source_hash)

            destination_split = "train" if split == "train" else "val"
            role = "synthetic_train" if split == "train" else "synthetic_heldout"
            if split == "train":
                train_counts.update(counts)
                source_counts["synthetic"] += 1
            else:
                validation_counts.update(counts)
            entries.append(
                _copy_registered(
                    source_image=source_image,
                    source_label=source_label,
                    destination_root=staging,
                    destination_split=destination_split,
                    destination_stem_prefix="synthetic__",
                    role=role,
                    source_image_display=f"images/{split}/{sample_id}.png",
                    source_label_display=f"labels/{split}/{sample_id}.txt",
                    counts=counts,
                )
            )

        if encoded_hashes["train"] & encoded_hashes["heldout"] or source_hashes["train"] & source_hashes["heldout"]:
            raise ValueError("synthetic train/heldout hash overlap")
        expected_sources = {key: int(value) for key, value in derivative["train_source_counts"].items()}
        if dict(source_counts) != expected_sources:
            raise ValueError(f"mixed training source counts mismatch: {dict(source_counts)}")
        expected_train = {key: int(value) for key, value in derivative["train_class_instance_counts"].items()}
        expected_validation = {key: int(value) for key, value in derivative["validation_class_instance_counts"].items()}
        if {"0": train_counts[0], "1": train_counts[1]} != expected_train:
            raise ValueError("mixed training class totals mismatch")
        if {"0": validation_counts[0], "1": validation_counts[1]} != expected_validation:
            raise ValueError("synthetic held-out class totals mismatch")
        if len([entry for entry in entries if entry["role"] != "synthetic_heldout"]) != int(derivative["train_image_count"]):
            raise ValueError("mixed training image count mismatch")
        if len([entry for entry in entries if entry["role"] == "synthetic_heldout"]) != int(derivative["validation_image_count"]):
            raise ValueError("synthetic held-out image count mismatch")

        yaml_path = staging / "dataset.yaml"
        yaml_path.write_text(
            "# ADR-0021: real train + synthetic train; synthetic held-out validation only.\n"
            "train: images/train\n"
            "val: images/val\n"
            "names:\n"
            "  0: ripe\n"
            "  1: unripe\n",
            encoding="utf-8",
        )
        canonical_rows = _canonical_rows(entries)
        manifest = {
            "schema_version": 1,
            "kind": "simulator_adaptation_training_derivative",
            "variant": contract["variant"],
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "contract": {
                "path": _display(contract_path),
                "size_bytes": contract_path.stat().st_size,
                "sha256": _sha256(contract_path),
            },
            "source_bindings": {
                key: {"path": _display(path), "size_bytes": path.stat().st_size, "sha256": _sha256(path)}
                for key, path in paths.items()
                if key not in ("authorization", "decision", "config", "base_weight")
            },
            "dataset_yaml_sha256": _sha256(yaml_path),
            "train_image_count": int(derivative["train_image_count"]),
            "validation_image_count": int(derivative["validation_image_count"]),
            "train_source_counts": expected_sources,
            "train_class_instance_counts": expected_train,
            "validation_class_instance_counts": expected_validation,
            "test_split_present": False,
            "audited_real_validation_copied": False,
            "formal_real_test_accessed": False,
            "entries": entries,
            "content_binding": {
                "algorithm": "sha256",
                "canonicalization": "selected-entry-fields-json-sort-keys-utf8-v1",
                "canonical_training_derivative_sha256": _canonical_sha256(canonical_rows),
            },
        }
        (staging / "training_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        staging.replace(output_dir)

    return verify_training_derivative(output_dir / "training_manifest.json", contract_path)


def verify_training_derivative(manifest_path: Path, contract_path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    contract, _ = _validate_contract(contract_path)
    manifest_path = _resolve(manifest_path).resolve()
    manifest = _load_object(manifest_path)
    output_dir = manifest_path.parent
    if manifest.get("kind") != "simulator_adaptation_training_derivative" or manifest.get("variant") != contract["variant"]:
        raise ValueError("unexpected simulator-adaptation derivative manifest")
    contract_binding = manifest.get("contract", {})
    if (
        contract_binding.get("path") != _display(contract_path)
        or contract_binding.get("size_bytes") != contract_path.stat().st_size
        or contract_binding.get("sha256") != _sha256(contract_path)
    ):
        raise ValueError("derivative contract binding mismatch")
    if manifest.get("test_split_present") is not False or manifest.get("formal_real_test_accessed") is not False:
        raise ValueError("derivative crossed the formal-test boundary")
    if manifest.get("audited_real_validation_copied") is not False:
        raise ValueError("audited real validation was exposed to the training derivative")
    if (output_dir / "images" / "test").exists() or (output_dir / "labels" / "test").exists():
        raise ValueError("derivative unexpectedly contains a test split")

    entries = manifest.get("entries")
    if not isinstance(entries, list):
        raise ValueError("derivative entries are missing")
    expected_total = int(contract["dataset_derivative"]["train_image_count"]) + int(contract["dataset_derivative"]["validation_image_count"])
    if len(entries) != expected_total:
        raise ValueError("derivative entry count mismatch")
    derived_images: set[str] = set()
    derived_labels: set[str] = set()
    for item in entries:
        for key, suffixes, seen in (
            ("derived_image", {".jpg", ".jpeg", ".png"}, derived_images),
            ("derived_label", {".txt"}, derived_labels),
        ):
            part = PurePosixPath(str(item[key]))
            if part.is_absolute() or ".." in part.parts or part.suffix.lower() not in suffixes:
                raise ValueError(f"unsafe derived path: {part}")
            if part.as_posix() in seen:
                raise ValueError(f"duplicate derived path: {part}")
            seen.add(part.as_posix())
            target = output_dir / part
            prefix = "derived_image" if key == "derived_image" else "derived_label"
            if (
                not target.is_file()
                or target.stat().st_size != int(item[f"{prefix}_size_bytes"])
                or _sha256(target) != item[f"{prefix}_sha256"]
            ):
                raise ValueError(f"derived file changed: {target}")
    canonical = _canonical_sha256(_canonical_rows(entries))
    if canonical != manifest.get("content_binding", {}).get("canonical_training_derivative_sha256"):
        raise ValueError("derivative canonical digest mismatch")
    yaml_path = output_dir / "dataset.yaml"
    if not yaml_path.is_file() or _sha256(yaml_path) != manifest.get("dataset_yaml_sha256"):
        raise ValueError("derivative dataset YAML changed")
    yaml_text = yaml_path.read_text(encoding="utf-8")
    if "test:" in yaml_text or "images/test" in yaml_text or "labels/test" in yaml_text:
        raise ValueError("dataset YAML exposes a test split")
    return {
        "variant": contract["variant"],
        "train_images": int(manifest["train_image_count"]),
        "validation_images": int(manifest["validation_image_count"]),
        "canonical_training_derivative_sha256": canonical,
        "audited_real_validation_copied": False,
        "formal_real_test_accessed": False,
        "training_started": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--verify", type=Path)
    arguments = parser.parse_args(argv)
    if arguments.verify is not None:
        result = verify_training_derivative(arguments.verify, arguments.contract)
    else:
        result = materialize(arguments.contract)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
