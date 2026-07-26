#!/usr/bin/env python3
"""Materialize ADR-0009's hash-bound train/audited-val derivative."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import sys
import tempfile
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from materialize_audited_validation import verify_audited_derivative  # noqa: E402


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT = Path("tools/perception/audited_opt2_contract.json")


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


def _verify_binding(binding: Mapping[str, Any], *, label: str) -> Path:
    path = _resolve(Path(str(binding.get("path", "")))).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"missing {label}: {path}")
    if path.stat().st_size != int(binding.get("size_bytes", -1)):
        raise ValueError(f"{label} size binding mismatch")
    if _sha256(path) != binding.get("sha256"):
        raise ValueError(f"{label} hash binding mismatch")
    return path


def _safe_split_path(value: str, kind: str, split: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if path.parent != PurePosixPath(kind, split):
        raise ValueError(f"unsafe {split} {kind} path: {value}")
    suffixes = {".jpg", ".jpeg", ".png"} if kind == "images" else {".txt"}
    if path.suffix.lower() not in suffixes:
        raise ValueError(f"unsupported {kind} suffix: {value}")
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


def _link_or_copy(source: Path, destination: Path) -> str:
    try:
        os.link(source, destination)
        return "hardlink"
    except OSError:
        shutil.copy2(source, destination)
        return "copy"


def _canonical_rows(entries: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "role": item["role"],
            "source_image": item["source_image"],
            "source_image_sha256": item["source_image_sha256"],
            "source_label": item["source_label"],
            "source_label_sha256": item["source_label_sha256"],
            "derived_image": item["derived_image"],
            "derived_image_sha256": item["derived_image_sha256"],
            "derived_label": item["derived_label"],
            "derived_label_sha256": item["derived_label_sha256"],
            "class_counts": item["class_counts"],
        }
        for item in sorted(entries, key=lambda row: (str(row["role"]), str(row["derived_image"])))
    ]


def _validate_contract(contract_path: Path) -> tuple[dict[str, Any], dict[str, Path]]:
    contract_path = contract_path.resolve()
    contract = _load_object(contract_path)
    if contract.get("kind") != "targeted_unripe_exposure_training_contract":
        raise ValueError("unexpected targeted-training contract kind")
    if contract.get("status") != "AUTHORIZED":
        raise ValueError("targeted-training contract is not authorized")
    if contract.get("formal_test", {}).get("access_authorized") is not False:
        raise ValueError("contract does not preserve the formal-test seal")

    authorization_path = _resolve(Path(str(contract["authorization_receipt"]))).resolve()
    authorization = _load_object(authorization_path)
    if authorization.get("kind") != "targeted_training_user_authorization":
        raise ValueError("unexpected training authorization kind")
    if authorization.get("status") != "AUTHORIZED" or authorization.get("new_training_authorized") is not True:
        raise ValueError("new training is not user-authorized")
    if authorization.get("formal_test_authorized") is not False or authorization.get("test_split_access_authorized") is not False:
        raise ValueError("authorization does not preserve the test seal")
    contract_binding = authorization.get("contract", {})
    if (
        contract_binding.get("path") != _display(contract_path)
        or contract_binding.get("size_bytes") != contract_path.stat().st_size
        or contract_binding.get("sha256") != _sha256(contract_path)
    ):
        raise ValueError("authorization contract binding mismatch")

    decision_path = _verify_binding(authorization["decision_record"], label="ADR-0009")
    if _display(decision_path) != contract.get("decision_record"):
        raise ValueError("contract decision-record path mismatch")
    config_path = _verify_binding(authorization["training_config"], label="Opt2 training config")
    if _display(config_path) != contract.get("training", {}).get("config"):
        raise ValueError("contract training-config path mismatch")
    upstream_path = _verify_binding(contract["upstream_handoff"], label="audited T30 handoff")
    split_manifest_path = _verify_binding(contract["source_split_manifest"], label="source split manifest")
    audited_manifest_path = _verify_binding(contract["audited_validation_manifest"], label="audited validation manifest")
    verification = verify_audited_derivative(audited_manifest_path)
    if verification["canonical_validation_derivative_sha256"] != contract["audited_validation_manifest"]["canonical_sha256"]:
        raise ValueError("audited validation canonical binding mismatch")
    return contract, {
        "authorization": authorization_path,
        "decision": decision_path,
        "config": config_path,
        "upstream": upstream_path,
        "split_manifest": split_manifest_path,
        "audited_manifest": audited_manifest_path,
    }


def materialize(contract_path: Path) -> dict[str, Any]:
    contract_path = _resolve(contract_path).resolve()
    contract, paths = _validate_contract(contract_path)
    derivative = contract["dataset_derivative"]
    output_dir = _resolve(Path(str(derivative["output_dir"]))).resolve()
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite training derivative: {output_dir}")
    if derivative.get("test_split_present") is not False:
        raise ValueError("training derivative contract must omit test")

    split_manifest = _load_object(paths["split_manifest"])
    if split_manifest.get("content_binding", {}).get("canonical_split_sha256", {}).get("train") != contract["source_split_manifest"]["canonical_train_sha256"]:
        raise ValueError("source train canonical digest mismatch")
    train_items = split_manifest.get("splits", {}).get("train")
    if not isinstance(train_items, list) or len(train_items) != int(derivative["original_train_image_count"]):
        raise ValueError("source training count mismatch")
    source_root = paths["split_manifest"].parent

    audited_manifest = _load_object(paths["audited_manifest"])
    val_items = audited_manifest.get("entries")
    if not isinstance(val_items, list) or len(val_items) != int(derivative["validation_image_count"]):
        raise ValueError("audited validation count mismatch")
    audited_root = paths["audited_manifest"].parent

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".unripe-x2-staging-", dir=output_dir.parent) as temporary:
        staging = Path(temporary)
        for split in ("train", "val"):
            (staging / "images" / split).mkdir(parents=True)
            (staging / "labels" / split).mkdir(parents=True)

        entries: list[dict[str, Any]] = []
        original_counts: Counter[int] = Counter()
        effective_counts: Counter[int] = Counter()
        unripe_image_count = 0
        image_modes: Counter[str] = Counter()

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
            counts = _count_classes(source_label)
            registered_counts = source_item.get("v1_class_counts", {})
            if counts[0] != int(registered_counts.get("0", 0)) or counts[1] != int(registered_counts.get("1", 0)):
                raise ValueError(f"registered train class count changed: {source_label}")
            original_counts.update(counts)

            copies = [("train_original", image_part.name, label_part.name)]
            if counts[1] > 0:
                unripe_image_count += 1
                copies.append(("train_unripe_duplicate", f"unripe_x2__{image_part.name}", f"unripe_x2__{label_part.name}"))
            for role, image_name, label_name in copies:
                derived_image_part = PurePosixPath("images", "train", image_name)
                derived_label_part = PurePosixPath("labels", "train", label_name)
                derived_image = staging / derived_image_part
                derived_label = staging / derived_label_part
                mode = _link_or_copy(source_image, derived_image)
                image_modes[mode] += 1
                shutil.copy2(source_label, derived_label)
                effective_counts.update(counts)
                entries.append(
                    {
                        "role": role,
                        "source_image": image_part.as_posix(),
                        "source_image_sha256": _sha256(source_image),
                        "source_label": label_part.as_posix(),
                        "source_label_sha256": _sha256(source_label),
                        "derived_image": derived_image_part.as_posix(),
                        "derived_image_sha256": _sha256(derived_image),
                        "derived_image_size_bytes": derived_image.stat().st_size,
                        "derived_label": derived_label_part.as_posix(),
                        "derived_label_sha256": _sha256(derived_label),
                        "derived_label_size_bytes": derived_label.stat().st_size,
                        "class_counts": {"0": counts[0], "1": counts[1]},
                        "image_materialization": mode,
                    }
                )

        expected_original = derivative["original_train_class_counts"]
        expected_effective = derivative["effective_train_class_counts"]
        if {"0": original_counts[0], "1": original_counts[1]} != expected_original:
            raise ValueError("original training class totals do not match the frozen contract")
        if {"0": effective_counts[0], "1": effective_counts[1]} != expected_effective:
            raise ValueError("effective training class totals do not match the frozen contract")
        if unripe_image_count != int(derivative["unripe_containing_image_count"]):
            raise ValueError("unripe-containing image count does not match the frozen contract")
        if len(entries) != int(derivative["effective_train_image_count"]):
            raise ValueError("effective training image count does not match the frozen contract")

        for item in sorted(val_items, key=lambda row: str(row["image"])):
            image_part = _safe_split_path(str(item["image"]), "images", "val")
            label_part = _safe_split_path(str(item["audited_label"]), "labels", "val")
            source_image = source_root / image_part
            source_label = audited_root / label_part
            if not source_image.is_file() or _sha256(source_image) != item.get("source_image_sha256") or source_image.stat().st_size != item.get("source_image_size_bytes"):
                raise ValueError(f"registered validation image changed: {source_image}")
            if not source_label.is_file() or _sha256(source_label) != item.get("audited_label_sha256") or source_label.stat().st_size != item.get("audited_label_size_bytes"):
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
                }
            )

        dataset_yaml = staging / "dataset.yaml"
        dataset_yaml.write_text(
            "# ADR-0009 derivative; deliberately contains no test split.\n"
            "train: images/train\n"
            "val: images/val\n"
            "names:\n"
            "  0: ripe\n"
            "  1: unripe\n",
            encoding="utf-8",
        )
        canonical_digest = _canonical_sha256(_canonical_rows(entries))
        manifest = {
            "schema_version": 1,
            "kind": "unripe_exposure_training_derivative",
            "variant": contract["variant"],
            "oversample_rule": derivative["oversample_rule"],
            "bindings": {
                "contract": {"path": _display(contract_path), "size_bytes": contract_path.stat().st_size, "sha256": _sha256(contract_path)},
                "authorization": {"path": _display(paths["authorization"]), "size_bytes": paths["authorization"].stat().st_size, "sha256": _sha256(paths["authorization"])},
                "decision_record": {"path": _display(paths["decision"]), "size_bytes": paths["decision"].stat().st_size, "sha256": _sha256(paths["decision"])},
                "source_split_manifest": {"path": _display(paths["split_manifest"]), "size_bytes": paths["split_manifest"].stat().st_size, "sha256": _sha256(paths["split_manifest"])},
                "audited_validation_manifest": {"path": _display(paths["audited_manifest"]), "size_bytes": paths["audited_manifest"].stat().st_size, "sha256": _sha256(paths["audited_manifest"])},
                "materializer": {"path": _display(Path(__file__)), "size_bytes": Path(__file__).stat().st_size, "sha256": _sha256(Path(__file__))},
            },
            "dataset_yaml": {"path": "dataset.yaml", "size_bytes": dataset_yaml.stat().st_size, "sha256": _sha256(dataset_yaml)},
            "counts": {
                "original_train_images": int(derivative["original_train_image_count"]),
                "unripe_containing_images": unripe_image_count,
                "duplicate_train_exposures": unripe_image_count,
                "effective_train_images": int(derivative["effective_train_image_count"]),
                "audited_validation_images": len(val_items),
                "original_train_classes": {"0": original_counts[0], "1": original_counts[1]},
                "effective_train_classes": {"0": effective_counts[0], "1": effective_counts[1]},
            },
            "image_materialization_counts": dict(sorted(image_modes.items())),
            "entries": entries,
            "content_binding": {
                "algorithm": "sha256",
                "canonicalization": "json-sort-keys-utf8-v1",
                "canonical_training_derivative_sha256": canonical_digest,
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


def verify_training_derivative(manifest_path: Path, contract_path: Path = DEFAULT_CONTRACT) -> dict[str, Any]:
    manifest_path = _resolve(manifest_path).resolve()
    contract_path = _resolve(contract_path).resolve()
    contract, _ = _validate_contract(contract_path)
    manifest = _load_object(manifest_path)
    if manifest.get("kind") != "unripe_exposure_training_derivative":
        raise ValueError("unexpected training derivative kind")
    if manifest.get("variant") != contract.get("variant"):
        raise ValueError("training derivative variant mismatch")
    if manifest.get("test_split_present") is not False or manifest.get("test_split_accessed") is not False:
        raise ValueError("training derivative does not preserve the test seal")
    if (manifest_path.parent / "images" / "test").exists() or (manifest_path.parent / "labels" / "test").exists():
        raise ValueError("training derivative unexpectedly contains test files")
    binding = manifest.get("bindings", {}).get("contract", {})
    if binding.get("sha256") != _sha256(contract_path) or binding.get("size_bytes") != contract_path.stat().st_size:
        raise ValueError("training derivative contract binding mismatch")
    dataset_yaml = manifest_path.parent / str(manifest["dataset_yaml"]["path"])
    if not dataset_yaml.is_file() or _sha256(dataset_yaml) != manifest["dataset_yaml"]["sha256"] or dataset_yaml.stat().st_size != manifest["dataset_yaml"]["size_bytes"]:
        raise ValueError("training derivative dataset YAML changed")
    if "test:" in dataset_yaml.read_text(encoding="utf-8"):
        raise ValueError("training derivative YAML contains a test key")

    entries = manifest.get("entries")
    expected_total = int(contract["dataset_derivative"]["effective_train_image_count"]) + int(contract["dataset_derivative"]["validation_image_count"])
    if not isinstance(entries, list) or len(entries) != expected_total:
        raise ValueError("training derivative entry count mismatch")
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
                raise ValueError(f"training derivative content changed: {path}")
        _count_classes(manifest_path.parent / str(item["derived_label"]))
    canonical = _canonical_sha256(_canonical_rows(entries))
    if canonical != manifest.get("content_binding", {}).get("canonical_training_derivative_sha256"):
        raise ValueError("training derivative canonical digest mismatch")
    return {
        "valid": True,
        "variant": manifest["variant"],
        "entry_count": len(entries),
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
