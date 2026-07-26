#!/usr/bin/env python3
"""Restore changed extracted images from the pinned ZIP, fail closed on hashes.

This is intentionally narrower than a full re-extraction: every expected raw
image hash comes from the frozen scene-group audit, every replacement is first
staged and verified, and only the mismatching files are atomically replaced.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import tempfile
from typing import Mapping, Sequence
import zipfile

from dataset_tools import IMAGE_SUFFIXES, file_digest, load_manifest, verify_file


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ARCHIVE = (
    REPOSITORY_ROOT / "data" / "raw" / "zenodo_6126677" / "strawberries.zip"
)
DEFAULT_SOURCE_ROOT = (
    REPOSITORY_ROOT
    / "data"
    / "raw"
    / "zenodo_6126677"
    / "extracted"
    / "strawberries"
)
DEFAULT_DATASET_MANIFEST = (
    REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677.json"
)
DEFAULT_AUDIT = (
    REPOSITORY_ROOT / "artifacts" / "data" / "zenodo_6126677_group_audit.json"
)
DEFAULT_REPORT = (
    REPOSITORY_ROOT
    / "artifacts"
    / "data"
    / "zenodo_6126677_extracted_repair.json"
)
DEFAULT_AUDIT_SHA256 = (
    "4d3cb231874fa3601a4f3830c4491c0e399e92b863e35c541be2e51cad9ca95b"
)


def _relative_image(value: object) -> str:
    relative = PurePosixPath(str(value).replace("\\", "/"))
    if (
        relative.is_absolute()
        or not relative.parts
        or ".." in relative.parts
        or relative.parts[0].endswith(":")
        or relative.suffix.lower() not in IMAGE_SUFFIXES
    ):
        raise ValueError(f"unsafe or unsupported audit image path: {value!r}")
    return relative.as_posix()


def _expected_images(audit_path: Path, expected_audit_sha256: str) -> Mapping[str, str]:
    actual_audit_sha256 = file_digest(audit_path, "sha256")
    if actual_audit_sha256 != expected_audit_sha256.lower():
        raise ValueError(
            "group audit SHA-256 mismatch: "
            f"expected {expected_audit_sha256.lower()}, got {actual_audit_sha256}"
        )
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    if audit.get("schema_version") != 1 or audit.get("dataset_id") != "zenodo_6126677":
        raise ValueError("unsupported group audit identity or schema")
    audit_input = audit.get("input")
    if not isinstance(audit_input, Mapping) or not isinstance(
        audit_input.get("images"), list
    ):
        raise ValueError("group audit does not contain input.images")
    expected = {}
    for item in audit_input["images"]:
        if not isinstance(item, Mapping):
            raise ValueError("group audit image entry must be an object")
        relative = _relative_image(item.get("path"))
        digest = str(item.get("sha256", "")).lower()
        if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
            raise ValueError(f"invalid SHA-256 for audit image {relative}")
        if relative in expected:
            raise ValueError(f"duplicate audit image path: {relative}")
        expected[relative] = digest
    if audit_input.get("image_count") != len(expected):
        raise ValueError("group audit image_count does not match input.images")
    return expected


def _verified_archive(
    archive: Path, dataset_manifest_path: Path
) -> tuple[Mapping[str, object], Mapping[str, object]]:
    manifest = load_manifest(dataset_manifest_path)
    if manifest.get("dataset_id") != "zenodo_6126677":
        raise ValueError("dataset manifest has the wrong dataset_id")
    artifacts = [item for item in manifest["files"] if item.get("name") == archive.name]
    if len(artifacts) != 1:
        raise ValueError(f"dataset manifest must declare {archive.name} exactly once")
    artifact = artifacts[0]
    checksum = artifact["checksum"]
    status = verify_file(
        archive,
        str(checksum["algorithm"]),
        str(checksum["value"]),
    )
    if not status["valid"]:
        raise ValueError(f"archive checksum failed: {status}")
    if archive.stat().st_size != int(artifact["size_bytes"]):
        raise ValueError("archive size differs from the dataset manifest")
    return manifest, status


def restore_changed_images(
    *,
    archive: Path,
    source_root: Path,
    dataset_manifest_path: Path,
    audit_path: Path,
    expected_audit_sha256: str,
    expected_mismatches: int,
) -> Mapping[str, object]:
    if expected_mismatches < 0:
        raise ValueError("expected_mismatches must be non-negative")
    archive = archive.resolve()
    source_root = source_root.resolve()
    audit_path = audit_path.resolve()
    expected = _expected_images(audit_path, expected_audit_sha256)
    manifest, archive_status = _verified_archive(archive, dataset_manifest_path)
    archive_metadata = manifest.get("archive")
    if not isinstance(archive_metadata, Mapping):
        raise ValueError("dataset manifest does not contain archive metadata")
    archive_root_path = PurePosixPath(str(archive_metadata.get("root_directory", "")))
    if (
        archive_root_path.is_absolute()
        or len(archive_root_path.parts) != 1
        or archive_root_path.parts[0] in {"", ".", ".."}
        or archive_root_path.parts[0].endswith(":")
    ):
        raise ValueError("archive root_directory must be one safe path segment")
    archive_root = archive_root_path.parts[0]

    actual_paths = {
        path.relative_to(source_root).as_posix()
        for path in source_root.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    }
    if actual_paths != set(expected):
        missing = sorted(set(expected) - actual_paths, key=str.lower)
        unexpected = sorted(actual_paths - set(expected), key=str.lower)
        raise ValueError(
            "extracted image set differs from the audit "
            f"(missing={missing[:3]}, unexpected={unexpected[:3]})"
        )

    mismatches = []
    for relative in sorted(expected, key=str.lower):
        path = source_root.joinpath(*PurePosixPath(relative).parts)
        actual = file_digest(path, "sha256")
        if actual != expected[relative]:
            mismatches.append(
                {
                    "path": relative,
                    "before_sha256": actual,
                    "expected_sha256": expected[relative],
                }
            )
    if len(mismatches) != expected_mismatches:
        raise ValueError(
            "raw mismatch count is not the explicitly authorized value: "
            f"expected {expected_mismatches}, found {len(mismatches)}"
        )

    with zipfile.ZipFile(archive) as bundle:
        member_counts = Counter(item.filename for item in bundle.infolist())
        members = {
            item["path"]: f"{archive_root}/{item['path']}" for item in mismatches
        }
        invalid_members = [
            member for member in members.values() if member_counts[member] != 1
        ]
        if invalid_members:
            raise ValueError(
                "archive replacement members must exist exactly once: "
                f"{invalid_members[:3]}"
            )

        with tempfile.TemporaryDirectory(
            prefix=".strawberry_restore_", dir=source_root.parent
        ) as temporary:
            staging = Path(temporary)
            staged = {}
            for index, item in enumerate(mismatches):
                relative = item["path"]
                staged_path = staging / f"{index:04d}.image"
                with bundle.open(members[relative]) as source, staged_path.open("wb") as output:
                    shutil.copyfileobj(source, output, length=8 * 1024 * 1024)
                staged_sha256 = file_digest(staged_path, "sha256")
                if staged_sha256 != item["expected_sha256"]:
                    raise ValueError(
                        f"archive member hash differs from audit for {relative}: "
                        f"{staged_sha256}"
                    )
                staged[relative] = staged_path

            for item in mismatches:
                relative = item["path"]
                destination = source_root.joinpath(*PurePosixPath(relative).parts)
                os.replace(staged[relative], destination)

    post_restore_mismatches = []
    for relative in sorted(expected, key=str.lower):
        path = source_root.joinpath(*PurePosixPath(relative).parts)
        actual = file_digest(path, "sha256")
        if actual != expected[relative]:
            post_restore_mismatches.append(relative)
    if post_restore_mismatches:
        raise RuntimeError(
            "post-restore audit failed for: " f"{post_restore_mismatches[:3]}"
        )

    return {
        "schema_version": 1,
        "dataset_id": "zenodo_6126677",
        "status": "restored" if mismatches else "already_clean",
        "archive": archive_status,
        "group_audit": {
            "path": str(audit_path),
            "sha256": expected_audit_sha256.lower(),
        },
        "source_root": str(source_root),
        "audited_image_count": len(expected),
        "restored_count": len(mismatches),
        "restored": mismatches,
        "post_restore_mismatch_count": 0,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT)
    parser.add_argument(
        "--dataset-manifest", type=Path, default=DEFAULT_DATASET_MANIFEST
    )
    parser.add_argument("--audit", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument(
        "--expected-audit-sha256", default=DEFAULT_AUDIT_SHA256
    )
    parser.add_argument("--expected-mismatches", type=int, required=True)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    report = restore_changed_images(
        archive=args.archive,
        source_root=args.source_root,
        dataset_manifest_path=args.dataset_manifest,
        audit_path=args.audit,
        expected_audit_sha256=args.expected_audit_sha256,
        expected_mismatches=args.expected_mismatches,
    )
    serialised = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    report_path = args.report.resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = report_path.with_name(f".{report_path.name}.tmp")
    temporary.write_text(serialised, encoding="utf-8")
    os.replace(temporary, report_path)
    print(serialised, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
