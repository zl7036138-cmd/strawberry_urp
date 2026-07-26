#!/usr/bin/env python3
"""Create a deterministic compact P6 delivery archive and receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import zipfile
from typing import Any

from verify_p5_release import verify


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARCHIVE = (
    REPOSITORY_ROOT / "artifacts" / "p6" / "strawberry_urp_release_v1.zip"
)
SOURCE_ROOTS = (
    "ros2_ws/src",
    "scripts",
    "config",
    "requirements",
    "tools",
    "docs",
)
ADDITIONAL_FILES = (
    "README.md",
    ".gitignore",
    ".cache/wheels/wheelhouse-manifest.json",
    "outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt",
    "outputs/perception/yolo11s_640_sim_adapt_v1/weights/best.pt",
    "artifacts/p5/p5_clean_reproduction_handoff_v1.json",
    "artifacts/p5/p5_final_release_handoff_v1.json",
    "artifacts/p5/video/strawberry_urp_demo_v1.mp4",
    "artifacts/p5/video/strawberry_urp_demo_v1.receipt.json",
    "artifacts/p5/video/strawberry_urp_demo_v1.source-metrics.json",
    "artifacts/p5/release_v1/evidence_manifest_v1.json",
)
FIXED_ZIP_TIME = (2026, 7, 24, 0, 0, 0)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative(path: Path) -> str:
    return path.resolve().relative_to(REPOSITORY_ROOT.resolve()).as_posix()


def _include_source(path: Path) -> bool:
    relative = _relative(path)
    parts = Path(relative).parts
    if "__pycache__" in parts:
        return False
    if path.suffix in {".pyc", ".pyo"}:
        return False
    return path.is_file()


def _bound_release_files() -> list[Path]:
    manifest_path = (
        REPOSITORY_ROOT / "artifacts" / "p5" / "release_v1"
        / "evidence_manifest_v1.json"
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    rows = [*manifest["inputs"], *manifest["outputs"], manifest["generator"]]
    return [REPOSITORY_ROOT / row["path"] for row in rows]


def _collect() -> list[Path]:
    paths: set[Path] = set()
    for relative_root in SOURCE_ROOTS:
        root = REPOSITORY_ROOT / relative_root
        paths.update(path for path in root.rglob("*") if _include_source(path))
    paths.update(REPOSITORY_ROOT / path for path in ADDITIONAL_FILES)
    paths.update(_bound_release_files())
    missing = sorted(_relative(path) for path in paths if not path.is_file())
    if missing:
        raise ValueError(f"delivery files are missing: {missing}")
    return sorted(paths, key=_relative)


def package(archive: Path) -> dict[str, Any]:
    verification = verify(REPOSITORY_ROOT / "artifacts" / "p5" / "release_v1")
    if verification["status"] != "VERIFIED_FINAL_RELEASE":
        raise ValueError("refusing to package an unverified P5 release")
    handoff = json.loads(
        (
            REPOSITORY_ROOT
            / "artifacts"
            / "p5"
            / "p5_final_release_handoff_v1.json"
        ).read_text(encoding="utf-8")
    )
    if handoff["status"] != "P5_COMPLETE_P6_DELIVERY_ONLY":
        raise ValueError("final P5 handoff status differs")

    files = _collect()
    inventory = [
        {
            "path": _relative(path),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
        }
        for path in files
    ]
    embedded_manifest = {
        "schema_version": 1,
        "kind": "p6_delivery_inventory",
        "release_id": "strawberry_urp_release_v1",
        "file_count": len(inventory),
        "uncompressed_size_bytes": sum(row["size_bytes"] for row in inventory),
        "files": inventory,
        "omitted_large_artifacts": [
            "data/raw/zenodo_6126677/strawberries.zip",
            ".cache/wheels/*.whl",
            "ros2_ws/build",
            "ros2_ws/install",
            "ros2_ws/log",
            "complete per-trial runtime directories beyond frozen summaries",
        ],
        "safety": {
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_repeated": False,
            "physical_robot_claim": False,
            "sim_to_real_claim": False,
        },
    }
    manifest_bytes = (
        json.dumps(
            embedded_manifest, ensure_ascii=False, indent=2, sort_keys=True
        )
        + "\n"
    ).encode("utf-8")

    archive = archive.resolve()
    archive.parent.mkdir(parents=True, exist_ok=True)
    temporary = archive.with_suffix(f"{archive.suffix}.tmp")
    if temporary.exists():
        temporary.unlink()
    with zipfile.ZipFile(
        temporary, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9
    ) as output:
        for path in files:
            info = zipfile.ZipInfo(_relative(path), FIXED_ZIP_TIME)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            output.writestr(info, path.read_bytes(), compresslevel=9)
        info = zipfile.ZipInfo("DELIVERY_INVENTORY.json", FIXED_ZIP_TIME)
        info.compress_type = zipfile.ZIP_DEFLATED
        info.external_attr = 0o100644 << 16
        output.writestr(info, manifest_bytes, compresslevel=9)
    temporary.replace(archive)

    with zipfile.ZipFile(archive) as value:
        bad_member = value.testzip()
        names = value.namelist()
    if bad_member is not None:
        raise ValueError(f"ZIP CRC verification failed: {bad_member}")
    if len(names) != len(files) + 1:
        raise ValueError("ZIP member count differs")

    receipt = {
        "schema_version": 1,
        "kind": "p6_delivery_archive_receipt",
        "status": "PASS",
        "release_id": "strawberry_urp_release_v1",
        "archive": {
            "path": _relative(archive),
            "size_bytes": archive.stat().st_size,
            "sha256": _sha256(archive),
            "member_count": len(names),
            "crc_verified": True,
        },
        "inventory": {
            "embedded_path": "DELIVERY_INVENTORY.json",
            "file_count": len(inventory),
            "uncompressed_size_bytes": embedded_manifest[
                "uncompressed_size_bytes"
            ],
            "sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        },
        "verification": verification,
        "scope": {
            "p5_complete": True,
            "p6_packaging_complete": True,
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_repeated": False,
        },
    }
    receipt_path = archive.with_suffix(".receipt.json")
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    arguments = parser.parse_args()
    result = package(arguments.archive)
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
