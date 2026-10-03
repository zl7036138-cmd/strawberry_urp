#!/usr/bin/env python3
"""Create a commit-bound final source/evidence archive from a clean Git tree."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARCHIVE = (
    ROOT
    / "artifacts"
    / "final_delivery"
    / "strawberry_urp_final_2026-10.zip"
)


def _git(*arguments: str) -> str:
    return subprocess.check_output(
        ["git", *arguments], cwd=ROOT, text=True
    ).strip()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument(
        "--prefix",
        default="strawberry_urp/",
        help="top-level directory stored in the ZIP",
    )
    args = parser.parse_args()

    dirty = _git("status", "--porcelain=v1", "--untracked-files=all")
    if dirty:
        raise SystemExit("refusing to package a dirty working tree")

    head = _git("rev-parse", "HEAD")
    branch = _git("branch", "--show-current")
    archive = args.archive if args.archive.is_absolute() else ROOT / args.archive
    archive = archive.resolve()
    archive.parent.mkdir(parents=True, exist_ok=True)
    temporary = archive.with_suffix(f"{archive.suffix}.tmp")

    subprocess.run(
        [
            "git",
            "archive",
            "--format=zip",
            f"--prefix={args.prefix}",
            f"--output={temporary}",
            head,
        ],
        cwd=ROOT,
        check=True,
    )
    temporary.replace(archive)

    with zipfile.ZipFile(archive) as package:
        bad_member = package.testzip()
        members = package.infolist()
    if bad_member is not None:
        raise SystemExit(f"ZIP CRC verification failed: {bad_member}")

    receipt = {
        "schema_version": 1,
        "kind": "strawberry_urp_final_delivery_receipt",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS",
        "git": {"commit": head, "branch": branch, "clean": True},
        "archive": {
            "path": archive.relative_to(ROOT).as_posix(),
            "bytes": archive.stat().st_size,
            "sha256": _sha256(archive),
            "member_count": len(members),
            "crc_verified": True,
            "prefix": args.prefix,
        },
        "scope": {
            "tracked_repository_files": True,
            "includes_generated_build_trees": False,
            "includes_ignored_training_data": False,
            "includes_ignored_model_weights": False,
            "note": (
                "The archive is the exact tracked Git tree. Externally managed "
                "datasets, model weights, and generated build products must be "
                "restored using the repository manifests and reproduction guide."
            ),
        },
    }
    receipt_path = archive.with_suffix(".receipt.json")
    receipt_path.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
