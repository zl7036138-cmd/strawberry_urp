#!/usr/bin/env python3
"""Download and checksum the dataset declared by its immutable manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from dataset_tools import download_file, extract_zip_safely, load_manifest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677.json"
DEFAULT_OUTPUT = REPOSITORY_ROOT / "data" / "raw" / "zenodo_6126677"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--extract-dir", type=Path)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    manifest = load_manifest(args.manifest)
    statuses = []
    for artifact in manifest["files"]:
        checksum = artifact["checksum"]
        destination = args.output_dir / artifact["name"]
        status = download_file(
            artifact["url"],
            destination,
            checksum["algorithm"],
            checksum["value"],
            force=args.force,
        )
        statuses.append(status)
        if args.extract_dir is not None:
            extract_zip_safely(destination, args.extract_dir, force=args.force)
    print(json.dumps({"dataset_id": manifest["dataset_id"], "files": statuses}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
