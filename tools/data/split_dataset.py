#!/usr/bin/env python3
"""Create a deterministic two-class YOLO view from an extracted source."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from dataset_tools import (
    DatasetVerificationError,
    SplitValidationError,
    make_splits,
    materialize_splits,
    verify_materialized_split_manifest,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = REPOSITORY_ROOT / "data" / "raw" / "zenodo_6126677" / "extracted"
DEFAULT_OUTPUT = REPOSITORY_ROOT / "data" / "processed" / "zenodo_6126677"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--groups-csv", type=Path)
    parser.add_argument(
        "--exclusions-csv",
        type=Path,
        help=(
            "audited image,reason,representative exclusions; valid only with "
            "--resplit-all"
        ),
    )
    parser.add_argument("--seed", type=int, default=20260710)
    parser.add_argument(
        "--resplit-all",
        action="store_true",
        help=(
            "combine all source directories and apply the grouped 70/15/15 "
            "policy instead of preserving an official split"
        ),
    )
    parser.add_argument(
        "--image-mode",
        choices=("hardlink", "copy"),
        default="copy",
        help=(
            "materialize isolated copies by default; hardlink remains available "
            "for diagnostics, while incomplete JPEGs are always copied and normalized"
        ),
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="verify output-dir/split_manifest.json without creating a split",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.verify_only:
        try:
            verification = verify_materialized_split_manifest(
                args.output_dir / "split_manifest.json"
            )
        except DatasetVerificationError as error:
            print(json.dumps(error.report, indent=2, ensure_ascii=False))
            return 2
        print(json.dumps(verification, indent=2, ensure_ascii=False))
        return 0
    try:
        dataset_root, splits, strategy = make_splits(
            args.source,
            args.seed,
            args.groups_csv,
            resplit_all=args.resplit_all,
            exclusions_csv=args.exclusions_csv,
        )
        report = materialize_splits(
            dataset_root,
            splits,
            args.output_dir,
            strategy,
            seed=args.seed,
            image_mode=args.image_mode,
            force=args.force,
            exclusions_csv=args.exclusions_csv,
        )
    except SplitValidationError as error:
        print(
            json.dumps(
                {
                    "dataset_id": "zenodo_6126677",
                    "validation": error.report,
                },
                indent=2,
            )
        )
        return 2
    except (FileNotFoundError, ValueError) as error:
        print(
            json.dumps(
                {
                    "dataset_id": "zenodo_6126677",
                    "validation": {
                        "schema_version": 1,
                        "valid": False,
                        "violations": [
                            {
                                "code": "SPLIT_GENERATION_ERROR",
                                "message": str(error),
                            }
                        ],
                    },
                },
                indent=2,
            )
        )
        return 2
    summary = {
        "dataset_id": report["dataset_id"],
        "strategy": report["strategy"],
        "seed": report["seed"],
        "counts": report["counts"],
        "exclusions_csv": report["exclusions_csv"],
        "exclusions_csv_sha256": report["exclusions_csv_sha256"],
        "source_image_count": report["source_image_count"],
        "excluded_count": report["excluded_count"],
        "retained_count": report["retained_count"],
        "output_root": report["output_root"],
        "validation": report["validation"],
        "canonical_dataset_sha256": report["content_binding"][
            "canonical_dataset_sha256"
        ],
        "canonical_split_sha256": report["content_binding"][
            "canonical_split_sha256"
        ],
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
