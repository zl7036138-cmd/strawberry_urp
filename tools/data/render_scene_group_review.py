#!/usr/bin/env python3
"""Render non-singleton scene groups as labelled contact sheets for review."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = (
    REPOSITORY_ROOT
    / "data"
    / "raw"
    / "zenodo_6126677"
    / "extracted"
    / "strawberries"
)
DEFAULT_GROUPS = (
    REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677_groups.csv"
)
DEFAULT_OUTPUT = REPOSITORY_ROOT / "artifacts" / "data" / "scene_group_review"


def _read_image(path: Path) -> np.ndarray:
    encoded = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"OpenCV could not decode {path}")
    return image


def _cell(image: np.ndarray, label: str, width: int, height: int) -> np.ndarray:
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    label_height = 26
    available_height = height - label_height
    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, available_height / source_height)
    resized = cv2.resize(
        image,
        (max(1, round(source_width * scale)), max(1, round(source_height * scale))),
        interpolation=cv2.INTER_AREA,
    )
    y = (available_height - resized.shape[0]) // 2
    x = (width - resized.shape[1]) // 2
    canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    cv2.putText(
        canvas,
        label,
        (5, height - 7),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (15, 15, 15),
        1,
        cv2.LINE_AA,
    )
    return canvas


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--groups-csv", type=Path, default=DEFAULT_GROUPS)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--columns", type=int, default=5)
    parser.add_argument("--cell-width", type=int, default=240)
    parser.add_argument("--cell-height", type=int, default=180)
    parser.add_argument("--include-singletons", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.columns < 1 or args.cell_width < 80 or args.cell_height < 80:
        raise ValueError("contact-sheet dimensions are too small")
    groups: dict[str, list[str]] = defaultdict(list)
    with args.groups_csv.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not {"image", "group_id"}.issubset(
            reader.fieldnames
        ):
            raise ValueError("groups CSV must have image and group_id columns")
        for row in reader:
            groups[row["group_id"]].append(row["image"])
    selected = [
        (group_id, sorted(paths, key=str.casefold))
        for group_id, paths in groups.items()
        if args.include_singletons or len(paths) > 1
    ]
    selected.sort(key=lambda item: (-len(item[1]), item[0]))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    index_rows = [("rank", "group_id", "image_count", "contact_sheet")]
    for rank, (group_id, paths) in enumerate(selected, start=1):
        rows = (len(paths) + args.columns - 1) // args.columns
        canvas = np.full(
            (rows * args.cell_height, args.columns * args.cell_width, 3),
            245,
            dtype=np.uint8,
        )
        for index, relative_path in enumerate(paths):
            image = _read_image(args.source / relative_path)
            cell = _cell(image, relative_path, args.cell_width, args.cell_height)
            row, column = divmod(index, args.columns)
            y = row * args.cell_height
            x = column * args.cell_width
            canvas[y : y + args.cell_height, x : x + args.cell_width] = cell
        filename = f"{rank:04d}_{len(paths):03d}_{group_id}.jpg"
        destination = args.output_dir / filename
        if not cv2.imwrite(str(destination), canvas, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise OSError(f"failed to write {destination}")
        index_rows.append((str(rank), group_id, str(len(paths)), filename))
    with (args.output_dir / "index.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        csv.writer(stream, lineterminator="\n").writerows(index_rows)
    print(
        f"rendered {len(selected)} group contact sheets under {args.output_dir}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
