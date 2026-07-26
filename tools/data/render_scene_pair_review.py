#!/usr/bin/env python3
"""Render ranked candidate pairs from a scene-group audit for human review."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Mapping, Sequence

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
DEFAULT_AUDIT = (
    REPOSITORY_ROOT / "artifacts" / "data" / "zenodo_6126677_group_audit.json"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "artifacts" / "data" / "scene_pair_review.jpg"
)


def _image(path: Path) -> np.ndarray:
    result = cv2.imdecode(
        np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    if result is None:
        raise ValueError(f"OpenCV could not decode {path}")
    return result


def _thumbnail(image: np.ndarray, width: int, height: int) -> np.ndarray:
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, height / source_height)
    resized = cv2.resize(
        image,
        (max(1, round(source_width * scale)), max(1, round(source_height * scale))),
        interpolation=cv2.INTER_AREA,
    )
    y = (height - resized.shape[0]) // 2
    x = (width - resized.shape[1]) // 2
    canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    return canvas


def _score(pair: Mapping[str, object], ranking: str) -> float:
    if ranking == "embedding":
        value = pair.get("embedding_similarity")
        return -1.0 if value is None else float(value)
    if ranking == "inliers":
        return float(pair["inliers"])
    if ranking == "phash":
        return -float(pair["phash_distance"])
    raise ValueError(f"unsupported ranking: {ranking}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--audit-json", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--decision", default="separate_no_geometric_evidence")
    parser.add_argument("--ranking", choices=("embedding", "inliers", "phash"), default="embedding")
    parser.add_argument("--limit", type=int, default=60)
    parser.add_argument("--pair-width", type=int, default=520)
    parser.add_argument("--pair-height", type=int, default=190)
    parser.add_argument("--columns", type=int, default=2)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.limit < 1 or args.columns < 1:
        raise ValueError("limit and columns must be positive")
    audit = json.loads(args.audit_json.read_text(encoding="utf-8"))
    pairs = [
        pair for pair in audit["pairs"] if pair["decision"] == args.decision
    ]
    pairs.sort(
        key=lambda pair: (
            -_score(pair, args.ranking),
            str(pair["image_a"]),
            str(pair["image_b"]),
        )
    )
    pairs = pairs[: args.limit]
    rows = (len(pairs) + args.columns - 1) // args.columns
    canvas = np.full(
        (rows * args.pair_height, args.columns * args.pair_width, 3),
        245,
        dtype=np.uint8,
    )
    label_height = 42
    half_width = args.pair_width // 2
    for index, pair in enumerate(pairs):
        left = _thumbnail(
            _image(args.source / pair["image_a"]),
            half_width,
            args.pair_height - label_height,
        )
        right = _thumbnail(
            _image(args.source / pair["image_b"]),
            args.pair_width - half_width,
            args.pair_height - label_height,
        )
        cell = np.full((args.pair_height, args.pair_width, 3), 245, dtype=np.uint8)
        cell[: args.pair_height - label_height, :half_width] = left
        cell[: args.pair_height - label_height, half_width:] = right
        similarity = pair.get("embedding_similarity")
        similarity_text = "n/a" if similarity is None else f"{float(similarity):.4f}"
        labels = (
            f"{index + 1}: {pair['image_a']} | {pair['image_b']}",
            f"emb={similarity_text} p={pair['phash_distance']} "
            f"good={pair['good_matches']} inliers={pair['inliers']}",
        )
        for line, text in enumerate(labels):
            cv2.putText(
                cell,
                text,
                (5, args.pair_height - 24 + line * 17),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
        row, column = divmod(index, args.columns)
        y = row * args.pair_height
        x = column * args.pair_width
        canvas[y : y + args.pair_height, x : x + args.pair_width] = cell
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), canvas, [cv2.IMWRITE_JPEG_QUALITY, 90]):
        raise OSError(f"failed to write {args.output}")
    print(f"rendered {len(pairs)} pairs to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
