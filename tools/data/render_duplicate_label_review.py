#!/usr/bin/env python3
"""Render the conflicting labels of the near-identical cross-directory pairs.

The scene-group audit contains 65 pairs that are the same 4000x3000 source
photograph under different JPEG encodings.  This script reproduces that
selection from the recorded geometry, identifies the 16 pairs whose v1 labels
cannot be matched one-to-one at IoU 0.5 with the same class, and renders both
label sets over the same deterministic representative image.

Raw images and labels are read-only.  The output JPEG is for human review; its
companion CSV records the exact selection, matching, crop, and box identities.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
import math
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
    REPOSITORY_ROOT / "artifacts" / "data" / "duplicate_label_review.jpg"
)
DEFAULT_INDEX = (
    REPOSITORY_ROOT
    / "artifacts"
    / "data"
    / "duplicate_label_review_index.csv"
)

CLASS_NAMES = {0: "RIPE", 1: "UNRIPE"}
# OpenCV uses BGR.  The requested review convention is RIPE=red, UNRIPE=green.
CLASS_COLORS = {0: (0, 0, 235), 1: (0, 185, 0)}
IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"})


@dataclass(frozen=True)
class LabelBox:
    class_id: int
    x: float
    y: float
    width: float
    height: float
    sequence: int
    source_line: int

    @property
    def name(self) -> str:
        return CLASS_NAMES[self.class_id]

    def pixel_xyxy(self, image_width: int, image_height: int) -> tuple[float, ...]:
        return (
            (self.x - self.width / 2.0) * image_width,
            (self.y - self.height / 2.0) * image_height,
            (self.x + self.width / 2.0) * image_width,
            (self.y + self.height / 2.0) * image_height,
        )


def _read_image(path: Path) -> np.ndarray:
    image = cv2.imdecode(
        np.frombuffer(path.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR
    )
    if image is None:
        raise ValueError(f"OpenCV could not decode {path}")
    return image


def _read_v1_boxes(source: Path, image_path: str) -> list[LabelBox]:
    label_path = source / Path(image_path).with_suffix(".txt")
    if not label_path.is_file():
        raise FileNotFoundError(f"label not found: {label_path}")
    result: list[LabelBox] = []
    for source_line, raw_line in enumerate(
        label_path.read_text(encoding="utf-8-sig").splitlines(), start=1
    ):
        fields = raw_line.strip().split()
        if not fields:
            continue
        if len(fields) != 5:
            raise ValueError(f"{label_path}:{source_line}: expected five fields")
        try:
            values = [float(field) for field in fields]
        except ValueError as error:
            raise ValueError(
                f"{label_path}:{source_line}: non-numeric YOLO field"
            ) from error
        class_value, x, y, width, height = values
        if not class_value.is_integer() or int(class_value) not in (0, 1, 2):
            raise ValueError(f"{label_path}:{source_line}: invalid class id")
        class_id = int(class_value)
        if class_id not in CLASS_NAMES:
            continue
        if not all(math.isfinite(value) for value in (x, y, width, height)):
            raise ValueError(f"{label_path}:{source_line}: non-finite box")
        if width <= 0.0 or height <= 0.0:
            raise ValueError(f"{label_path}:{source_line}: non-positive box extent")
        if not all(0.0 <= value <= 1.0 for value in (x, y, width, height)):
            raise ValueError(f"{label_path}:{source_line}: non-normalised box")
        result.append(
            LabelBox(
                class_id=class_id,
                x=x,
                y=y,
                width=width,
                height=height,
                sequence=len(result) + 1,
                source_line=source_line,
            )
        )
    return result


def _iou(left: LabelBox, right: LabelBox) -> float:
    left_x1 = left.x - left.width / 2.0
    left_y1 = left.y - left.height / 2.0
    left_x2 = left.x + left.width / 2.0
    left_y2 = left.y + left.height / 2.0
    right_x1 = right.x - right.width / 2.0
    right_y1 = right.y - right.height / 2.0
    right_x2 = right.x + right.width / 2.0
    right_y2 = right.y + right.height / 2.0
    intersection = max(0.0, min(left_x2, right_x2) - max(left_x1, right_x1)) * max(
        0.0, min(left_y2, right_y2) - max(left_y1, right_y1)
    )
    union = left.width * left.height + right.width * right.height - intersection
    return intersection / union if union > 0.0 else 0.0


def _match_boxes(
    training: Sequence[LabelBox],
    validation: Sequence[LabelBox],
    threshold: float = 0.5,
) -> tuple[list[tuple[float, LabelBox, LabelBox]], list[LabelBox], list[LabelBox]]:
    candidates = sorted(
        (
            (_iou(training_box, validation_box), training_index, validation_index)
            for training_index, training_box in enumerate(training)
            for validation_index, validation_box in enumerate(validation)
            if _iou(training_box, validation_box) >= threshold
        ),
        reverse=True,
    )
    used_training: set[int] = set()
    used_validation: set[int] = set()
    matches: list[tuple[float, LabelBox, LabelBox]] = []
    for overlap, training_index, validation_index in candidates:
        if training_index in used_training or validation_index in used_validation:
            continue
        used_training.add(training_index)
        used_validation.add(validation_index)
        matches.append(
            (overlap, training[training_index], validation[validation_index])
        )
    unmatched_training = [
        box for index, box in enumerate(training) if index not in used_training
    ]
    unmatched_validation = [
        box for index, box in enumerate(validation) if index not in used_validation
    ]
    return matches, unmatched_training, unmatched_validation


def _normalise_pair(pair: Mapping[str, object]) -> tuple[str, str]:
    paths = (str(pair["image_a"]), str(pair["image_b"]))
    training = [path for path in paths if Path(path).parts[0] == "training"]
    validation = [path for path in paths if Path(path).parts[0] == "validation"]
    if len(training) != 1 or len(validation) != 1:
        raise ValueError(f"pair is not training/validation: {paths}")
    return training[0], validation[0]


def _select_review_pairs(
    source: Path, audit: Mapping[str, object]
) -> list[Mapping[str, object]]:
    image_metadata = {
        str(item["path"]): item for item in audit["input"]["images"]
    }
    duplicate_pairs: list[Mapping[str, object]] = []
    for pair in audit["pairs"]:
        try:
            training_path, validation_path = _normalise_pair(pair)
        except ValueError:
            continue
        training_metadata = image_metadata[training_path]
        validation_metadata = image_metadata[validation_path]
        dimensions = (
            int(training_metadata["width"]),
            int(training_metadata["height"]),
        )
        if (
            dimensions == (4000, 3000)
            and dimensions
            == (
                int(validation_metadata["width"]),
                int(validation_metadata["height"]),
            )
            and int(pair["good_matches"]) > 1000
            and float(pair["inlier_ratio"]) > 0.99
        ):
            duplicate_pairs.append(pair)
    if len(duplicate_pairs) != 65:
        raise ValueError(
            f"duplicate-pair contract changed: expected 65, got {len(duplicate_pairs)}"
        )

    review_pairs: list[Mapping[str, object]] = []
    for pair in duplicate_pairs:
        training_path, validation_path = _normalise_pair(pair)
        training_boxes = _read_v1_boxes(source, training_path)
        validation_boxes = _read_v1_boxes(source, validation_path)
        matches, unmatched_training, unmatched_validation = _match_boxes(
            training_boxes, validation_boxes
        )
        if (
            unmatched_training
            or unmatched_validation
            or any(left.class_id != right.class_id for _, left, right in matches)
        ):
            review_pairs.append(pair)
    if len(review_pairs) != 16:
        raise ValueError(
            f"label-review contract changed: expected 16, got {len(review_pairs)}"
        )
    return sorted(review_pairs, key=lambda pair: _natural_path(_normalise_pair(pair)[0]))


def _natural_path(value: str) -> tuple[object, ...]:
    path = Path(value)
    try:
        stem: object = int(path.stem)
    except ValueError:
        stem = path.stem.casefold()
    return (*[part.casefold() for part in path.parts[:-1]], stem, path.suffix.casefold())


def _crop_bounds(
    boxes: Sequence[LabelBox], image_width: int, image_height: int
) -> tuple[int, int, int, int]:
    if not boxes:
        return 0, 0, image_width, image_height
    coordinates = [box.pixel_xyxy(image_width, image_height) for box in boxes]
    x1 = min(item[0] for item in coordinates)
    y1 = min(item[1] for item in coordinates)
    x2 = max(item[2] for item in coordinates)
    y2 = max(item[3] for item in coordinates)
    centre_x = (x1 + x2) / 2.0
    centre_y = (y1 + y2) / 2.0
    # Keep substantial context: at least 45% of each source dimension and a
    # 30% margin around the complete union of both annotation sets.
    crop_width = min(
        float(image_width), max(image_width * 0.45, (x2 - x1) * 1.60)
    )
    crop_height = min(
        float(image_height), max(image_height * 0.45, (y2 - y1) * 1.60)
    )
    crop_x1 = max(0.0, min(image_width - crop_width, centre_x - crop_width / 2.0))
    crop_y1 = max(0.0, min(image_height - crop_height, centre_y - crop_height / 2.0))
    return (
        int(math.floor(crop_x1)),
        int(math.floor(crop_y1)),
        int(math.ceil(crop_x1 + crop_width)),
        int(math.ceil(crop_y1 + crop_height)),
    )


def _put_text(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    *,
    scale: float = 0.5,
    color: tuple[int, int, int] = (25, 25, 25),
    thickness: int = 1,
) -> None:
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _render_panel(
    source_image: np.ndarray,
    boxes: Sequence[LabelBox],
    crop: tuple[int, int, int, int],
    width: int,
    height: int,
    heading: str,
) -> np.ndarray:
    panel = np.full((height, width, 3), 248, dtype=np.uint8)
    heading_height = 38
    _put_text(panel, heading, (9, 25), scale=0.54, thickness=1)
    x1, y1, x2, y2 = crop
    cropped = source_image[y1:y2, x1:x2]
    image_height = height - heading_height
    scale = min(width / cropped.shape[1], image_height / cropped.shape[0])
    resized_width = max(1, round(cropped.shape[1] * scale))
    resized_height = max(1, round(cropped.shape[0] * scale))
    resized = cv2.resize(
        cropped,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA,
    )
    offset_x = (width - resized_width) // 2
    offset_y = heading_height + (image_height - resized_height) // 2
    panel[offset_y : offset_y + resized_height, offset_x : offset_x + resized_width] = resized

    source_height, source_width = source_image.shape[:2]
    for box in boxes:
        box_x1, box_y1, box_x2, box_y2 = box.pixel_xyxy(
            source_width, source_height
        )
        left = round(offset_x + (box_x1 - x1) * scale)
        top = round(offset_y + (box_y1 - y1) * scale)
        right = round(offset_x + (box_x2 - x1) * scale)
        bottom = round(offset_y + (box_y2 - y1) * scale)
        left = max(offset_x, min(offset_x + resized_width - 1, left))
        right = max(offset_x, min(offset_x + resized_width - 1, right))
        top = max(offset_y, min(offset_y + resized_height - 1, top))
        bottom = max(offset_y, min(offset_y + resized_height - 1, bottom))
        color = CLASS_COLORS[box.class_id]
        cv2.rectangle(panel, (left, top), (right, bottom), (10, 10, 10), 5)
        cv2.rectangle(panel, (left, top), (right, bottom), color, 3)
        label = f"#{box.sequence} {box.name} L{box.source_line}"
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
        )
        label_top = max(offset_y, top - text_height - baseline - 6)
        label_right = min(width - 1, left + text_width + 8)
        cv2.rectangle(
            panel,
            (left, label_top),
            (label_right, label_top + text_height + baseline + 6),
            color,
            -1,
        )
        _put_text(
            panel,
            label,
            (left + 4, label_top + text_height + 2),
            scale=0.48,
            color=(255, 255, 255),
            thickness=1,
        )
    return panel


def _box_text(boxes: Sequence[LabelBox]) -> str:
    return ";".join(
        f"#{box.sequence}:L{box.source_line}:{box.name}:"
        f"{box.x:.6f},{box.y:.6f},{box.width:.6f},{box.height:.6f}"
        for box in boxes
    )


def _match_text(matches: Sequence[tuple[float, LabelBox, LabelBox]]) -> str:
    return ";".join(
        f"T#{training.sequence}-V#{validation.sequence}:iou={overlap:.6f}:"
        f"{'SAME' if training.class_id == validation.class_id else 'CLASS_CONFLICT'}"
        for overlap, training, validation in matches
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--audit-json", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--index-csv", type=Path, default=DEFAULT_INDEX)
    parser.add_argument("--columns", type=int, default=2)
    parser.add_argument("--card-width", type=int, default=1500)
    parser.add_argument("--card-height", type=int, default=720)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.columns < 1 or args.card_width < 900 or args.card_height < 500:
        raise ValueError("columns/card dimensions are too small for a readable review")
    source = args.source.resolve()
    if not source.is_dir():
        raise FileNotFoundError(f"source directory not found: {source}")
    if not args.audit_json.is_file():
        raise FileNotFoundError(f"group audit not found: {args.audit_json}")
    for output in (args.output, args.index_csv):
        if output.exists() and not args.force:
            raise FileExistsError(f"output already exists: {output}")

    audit = json.loads(args.audit_json.read_text(encoding="utf-8"))
    review_pairs = _select_review_pairs(source, audit)
    rows = (len(review_pairs) + args.columns - 1) // args.columns
    canvas = np.full(
        (rows * args.card_height, args.columns * args.card_width, 3),
        225,
        dtype=np.uint8,
    )
    index_rows: list[Mapping[str, object]] = []

    for review_index, pair in enumerate(review_pairs, start=1):
        training_path, validation_path = _normalise_pair(pair)
        training_boxes = _read_v1_boxes(source, training_path)
        validation_boxes = _read_v1_boxes(source, validation_path)
        matches, unmatched_training, unmatched_validation = _match_boxes(
            training_boxes, validation_boxes
        )
        conflicts = [
            item for item in matches if item[1].class_id != item[2].class_id
        ]
        candidates = (training_path, validation_path)
        representative_path = max(
            candidates,
            key=lambda value: (
                (source / value).stat().st_size,
                value.casefold(),
            ),
        )
        representative = _read_image(source / representative_path)
        image_height, image_width = representative.shape[:2]
        crop = _crop_bounds(
            [*training_boxes, *validation_boxes], image_width, image_height
        )

        card = np.full(
            (args.card_height - 8, args.card_width - 8, 3),
            250,
            dtype=np.uint8,
        )
        _put_text(
            card,
            f"REVIEW {review_index:02d}/16 | representative={representative_path} "
            "(larger JPEG)",
            (12, 25),
            scale=0.58,
            thickness=1,
        )
        _put_text(
            card,
            f"good={pair['good_matches']} inliers={pair['inliers']} "
            f"ratio={float(pair['inlier_ratio']):.5f} | matches={len(matches)} "
            f"class_conflicts={len(conflicts)} unmatched T/V="
            f"{len(unmatched_training)}/{len(unmatched_validation)}",
            (12, 50),
            scale=0.51,
            thickness=1,
        )
        panel_y = 62
        footer_height = 34
        panel_height = card.shape[0] - panel_y - footer_height
        half_width = card.shape[1] // 2
        training_panel = _render_panel(
            representative,
            training_boxes,
            crop,
            half_width,
            panel_height,
            f"TRAINING LABELS | {training_path}",
        )
        validation_panel = _render_panel(
            representative,
            validation_boxes,
            crop,
            card.shape[1] - half_width,
            panel_height,
            f"VALIDATION LABELS | {validation_path}",
        )
        card[panel_y : panel_y + panel_height, :half_width] = training_panel
        card[panel_y : panel_y + panel_height, half_width:] = validation_panel
        cv2.line(
            card,
            (half_width, panel_y),
            (half_width, panel_y + panel_height),
            (40, 40, 40),
            2,
        )
        _put_text(
            card,
            "RED=RIPE  GREEN=UNRIPE  |  #=v1 box sequence  L=source label line",
            (12, card.shape[0] - 10),
            scale=0.52,
            thickness=1,
        )

        row, column = divmod(review_index - 1, args.columns)
        canvas_y = row * args.card_height + 4
        canvas_x = column * args.card_width + 4
        canvas[
            canvas_y : canvas_y + card.shape[0],
            canvas_x : canvas_x + card.shape[1],
        ] = card

        index_rows.append(
            {
                "review_id": review_index,
                "training_image": training_path,
                "validation_image": validation_path,
                "representative_image": representative_path,
                "representative_rule": "larger_encoded_byte_size_then_path",
                "training_jpeg_bytes": (source / training_path).stat().st_size,
                "validation_jpeg_bytes": (source / validation_path).stat().st_size,
                "good_matches": int(pair["good_matches"]),
                "inliers": int(pair["inliers"]),
                "inlier_ratio": f"{float(pair['inlier_ratio']):.9f}",
                "training_box_count": len(training_boxes),
                "validation_box_count": len(validation_boxes),
                "matched_iou50_count": len(matches),
                "same_class_match_count": len(matches) - len(conflicts),
                "class_conflict_count": len(conflicts),
                "training_unmatched_count": len(unmatched_training),
                "validation_unmatched_count": len(unmatched_validation),
                "training_boxes": _box_text(training_boxes),
                "validation_boxes": _box_text(validation_boxes),
                "matches": _match_text(matches),
                "training_unmatched": _box_text(unmatched_training),
                "validation_unmatched": _box_text(unmatched_validation),
                "crop_xyxy": ",".join(str(value) for value in crop),
            }
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.index_csv.parent.mkdir(parents=True, exist_ok=True)
    encoded, data = cv2.imencode(
        ".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 92]
    )
    if not encoded:
        raise OSError(f"failed to encode review image: {args.output}")
    args.output.write_bytes(data.tobytes())
    fieldnames = list(index_rows[0])
    with args.index_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(index_rows)
    print(
        json.dumps(
            {
                "duplicate_pairs": 65,
                "review_pairs": len(review_pairs),
                "output": str(args.output.resolve()),
                "index_csv": str(args.index_csv.resolve()),
                "canvas_width": int(canvas.shape[1]),
                "canvas_height": int(canvas.shape[0]),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, OSError) as error:
        print(f"duplicate label review failed: {error}")
        raise SystemExit(2) from error
