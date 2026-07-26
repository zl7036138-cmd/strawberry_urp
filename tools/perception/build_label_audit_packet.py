#!/usr/bin/env python3
"""Build a read-only visual review packet for validation label candidates."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ANALYSIS = Path(
    "artifacts/perception/diagnostics/yolo11s_val_threshold_031/summary.json"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/label_audit/yolo11s_val_threshold_031_candidates_v1"
)
ALLOWED_REVIEW_DECISIONS = (
    "CONFIRMED_MISSING_RIPE",
    "CONFIRMED_MISSING_UNRIPE",
    "MODEL_FALSE_POSITIVE",
    "DUPLICATE_EXISTING_LABEL",
    "AMBIGUOUS_EXCLUDE",
)
CLASS_NAMES = {0: "ripe", 1: "unripe"}


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else (REPOSITORY_ROOT / path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _xywh_to_xyxy(values: Sequence[float]) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError("YOLO box must contain x, y, width, height")
    x, y, width, height = (float(value) for value in values)
    result = (
        max(0.0, x - width / 2.0),
        max(0.0, y - height / 2.0),
        min(1.0, x + width / 2.0),
        min(1.0, y + height / 2.0),
    )
    if not all(math.isfinite(value) for value in result):
        raise ValueError("YOLO box must be finite")
    if result[2] <= result[0] or result[3] <= result[1]:
        raise ValueError("YOLO box must have positive area")
    return result


def _load_labels(path: Path) -> list[dict[str, Any]]:
    labels = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError(f"invalid label at {path}:{line_number}")
        class_id = int(fields[0])
        if class_id not in CLASS_NAMES:
            raise ValueError(f"unexpected class at {path}:{line_number}")
        labels.append(
            {
                "class_id": class_id,
                "class_name": CLASS_NAMES[class_id],
                "bbox_xyxy_normalized": list(
                    _xywh_to_xyxy(tuple(float(value) for value in fields[1:]))
                ),
            }
        )
    return labels


def _context_box(
    bbox: Sequence[float],
    *,
    image_width: int,
    image_height: int,
    expansion: float = 3.0,
) -> tuple[int, int, int, int]:
    """Return a clipped pixel crop with context around a normalized box."""

    if len(bbox) != 4 or image_width <= 0 or image_height <= 0:
        raise ValueError("bbox and image dimensions are invalid")
    if not math.isfinite(expansion) or expansion < 1.0:
        raise ValueError("expansion must be finite and at least one")
    x1, y1, x2, y2 = (float(value) for value in bbox)
    if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
        raise ValueError("normalized bbox is invalid")
    center_x = (x1 + x2) * image_width / 2.0
    center_y = (y1 + y2) * image_height / 2.0
    width = max((x2 - x1) * image_width * expansion, image_width * 0.18)
    height = max((y2 - y1) * image_height * expansion, image_height * 0.18)
    left = max(0, int(math.floor(center_x - width / 2.0)))
    top = max(0, int(math.floor(center_y - height / 2.0)))
    right = min(image_width, int(math.ceil(center_x + width / 2.0)))
    bottom = min(image_height, int(math.ceil(center_y + height / 2.0)))
    if right <= left or bottom <= top:
        raise ValueError("context crop is empty")
    return left, top, right, bottom


def _draw_box(draw, bbox, width, height, color, label, line_width) -> None:
    x1, y1, x2, y2 = bbox
    pixels = (x1 * width, y1 * height, x2 * width, y2 * height)
    draw.rectangle(pixels, outline=color, width=line_width)
    draw.text(
        (pixels[0] + 2, max(0, pixels[1] - 14)),
        label,
        fill=color,
        stroke_width=2,
        stroke_fill="black",
    )


def _render_candidate_assets(
    candidates: list[dict[str, Any]], output_dir: Path
) -> list[dict[str, Any]]:
    try:
        from PIL import Image, ImageDraw
    except ImportError as error:  # pragma: no cover - environment dependency
        raise RuntimeError("Pillow is required to build the label audit packet") from error

    overlay_dir = output_dir / "overlays"
    crop_dir = output_dir / "crops"
    page_dir = output_dir / "contact_sheets"
    overlay_dir.mkdir(parents=True)
    crop_dir.mkdir(parents=True)
    page_dir.mkdir(parents=True)
    rendered = []
    contact_tiles = []
    tile_size = (560, 420)

    for candidate in candidates:
        image_path = Path(candidate["absolute_image_path"])
        with Image.open(image_path) as opened:
            image = opened.convert("RGB")
        width, height = image.size
        line_width = max(3, round(max(width, height) / 450))
        overlay = image.copy()
        overlay_draw = ImageDraw.Draw(overlay)
        for label in candidate["existing_labels"]:
            _draw_box(
                overlay_draw,
                label["bbox_xyxy_normalized"],
                width,
                height,
                "lime",
                f"GT {label['class_name']}",
                line_width,
            )
        _draw_box(
            overlay_draw,
            candidate["bbox_xyxy_normalized"],
            width,
            height,
            "red",
            f"{candidate['candidate_id']} P {candidate['predicted_class_name']} "
            f"{candidate['confidence']:.3f}",
            line_width + 1,
        )
        overlay_name = f"{candidate['candidate_id']}_{image_path.stem}.jpg"
        overlay_path = overlay_dir / overlay_name
        overlay.save(overlay_path, "JPEG", quality=92, optimize=True)

        crop_bounds = _context_box(
            candidate["bbox_xyxy_normalized"],
            image_width=width,
            image_height=height,
        )
        crop = overlay.crop(crop_bounds)
        crop.thumbnail((tile_size[0] - 20, tile_size[1] - 70), Image.Resampling.LANCZOS)
        tile = Image.new("RGB", tile_size, "#202020")
        tile.paste(
            crop,
            ((tile_size[0] - crop.width) // 2, 48 + (tile_size[1] - 60 - crop.height) // 2),
        )
        tile_draw = ImageDraw.Draw(tile)
        tile_draw.text(
            (10, 8),
            f"{candidate['candidate_id']} | predicted {candidate['predicted_class_name']} "
            f"{candidate['confidence']:.3f}",
            fill="white",
        )
        tile_draw.text((10, 26), candidate["image"], fill="#dddddd")
        crop_path = crop_dir / f"{candidate['candidate_id']}.jpg"
        tile.save(crop_path, "JPEG", quality=92, optimize=True)
        contact_tiles.append((candidate["candidate_id"], tile))
        rendered.append(
            {
                "candidate_id": candidate["candidate_id"],
                "overlay": overlay_path.relative_to(output_dir).as_posix(),
                "overlay_sha256": _sha256(overlay_path),
                "crop": crop_path.relative_to(output_dir).as_posix(),
                "crop_sha256": _sha256(crop_path),
                "crop_bounds_pixels": list(crop_bounds),
            }
        )

    for page_index, start in enumerate(range(0, len(contact_tiles), 6), start=1):
        page_items = contact_tiles[start : start + 6]
        page = Image.new("RGB", (tile_size[0] * 2, tile_size[1] * 3 + 50), "#101010")
        page_draw = ImageDraw.Draw(page)
        page_draw.text(
            (12, 12),
            f"Validation label audit candidates | page {page_index} | red=candidate green=existing GT",
            fill="white",
        )
        for index, (_, tile) in enumerate(page_items):
            column = index % 2
            row = index // 2
            page.paste(tile, (column * tile_size[0], 50 + row * tile_size[1]))
        page_path = page_dir / f"page_{page_index:02d}.png"
        page.save(page_path, "PNG", optimize=True)
    return rendered


def build_packet(analysis_path: Path, output_dir: Path) -> dict[str, Any]:
    analysis_path = analysis_path.resolve(strict=True)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError("output directory must be absent or empty")
    analysis = _load_object(analysis_path)
    if analysis.get("kind") != "validation_error_analysis":
        raise ValueError("input is not a validation error analysis")
    scope = analysis.get("scope", {})
    if scope.get("split") != "val" or scope.get("test_split_accessed") is not False:
        raise ValueError("audit packet accepts validation-only evidence")
    screen = analysis.get("label_incompleteness_screen", {})
    raw_candidates = screen.get("candidates")
    if not isinstance(raw_candidates, list) or not raw_candidates:
        raise ValueError("analysis does not contain label candidates")

    split_manifest_path = _resolve(Path(analysis["bindings"]["split_manifest"])).resolve(
        strict=True
    )
    if _sha256(split_manifest_path) != analysis["bindings"]["split_manifest_sha256"]:
        raise ValueError("split manifest hash differs from analysis binding")
    split_manifest = _load_object(split_manifest_path)
    dataset_root = split_manifest_path.parent
    validation_entries = {
        str(entry["output_image"]): entry for entry in split_manifest["splits"]["val"]
    }
    output_dir.mkdir(parents=True, exist_ok=True)

    candidates = []
    for index, raw in enumerate(raw_candidates, start=1):
        image_key = str(raw["image"])
        if not image_key.startswith("images/val/") or image_key not in validation_entries:
            raise ValueError(f"candidate is outside registered validation split: {image_key}")
        entry = validation_entries[image_key]
        label_key = str(entry["output_label"])
        image_path = (dataset_root / image_key).resolve(strict=True)
        label_path = (dataset_root / label_key).resolve(strict=True)
        if dataset_root.resolve() not in image_path.parents or dataset_root.resolve() not in label_path.parents:
            raise ValueError("candidate path escapes dataset root")
        if _sha256(image_path) != entry["output_image_sha256"]:
            raise ValueError(f"validation image hash mismatch: {image_key}")
        if _sha256(label_path) != entry["output_label_sha256"]:
            raise ValueError(f"validation label hash mismatch: {label_key}")
        class_id = int(raw["class_id"])
        if class_id not in CLASS_NAMES:
            raise ValueError("candidate class is outside the v1 contract")
        candidates.append(
            {
                "candidate_id": f"C{index:03d}",
                "image": image_key,
                "label": label_key,
                "source_image": str(entry["source_image"]),
                "source_label": str(entry["source_label"]),
                "group_id": str(entry["group_id"]),
                "image_sha256": entry["output_image_sha256"],
                "label_sha256": entry["output_label_sha256"],
                "predicted_class_id": class_id,
                "predicted_class_name": CLASS_NAMES[class_id],
                "confidence": float(raw["confidence"]),
                "bbox_xyxy_normalized": [
                    float(value) for value in raw["bbox_xyxy_normalized"]
                ],
                "nearest_annotation_iou": float(raw["related_iou"]),
                "existing_labels": _load_labels(label_path),
                "review_status": "PENDING_HUMAN_REVIEW",
                "absolute_image_path": str(image_path),
            }
        )

    rendered = _render_candidate_assets(candidates, output_dir)
    rendered_by_id = {item["candidate_id"]: item for item in rendered}
    public_candidates = []
    for candidate in candidates:
        public = {
            key: value
            for key, value in candidate.items()
            if key != "absolute_image_path"
        }
        public.update(rendered_by_id[candidate["candidate_id"]])
        public_candidates.append(public)

    manifest = {
        "schema_version": 1,
        "kind": "validation_label_audit_packet",
        "scope": {
            "split": "val",
            "test_split_accessed": False,
            "candidate_count": len(public_candidates),
            "candidate_image_count": len({item["image"] for item in public_candidates}),
            "labels_modified": False,
        },
        "bindings": {
            "validation_error_analysis": analysis_path.relative_to(REPOSITORY_ROOT).as_posix(),
            "validation_error_analysis_sha256": _sha256(analysis_path),
            "split_manifest": split_manifest_path.relative_to(REPOSITORY_ROOT).as_posix(),
            "split_manifest_sha256": _sha256(split_manifest_path),
            "canonical_val_sha256": analysis["bindings"]["canonical_val_sha256"],
        },
        "review_protocol": {
            "allowed_decisions": list(ALLOWED_REVIEW_DECISIONS),
            "required_reviewers": 2,
            "adjudication_required_on_disagreement": True,
            "model_prediction_is_not_ground_truth": True,
            "no_label_change_authorized_by_this_packet": True,
        },
        "candidates": public_candidates,
    }
    manifest_path = output_dir / "candidate_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    template_path = output_dir / "review_template.csv"
    fields = (
        "candidate_id",
        "image",
        "predicted_class_name",
        "confidence",
        "bbox_xyxy_normalized",
        "reviewer_1",
        "reviewer_1_decision",
        "reviewer_2",
        "reviewer_2_decision",
        "adjudicator",
        "final_decision",
        "notes",
    )
    with template_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for item in public_candidates:
            writer.writerow(
                {
                    "candidate_id": item["candidate_id"],
                    "image": item["image"],
                    "predicted_class_name": item["predicted_class_name"],
                    "confidence": item["confidence"],
                    "bbox_xyxy_normalized": json.dumps(
                        item["bbox_xyxy_normalized"], separators=(",", ":")
                    ),
                }
            )
    packet_summary = {
        "schema_version": 1,
        "kind": "validation_label_audit_packet_summary",
        "candidate_count": len(public_candidates),
        "candidate_image_count": len({item["image"] for item in public_candidates}),
        "labels_modified": False,
        "test_split_accessed": False,
        "candidate_manifest": manifest_path.relative_to(output_dir).as_posix(),
        "candidate_manifest_sha256": _sha256(manifest_path),
        "review_template": template_path.relative_to(output_dir).as_posix(),
        "review_template_sha256": _sha256(template_path),
        "contact_sheets": [
            {
                "path": path.relative_to(output_dir).as_posix(),
                "sha256": _sha256(path),
            }
            for path in sorted((output_dir / "contact_sheets").glob("*.png"))
        ],
        "status": "PENDING_TWO_REVIEWER_DECISIONS",
    }
    summary_path = output_dir / "packet_summary.json"
    summary_path.write_text(
        json.dumps(packet_summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return packet_summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis", type=Path, default=DEFAULT_ANALYSIS)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args(argv)
    summary = build_packet(_resolve(arguments.analysis), _resolve(arguments.output_dir))
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
