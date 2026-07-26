#!/usr/bin/env python3
"""Build ADR-0024's bounded human-review packet from train predictions."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from build_label_audit_packet import _context_box, _draw_box  # noqa: E402
from workflow import read_json, sha256_file  # noqa: E402


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUNDLE = Path(
    "artifacts/perception/label_audit/t30_train_label_screen_v1/predictions.json"
)
DEFAULT_OUTPUT = Path(
    "artifacts/perception/label_audit/t30_train_label_screen_v1/review_packet"
)
MISSING_CONFIDENCE = 0.75
CLASS_DISAGREEMENT_CONFIDENCE = 0.50
MISSING_MAX_IOU = 0.10
CLASS_MATCH_IOU = 0.50
PACKET_LIMIT = 60
CLASS_NAMES = {0: "ripe", 1: "unripe"}
ALLOWED_DECISIONS = (
    "CONFIRMED_ADD_RIPE",
    "CONFIRMED_ADD_UNRIPE",
    "CONFIRMED_RELABEL_RIPE",
    "CONFIRMED_RELABEL_UNRIPE",
    "MODEL_ERROR_KEEP_LABELS",
    "AMBIGUOUS_EXCLUDE",
)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def _bbox(item: Mapping[str, Any]) -> tuple[float, float, float, float]:
    values = item.get("bbox_xyxy_normalized")
    if not isinstance(values, list) or len(values) != 4:
        raise ValueError("candidate box must contain four coordinates")
    box = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in box):
        raise ValueError("candidate box must be finite")
    if not (0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1):
        raise ValueError("candidate box must have normalized positive area")
    return box


def _iou(left: Sequence[float], right: Sequence[float]) -> float:
    x1, y1 = max(left[0], right[0]), max(left[1], right[1])
    x2, y2 = min(left[2], right[2]), min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = (left[2] - left[0]) * (left[3] - left[1])
    right_area = (right[2] - right[0]) * (right[3] - right[1])
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0


def screen_candidates(records: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    candidates = []
    for record in records:
        image = str(record.get("image", "")).replace("\\", "/")
        if not image.startswith("images/train/"):
            raise ValueError(f"audit record is outside the training split: {image}")
        truths = record.get("ground_truth")
        predictions = record.get("predictions")
        if not isinstance(truths, list) or not isinstance(predictions, list):
            raise ValueError("training prediction record is malformed")
        for prediction in predictions:
            class_id = int(prediction["class_id"])
            if class_id not in CLASS_NAMES:
                raise ValueError("prediction class is outside the maturity contract")
            confidence = float(prediction["confidence"])
            box = _bbox(prediction)
            same = [
                _iou(box, _bbox(truth))
                for truth in truths
                if int(truth["class_id"]) == class_id
            ]
            opposite = [
                _iou(box, _bbox(truth))
                for truth in truths
                if int(truth["class_id"]) != class_id
            ]
            max_same = max(same, default=0.0)
            max_opposite = max(opposite, default=0.0)
            max_any = max(max_same, max_opposite)
            if (
                confidence >= CLASS_DISAGREEMENT_CONFIDENCE
                and max_opposite >= CLASS_MATCH_IOU
                and max_same < CLASS_MATCH_IOU
            ):
                reason = "HIGH_CONFIDENCE_CROSS_CLASS"
                priority = 0
            elif confidence >= MISSING_CONFIDENCE and max_any < MISSING_MAX_IOU:
                reason = "HIGH_CONFIDENCE_UNMATCHED_PREDICTION"
                priority = 1
            else:
                continue
            candidates.append(
                {
                    "image": image,
                    "reason": reason,
                    "priority": priority,
                    "predicted_class_id": class_id,
                    "predicted_class_name": CLASS_NAMES[class_id],
                    "confidence": confidence,
                    "bbox_xyxy_normalized": list(box),
                    "max_same_class_iou": max_same,
                    "max_opposite_class_iou": max_opposite,
                    "max_any_annotation_iou": max_any,
                    "existing_labels": [
                        {
                            "class_id": int(truth["class_id"]),
                            "class_name": CLASS_NAMES[int(truth["class_id"])],
                            "bbox_xyxy_normalized": list(_bbox(truth)),
                        }
                        for truth in truths
                    ],
                }
            )
    candidates.sort(
        key=lambda item: (
            int(item["priority"]),
            -float(item["confidence"]),
            str(item["image"]),
            int(item["predicted_class_id"]),
            tuple(float(value) for value in item["bbox_xyxy_normalized"]),
        )
    )
    return candidates


def _render(candidates: list[dict[str, Any]], output_dir: Path) -> list[dict[str, Any]]:
    try:
        from PIL import Image, ImageDraw
    except ImportError as error:  # pragma: no cover - environment dependency
        raise RuntimeError("Pillow is required to render the audit packet") from error

    overlay_dir = output_dir / "overlays"
    crop_dir = output_dir / "crops"
    page_dir = output_dir / "contact_sheets"
    for directory in (overlay_dir, crop_dir, page_dir):
        directory.mkdir(parents=True)
    tile_size = (560, 420)
    rendered = []
    tiles = []
    for candidate in candidates:
        image_path = Path(candidate["absolute_image_path"])
        with Image.open(image_path) as opened:
            image = opened.convert("RGB")
        width, height = image.size
        line_width = max(3, round(max(width, height) / 450))
        overlay = image.copy()
        draw = ImageDraw.Draw(overlay)
        for label in candidate["existing_labels"]:
            _draw_box(
                draw,
                label["bbox_xyxy_normalized"],
                width,
                height,
                "lime",
                f"GT {label['class_name']}",
                line_width,
            )
        _draw_box(
            draw,
            candidate["bbox_xyxy_normalized"],
            width,
            height,
            "red",
            f"{candidate['candidate_id']} P {candidate['predicted_class_name']} "
            f"{candidate['confidence']:.3f}",
            line_width + 1,
        )
        overlay_path = overlay_dir / f"{candidate['candidate_id']}_{image_path.stem}.jpg"
        overlay.save(overlay_path, "JPEG", quality=92, optimize=True)

        bounds = _context_box(
            candidate["bbox_xyxy_normalized"],
            image_width=width,
            image_height=height,
        )
        crop = overlay.crop(bounds)
        crop.thumbnail((tile_size[0] - 20, tile_size[1] - 80), Image.Resampling.LANCZOS)
        tile = Image.new("RGB", tile_size, "#202020")
        tile.paste(crop, ((tile_size[0] - crop.width) // 2, 60))
        tile_draw = ImageDraw.Draw(tile)
        tile_draw.text(
            (10, 8),
            f"{candidate['candidate_id']} | {candidate['reason']} | "
            f"{candidate['predicted_class_name']} {candidate['confidence']:.3f}",
            fill="white",
        )
        tile_draw.text((10, 28), candidate["image"], fill="#dddddd")
        crop_path = crop_dir / f"{candidate['candidate_id']}.jpg"
        tile.save(crop_path, "JPEG", quality=92, optimize=True)
        tiles.append(tile)
        rendered.append(
            {
                "candidate_id": candidate["candidate_id"],
                "overlay": overlay_path.relative_to(output_dir).as_posix(),
                "overlay_sha256": sha256_file(overlay_path),
                "crop": crop_path.relative_to(output_dir).as_posix(),
                "crop_sha256": sha256_file(crop_path),
                "crop_bounds_pixels": list(bounds),
            }
        )

    for page_number, offset in enumerate(range(0, len(tiles), 6), start=1):
        page = Image.new("RGB", (1120, 1310), "#101010")
        page_draw = ImageDraw.Draw(page)
        page_draw.text(
            (12, 12),
            f"Training-label audit | page {page_number} | red=candidate green=current GT",
            fill="white",
        )
        for index, tile in enumerate(tiles[offset : offset + 6]):
            page.paste(tile, ((index % 2) * 560, 50 + (index // 2) * 420))
        page_path = page_dir / f"page_{page_number:02d}.png"
        page.save(page_path, "PNG", optimize=True)
    return rendered


def build_packet(bundle_path: Path, output_dir: Path) -> dict[str, Any]:
    if not bundle_path.is_file():
        raise FileNotFoundError(bundle_path)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError("output packet directory must be absent or empty")
    bundle = read_json(bundle_path)
    if bundle.get("kind") != "training_label_audit_prediction_bundle":
        raise ValueError("unexpected training-label prediction bundle")
    scope = bundle.get("scope", {})
    if (
        scope.get("split") != "train"
        or int(scope.get("image_count", -1)) != 501
        or scope.get("held_out_test_accessed") is not False
        or scope.get("training_started") is not False
    ):
        raise ValueError("prediction bundle violates the training-label audit scope")
    bindings = bundle.get("bindings", {})
    rebaseline_binding = bindings.get("rebaseline", {})
    rebaseline_path = _resolve(Path(str(rebaseline_binding.get("path", "")))).resolve()
    if (
        not rebaseline_path.is_file()
        or rebaseline_path.stat().st_size != int(rebaseline_binding.get("size_bytes", -1))
        or sha256_file(rebaseline_path) != rebaseline_binding.get("sha256")
    ):
        raise ValueError("rebaseline binding mismatch")
    rebaseline = read_json(rebaseline_path)
    if (
        rebaseline.get("decision", {})
        .get("authorization", {})
        .get("training_label_audit")
        is not True
    ):
        raise ValueError("rebaseline no longer authorizes this audit")

    records = bundle.get("records")
    if not isinstance(records, list) or len(records) != 501:
        raise ValueError("training-label audit requires all 501 training records")
    all_candidates = screen_candidates(records)
    selected = all_candidates[:PACKET_LIMIT]
    split_binding = bindings.get("split_manifest", {})
    split_path = _resolve(Path(str(split_binding.get("path", "")))).resolve()
    if (
        not split_path.is_file()
        or split_path.stat().st_size != int(split_binding.get("size_bytes", -1))
        or sha256_file(split_path) != split_binding.get("sha256")
    ):
        raise ValueError("split manifest binding mismatch")
    split_manifest = read_json(split_path)
    train_entries = {
        str(item["output_image"]).replace("\\", "/"): item
        for item in split_manifest["splits"]["train"]
    }
    dataset_root = split_path.parent.resolve()
    public = []
    for index, candidate in enumerate(selected, start=1):
        image_key = candidate["image"]
        entry = train_entries.get(image_key)
        if entry is None:
            raise ValueError(f"candidate is not in registered training split: {image_key}")
        image_path = (dataset_root / image_key).resolve()
        try:
            image_path.relative_to(dataset_root)
        except ValueError as error:
            raise ValueError("candidate image escapes dataset root") from error
        if (
            not image_path.is_file()
            or image_path.stat().st_size != int(entry["output_image_size_bytes"])
            or sha256_file(image_path) != entry["output_image_sha256"]
        ):
            raise ValueError(f"candidate image binding mismatch: {image_key}")
        candidate = {
            **candidate,
            "candidate_id": f"T{index:03d}",
            "group_id": str(entry["group_id"]),
            "image_sha256": entry["output_image_sha256"],
            "label": str(entry["output_label"]),
            "label_sha256": entry["output_label_sha256"],
            "review_status": "PENDING_HUMAN_REVIEW",
            "absolute_image_path": str(image_path),
        }
        public.append(candidate)

    output_dir.mkdir(parents=True, exist_ok=True)
    rendered = _render(public, output_dir)
    rendered_by_id = {item["candidate_id"]: item for item in rendered}
    manifest_candidates = []
    for candidate in public:
        item = {key: value for key, value in candidate.items() if key not in ("absolute_image_path", "priority")}
        item.update(rendered_by_id[candidate["candidate_id"]])
        manifest_candidates.append(item)

    reason_counts = {
        reason: sum(item["reason"] == reason for item in all_candidates)
        for reason in (
            "HIGH_CONFIDENCE_CROSS_CLASS",
            "HIGH_CONFIDENCE_UNMATCHED_PREDICTION",
        )
    }
    manifest = {
        "schema_version": 1,
        "kind": "training_label_audit_packet",
        "scope": {
            "split": "train",
            "source_image_count": 501,
            "candidate_count_before_limit": len(all_candidates),
            "packet_limit": PACKET_LIMIT,
            "candidate_count": len(manifest_candidates),
            "excluded_by_limit_count": max(0, len(all_candidates) - PACKET_LIMIT),
            "candidate_image_count": len({item["image"] for item in manifest_candidates}),
            "reason_counts_before_limit": reason_counts,
            "validation_split_accessed": False,
            "held_out_test_accessed": False,
            "training_started": False,
            "labels_modified": False,
        },
        "bindings": {
            "prediction_bundle": {
                "path": bundle_path.relative_to(REPOSITORY_ROOT).as_posix(),
                "size_bytes": bundle_path.stat().st_size,
                "sha256": sha256_file(bundle_path),
            },
            "rebaseline": rebaseline_binding,
            "split_manifest": split_binding,
            "weights": bindings.get("weights"),
        },
        "screen": {
            "missing_annotation": {
                "confidence_min": MISSING_CONFIDENCE,
                "max_any_annotation_iou_exclusive": MISSING_MAX_IOU,
            },
            "class_disagreement": {
                "confidence_min": CLASS_DISAGREEMENT_CONFIDENCE,
                "opposite_class_iou_min": CLASS_MATCH_IOU,
                "same_class_iou_exclusive": CLASS_MATCH_IOU,
            },
            "selection_order": "cross_class_then_unmatched_then_confidence_desc_then_stable_image_box",
        },
        "review_protocol": {
            "allowed_decisions": list(ALLOWED_DECISIONS),
            "model_prediction_is_not_ground_truth": True,
            "ambiguous_or_disagreed_items_are_excluded": True,
            "no_label_change_authorized_by_this_packet": True,
        },
        "candidates": manifest_candidates,
    }
    manifest_path = output_dir / "candidate_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    review_path = output_dir / "review_template.csv"
    fields = (
        "candidate_id",
        "reason",
        "image",
        "predicted_class_name",
        "confidence",
        "bbox_xyxy_normalized",
        "decision",
        "notes",
    )
    with review_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for item in manifest_candidates:
            writer.writerow(
                {
                    "candidate_id": item["candidate_id"],
                    "reason": item["reason"],
                    "image": item["image"],
                    "predicted_class_name": item["predicted_class_name"],
                    "confidence": item["confidence"],
                    "bbox_xyxy_normalized": json.dumps(item["bbox_xyxy_normalized"], separators=(",", ":")),
                }
            )
    summary = {
        "schema_version": 1,
        "kind": "training_label_audit_packet_summary",
        "candidate_count_before_limit": len(all_candidates),
        "candidate_count": len(manifest_candidates),
        "reason_counts_before_limit": reason_counts,
        "labels_modified": False,
        "training_started": False,
        "held_out_test_accessed": False,
        "candidate_manifest": {
            "path": manifest_path.relative_to(output_dir).as_posix(),
            "sha256": sha256_file(manifest_path),
        },
        "review_template": {
            "path": review_path.relative_to(output_dir).as_posix(),
            "sha256": sha256_file(review_path),
        },
        "contact_sheets": [
            {
                "path": path.relative_to(output_dir).as_posix(),
                "sha256": sha256_file(path),
            }
            for path in sorted((output_dir / "contact_sheets").glob("*.png"))
        ],
        "status": "PENDING_HUMAN_REVIEW",
    }
    summary_path = output_dir / "packet_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    result = build_packet(_resolve(args.bundle).resolve(), _resolve(args.output_dir).resolve())
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
