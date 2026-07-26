#!/usr/bin/env python3
"""Summarize paired A/B/C strawberry visual-asset Shadow evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics

from validate_t70_strawberry_asset_ablation import load_manifest


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def summarize(output_dir: Path, manifest_path: Path, model_path: Path) -> dict:
    manifest, rows = load_manifest(manifest_path, model_path)
    grouped: dict[tuple[str, str], dict] = {}
    scenario_records = []
    for scenario_id, variant, maturity, position_label, *_ in rows:
        scenario_dir = output_dir / scenario_id
        receipt_path = scenario_dir / "receipt.json"
        window_path = scenario_dir / "shadow_window.json"
        if not receipt_path.is_file() or not window_path.is_file():
            raise ValueError(f"scenario is incomplete: {scenario_id}")
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        window = json.loads(window_path.read_text(encoding="utf-8"))
        if receipt.get("variant") != variant or receipt.get("target_maturity") != maturity:
            raise ValueError(f"receipt identity mismatch: {scenario_id}")
        if receipt.get("robot_motion_started") is not False:
            raise ValueError(f"motion boundary violated: {scenario_id}")
        if window.get("formal_acceptance") is not False:
            raise ValueError(f"window incorrectly claims acceptance: {scenario_id}")
        expected_frames = int(manifest["measurement"]["fixed_measurement_frames"])
        if int(window.get("required_frames", -1)) != expected_frames:
            raise ValueError(f"wrong fixed window size: {scenario_id}")
        if int(window.get("warmup_frames_discarded", -1)) != int(
            manifest["measurement"]["warmup_detection_frames"]
        ):
            raise ValueError(f"wrong warmup size: {scenario_id}")

        group = grouped.setdefault(
            (variant, maturity),
            {
                "frames": 0,
                "correct_frames": 0,
                "wrong_frames": 0,
                "target_pose_frames": 0,
                "correct_confidences": [],
                "all_confidences": [],
            },
        )
        for frame in window["frames"]:
            ripe_count = int(frame["ripe_detection_count"])
            unripe_count = int(frame["unripe_detection_count"])
            if maturity == "RIPE":
                correct_count, wrong_count = ripe_count, unripe_count
                correct_confidences = frame.get("ripe_confidences", [])
            else:
                correct_count, wrong_count = unripe_count, ripe_count
                correct_confidences = frame.get("unripe_confidences", [])
            group["frames"] += 1
            group["correct_frames"] += int(correct_count > 0)
            group["wrong_frames"] += int(wrong_count > 0)
            group["target_pose_frames"] += int(bool(frame["target_pose_received"]))
            group["correct_confidences"].extend(float(v) for v in correct_confidences)
            group["all_confidences"].extend(
                float(v) for v in frame.get("detection_confidences", [])
            )

        screenshot = scenario_dir / "rgb.png"
        scenario_records.append(
            {
                "scenario_id": scenario_id,
                "variant": variant,
                "target_maturity": maturity,
                "position_label": position_label,
                "window_sha256": _sha256(window_path),
                "world_sha256": receipt["materialized_world"]["sha256"],
                "screenshot": (
                    {
                        "path": str(screenshot.resolve()),
                        "size_bytes": screenshot.stat().st_size,
                        "sha256": _sha256(screenshot),
                    }
                    if screenshot.is_file()
                    else None
                ),
            }
        )

    group_records = []
    by_identity = {}
    for (variant, maturity), raw in sorted(grouped.items()):
        frames = int(raw["frames"])
        record = {
            "variant": variant,
            "target_maturity": maturity,
            "frames": frames,
            "correct_class_frame_rate": raw["correct_frames"] / frames,
            "wrong_class_frame_rate": raw["wrong_frames"] / frames,
            "target_pose_frame_rate": raw["target_pose_frames"] / frames,
            "correct_detection_count": len(raw["correct_confidences"]),
            "mean_correct_confidence": _mean(raw["correct_confidences"]),
            "mean_all_detection_confidence": _mean(raw["all_confidences"]),
        }
        group_records.append(record)
        by_identity[(variant, maturity)] = record

    thresholds = manifest["diagnostic_interpretation"]
    frame_threshold = float(thresholds["material_effect_min_absolute_frame_rate_delta"])
    confidence_threshold = float(
        thresholds["material_effect_min_absolute_mean_confidence_delta"]
    )
    deltas = []
    material_effect_supported = False
    for variant in ("B", "C"):
        for maturity in ("RIPE", "UNRIPE"):
            baseline = by_identity[("A", maturity)]
            candidate = by_identity[(variant, maturity)]
            frame_delta = (
                candidate["correct_class_frame_rate"]
                - baseline["correct_class_frame_rate"]
            )
            wrong_delta = (
                candidate["wrong_class_frame_rate"]
                - baseline["wrong_class_frame_rate"]
            )
            base_confidence = baseline["mean_correct_confidence"]
            candidate_confidence = candidate["mean_correct_confidence"]
            confidence_delta = (
                candidate_confidence - base_confidence
                if base_confidence is not None and candidate_confidence is not None
                else None
            )
            effect = abs(frame_delta) >= frame_threshold or (
                confidence_delta is not None
                and abs(confidence_delta) >= confidence_threshold
            )
            material_effect_supported = material_effect_supported or effect
            deltas.append(
                {
                    "comparison": f"{variant}-A",
                    "target_maturity": maturity,
                    "correct_class_frame_rate_delta": frame_delta,
                    "wrong_class_frame_rate_delta": wrong_delta,
                    "mean_correct_confidence_delta": confidence_delta,
                    "meets_frozen_material_effect_threshold": effect,
                }
            )

    return {
        "schema_version": 1,
        "diagnostic_id": manifest["diagnostic_id"],
        "scope": manifest["scope"],
        "formal_acceptance": False,
        "held_out_real_test_consumed": False,
        "formal_simulator_matrix_consumed": False,
        "robot_motion_started": False,
        "training_started": False,
        "candidate_promotion_authorized": False,
        "perception_control_authorized": False,
        "scenario_count": len(scenario_records),
        "group_metrics": group_records,
        "paired_deltas": deltas,
        "material_effect_supported": material_effect_supported,
        "interpretation_boundary": (
            "A material effect does not prove model acceptance, real-world fidelity, "
            "or readiness for perception control."
        ),
        "scenarios": scenario_records,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    options = parser.parse_args()
    summary = summarize(options.output_dir, options.manifest, options.model)
    summary_path = options.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
