"""Fail-closed selection of one bounded wrist observation preset."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence


PRESETS = {
    1: {
        "name": "lower",
        "focus_roi_xyxy_px": [320, 240, 640, 480],
    },
    3: {
        "name": "center",
        "focus_roi_xyxy_px": [0, 0, 320, 480],
    },
}


def select_observation_preset(
    frames: Sequence[Mapping[str, object]],
    *,
    minimum_support_fraction: float = 0.8,
) -> dict[str, object]:
    """Select one preset from stable base-view target-pose identity support."""

    if not 0.0 < minimum_support_fraction <= 1.0:
        raise ValueError("minimum support fraction must be in (0, 1]")
    if not frames:
        raise ValueError("base overview contains no frames")
    counts: Counter[int] = Counter()
    for frame in frames:
        identities = {
            int(value)
            for value in frame.get("target_pose_ids", [])
            if int(value) > 0
        }
        counts.update(identities)
    if not counts:
        raise ValueError("base overview produced no localized ripe candidate")
    ordered = counts.most_common()
    if len(ordered) > 1 and ordered[0][1] == ordered[1][1]:
        raise ValueError("base overview candidate identity is ambiguous")
    target_id, support_frames = ordered[0]
    support_fraction = support_frames / len(frames)
    if support_fraction < minimum_support_fraction:
        raise ValueError(
            "base overview support is below the bounded observation threshold"
        )
    if target_id not in PRESETS:
        raise ValueError(f"no bounded wrist preset exists for target {target_id}")
    preset = PRESETS[target_id]
    return {
        "candidate_target_id": target_id,
        "support_frames": support_frames,
        "frame_count": len(frames),
        "support_fraction": support_fraction,
        "selected_preset": preset["name"],
        "focus_roi_xyxy_px": list(preset["focus_roi_xyxy_px"]),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--minimum-support-fraction", type=float, default=0.8)
    options = parser.parse_args(args)
    if options.output_json.exists():
        raise ValueError(f"refusing to overwrite {options.output_json}")

    source = json.loads(options.input_json.read_text(encoding="utf-8"))
    if source.get("formal_acceptance") is not False:
        raise ValueError("base overview must be explicitly non-acceptance")
    if source.get("window_boundary") != "base_overview_before_robot_motion":
        raise ValueError("base overview window boundary is not pre-motion")
    selection = select_observation_preset(
        source.get("frames", []),
        minimum_support_fraction=options.minimum_support_fraction,
    )
    result = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "NON_ACCEPTANCE_DUAL_CAMERA_OBSERVATION_SELECTION",
        "formal_acceptance": False,
        "held_out_test_consumed": False,
        "identity_source": "simulation_ground_truth_association",
        "base_window": {
            "path": str(options.input_json),
            "sha256": _sha256(options.input_json),
        },
        **selection,
        "wrist_observation_authorized": True,
        "pick_authorized": False,
        "state_history": [
            "OVERVIEW_ACQUIRE",
            "OVERVIEW_CANDIDATE_VALIDATED",
            "WRIST_PRESET_SELECTED",
        ],
    }
    options.output_json.parent.mkdir(parents=True, exist_ok=True)
    options.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
