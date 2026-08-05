#!/usr/bin/env python3
"""Evaluate legacy and geometry-layer depth estimators without ROS or motion."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Callable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws" / "src" / "strawberry_localization"
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_localization.core import (  # noqa: E402
    BoundingBox,
    CameraIntrinsics,
    LocalizationError,
    robust_center_depth,
    robust_geometry_layer_depth,
)


DEFAULT_OUTPUT = (
    ROOT
    / "results"
    / "development"
    / "occlusion_aware_depth_layer_v1"
    / "receipt.json"
)
BOX = BoundingBox(20, 20, 60, 60)
INTRINSICS = CameraIntrinsics(600.0, 600.0, 50.0, 50.0)
TARGET_RADIUS_M = 0.026
TARGET_SURFACE_DEPTH_M = 0.50


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _base_scene() -> np.ndarray:
    return np.full((100, 100), 1.8, dtype=np.float32)


def _fruit_scene(*, center_occluded: bool) -> np.ndarray:
    depth = _base_scene()
    yy, xx = np.ogrid[:100, :100]
    fruit_mask = (xx - 50) ** 2 + (yy - 50) ** 2 <= 28**2
    depth[fruit_mask] = TARGET_SURFACE_DEPTH_M
    if center_occluded:
        depth[20:80, 40:60] = 0.18
    return depth


def _fruit_absent_scene() -> np.ndarray:
    depth = _base_scene()
    depth[20:80, 40:60] = 0.18
    return depth


def _ambiguous_scene() -> np.ndarray:
    depth = _base_scene()
    depth[20:80, 20:50] = 0.48
    depth[20:80, 50:80] = 0.51
    return depth


def _run_estimator(
    callback: Callable[[], Any],
) -> dict[str, Any]:
    try:
        estimate = callback()
    except (LocalizationError, TypeError, ValueError) as exc:
        return {"status": "REJECTED", "reason": str(exc)}
    return {
        "status": "ESTIMATED",
        "depth_m": float(estimate.depth_m),
        "absolute_error_m": abs(
            float(estimate.depth_m) - TARGET_SURFACE_DEPTH_M
        ),
        "sigma_m": float(estimate.sigma_m),
        "valid_pixels": int(estimate.valid_pixels),
        "selected_pixel": [float(estimate.center_u), float(estimate.center_v)],
    }


def _evaluate_scene(depth: np.ndarray) -> dict[str, Any]:
    return {
        "legacy_center_median": _run_estimator(
            lambda: robust_center_depth(depth, BOX)
        ),
        "geometry_layer": _run_estimator(
            lambda: robust_geometry_layer_depth(
                depth,
                BOX,
                INTRINSICS,
                target_radius_m=TARGET_RADIUS_M,
                expected_depth_tolerance_m=0.08,
            )
        ),
    }


def evaluate() -> dict[str, Any]:
    scenarios = {
        "clear_fruit": _evaluate_scene(_fruit_scene(center_occluded=False)),
        "center_occluded_fruit_visible_at_edges": _evaluate_scene(
            _fruit_scene(center_occluded=True)
        ),
        "fruit_depth_absent": _evaluate_scene(_fruit_absent_scene()),
        "ambiguous_candidate_layers": _evaluate_scene(_ambiguous_scene()),
    }
    checks = {
        "clear_legacy_accurate": (
            scenarios["clear_fruit"]["legacy_center_median"]["status"]
            == "ESTIMATED"
            and scenarios["clear_fruit"]["legacy_center_median"]
            ["absolute_error_m"]
            <= 1e-6
        ),
        "clear_geometry_accurate": (
            scenarios["clear_fruit"]["geometry_layer"]["status"]
            == "ESTIMATED"
            and scenarios["clear_fruit"]["geometry_layer"]["absolute_error_m"]
            <= 1e-6
        ),
        "occluded_legacy_selects_foreground": (
            scenarios["center_occluded_fruit_visible_at_edges"]
            ["legacy_center_median"]["status"]
            == "ESTIMATED"
            and scenarios["center_occluded_fruit_visible_at_edges"]
            ["legacy_center_median"]["absolute_error_m"]
            >= 0.30
        ),
        "occluded_geometry_recovers_fruit": (
            scenarios["center_occluded_fruit_visible_at_edges"]
            ["geometry_layer"]["status"]
            == "ESTIMATED"
            and scenarios["center_occluded_fruit_visible_at_edges"]
            ["geometry_layer"]["absolute_error_m"]
            <= 1e-6
        ),
        "missing_fruit_fails_closed": (
            scenarios["fruit_depth_absent"]["geometry_layer"]["status"]
            == "REJECTED"
        ),
        "ambiguous_layers_fail_closed": (
            scenarios["ambiguous_candidate_layers"]["geometry_layer"]["status"]
            == "REJECTED"
        ),
    }
    return {
        "schema_version": 1,
        "kind": "occlusion_aware_depth_layer_offline_diagnostic",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "inputs": {
            "image_shape": [100, 100],
            "bounding_box_xywh": [BOX.x, BOX.y, BOX.width, BOX.height],
            "intrinsics": {
                "fx": INTRINSICS.fx,
                "fy": INTRINSICS.fy,
                "cx": INTRINSICS.cx,
                "cy": INTRINSICS.cy,
            },
            "target_radius_m": TARGET_RADIUS_M,
            "target_surface_depth_m": TARGET_SURFACE_DEPTH_M,
            "geometry_expected_depth_tolerance_m": 0.08,
        },
        "checks": checks,
        "scenarios": scenarios,
        "scope": {
            "synthetic_depth_only": True,
            "ros_runtime": False,
            "robot_motion": False,
            "formal_p3_or_p4_evidence": False,
            "held_out_real_test_consumed": False,
            "runtime_promotion_authorized": False,
        },
        "source": {
            "core_path": (
                "ros2_ws/src/strawberry_localization/"
                "strawberry_localization/core.py"
            ),
            "core_sha256": _sha256(
                PACKAGE_ROOT / "strawberry_localization" / "core.py"
            ),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    receipt = evaluate()
    output = arguments.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True)
            + "\n"
        )
    print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if receipt["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
