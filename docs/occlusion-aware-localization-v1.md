# Occlusion-aware localization v1

## Purpose

The frozen localization baseline takes the median depth from the central 30%
of a ripe detection. This is accurate when the fruit occupies the box centre,
but it can return a foreground leaf or benchmark occluder when the fruit is
only visible near the box edges. P4 preserved the resulting failure: ripe
detection reached 300/300 heavy-occlusion frames while target-pose output
remained 0/300.

This development change introduces a geometry-guided depth-layer estimator.
It is a localization study only. It does not rerun P3 or P4, use the rejected
P4 checkpoint as a control source, access the sealed real test, or authorize
robot motion.

## Algorithm

For one ripe detection, `geometry_layer`:

1. reads all valid depth pixels in a configurable central fraction of the box;
2. sorts the depths and splits them at discontinuities greater than
   `geometry_layer_gap_m`;
3. rejects layers below either `min_valid_pixels` or the configured minimum
   fraction of the box area;
4. predicts a surface depth from the detection's apparent diameter, the camera
   focal length, and the scene-bound fruit radius;
5. selects the nearest compatible layer only when its error is within
   `geometry_expected_depth_tolerance_m`;
6. rejects the observation if the two nearest layers differ by less than
   `geometry_ambiguity_margin_m`; and
7. projects the selected layer's pixel centroid instead of the bounding-box
   centre.

The existing `surface_to_center_offset_m` is also the target radius used by
the geometric prior. A zero or invalid radius is rejected in geometry mode.
Simulation ground truth remains limited to identity and audit checks after the
point is reconstructed; it is never used to select a depth layer.

## Parameters

| Parameter | Baseline value | Meaning |
|---|---:|---|
| `depth_estimator_mode` | `center_median` | `geometry_layer` is opt-in |
| `geometry_search_fraction` | `1.0` | Fraction of the detection box searched |
| `geometry_layer_gap_m` | `0.015` | Minimum discontinuity between depth layers |
| `geometry_min_layer_fraction` | `0.03` | Minimum box-area support for a candidate layer |
| `geometry_expected_depth_tolerance_m` | `0.08` | Maximum difference from the size-derived surface depth |
| `geometry_ambiguity_margin_m` | `0.01` | Minimum advantage over the second candidate layer |
| `geometry_ambiguity_min_support_ratio` | `0.50` | Runner-up support required before a near tie is rejected |
| `geometry_bbox_quantization_margin_px` | `0.0` | Optional integer-box interval; the rendered oracle-only config uses `2.0` |
| `min_valid_pixels` | `9` | Minimum samples in the selected layer |

The frozen Blender-v2 and legacy YAML files remain byte-for-byte unchanged and
therefore use the node default `center_median`. The separate
`localization_geometry_layer_v1.yaml` file opts into the new estimator. A
future ROS test must use that development config and a new output directory.

ADR 0062 adds a support-aware ambiguity rule after the first identical-frame
matrix exposed a false rejection under 10% detection-box shrinkage. Geometric
near ties still reject when the runner-up has at least half the selected
layer's support; equal-support ambiguity remains fail-closed.

## Offline diagnostic

Run:

```bash
source /opt/strawberry_venv/bin/activate
python scripts/evaluate_occlusion_depth_estimator.py
```

The deterministic synthetic receipt covers four cases:

| Case | Legacy centre median | Geometry layer |
|---|---|---|
| Clear fruit | estimates 0.50 m | estimates 0.50 m |
| Fruit visible at edges, centre occluded at 0.18 m | selects 0.18 m foreground | recovers 0.50 m fruit layer |
| Fruit layer absent | selects 0.18 m foreground | rejects as size-inconsistent |
| Two equally plausible layers | returns their central median | rejects as ambiguous |

The receipt is written to
`results/development/occlusion_aware_depth_layer_v1/receipt.json`. Its status
must be `PASS`, but it remains synthetic-depth-only evidence.

## Field-v3 no-motion Shadow

The first fresh runtime study is stored at
`results/development/field_v3_geometry_layer_shadow_v1`. The fail-closed
summary reports:

| Measurement | Result |
|---|---:|
| Measured frames | `60/60` |
| Frames with a ripe detection | `60/60` |
| Frames with a target pose | `60/60` |
| Accuracy samples for target 1 | `60` |
| Median / maximum position error | `3.148 / 3.148 mm` |
| Median reported position sigma | `0.028098 m` |
| Observed joint delta | `0.0 rad` |
| Control, pick, trajectory, or gripper commands | `0` |
| Summary status | `PASS` |

The summary SHA-256 is
`c2e3e48d9aa31fde40d1e0bf3d906da0668cfe215db9b2d06d8f3b14160d8e72`.
The 3.148 mm error is lower than the historical 4.856 mm field-v3
centre-median record, but that is a scene-matched observation, not an
identical-frame formal comparison. The runtime run covers one fixed target and
natural field-scene occlusion only. It does not reproduce the synthetic P4
benchmark occluder or prove the missing-fruit and ambiguous-layer rejection
paths in Gazebo.

## Promotion gate

Runtime promotion remains blocked. The Shadow study establishes ROS topic and
TF integration under zero motion, but the 28.1 mm median uncertainty is large
relative to the 3.15 mm observed error. It must not be tuned away from this one
scene. A promotion decision needs a new no-motion matrix with varied target
poses, apparent sizes, and controlled occlusions, including paired
`center_median` and `geometry_layer` evaluation on identical recorded frames.
The matrix must also exercise absent-fruit and ambiguous-layer rejection.

Only after that study passes may a separate decision consider enabling
`geometry_layer` for field-v3 planning. Motion still requires explicit
authorization. The frozen P3/P4 artifacts and the sealed real test remain
untouched.

The paired 40-frame matrix, preserved failure, repaired result, and remaining
promotion blockers are documented in
[`paired-depth-estimator-matrix-v3.md`](paired-depth-estimator-matrix-v3.md).

The subsequent five-position rendered matrix completed all 300 paired frames
but failed its frozen diagnostic because one no-occluder ROI contaminated the
blue colour metric and one clear position produced only 18/20 geometry poses.
All three geometry P95 error checks passed, including `26.962 mm` under heavy
occlusion versus `361.727 mm` for the centre median, but sigma and broader
viewpoint/detector gates still block promotion. See
[`rendered-occlusion-localization-matrix-v1.md`](rendered-occlusion-localization-matrix-v1.md)
and ADR 0064.
