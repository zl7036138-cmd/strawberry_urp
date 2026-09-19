# ADR 0063: Freeze the rendered-occlusion localization matrix

- Recorded: 2026-08-05
- Status: Accepted; one non-formal execution authorized
- Scope: Blender-v2 simulator localization isolation, no robot motion

## Context

ADR 0062 passes a 40-frame paired-depth diagnostic, but its occluders are
offline depth interventions and its two captures are slight variations of one
requested wrist pose. Runtime promotion therefore remains blocked. The next
bounded question is whether geometry-layer localization works across distinct
target viewpoints when the RGB-D renderer, rather than an array edit, produces
the foreground surface.

This is a new post-submission research matrix. It is not a rerun or rewrite of
the consumed P3/P4 experiments.

## Decision

1. Use the Blender-v2 scene, 26 mm ripe-fruit radius, fixed base RGB-D camera,
   and exactly five previously reviewed camera-visible target positions.
2. Test nominal lighting with `none`, `partial`, and `heavy` visual-only blue
   occluders, for 15 separately materialized Gazebo worlds.
3. Use simulator truth only to construct an unpadded (1.00) spherical oracle box and
   to measure error. Do not start a perception model. This isolates
   localization from classification and detector-box variation.
4. Run `center_median` and `geometry_layer` simultaneously. Publish each oracle
   detection once so both nodes consume the same stamped depth frame. Disable
   their simulator truth-association guard so unsafe legacy estimates remain
   observable as measurements; no output may be connected to control.
5. Collect exactly 20 paired frames per scenario, for 300 frames and 600
   estimator observations.
6. Require all 15 scenarios to complete. At every position, rendered blue
   coverage must be at most 0.02 for none, at least 0.05 for partial, within
   0.35--0.98 for heavy, and heavy must exceed partial by at least 0.15.
7. Require geometry-layer acceptance at every position to be at least 0.95 for
   none, 0.80 for partial, and 0.60 for heavy. Its aggregate P95 error must not
   exceed 30 mm for any occlusion level.
8. Keep 15 mm as the separate planning-sigma screen. A localization diagnostic
   may pass while runtime promotion remains blocked.
9. Permit only simulated fruit pose changes before measurement and visual-only
   occluders. Robot motion, gripper commands, manipulation, orchestration,
   attachment, perception-model inference, held-out real-test access, and
   formal acceptance are prohibited.
10. Execute the hash-bound matrix once into a fresh directory. Preserve a
    failed result; do not tune positions, occluders, thresholds, or estimator
    parameters after observing it.

## Pre-execution amendment

The full 15-scenario execution had not started when the single centre-position
preflight exposed three infrastructure/modeling defects: ordinary Python was
given ROS-only arguments, renamed nodes did not match name-scoped parameter
files, and integer `floor`/`ceil` oracle boxes were treated as exact continuous
diameters. The runner now uses unique node names with wildcard-scoped parameter
files, waits for sensor and TF readiness, and enables an explicit 2 px
quantization interval only for the geometry-layer development configuration.
The pre-existing zero-margin default and 40-frame offline matrix behavior are
preserved.

The same no-motion preflight calibrated the visual-only widths to 0.022 m for
partial and 0.035 m for heavy, while leaving every acceptance/error threshold
unchanged. At the centre position their rendered blue coverage was 0.442 and
0.613. Geometry-layer error was 18.8 mm and 27.0 mm; the heavy legacy estimate
was wrong by about 351 mm. These are calibration observations, not formal
matrix evidence. After binding this amendment, no further parameter tuning is
allowed from the full matrix result.

## Consequences

A passing result would establish multi-position localization behavior under
rendered geometric occlusion, not end-to-end perception or grasp success.
Natural leaf occlusion, YOLO boxes, the wrist camera in field-v3, and robot
motion require later independent gates. A failed result identifies the next
localization limitation without weakening any safety threshold.
