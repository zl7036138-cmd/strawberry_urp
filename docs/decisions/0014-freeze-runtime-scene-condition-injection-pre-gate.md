# ADR 0014: Freeze a non-acceptance runtime scene-condition injection pre-gate

- Status: Accepted
- Date: 2026-07-15

## Context

The formal P4 matrix requires three lighting levels and three occlusion levels,
but the existing benchmark labels did not yet prove that Gazebo actually
rendered nine distinct conditions. Running the formal 165-trial benchmark before
this infrastructure check would make a failed or unchanged injection
indistinguishable from a system robustness result.

This check must not consume the sealed real-image test set, close T30/P2/P3/P4,
or allow perception output to control the robot.

## Decision

Freeze `config/t70_scene_conditions.json` as the pre-gate contract. Generate a
new SDF from the immutable base world for each of the nine `dim|nominal|bright`
by `none|partial|heavy` combinations. Each world and its receipt are SHA-256
bound to the base world and configuration.

Lighting is changed through the world ambient light and sun diffuse/specular
values. Occlusion is injected as a blue, static, visual-only box with no
collision element. This ensures that the pre-gate measures camera-image change
without changing MoveIt or Gazebo collision outcomes.

For seed `20260710`, fruit 1 is placed at the frozen validation pose and fruit 2
and fruit 3 are parked outside the working scene. Five settled RGB frames are
measured for every condition. The gate checks ordered luminance gaps, ordered
blue coverage in the projected target ROI, nine distinct world hashes, complete
runtime receipts, and clean logs.

The generated summary must always state `formal_acceptance=false`,
`held_out_test_consumed=false`, and that it cannot close T30, P2, P3, or P4.

The first runtime execution is retained as
`results/t70/scene_condition_injection_pre_gate_v1`. All nine infrastructure
records were valid, but the heavy occluder covered 100% of the projected target
ROI and correctly failed the frozen 98% maximum. Before any acceptance use, the
heavy box was revised from pose x `0.329 m` and size `0.060 x 0.018 x 0.100 m`
to the target-aligned x `0.346 m` and size `0.050 x 0.018 x 0.090 m`. The v1
artifact remains immutable failure evidence; the revised configuration requires
a complete nine-world v2 rerun with new hashes.

## Consequences

Passing establishes only that the runtime condition injector is observable and
reproducible. It does not establish detector quality, manipulation robustness,
sim-to-real transfer, or the formal 80% end-to-end target. A failed check blocks
the formal robustness matrix until its injection infrastructure is corrected.
