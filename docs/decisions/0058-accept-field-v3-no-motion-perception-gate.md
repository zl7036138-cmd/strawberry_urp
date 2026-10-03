# ADR 0058: Accept the field-v3 no-motion perception gate

- Recorded: 2026-07-28
- Status: Accepted
- Scope: Opt-in `st1.blend` field scene, dual-camera health, target-1
  selection, and controller-free RGB-D localization

## Context

The supplied `st1.blend` asset was exported as an instanced Gazebo field with
101 background plants and three simplified ridge collisions. Two source plants
were removed from one outer-row bay and replaced with the already-qualified
Blender-v2 workcell plant and its three independent fruit models. The accepted
v2 world remains the default.

The no-motion render and sensor result at
`results/development/field_v3_no_motion_v2` passed both camera health probes
with no violations. The base and wrist RGB streams measured 3.625 Hz and
8.181 Hz respectively during the bounded wall-time probes, both camera TF
chains were available, and the arm spawned directly in a previously qualified
observation posture without receiving a trajectory command.

The first field perception shadow run failed closed. Its detector preferred
the upper ripe fruit while the control contract expected target 1. The
localizer consistently and correctly associated that fruit with simulator
target 3, producing 684 identity mismatches rather than silently relabelling
it. This showed that the failure was candidate selection, not loss of
localization.

The revised run binds target 1 to the reviewed lower-right wrist ROI. Its
`results/development/field_v3_perception_shadow_v2/shadow_window.json` has
SHA-256
`311d9535634ffd56e525eb92f662272662b335685d65a15e5f125a0c8895f9f7`.
All 60 measured frames contained a ripe detection and target pose, and the
15-consecutive-frame exact-identity readiness gate passed. Its
`target_pose_accuracy.json` has SHA-256
`ee5da7bb5a7e9e6356f6fb30f0350d8e7b4dec56c1563e7f99a42c2530f5f044`.
Across 60 samples, target-1 centre error was 4.441 mm median, 4.444 mm P95,
and 4.445 mm maximum, below the frozen 15 mm / 30 mm limits. The run sent zero
control commands and did not start a pick action.

## Decision

1. Accept field-v3 as an opt-in alternative scene for rendering, dual-camera
   sensing, target-1 selection, and no-motion three-dimensional localization.
2. Preserve the failed first shadow result as evidence that exact target
   identity fails closed.
3. Keep the target-1 wrist ROI scene-bound in `scene_field_v3.yaml`; do not
   change the canonical v2 selection behavior.
4. Treat the accepted result as non-formal development evidence. It does not
   replace the accepted Blender-v2 end-to-end pick.
5. Keep `motion_authorized: false`, attachment disabled, and pose control
   disabled for the field launcher.
6. Require field-specific MoveIt collision parity for the three ridges,
   ground/workcell, and relocated bin before any planning or motion
   authorization.
7. After collision parity passes, run a controller-free planning-scene and
   pre-grasp audit before considering an Oracle execution.

## Consequences

The project now has a reproducible large-field scene in which the base and
wrist cameras, detector, exact target selection, depth, TF, and
three-dimensional localization work without moving the robot. The next risk is
not perception; it is disagreement between Gazebo and MoveIt environment
geometry. Until that is resolved and audited, a successful field-v3 grasp or
full-process simulation must not be claimed.
