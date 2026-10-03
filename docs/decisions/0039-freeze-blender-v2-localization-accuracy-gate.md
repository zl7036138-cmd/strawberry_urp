# ADR 0039: Freeze the Blender-v2 localization accuracy gate

- Recorded: 2026-07-27
- Status: Accepted
- Scope: New non-formal v2 geometry qualification after ADR 0038

## Context

The historical T40 localization evidence belongs to the archived tabletop-v1
scene and its rigid 35 mm fruit. The canonical Blender-v2 fruit uses a 26 mm
collision radius and an irregular visual mesh. A fresh isolated diagnostic
shows that the production v2 localization path can publish TargetPose messages,
but it does not measure three-dimensional accuracy over a representative
volume. Reusing the old T40 result would therefore overstate the v2 evidence.

Natural plant occlusion and localization geometry are separate questions.
Measuring both in one run would make a failed depth sample ambiguous: it could
come from the 26 mm surface-to-centre model or from a foreground leaf. The
first v2 accuracy gate must isolate geometry before the later multi-view,
natural-plant robustness work.

## Decision

1. Create one new development gate named
   `blender_v2_localization_accuracy_100_v1`. It is not T40, formal
   acceptance, real-world evidence, or a reopening of P3/P4.
2. Use the canonical fixed base RGB-D camera, production localization node,
   Blender-v2 ripe-fruit mesh, 26 mm scene radius, simulator ground-truth
   stream, and an oracle bounding box derived from truth. YOLO is intentionally
   excluded so the result measures depth projection and TF localization.
3. Materialize a camera-clear world from the canonical Blender-v2 world. Keep
   the camera, table, planter, robot, lighting, and target fruit unchanged.
   Park the plant and both non-target fruits outside the camera volume. This
   run may move only simulated fruit entities; arm trajectories, gripper
   commands, attachment, planning, and pick actions remain disabled.
4. Move `strawberry_1` through exactly 100 distinct positions in a frozen
   `5 × 5 × 4` Cartesian grid:

   - X: `0.44, 0.47, 0.50, 0.53, 0.56 m`;
   - Y: `-0.12, -0.075, -0.03, 0.015, 0.06 m`;
   - Z: `0.51, 0.53, 0.55, 0.57 m`.

   Keep the target orientation at the identity quaternion for every sample.
5. At each position, wait for fresh truth and at least three settled depth
   frames. Allow at most three sensor/publication attempts for that position.
   An attempt is not an experiment rerun and may not change any parameter.
6. Freeze the pass conditions before execution:

   - exactly `100/100` valid measurements;
   - three-dimensional median error no greater than `15 mm`;
   - three-dimensional P95 error no greater than `30 mm`;
   - no unhandled runtime failure; and
   - no robot or gripper command.

7. Execute the frozen gate once into a fresh, non-overwriting result directory.
   A failed result is preserved and diagnosed; thresholds, radius, crop,
   positions, retries, or samples may not be changed under this decision.

## Boundaries

Passing this gate qualifies only the camera-clear Blender-v2 localization
geometry. It does not qualify natural leaf occlusion, detector performance,
multi-view fusion, gripper geometry, attachment, trajectory execution,
perception-controlled picking, physical hardware, or sim-to-real behavior.
The formal real held-out test remains sealed.
