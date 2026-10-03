# ADR 0076: Use an opposite-side overview and prove partial batch recovery

## Status

Accepted as a development runtime milestone. A two-ripe global scene and a
truth-free physical pick after another target is skipped are proved. Two fruits
harvested in one batch and the formal generalization gate are not yet proved.

## Context

The prior base camera position at `[-0.35, -0.42, 1.05]` was reliable but the
ready gripper occluded the plant centre. Development seed 44008 therefore
produced one stable ripe detection even though its scene truth contains four
ripe fruits. Increasing the base RGB-D resolution to 640x480 overloaded the
development runtime and was rejected. Re-aiming the same-side 320x240 camera
toward the plant centroid still produced only one ripe detection.

The generalized localizer also charged the full apparent-size residual to the
base-camera position sigma. The frozen depth-layer estimator already rejects
unsupported, ambiguous and geometrically implausible layers before the
generalized runtime applies its bounded residual calibration. The wrist path
already used a factor of 0.5 for quantized detector boxes.

## Decision

- Keep the base stream at 320x240 and 10 simulated Hz.
- Mount the overview mast at `[-0.35, 0.45, 0.05]` and aim its camera with RPY
  `[0, 0.543, -0.480]`, viewing the plants from the side opposite the bin-side
  ready gripper.
- Apply the existing 0.5 apparent-size residual calibration to the generalized
  base localizer only after all depth-layer rejection gates pass. Do not relax
  the 15 mm selection limit, maturity gate, reach bounds or collision checks.
- Preserve fresh-scan barriers after every arm motion and suppress completed
  targets without replaying a cached selection.
- Preserve strict 0.05 rad controller path/goal tolerances and the independent
  live seven-joint limit monitor.
- Keep the 30 fixed formal seeds sealed. These results are development evidence
  from seed 44008 only.

## Evidence

The 60-frame no-motion shadow `opposite_side_shadow_v57` completed with two
ripe and two unripe detections in every frame, 60/60 frames with a localized
ripe target, and a clean process-group shutdown. Ripe confidence remained
between 0.8273 and 0.8328. Receipt SHA-256:
`d757f1da33e43ac461b881c8a0ed349aa87d25533b4e9c15f377596f51c369fe`.
The saved overview image SHA-256 is
`eb5876494de17708879400c55357171285868c03a72e2d744eb2c2827fed3f1e`.

The full truth-free runtime `opposite_side_v59` established two stable ripe
tracks. Track 1 failed bounded pre-grasp IK/collision feasibility, was retried
once, returned home and was skipped. The batch then selected track 2, used the
fourth of six dynamic wrist views, accepted a 10.37 mm wrist correction, and
completed physical bilateral contact. Anonymous contact resolved the attached
simulation object to fruit entity 4 without a truth identity lookup. Retreat,
release, bin verification and recovery home completed, yielding
`PARTIAL_SUCCESS`, `harvested=[2]`, one skipped target, zero execution failures
and `CLEAN` shutdown. Runtime receipt SHA-256:
`d2e64f9e00c00c0c4609aa456a201d829606f3ef7e60029e14527eb7c574a969`.

The dependency-light suite passes 697 tests with zero failures or errors; two
environment-specific tests remain skipped by design.

## Consequences

The runtime now proves that a failed target does not terminate the batch and
that a later target can complete the full physical pick/place loop. The
opposite-side camera doubles stable ripe visibility for this development seed.

The selector still ranks candidates using conservative geometry before MoveIt
feasibility is known. Consequently it publishes track 1 as selected, and the
orchestrator must discover the infeasible path and skip it before attempting
track 2. The next change must bring bounded MoveIt feasibility into the ranking
decision, then demonstrate at least two successful harvests in a scene where
two mature targets are genuinely feasible.
