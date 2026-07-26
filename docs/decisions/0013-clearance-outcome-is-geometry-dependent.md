# ADR 0013: Treat multi-fruit clearance as geometry-dependent, not a scalar threshold

- Status: Accepted
- Date: 2026-07-15
- Owners: T00 / T50 / T60

## Context

ADR 0012 paired five already-reachable target poses with one fixed two-neighbor
layout. The first run was incomplete because the fifth world's Gazebo
`set_pose` service did not answer within five seconds. Its first four trials are
useful defect/diagnostic evidence, but its aggregate is not authoritative.

The client now retries a timed-out pose request exactly once and records the
attempt count. The summary also classifies setup failures as infrastructure
failures instead of inferring a collision from generic MoveIt startup text.
The complete v2 run used first-attempt pose configuration for all fifteen model
placements.

## Evidence

`results/t60/oracle_shadow_multifruit_clearance_diagnostic_v2/summary.json`
is infrastructure-valid in 5/5 fresh worlds. Oracle motion succeeds at four
positions; Shadow emits ripe detections and target poses in all five.

The approximately 133.961, 108.676, and 91.555 mm surface-clearance scenarios
succeed. The 64.629 mm scenario fails during `APPROACH`: both v1 and v2 reject
the same final Cartesian waypoint after reporting contact between
`panda_hand` and `strawberry_fruit_2`. The closer 30.000 mm scenario succeeds.

## Decision

1. Do not derive or publish a scalar minimum fruit-to-fruit clearance from this
   result. Clearance magnitude alone is not monotonic with outcome.
2. Attribute the repeated fourth-position failure to the relative target,
   neighbor, and hand-path geometry. Preserve the collision rejection as a
   correct fail-closed result; do not delete the obstacle, relax collision
   checking, expand tolerances, or add retries.
3. Preserve all five coordinates as valid single-target reachability
   candidates. For this exact multi-fruit layout, positions 1, 2, 3, and 5 are
   passing diagnostic candidates; position 4 is not.
4. Do not optimize the guarded approach yet. P4 permits one measured bottleneck
   intervention, and the project must first validate lighting/occlusion scene
   injection and the complete integration pipeline before choosing it.
5. If crowded-scene manipulation becomes the selected P4 bottleneck, the next
   experiment must hold one reachable target fixed and vary neighbor direction
   and height at a fixed center distance. That design can isolate geometry from
   distance and target reachability.
6. This remains Oracle-controlled non-acceptance evidence. It cannot close T30,
   P2, full P3, or P4, and the held-out real-image test remains sealed.

## Consequences

- Multi-fruit planning is demonstrated at 4/5 paired layouts without weakening
  safety behavior.
- The failed layout is reproducible and precisely attributed, so it can be
  revisited later without blocking lighting/occlusion infrastructure work.
- Formal documentation must report neighbor pose, relative vector, and first
  colliding link rather than only a scalar clearance value.
