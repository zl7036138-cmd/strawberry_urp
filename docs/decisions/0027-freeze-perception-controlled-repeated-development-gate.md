# ADR 0027: Freeze the repeated perception-controlled development gate

- Recorded: 2026-07-17
- Status: Accepted
- Scope: Non-formal P3 simple-scene development evidence

## Context

ADR 0026 authorizes the hash-bound below-gate model for bounded simulator
control. Its first isolated perception-controlled smoke completed 1/1, but a
single success is insufficient to establish repeatability or only-unripe
`NO_PICK` behavior.

## Decision

1. Run ten positive and ten negative trials. Every trial starts a fresh Gazebo
   world and uses a distinct ROS domain ID.
2. Positive worlds expose only `strawberry_1` at the already verified base pose
   `(0.42, -0.12, 0.52)` m. The other fruit are parked outside the work scene.
3. Negative worlds expose only unripe `strawberry_2` at the same pose and park
   both ripe fruit outside the scene.
4. Before each negative trial, require at least ten post-setup detection-array
   frames. This prevents an unavailable perception node from masquerading as a
   safe `NO_PICK`.
5. Accept the development gate only when positive success is at least 80%, all
   ten negative trials finish as `NO_PICK`, the false-pick rate is at most 5%,
   no negative trial attempts control, all infrastructure and shutdown checks
   pass, and successful positive planning p95 is at most five seconds.
6. Keep the exact ADR-0026 checkpoint, threshold `0.58`, topics, Oracle-off
   routing, and held-out-test seal.

## Boundaries

This gate measures fresh-world repeatability at one clear pose. It is not the
formal 135-positive plus 30-negative matrix and cannot establish robustness to
position, lighting, occlusion, seeds, real images, or sim-to-real transfer.
Its results may authorize the next P3 development step, not formal P3/P4
acceptance.
