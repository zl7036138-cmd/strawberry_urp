# ADR 0011: Synchronize the MoveIt fruit collision scene from live simulation truth

- Status: Accepted
- Date: 2026-07-15
- Owners: T50 / T60

## Context

ADR 0010 authorized a diagnostic-only path in which Gazebo moves the target
fruit through explicit positions while Oracle remains the sole motion source
and YOLO runs only on shadow topics. A follow-on single-target isolation run
also parked the two non-target fruit outside the work area. Gazebo and the
Oracle target stream acknowledged those new poses, but MoveIt continued to use
the immutable initial fruit coordinates from the scene manifest.

The pre-fix isolation run therefore succeeded in only 2/5 scenarios and
reported three collisions with `strawberry_fruit_2`, even though that model had
been parked at `y=-2.0 m`. Those contacts were planning-scene ghost obstacles,
not physical multi-fruit clearance failures. The run remains preserved as
defect evidence, but it is not a valid single-target reachability result.

## Decision

1. The manipulation action server subscribes to
   `/strawberry/ground_truth/poses` in simulation and maintains a thread-safe,
   timestamped snapshot of every manifest fruit pose.
2. Immediately before preparing a pick, MoveIt replaces all fruit collision
   spheres with the fresh snapshot. The expected ID set must match the scene
   manifest exactly, every position must be finite, and the snapshot must be no
   more than 2.0 seconds old. Missing, stale, incomplete, or malformed truth
   fails closed before arm motion.
3. The action goal remains authoritative for the selected target pose. After
   synchronizing all obstacles, the selected collision sphere is updated from
   the action goal so the contact corridor is centered on the commanded pose.
4. After every post-prepare exit, the selected collision sphere is restored at
   its latest live simulation position rather than its immutable initial
   manifest position. Non-target fruit are never deleted, collision checking is
   not weakened, and retries or tolerances are not expanded.
5. Simulation truth may synchronize the planning scene, but it may not select a
   target, generate a detector result, replace the perception target pose, or
   change the Oracle/Shadow routing frozen by ADR 0010.
6. The pre-fix single-target v1 result is retained only as defect evidence. The
   first valid isolated calibration result is the post-fix v2 run.

## Verification

- Dependency-light WSL suite: 291 tests, zero failures/errors, one conditional
  skip.
- Full ROS workspace: seven packages, 184 tests, zero failures/errors/skips.
- Post-fix runtime logs record `MoveIt live fruit scene synchronized` in all
  five fresh worlds.
- `results/t60/oracle_shadow_single_target_position_diagnostic_v2/summary.json`
  is complete: Oracle motion succeeds 5/5, no failure attribution or collision
  object is recorded, and the baseline YOLO shadow emits ripe detections and a
  target pose in 5/5 scenarios.

## Consequences

- Teleported or procedurally placed simulation fruit and MoveIt now share one
  current geometric state, eliminating the diagnosed ghost-obstacle defect.
- The five camera-clear coordinates are valid single-target reachability
  calibration candidates, but are still diagnostic-only coordinates.
- Multi-fruit clearance remains a separate problem; the original 1/5
  multi-fruit run must not be reinterpreted as a reachability gate.
- Before a formal perception-controlled P3 run, T00 must explicitly freeze the
  allowed use of simulator truth for non-target planning-scene obstacles. Such
  truth may never be counted as perception evidence or used to command the
  target.
- T30, P2, full P3, and P4 remain unaccepted, and the held-out real-image test
  remains sealed.
