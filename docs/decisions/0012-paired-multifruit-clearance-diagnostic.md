# ADR 0012: Pair reachable positions with a bounded multi-fruit clearance sweep

- Status: Accepted
- Date: 2026-07-15
- Owners: T00 / T50 / T60

## Context

The post-ADR-0011 single-target diagnostic proves that all five camera-clear
target coordinates are reachable when the two non-target fruit are outside the
work area. The earlier 1/5 multi-fruit run used different target coordinates
and a partly implicit scene, so it cannot separate target reachability from
neighbor clearance.

The project needs a bounded paired comparison before changing approach paths or
freezing formal positions. The below-gate detector must remain observational,
and no result from this diagnostic may be promoted into a perception or
robustness acceptance claim.

## Decision

1. Reuse the exact five target coordinates from the valid single-target v2
   diagnostic. This makes its 5/5 Oracle result the paired no-neighbor baseline.
2. Place `strawberry_2` at `[0.44, 0.10, 0.52] m` and `strawberry_3` at
   `[0.38, 0.24, 0.50] m` in every fresh world. Their identities, poses, and
   0.035 m collision radius are hash-bound by the executable manifest.
3. The resulting minimum target-to-neighbor surface clearances, in scenario
   order, are approximately 133.961, 108.676, 91.555, 64.629, and 30.000 mm.
   The contract rejects overlap, missing neighbors, duplicate identities, or a
   non-decreasing sweep.
4. Oracle on `/strawberry/oracle/target_pose` remains the sole control source.
   `baseline__best` remains hash-bound at confidence 0.31 and may publish only
   on `/strawberry/shadow/*` topics.
5. MoveIt must retain both non-target collision spheres and all existing
   collision checks. No tolerance, retry count, approach geometry, or collision
   allowance may change during this diagnostic.
6. Gazebo truth is authorized here only to synchronize the deliberately placed
   simulation collision objects. It may not choose the target or replace the
   action target pose. This authorization is diagnostic-only and does not
   authorize live fruit truth in a formal perception-controlled P3/P4 run.
7. Motion failure is valid evidence when scene placement, source isolation,
   Shadow evidence, and shutdown remain valid. The first collision object and
   failure stage must be retained.

The executable contract is
`config/t60_oracle_shadow_multifruit_clearance_diagnostic.json`.

## Exit interpretation

- A 5/5 result means the current guarded approach tolerates this particular
  paired 30-134 mm surface-clearance sweep; it does not prove arbitrary crowded
  scenes.
- A failure at one or more positions identifies a clearance/approach bottleneck
  relative to that position's already-passing isolated baseline. It does not
  justify weakening collision checks.
- Any Shadow result remains observational and cannot close T30, P2, full P3,
  or P4.
- The held-out real-image test remains sealed.
