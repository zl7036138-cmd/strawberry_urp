# ADR 0077: Preflight MoveIt before publishing a target

## Status

Accepted as a development runtime milestone. The selector now rejects a
geometrically plausible but MoveIt-infeasible target before the continuous
harvest state machine consumes it. Multi-fruit success in one batch and the
formal generalization gate are not yet proved.

## Context

Development seed 44008 exposes two stable ripe tracks. In milestone v59, the
selector ranked track 1 first from maturity, confidence, uncertainty, clearance
and a coarse Cartesian reach box. The orchestrator then discovered that every
bounded pre-grasp IK/collision candidate failed, retried once, returned home,
and skipped track 1 before harvesting track 2. That behavior was safe, but it
spent a complete failure/recovery cycle on a target whose infeasibility was
already knowable without moving the arm.

The `/strawberry/evaluate_target` service already checks a connected route from
the live joint state through pre-grasp, guarded approach, grasp and retreat. It
does not execute a trajectory. Keeping that result outside selection meant the
selector's published ordering did not satisfy the intended path-feasibility
priority.

## Decision

- Call `/strawberry/evaluate_target` in the selector before publishing
  `TARGET_SELECTED`.
- Preserve the existing fail-closed maturity, 15 mm uncertainty, freshness,
  observation-count, clearance and conservative-workspace gates.
- Evaluate candidates by deterministic clearance, uncertainty and confidence
  groups. Within the first group containing feasible candidates, use MoveIt's
  measured joint travel and then stable track ID as final tie-breaks.
- Cache a path result only while the tracked fruit remains within 5 mm of the
  evaluated pose. Invalidate every cached current-state result when any target
  is completed or skipped.
- Ignore callback results from an older selection epoch after a completion
  tombstone or restart.
- Treat a busy/unavailable backend and collision-scene synchronization failures
  as deferred evaluation, not as evidence that a fruit is unreachable.
- Continue publishing a preflighted identity from fresh tracking frames until
  its completion tombstone. This lets a later-started batch consume the target
  without issuing MoveIt requests during active arm motion.
- Keep the orchestrator's execution-boundary feasibility check and final
  post-wrist-refinement check as independent safety revalidation.
- Keep the formal 30-seed matrix sealed.

## Evidence

The dependency-light suite passes 702 tests with zero failures or errors; two
environment-specific tests remain skipped by design.

The full truth-free development runtime
`generalized_harvest_seed_44008_moveit_preselection_v60` first evaluated tracks
1 and 2 without motion. The orchestrator's first and only selected identity was
track 2; track 1 never entered the retry/skip state machine. Track 2 used the
fourth of six dynamic wrist views, accepted an 8.63 mm raw visual correction
(0.83 mm after fusion, 9.46 mm fused sigma), established physical bilateral
contact with anonymous simulation entity 4, and completed retreat, release,
verification and home. The terminal receipt is `SUCCESS`, `harvested=[2]`, no
skipped targets, no failures, elapsed 245.06 seconds, and clean shutdown.

After the fresh post-pick scan, track 1 was reported as
`MOVEIT_PATH_INFEASIBLE` with `NO_MOVEIT_FEASIBLE_TARGET`, so the system ended
without relaxing a safety threshold. Compared with v59, the same seed avoided
the redundant track-1 retry/skip cycle and reduced elapsed time by about 65.07
seconds. Runtime receipt SHA-256:
`8bdf12d3ef78e7047a094a116c550e366b2dc395485ff4cfa837359e6b87ae4b`.
Launch log SHA-256:
`3b0c9dc52898b32c243548f43443ff697e65f725effb8b84f4b993ad96276369`.
Cleanup receipt SHA-256:
`b0d5e4fc5648d375c5beb561bf6bb33fc7b0b7cd3f731f5ab4f97e7dd063bfed`.

## Consequences

The target selector now implements the required safety ordering with real
MoveIt path feasibility before hand-off. A known-infeasible high-quality target
no longer consumes the batch's one retry, and backend contention is not cached
as a false unreachable result.

This evidence is still one development seed with one physically harvested
fruit. The next primary requirement is to generate a development scene with at
least two genuinely feasible mature targets and prove two `TARGET_HARVESTED`
outcomes in one continuous batch before expanding to several untrained
development seeds.
