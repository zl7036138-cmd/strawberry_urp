# ADR 0087: Add bounded grasp-candidate search

## Status

Accepted for staged implementation.  Stage A (pure candidate geometry) is
implemented and covered by pure tests.  Search, certificate/execution identity
binding, and runtime qualification remain open.

## Context

ADR 0086 can certify a nominal grasp through pregrasp, approach, virtual
attachment, escape, transport and bin approach.  A target can still be safe
and reachable through a nearby approach direction when its one nominal grasp
is not.  Manually changing an approach angle is not reproducible or suitable
for autonomous selection.

## Decision

- Generate a finite, deterministic set of fifteen local approach-axis tilts
  around the nominal orientation.  The frozen order begins with nominal `G00`.
- Tilts use local X/Y rotations only; first version does not use finger-roll
  (local Z/yaw), random sampling, scoring, or learned grasp generation.
- Every `GraspCandidate` is immutable and owns its `grasp_pose`,
  `pregrasp_pose`, `escape_pose`, tilt values, stable ID, and geometry
  fingerprint.  Pregrasp and escape both follow that candidate's own local
  tool Z axis.
- A later coordinator will evaluate candidates in order through ADR 0086 and
  select the first feasible candidate.  It must bind certificate identity to
  execution identity: the certified candidate geometry is the only geometry
  eligible for execution.
- Candidate-set total budget and per-candidate budget will be added with the
  search coordinator.  ADR 0086's per-candidate safety semantics remain
  unchanged.

## Non-goals

This ADR does not alter `whole_chain.py`, collision policy, vegetation contact,
MoveIt Task Constructor, Servo, candidate scoring, manipulability ranking,
random sampling, or the public `PickAndPlace.action` contract.

## Verification

Stage A pure tests prove fixed count/order, nominal-first behavior, unique IDs
and geometry fingerprints, normalized orientations, fail-closed input checks,
and candidate-local pregrasp/escape geometry.

Future stages must prove first-feasible early exit, total-budget behavior,
certificate/execution identity equality, plan-only runtime enumeration, and a
runtime case where nominal fails but a non-nominal candidate completes the
certified execution chain.
