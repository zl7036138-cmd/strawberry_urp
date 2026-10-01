# ADR 0087: Add bounded grasp-candidate search

## Status

Accepted for staged implementation. Stages A (pure candidate geometry), B
(bounded first-feasible search), and C (certificate-to-execution identity
binding) are implemented and covered by pure/executor-contract tests. Runtime
plan-only qualification and a real non-nominal execution remain open.

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
  fingerprint. The fingerprint is verified against all candidate geometry at
  construction time. Pregrasp and escape both follow that candidate's own
  local tool Z axis.
- The pure search coordinator evaluates candidates in order through ADR 0086
  and selects the first feasible candidate.  It retains one immutable trace
  row per evaluated candidate, including ID, geometry fingerprint, original
  ADR 0086 result and allocated per-candidate budget.
- The coordinator enforces both candidate-set total budget and per-candidate
  budget without changing ADR 0086's per-candidate safety semantics.
- `AuthorizedGraspPlan` binds a positive target ID, the selected immutable
  candidate, a feasible ADR 0086 result, scene signature, certificate time,
  and a self-verifying certificate fingerprint.
- `PickAndPlaceExecutor.execute_authorized()` requires a complete
  `ExecutionIdentity` (target ID, candidate ID, geometry fingerprint, and
  scene signature) equal to that certificate. Missing or mismatched evidence
  fails before `prepare_pick`, planner, gripper, or attachment calls.
- An authorized execution uses the certificate's exact `pregrasp_pose`,
  `grasp_pose`, and `escape_pose`. It forbids target-pose refinement and all
  historical reorientation, centering, or alternate-grasp retries that could
  change certified geometry.
- The existing nominal `execute()` path remains temporarily for the public
  action server. It is not the ADR 0087 path; stage D must connect runtime
  candidate search and scene-signature capture to `execute_authorized()`.

## Non-goals

This ADR does not alter `whole_chain.py`, collision policy, vegetation contact,
MoveIt Task Constructor, Servo, candidate scoring, manipulability ranking,
random sampling, or the public `PickAndPlace.action` contract.

## Verification

Stage A pure tests prove fixed count/order, nominal-first behavior, unique IDs
and geometry fingerprints, normalized orientations, fail-closed input checks,
and candidate-local pregrasp/escape geometry.  Stage B tests prove nominal
early exit, first-later-feasible selection, full rejection traces, bounded
total/per-candidate budget behavior, invalid-set fail-closed behavior, and
unmodified candidate geometry entering the ADR 0086 request adapter.

Stage C tests prove that a feasible certificate creates execution authority,
that candidate/geometry/scene/target mismatches and missing authority cause
zero backend commands, that certificate tampering is rejected, and that the
executor sends the exact certified pregrasp/grasp/escape geometry without
falling back to nominal or alternate geometry.

Future stages must prove plan-only runtime enumeration and a runtime case where
nominal fails but a non-nominal candidate completes the certified execution
chain.
