# ADR 0087: Add bounded grasp-candidate search

## Status

Accepted and runtime-qualified for the bounded controlled-challenge scope.
Stages A (pure candidate geometry), B (bounded first-feasible search), C
(certificate-to-execution identity binding), D (runtime plan-only
qualification), and E (controlled non-nominal physical execution) are
implemented. Stage E now has a clean-commit-bound positive receipt in which
`G00`–`G02` fail, `G03` is certified and executes the complete grasp, transport,
release and return chain. Multi-fruit and formal 30-seed acceptance remain
outside this ADR.

## Context

ADR 0086 can certify a nominal grasp through pregrasp, approach, virtual
attachment, escape, transport and bin approach.  A target can still be safe
and reachable through a nearby approach direction when its one nominal grasp
is not.  Manually changing an approach angle is not reproducible or suitable
for autonomous selection.

## Decision

- Generate a finite, deterministic set of fifteen local approach-axis tilts
  around the nominal orientation. The order begins with nominal `G00`; after
  the 10-degree terminal-grasp ring repeatedly produced mirrored single-finger
  contact in physical simulation, the evidence-qualified v2 geometry uses a
  5-degree first ring and a 10-degree outer ring. Geometry fingerprints make
  every certificate from the superseded 10/20-degree ring non-reusable.
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
  action server. It is not the ADR 0087 path.
- Stage D exposes `/strawberry/qualify_grasp_candidates`: it prepares the
  target collision lifecycle, generates `G00`–`G14` from the perception pose,
  evaluates them in copied PlanningScenes, emits an ordered trace, and then
  restores the target collision object. The service is explicitly plan-only:
  it never invokes the executor, does not retain an executable plan after
  cleanup, and returns its certificate only as an audit artifact.
- Stage E must re-qualify inside one live collision lifecycle and pass the
  exact certified identity to `execute_authorized()` immediately before a
  non-nominal execution. A plan-only certificate is intentionally not valid
  for later execution after its target-collision cleanup.
- Stage E freezes one perception-derived collision manifest for the bounded
  action. Lease acquisition may wait at most three seconds for a message that
  satisfies the unchanged 0.75-second tracker freshness contract; it issues
  no command while waiting and fails closed when no fresh scene arrives.
- A certified physical grasp that returns `EMPTY` and safely reaches home may
  consume exactly one contact-driven reauthorization. A unique single-fruit
  contact supplies stronger directional evidence than an untried
  orientation, so the coordinator evaluates the bounded full-to-partial
  contact-centering translation ladder while preserving the certified source
  orientation. Every correction receives a fresh ADR 0086 certificate; no
  third physical attempt is permitted.

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

Stage D tests prove that plan-only qualification fails closed for unstable
perception, unavailable/changed scenes, non-empty payloads, and incomplete
cleanup. They also validate the runtime receipt sequence, frozen candidate
order, candidate fingerprints, scene isolation, collision restoration, and
the absence of command/gripper/attachment/contact/holding evidence. The
development receipt in `docs/p0/ADR0087_PLAN_ONLY_EVIDENCE_2026-10-01.md`
records an actual perception-derived `G00` certificate with no dispatched
execution.

Stage E required a runtime case where nominal fails but a non-nominal
candidate completes the newly certified execution chain. The bound positive
receipt is recorded in
`docs/p0/ADR0087E_CONTROLLED_CHALLENGE_POSITIVE_2026-10-02.md`.

The first controlled Stage E challenge is retained as a negative result in
`docs/p0/ADR0087E_CONTROLLED_CHALLENGE_NEGATIVE_2026-10-02.md`. It proves
scene isolation, exact non-nominal dispatch, safe empty-payload recovery and
bounded follow-on rejection, but it does not satisfy the positive completion
criterion above. Follow-up mirrored challenges show that the current
10-degree terminal grasp tilts produce repeatable single-finger contact in
this plant geometry. The next Stage E iteration must therefore qualify a
smaller tilt ring or separate the obstacle-avoiding approach direction from
the final bilateral closure orientation; safety thresholds and retry limits
remain unchanged. The qualified 5-degree ring subsequently completed `G03`
without contact reauthorization, closing Stage E for its controlled scope.
