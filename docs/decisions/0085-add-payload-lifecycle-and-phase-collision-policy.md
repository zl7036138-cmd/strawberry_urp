# ADR 0085: Separate payload lifecycle from collision semantics

## Status

Accepted as a code-structure and fail-closed safety change. Pure state-machine,
collision-policy, asset-contract and backend-unit tests pass; no new Gazebo
behavior result is claimed by this decision.

## Context

The earlier executor represented attachment with one local boolean. Its generic
failure handler detached the fruit, opened the gripper and attempted recovery
motion even after a successful grasp. A post-grasp transport or place-plan
failure could therefore convert into an uncontrolled release outside the bin.
At the same time the planner treated all collision objects as anonymous boxes,
while the selected fruit already had an implicit one-off contact exception.

That mixes three distinct policies: physical payload ownership, collision
semantics, and whether a recovery trajectory is authorized.

## Decision

- Add the pure `payload_lifecycle` module with the explicit states
  `EMPTY → CONTACT → HOLDING → ESCAPED → AT_BIN → RELEASED`.
- Keep `PickAndPlace.action` wire-compatible. Feedback emits the lifecycle
  stages; the existing `recovery_disposition=MOTION_WITHHELD` and result message
  expose a retained payload to the orchestrator without changing the action
  schema.
- After successful attach, ordinary failure no longer calls detach, open or
  home. The carried-body collision model remains installed, the target-collision
  restoration is skipped, and a process-lifetime payload interlock rejects any
  later pick goal until an explicit external intervention restarts in a known
  empty state.
- Detach remains legal only after the arm reaches `AT_BIN`; after detachment,
  release/verification failures retain the existing bounded recorded-route
  recovery because the fruit is no longer held.
- Change escape to the reverse local tool axis of the active grasp pose. This
  is equivalent to the prior base-Z retreat for the current top-down grasp but
  stays correct for future bounded orientation candidates.
- Add a dependency-free collision-policy module. Tables, planters, bin walls,
  field ground/ridges, self-collision and non-selected fruit remain hard. The
  selected fruit is a task object only during `GRASP_CONTACT`. The future
  `SOFT` plant category has no permitted phase until a bounded contact model
  and independent telemetry exist, so no present collision constraint is
  relaxed.

## Verification

- 40 manipulation-core tests pass, including retained-payload, interlock,
  lifecycle-feedback, release and escape-direction cases.
- 9 collision-policy/scene-geometry tests pass.
- 117 interface-contract, manipulation asset and backend tests pass.

The v9 two-fruit run in ADR 0084 was executed before this change. Its success
does not validate the new retained-payload behavior and is preserved as prior
evidence only. The next runtime test must use a fresh development run and must
record either a normal release path or a `PAYLOAD_HELD_MOTION_WITHHELD` outcome.

## Consequences

This closes the semantic gap where a planning failure could drop a correctly
grasped fruit. It deliberately makes an unresolved carrying failure more
conservative: the batch terminates in a visible held state rather than trying a
long unreviewed motion. The next architectural step is whole-chain evaluation
before attachment, so the new interlock is a rare safety backstop rather than a
normal planning outcome.
