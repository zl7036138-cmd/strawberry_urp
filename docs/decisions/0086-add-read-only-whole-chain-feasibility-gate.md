# ADR 0086: Add a read-only whole-chain feasibility gate

## Status

Accepted and runtime-qualified for both positive and deliberately
blocked-transport paths.

## Context

ADR 0085 prevents a held strawberry from being released automatically after a
post-grasp planning failure.  That is a safety backstop, not a way to avoid
discovering an impossible escape or bin route only after closure.

The old feasibility service could prove a route to pregrasp, grasp, and a
short retreat.  It could not prove that the attached fruit could leave the
plant, cross the field, and enter the collection bin.

## Decision

- Add `whole_chain.py`, a ROS-free coordinator with stable results:
  `FEASIBLE`, `PREGRASP_FAILED`, `APPROACH_FAILED`,
  `GRASP_STATE_INVALID`, `ESCAPE_FAILED`, `TRANSPORT_FAILED`,
  `BIN_APPROACH_FAILED`, `SCENE_INVALID`, and `TIME_BUDGET_EXCEEDED`.
- The evaluator receives one existing nominal grasp.  It does not generate or
  score alternative grasps; bounded candidate generation remains ADR 0087.
- Its adapter creates a `copy.deepcopy()` snapshot of the MoveIt PlanningScene
  while holding only `read_only()`.  Target contact is opened, the target is
  converted to an attached collision sphere, and all post-grasp checks happen
  only in that copied scene.  No live monitor write lock, controller action,
  gripper command, simulation attach service, or payload-lifecycle transition
  is permitted in evaluation.
- Each successful stage propagates the prior terminal robot state and virtual
  scene to the next stage.  The certificate is an existence check, not an
  executable trajectory: execution must still validate current state and plan
  its next segment at every normal stage.
- `PickAndPlaceExecutor` accepts an authorization callback.  The ROS action
  server supplies it before the first `open_gripper()` command.  A rejection
  restores the prepared target collision object and returns with
  `payload_state=EMPTY`; it cannot enter contact or close a gripper.
- The `PickAndPlace.action` wire contract remains unchanged.

## Verification

- Pure tests cover every stable stage rejection, terminal-state propagation,
  time-budget fail-closed behavior, and the zero-command virtual-attach
  contract.
- Core test verifies a `TRANSPORT_FAILED` certificate occurs after target
  preparation but before any motion, gripper or attach call, and restores the
  target collision object.
- Existing core and backend unit tests continue to pass.
- Development runtime evidence `adr0086_positive_v3` (seed 45504, 2026-09-28)
  recorded two independent `FEASIBLE` certificates.  Both include all seven
  stages, `payload_state=EMPTY`, zero controller/gripper/physical-attach
  commands during evaluation, and an unchanged collision-scene fingerprint.
  Both certificates preceded their first controller command and both targets
  subsequently reached the normal `TARGET_HARVESTED` terminal path.  The
  immutable receipt hash and the bounded claim are recorded in
  `docs/p0/ADR0086_RUNTIME_EVIDENCE_2026-09-28.md`.
- Development negative evidence `adr0086_blocked_bin_v2` (seed 45504) adds a
  virtual-only obstruction to the copied scene's transport stage.  Two
  independent targets reached `TRANSPORT_FAILED` only after PREGRASP,
  APPROACH, GRASP_STATE, VIRTUAL_ATTACH and ESCAPE succeeded.  Each rejection
  remained EMPTY, changed neither live collision scene fingerprint, issued no
  gripper/attach evidence, and was followed by an explicit same-target
  `TARGET_COLLISION_RESTORED` event.  The bounded receipt hash and the
  qualification result are recorded in
  `docs/p0/ADR0086_BLOCKED_TRANSPORT_EVIDENCE_2026-10-01.md`.

## Consequences

The action server now has an explicit `AUTHORIZE_PICK` boundary between target
selection and any gripper closure.  Empty-payload recovery-home reliability
remains a separate technical-debt item: it does not alter the authorization
claim because a rejected Pick has not entered CONTACT or HOLDING.  Servo, soft
vegetation contact, MTC migration and multi-candidate scoring remain outside
this ADR.
