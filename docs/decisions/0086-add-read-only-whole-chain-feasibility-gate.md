# ADR 0086: Add a read-only whole-chain feasibility gate

## Status

Accepted as a code-structure and fail-closed authorization change.  It has
pure/unit and backend-source verification only; it has **not** yet established
a new Gazebo runtime success or safety result.

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

## Consequences

The action server now has an explicit `AUTHORIZE_PICK` boundary between target
selection and any gripper closure.  A fresh field-v3 positive run and a
deliberately blocked-bin negative run are still required to validate the
MoveItPy copied-scene adapter in the actual Gazebo process.  Servo, soft
vegetation contact, MTC migration and multi-candidate scoring remain outside
this ADR.
