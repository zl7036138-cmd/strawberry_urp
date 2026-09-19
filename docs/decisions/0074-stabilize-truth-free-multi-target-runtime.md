# ADR 0074: Stabilize the truth-free multi-target runtime

## Status

Accepted as a development runtime milestone. Multi-target scheduling and safe
failure isolation are runtime-proved; multi-fruit harvest success is not.

## Context

The first complete truth-free single-fruit run exposed several transition
faults that did not appear in the fixed-scene baseline. Base-camera tracks can
change while the arm occludes the plant, a safe next target can be published
just before the batch state changes back to scanning, and observation IK can
finish only milliradians from a Panda hard joint limit. Recovery-home motions
also occasionally exceeded the simulator controller's 0.5 rad path tolerance
by 1--5 mrad.

Development seed 44008 later processed two ripe targets in one batch. Both
were physically grasped, but neither maintained the required one simulated
second of collection-bin contact after release. The earlier five-second
client deadline also expired before the simulator's twelve-second verification
service could return its physical verdict.

## Decision

- Freeze the complete visual fruit obstacle inventory immediately before the
  first eye-in-hand motion for a target. The snapshot is usable only for that
  target, for at most 120 wall seconds, and only while the fused target has
  moved no more than 50 mm from the visual snapshot. A different target must
  acquire a new live visual scene. No simulator truth enters planning.
- Publish scan diagnostics at harvest, skip and terminal transitions. Include
  all visible tracks, maturity, uncertainty, confidence, observation count,
  the latest selector decision, and wall ages for both diagnostic snapshots.
- When a completed identity reaches the selector, re-run deterministic
  selection once over its cached tracking message. The normal 0.5 s age,
  maturity, uncertainty, reach and clearance gates remain authoritative.
- Reject every seven-joint path whose waypoint enters a 0.01 rad soft margin
  around the Panda hard limits. An unsafe observation candidate is skipped
  before motion and the bounded dynamic view bank continues.
- Use 0.10 rad/s for named-home trajectories while normal manipulation keeps
  its 0.30 rad/s bound. Retry a home plan once only when the first failure
  executed no motion. Never retry automatically after controller execution
  began.
- After an unattached grasp-contact failure, open the gripper and reverse to
  the last collision-checked pre-grasp pose before requesting home. If that
  retreat fails, withhold the long home motion.
- Give collection-bin verification its own 15 s client deadline. The simulator
  still decides success from one continuous simulated second of physical bin
  contact; no coordinate or timeout shortcut is accepted.

## Evidence

All runs below are development-only seed 44008. No hidden formal scenario was
opened.

- `zero_motion_home_replan_v31` completed one truth-free physical grasp,
  attachment, retreat, release, bin verification and final home. It also
  showed a second ripe selected track after completion, but the old batch
  hand-off missed that publication. Runtime receipt SHA-256:
  `f7e69605701ffdd10d1740f4b13178c2527ba730325d77c5440fe9059af97b42`.
- `slower_home_v35` processed target IDs 1 and 2 in one batch, performed safe
  recovery-home between failures, and terminated normally after excluding
  both. This proves multi-target dispatch and failure isolation, not harvest
  success. Runtime receipt SHA-256:
  `426db8bcd1d57ad9c09d954c26cd3a52ce20f59311805b836e64c37dace3decf`.
- `bin_verify_timeout_v36` again processed both ripe targets. Each passed
  dynamic observation, wrist confirmation, grasp and physical contact-resolved
  attachment. Both then received the simulator's explicit `no stable physical
  collection-bin contact before timeout` verdict. The unripe target was not
  selected, recovery-home succeeded, and cleanup was `CLEAN`. Runtime receipt
  SHA-256:
  `0dc3abc7c5e473d6d13bfa1fdfd7398a0c1a63121ea9455402b729c06dc49e51`.

The dependency-light regression suite passes 683 tests with zero failures or
errors; two environment-specific tests are skipped by design. Machine-readable
evidence is recorded in `config/generalized_runtime_progress_v11.json`.

## Consequences

The generalized runtime no longer depends on a single target publication or a
hard-limit observation pose to make progress. It can process multiple visual
identities, isolate one target's failure, return safely to the global camera,
and continue with another mature target.

The next priority is now the release geometry and collection-bin contact
contract. It must be solved without deleting newly observed collision objects,
using ground-truth coordinates for control, or weakening the one-second
physical stability requirement. A development batch with at least two
`TARGET_HARVESTED` outcomes is still required before opening the formal matrix.
