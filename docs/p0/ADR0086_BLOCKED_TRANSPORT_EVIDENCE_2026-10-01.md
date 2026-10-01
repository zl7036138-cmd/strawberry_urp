# ADR 0086 blocked-transport evidence — 2026-10-01

## Scope

Development-only negative qualification for ADR 0086.  A default-disabled
test switch adds `development_virtual_blocked_bin` only to each copied
PlanningScene transport branch.  It does not alter Gazebo, the live MoveIt
scene, the executed trajectory, perception inputs, or safety thresholds.

## Immutable receipt

- Scenario: `generalized_seed_045504` (development seed `45504`)
- Run: `adr0086_blocked_bin_v2`
- Receipt: `.codex_tmp/generalized_harvest_seed_45504_adr0086_blocked_bin_v2/runtime_probe.json`
- Receipt SHA-256: `6D7F088E9942661FA68580FEF1CACF30236429D557673CB7D7A9D224112FEFC0`

## Qualification result

For targets 2 and 3, the automated qualification checker passed with:

```text
expected_result = TRANSPORT_FAILED
expect_motion = false
require_target_collision_restore = true
```

For each target the receipt orders the relevant evidence as:

```text
WHOLE_CHAIN_EVALUATION_STARTED
  < WHOLE_CHAIN_EVALUATION_RESULT(TRANSPORT_FAILED)
  < TARGET_COLLISION_RESTORED(same target)
```

The result lists successful `PREGRASP`, `APPROACH`, `GRASP_STATE`,
`VIRTUAL_ATTACH`, and `ESCAPE` stages, then rejects `TRANSPORT`.  During each
evaluation the real payload was `EMPTY`, all controller/gripper/physical-attach
counts were zero, and the live collision-scene fingerprint was unchanged.
There are no gripper or physical-attach evidence events in the receipt.

## Recovery interpretation

The rejected Pick itself issued no grasp, approach, transport, close-gripper,
or attach operation.  Later recovery-home motion is separately authorized
while payload remains empty.  In this v2 receipt it reached
`RECOVERY_HOME_REACHED`; a prior v1 run had `RECOVERY_HOME_FAILED`.  That is
recorded as an empty-payload recovery reliability issue, not as an ADR 0086
authorization failure.

## Boundary

ADR 0086 is runtime-qualified for both its positive and intentionally blocked
transport paths.  It does not add alternate grasp generation; that remains the
separate ADR 0087 responsibility.
