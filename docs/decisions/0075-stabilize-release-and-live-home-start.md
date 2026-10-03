# ADR 0075: Stabilize release and verify live home starts

## Status

Accepted as a development runtime milestone. The isolated physical release
gate and recovery-home defect are resolved; a completed multi-fruit visual
batch is still not proved.

## Context

Development seed 44008 had produced two physical grasp-and-place attempts that
failed the unchanged one-simulated-second collection-bin contact requirement.
It also exposed a repeatable recovery-home failure after wrist confirmation:
Panda joint 4 exceeded the controller's 0.50 rad path tolerance by 1--6 mrad.

The release failure was caused by opening the fingers while the fruit remained
rigidly attached to the hand. The later detach released an asymmetric preload.
The home failure was not fixed by merely partitioning the planned path. A fresh
run showed that the preceding wrist action could time out and be cancelled,
while repeated stale planning-scene samples looked stationary. The next plan
could therefore start from a state that did not match fresh `/joint_states`.

## Decision

- Detach the simulated fruit before opening the physical fingers. A detach
  failure stops the release and withholds every later motion.
- Assign successful harvests to a deterministic, bounded nine-slot drop bank.
  Retries retain the same slot; indices outside the bank fail closed.
- Execute named-home paths in controller segments of at most four nominal
  seconds. Validate the complete MoveIt path before segmentation and verify
  each segment endpoint against live joints.
- Track all seven Panda arm joints from `/joint_states`. Settling requires new
  messages rather than repeated reads of one cached planning-scene state.
- Reject any trajectory before controller submission when its planned start
  differs from fresh live joints by more than 0.05 rad. A zero-motion rejection
  may trigger the existing single bounded home replan; execution failures never
  trigger blind retries.
- Keep every collision, reach, maturity, uncertainty and controller tolerance
  unchanged. Gazebo truth remains excluded from runtime decisions.

## Evidence

No hidden formal scenario was opened.

- Isolated development release repetitions `v37`, `v38` and `v39` each
  completed bilateral physical contact, attachment, retreat, detach-before-open
  release and one continuous simulated second of bin-floor contact. Their
  receipt SHA-256 values are respectively
  `330007e80a95f6087213f732d7882f7c54a375344e5e7a36766fbd713a16e78a`,
  `8614aeb79d7f9e43fe7f96b7783c559f61041d2b208dcd02d87b6303df64cdf7`
  and `540f0bbc9101cf0bd76db0b89951b1bcf8eeaa5547fdde91297a8e2c73b84736`.
- Full visual runtime `live_start_gate_v43` passed base selection, dynamic wrist
  observation, wrist/base fusion, connected approach and the complete
  three-segment recovery-home route. It emitted `GLOBAL_SCAN_HOME_REACHED` with
  no path-tolerance violation and cleaned its process group. Runtime receipt
  SHA-256: `93b0438f73dc73fc253789997f72a925768cd691c5d5a8a233e3aca062e9b8d2`.
- The dependency-light suite passes 690 tests with zero failures or errors;
  two environment-specific tests remain skipped by design.

## Consequences

Release geometry is no longer the primary blocker, and a failed pick can return
to global scan without the prior stale-start controller fault. The complete
visual batch still failed safely because the left gripper action result timed
out twice, even though raw physical contact was classified
`BILATERAL_SAME_FRUIT`. Runtime correctly refused to treat contact alone as a
validated close command.

The next task is to make dual-gripper result collection independent rather than
waiting for the left result before observing the already-completed right result.
That change must retain measured joint-position validation and must not convert
physical contact into implicit action success.
