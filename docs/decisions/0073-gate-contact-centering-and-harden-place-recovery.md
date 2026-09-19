# ADR 0073: Gate contact centering and harden place recovery

## Status

Accepted as a development safety and recovery milestone. Contact-directed
centering and the extended floor-contact window still require a successful
runtime requalification before continuous harvest or the formal 30-seed
matrix can be accepted.

## Context

After ADR 0072 removed runtime truth identity, development seed 44008 reached
the physical grasp repeatedly. A first correction estimated both direction
and distance from the difference between the two final finger joint positions.
That reduced one asymmetry, but it could not distinguish a finger stopped by a
fruit from one stopped by foliage or the gripper mechanism.

A later probe also completed a truth-free bilateral attachment, retreat,
transport, release and detach. It failed the strict collection-bin floor
contact check. The generalized GPU pipeline was running at roughly one third
real time, so the 3 s wall timeout covered little more than the required 1 s
of simulated stable contact and left almost no interval for the fruit to fall.

Finally, a failed action could already have returned the arm to the frozen
`ready` posture before the batch orchestrator requested global-scan home. In
that state MoveIt can return a stationary plan. The backend treated every plan
with fewer than two waypoints as failure even when live joints independently
confirmed that the robot was already home.

## Decision

- Add an identity-free simulator service that classifies fresh raw pad contact
  as no fruit, unique left-side fruit, unique right-side fruit, the same fruit
  on both sides, or ambiguous fruit contact. It exposes no target ID, pose,
  maturity, scene truth coordinate or tracker association.
- In the generalized runtime, allow one centering retry only for a unique
  single-sided fruit contact. The contact pad selects the local-Y direction;
  the absolute finger-position difference selects the correction magnitude.
  Corrections below 1 mm are considered unreliable and corrections above
  10 mm fail closed. No-fruit, bilateral, ambiguous and unavailable contact
  classes do not move the arm.
- Preserve the bounded orthogonal contact retry only for the historical fixed
  scene backend, where the anonymous physical contact service is not enabled.
- Keep the generalized success condition unchanged: the released fruit must
  maintain fresh collection-bin floor contact for 1 s of simulated time.
  Increase only its wall-clock service deadline from 3 s to 12 s so slower
  simulation has time for falling and stability accumulation.
- A stationary named-home plan succeeds only when all seven live arm joints
  are within 0.03 rad of the frozen `ready` joint values. A stationary plan
  without that independent confirmation still fails closed.

## Evidence

All runtime evidence is development-only and uses seed 44008. No hidden formal
scenario was opened.

- The `contact_class_v23b` probe obtained fresh bilateral physical contact,
  attached a gravity-enabled ripe fruit without runtime truth association,
  completed retreat and guarded transport, opened and detached above the bin,
  then timed out on physical floor contact. This proves that the bottleneck
  had moved beyond grasp attachment, but does not prove a completed placement.
- The `bin_timeout_home_v25` probe observed
  `LEFT_SINGLE_FRUIT` while the raw signed half-difference was -6.533 mm. This
  falsified using joint-difference sign as the centering direction and supports
  the pad-directed rule. That run also logged a stationary home plan accepted
  only after live seven-joint confirmation.
- The final controlled `contact_directed_centering_v26c` probe did not reach
  grasp because final connected feasibility and a later tracked-scene sync
  failed. It skipped safely, returned to global-scan home and terminated with
  a stable process-group cleanup. Therefore it validates recovery, not the new
  centering motion or extended placement deadline.

The Linux/WSL dependency-light regression suite passes 668 tests with zero
errors and failures; two environment-specific tests are skipped by design.
Machine-readable evidence is recorded in
`config/generalized_runtime_progress_v10.json`.

## Consequences

The generalized gripper no longer converts unexplained joint asymmetry into an
ungated arm displacement. Recovery-home is idempotent but still independently
measured, and the physical placement criterion is not weakened to a coordinate
or timeout-based success.

The next required runtime evidence is a seed that reaches either single-sided
fruit contact or bilateral attachment: it must demonstrate the pad-directed
centering move when needed, then complete gravity release and 1 s physical
floor contact within the extended wall deadline. Multi-fruit and multi-seed
acceptance remains deferred.
