# ADR 0078: Audit and seal the generalized multi-fruit development gate

## Status

Accepted as a pre-qualification implementation milestone. Formal 30-seed
claims remain sealed and no qualification seed has been consumed by this
decision.

## Context

The generalized runtime had proved one complete perception-controlled pick and
safe recovery, but it had not proved two harvested targets in one batch. The
existing selector mixed asynchronous MoveIt callbacks with ROS node state,
development receipts did not preserve every rejection event, and formal result
validation did not bind every materialized input strongly enough for a
one-attempt hidden evaluation.

The next runtime experiment must not choose favorable scenes using motion, must
not allow simulator truth into control, and must not consume the formal matrix
while the five-scene development gate is still closed.

## Decision

- Disable localization truth association by default and create no truth
  subscriptions when it is disabled. Audit the live no-motion ROS graph and
  allow truth subscribers only at explicit simulation/scoring boundaries.
- Extract asynchronous MoveIt candidate evaluation into a pure coordinator
  that rejects stale callbacks, invalidates drifted poses and caches, and
  distinguishes a temporarily busy backend from an infeasible path.
- Record development evidence as schema v3, including every selection event.
  Publish simulation contact/placement truth only as scoring events; they are
  unavailable to runtime control and are associated with selected tracks after
  the run.
- Add a no-motion feasibility probe and 18-seed sweep. A scene is eligible only
  when at least two stable ripe tracked targets pass the existing MoveIt path
  evaluator. Do not execute arm trajectories during selection.
- Freeze discovery seeds at 45001–45018. Start qualification with untouched
  seeds 46001–46018 on a clean Git commit, selecting the first five eligible
  scenes in matrix order. If fewer than five exist, stop without lowering a
  threshold or executing a behavior run.
- Require immutable five-run receipts and exact gate aggregation. Any behavior
  change invalidates a qualification batch and advances to the next hundred
  seed block.
- Upgrade formal acceptance to strict schema v2 with hashes for the matrix,
  materialized scenes/worlds, model, runtime configuration, and Git commit.
  Require an atomic non-overwriting claim, one behavior attempt per scene,
  clean shutdown, and zero unsafe motion attempts. Do not create a formal claim
  before the development gate passes.

## Evidence

The dependency-light suite passes 735 tests with zero failures or errors; two
environment-specific tests remain skipped. The full ROS 2/colcon suite passes
617 tests with zero failures, errors, or skips. A live no-motion launch passes
the truth-isolation graph audit and exits cleanly.

Two discovery sweeps over seeds 45001–45018 both completed 18/18 cleanups and
18/18 truth-isolation audits. Of 29 evaluated candidate tracks, seven were
MoveIt feasible, and only seed 45007 contained at least two stable ripe
feasible tracks. Making pre-grasp and grasp orientation branches consistent is
a correctness improvement but did not change this 1/18 eligibility result.

## Consequences

Truth isolation, asynchronous safety selection, evidence schemas, no-motion
screening, qualification execution, and formal claim validation now have
testable fail-closed contracts. The remaining result is empirical: the unused
qualification block must either produce five eligible scenes or stop with an
insufficient-eligibility receipt. No evidence in this ADR proves same-batch
multi-fruit success or formal generalization.
