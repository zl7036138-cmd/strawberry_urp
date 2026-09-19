# ADR 0066: Resolve depth-layer near ties without relaxing the runtime gate

## Status

Accepted on 2026-08-12.

## Context

The generalized detector had already passed its independent qualification, but
development seed 44003 formed only one stable ripe runtime track. A read-only
RGB-D frame diagnostic showed that another detected ripe fruit was rejected by
the geometry-layer localizer: a 17-pixel layer was marginally closer to the
expected fruit depth than the real 280-pixel fruit layer, so the old asymmetric
ambiguity test treated the well-supported layer as the runner-up and rejected
the box.

The runtime also needed clearer evidence for safe `NO_PICK` outcomes and parity
between target feasibility checks and the executor's existing bounded retry.

## Decision

Within the configured geometry-error ambiguity margin, rank depth-layer
candidates by support before geometry error. Keep the minimum geometry error
as the implausibility guard, and still reject near ties when the second layer
has substantial support relative to the selected layer. This resolves weak
near-tie artifacts without broadening the accepted depth interval or weakening
the ambiguity threshold.

Expose stable rejection codes for every target-selection gate. A `NO_PICK`
status now records whether each track was excluded, unripe, low confidence,
uncertain, under-observed, stale, too close to an obstacle, or outside the
conservative reach envelope.

Use the same finite two-pose pregrasp set for non-executing feasibility checks
and execution: the primary pregrasp and the existing bounded 90-degree retry.
Both candidates must fail IK or collision checking before the target is
rejected at that stage.

Keep the formal 30-scene matrix sealed. Simulator truth remains available only
to the development diagnostic for offline projection and localization scoring;
it is never published into runtime selection or control.

## Evidence

- Localization tests: 33 passed, including the 17-versus-280-pixel near-tie
  regression and a zero-margin regression.
- Manipulation core tests: 22 passed, including exact primary/retry candidate
  parity.
- Harvest-planning tests: 7 passed, including all structured rejection gates.
- The three changed ROS 2 packages built successfully with `colcon`.
- On development seed 44003, the previously rejected ripe fruit localized at
  4.3 mm scoring error and the online stable ripe-track count increased from
  one to two. Unsafe candidates still ended in `NO_PICK`.
- Development seed 44012 passed runtime selection with 32.5 mm reported
  clearance and reached wrist observation with a 12-pose observation plan.

Seed 44012 did not prove a completed pick. Wrist localization later reported a
large disagreement and MoveIt reported an out-of-bounds start state. The run's
terminal receipt was also invalidated when the host C drive filled, so this
probe is evidence of pipeline reachability only, not task success.

## Consequences

The RGB-D localization recall defect identified on seed 44003 is repaired while
all safety gates remain active and explainable. The next product bottleneck is
wrist re-localization and motion-state validity, followed by a fresh complete
development pick and multi-fruit batch.

The last full regression remains the 483-test clean baseline. The new 62-test
targeted set passed, but the full suite must be rerun after host disk space is
restored. The machine-readable development record is
`config/generalized_runtime_progress_v3.json`.
