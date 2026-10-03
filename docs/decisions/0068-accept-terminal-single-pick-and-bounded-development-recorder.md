# ADR 0068: Accept one terminal perception-controlled pick and bounded development recorder

## Status

Accepted on 2026-08-13.

## Context

The preceding seed-44012 probe reached guarded placement, but its fixed
210-second recorder expired while the action was still active. It could not
prove either success or failure, and the runner could leave Gazebo descendants
behind after its launch parent exited. Wrist confirmation receipts also lacked
the measured sigma, confidence, and correction values needed to diagnose a
safe rejection.

The 1.5-second confirmation timer began immediately after resetting wrist
tracks and required three new stationary observations. Development replays
showed that this could expire before the detector and RGB-D localizer completed
the bounded observation set even when a valid target was present.

## Decision

Install a reusable development recorder with three explicit time bounds:

- 120 seconds to receive the first batch status;
- 180 seconds without a distinct state or action-progress event;
- a non-extendable 900-second hard limit.

Propagate manipulation feedback stages through `/strawberry/harvest_status` so
the recorder distinguishes slow, bounded progress from a stalled action.
Record the actual wrist IDs, correction, confidence, sigma, and corresponding
thresholds for every accepted or rejected confirmation.

Use a 3.0-second stationary wrist-confirmation window. Keep the 15 mm
uncertainty gate, 50 mm correction gate, detector confidence gate, finite view
bank, collision checks, and one-retry policy unchanged.

For process cleanup, treat the PID returned by the asynchronous `setsid`
launch as the process-group ID. Do not query `ps` immediately after spawning:
that race can observe the child before `setsid` and return the runner's own
group. Require the launch group to remain absent across consecutive checks and
emit a separate cleanup receipt.

Keep all formal seeds sealed. These changes and replays are development-only.

## Evidence

Development seed 44012 produced a schema-v2 terminal receipt after 440.1 wall
seconds:

- target 2 passed its first wrist confirmation, but grasp contact or simulated
  attachment failed; its one allowed retry later timed out at wrist
  confirmation and it was skipped;
- the batch continued to target 7;
- target 7 passed wrist confirmation with 6.9 mm correction and 14.67 mm
  reported wrist sigma under the unchanged 15 mm gate;
- the action emitted `PLAN`, `APPROACH`, `GRASP`, `RETREAT`, `PLACE`, `VERIFY`,
  and `DONE`;
- the target was recorded as `HARVESTED`, and the terminal batch outcome was
  `PARTIAL_SUCCESS`.

The runtime receipt SHA-256 is
`0871181698759cd9c351593b8f57f41d033efa0038b9c65670c276aedf1519d0`.
The corresponding launch-log SHA-256 is
`f0a15eb725da4bde19c13871d08474d1d024df3f41d52fa1cc49f12cb14b1d86`.

That successful action run exposed the process-group discovery race and
required manual cleanup, so it is not used to claim cleanup correctness. A
later ready-only ROS/Gazebo smoke exited with code zero, emitted `CLEAN`, and
left zero matching processes. Dynamic tests also terminate both ordinary and
TERM-resistant process groups.

The complete workspace passed 504 tests before the final PGID fix. After the
fix, the tracked build-and-test entry point passed 505 tests with zero errors,
failures, or skips; the change-specific suite passed 31 tests.

## Consequences

The project now has valid evidence for one complete perception-controlled
single-fruit pick and place in a randomized multi-target development scene. It
also demonstrates that one failed target does not abort the batch: another
target can be selected, confirmed, harvested, verified, and reported in the
same terminal run.

This is not evidence that all reachable fruit were harvested, that two fruit
were successfully picked in one batch, or that the formal generalization
metrics pass. The next milestone is a development batch with at least two
successful harvests, followed by repeatability checks on fresh development
seeds. The formal 30-seed matrix remains sealed.

The machine-readable record is `config/generalized_runtime_progress_v5.json`.
