# ADR 0050: Freeze v2 Oracle five-world repeat

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Five fresh-world development repetitions after the v2 single pass

## Context

ADR 0049 preserved the failed v1 evidence and authorized one measurement-only
v2 requalification. The consumed v2 result at
`results/development/blender_v2_oracle_execution_round_trip_v2/summary.json`
passes without violations:

- complete ten-sample initial-state preflight;
- exact seven-stage action success;
- planning/execution time: `0.0343/89.9585 s`;
- raw left/right target contacts: `320/315`;
- confirmed attach/detach round trip;
- no unexpected fruit contact;
- initial ready error: `7.78e-11 rad`;
- initial-to-final home delta: `7.71e-11 rad`;
- gripper reopen error: `1.12e-7 m`; and
- clean shutdown.

One pass establishes feasibility but not fresh-world repeatability.

## Decision

1. Freeze a non-acceptance development gate named
   `blender_v2_oracle_execution_repeat_5_v1`.
2. Run exactly five trials. Every trial starts a fresh canonical Blender-v2
   Gazebo world and fresh ROS domain, captures a new read-only initial-state
   preflight, sends one target-1 ground-truth action, and shuts down before the
   next trial.
3. Keep all v2 motion and measurement inputs unchanged: dual cameras,
   `blender_v2_26mm`, `0.0964 m` tool offset, `0.022/0.040 m` close/open,
   place position `[0.35, -0.45, 0.45] m`, production collision rules, and
   existing controller tolerances.
4. Keep perception, localization, Oracle provider, orchestrator, and Gazebo
   pose control stopped.
5. Evaluate every trial with the same v2 validator. Require all five to pass;
   there is no per-trial retry and no 90-percent rounding for this small
   deterministic sample.
6. Continue the frozen five-trial matrix after an individual failure so the
   fixed sample cannot be selectively truncated. Missing evidence counts as a
   failed trial.
7. Write once into a fresh, non-overwriting directory. No sixth trial,
   alternate target, parameter change, or tolerance relaxation is authorized.

## Boundaries

A 5/5 pass qualifies only Oracle-controlled repeatability at the canonical
pose in simulation. It does not authorize perception-derived execution,
position variation, natural-plant contact robustness, formal acceptance,
held-out-test access, or physical hardware. General `pick_authorized` remains
false.
