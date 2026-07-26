# ADR 0028: Accept the repeated perception development-gate outcome

- Recorded: 2026-07-17
- Status: Accepted
- Scope: Non-formal P3 simple-scene development evidence

## Context

ADR 0027 froze a twenty-trial development gate for the exact ADR-0026 model:
ten isolated ripe scenes and ten isolated unripe scenes, each in a fresh world
and distinct ROS domain. The held-out real-image test and formal simulator
matrix remained outside the run.

## Evidence

`results/p3/perception_repeated_dev_gate_v1/summary.json` records:

- 10/10 positive trials at `DONE/SUCCESS`;
- 10/10 negative trials at `NO_PICK`;
- zero negative false picks and zero negative control attempts;
- at least ten settled detection frames in every negative trial;
- 0.0713023663 s positive planning p95; and
- valid infrastructure, routing, liveness, and clean shutdown for all trials.

The run used ROS domains 180 through 199. Oracle was not started and
`/strawberry/target_pose` was the control source.

## Decision

Accept ADR 0027's repeated simple-scene development gate. The result is
sufficient to proceed to definition of the formal P3 matrix without another
perception-control smoke.

The acceptance remains `ACCEPTED_WITH_WAIVER`: the model's real-validation
macro-F1 is 0.8006746847 rather than the required 0.85, the held-out real test
is sealed, and no formal test score exists.

## Boundaries

This decision does not accept T30 numerically, close formal P3, establish
lighting/occlusion/position robustness, validate unripe classification recall,
or support a real-image or sim-to-real claim. The formal 135-positive plus
30-negative matrix requires its own frozen contract and evidence.
