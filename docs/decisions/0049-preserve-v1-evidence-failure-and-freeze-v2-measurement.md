# ADR 0049: Preserve v1 evidence failure and freeze v2 measurement

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One measurement-only repair after the consumed ADR-0048 gate

## Context

The single ADR-0048 execution was consumed and may not be rerun. Its immutable
trial result records a successful seven-stage action:

- planning/execution time: `0.0403/92.2741 s`;
- raw left/right target contacts: `281/276`;
- processed contact observed on both fingers;
- no unexpected non-finger fruit contact;
- detached-to-attached-to-detached transition confirmed;
- final fruit position:
  `[0.350025718, -0.449980244, 0.450040094] m`;
- final arm state within approximately `6.23e-13 rad` of `ready`; and
- final gripper opening within approximately `1.12e-7 m` of `0.040 m`.

The action itself succeeded, but the gate did not. The client saved its initial
arm snapshot immediately after the target and action server became available,
before it had received a `/joint_states` sample. The initial dictionary was
therefore empty and the initial-to-final delta was `null`. The validator then
attempted `float(null)` and raised `TypeError` instead of emitting a fail-closed
violation. The preserved v1 summary consequently records
`passed=false`, `action_execution_succeeded=true`, and
`evidence_complete=false`.

## Decision

1. Preserve the v1 contract, runner, client, validator, trial JSON, launch log
   fingerprint, and failed summary. Do not rewrite or reinterpret v1 as a
   pass.
2. Authorize one separately named gate,
   `blender_v2_oracle_execution_round_trip_v2`, solely to repair measurement.
3. Before sending the unchanged action, run a read-only preflight probe that
   requires:

   - at least ten complete seven-joint samples;
   - all seven arm joints within `0.02 rad` of `ready`;
   - both gripper joints present; and
   - target 1 explicitly detached.

4. Feed the frozen preflight snapshot into a new validator. Treat missing,
   null, non-finite, or incomplete numeric evidence as a normal violation,
   never an exception.
5. Keep the world, target, target topic, camera mode, place pose, scene-bound
   grasp profile, motion code, controller configuration, timeouts, collision
   rules, and thresholds identical to v1.
6. Execute v2 at most once in a fresh world and non-overwriting directory. It
   is not a retry of a mechanical failure and does not authorize parameter
   changes or a third run.

## Consequences

V1 remains failed because its recovery evidence is incomplete, despite the
successful action. V2 can establish a complete initial-to-final recovery
measurement without changing the manipulation behavior. Repeated execution
remains unauthorized until v2 passes and a separate repeat contract is
accepted. Perception, formal acceptance, held-out-test access, and general
`pick_authorized` remain false.
