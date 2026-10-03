# ADR 0052: Accept v2 Oracle execution repeat

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Close the canonical-pose Oracle execution repeat subgate

## Context

ADR 0048 exposed an evidence-capture defect after a mechanically successful
single action. ADR 0049 preserved that failed v1 evidence and authorized a
measurement-only v2 run, which passed without violations. ADR 0050 then froze
five fresh-world repetitions. Its first runner invocation was stopped before
any world or action because ROS CLI self-nodes were mistaken for occupied
domains; ADR 0051 preserved the zero-trial infrastructure failure and repaired
only that domain filter.

The consumed repeat-v2 result at
`results/development/blender_v2_oracle_execution_repeat_5_v2/summary.json`
has SHA-256
`14e352d3195dc8878daa93c4ff8d006343d79fb4f623af30f79198a5afc8765f`
and passes `5/5` with zero violations:

- planning time range: `0.0357-0.0777 s`;
- execution time range: `89.9871-93.3766 s`;
- raw left/right target contacts per trial:
  `286-327` / `281-314`;
- processed bilateral contact in every trial;
- attach/detach round trip in every trial;
- no unexpected fruit contact;
- maximum initial-to-final home delta: `9.52e-11 rad`;
- maximum gripper reopen error: `1.12e-7 m`; and
- clean shutdown in every trial.

Every trial used a fresh world and ROS domain, exactly one target-1 action,
the canonical dual-camera scene, and the same `blender_v2_26mm` profile.
Perception, localization, Oracle provider, orchestrator, and pose control
remained stopped.

## Decision

1. Accept the repeat-v2 result as the canonical-pose, Oracle-controlled
   Blender-v2 execution repeat qualification.
2. Treat the final approach, bilateral close/contact, guarded attachment,
   attached retreat, bin placement, detach, target verification, gripper
   reopen, final home, collision restoration, and clean shutdown path as
   qualified for this exact simulation scene and target pose.
3. Keep the new final-home return check in the production executor. A failed
   home motion is a failed action and is never automatically retried.
4. Preserve the v1 single evidence failure and repeat-v1 infrastructure
   failure; neither is rewritten as a pass.
5. Require a separately frozen gate before perception-derived execution,
   target/pose variation, occlusion robustness, natural plant contact, or
   physical hardware is attempted.

## Consequences

The project now has a repeatable Oracle-controlled full manipulation round
trip for the canonical Blender-v2 plant scene. This closes the current
geometry-to-execution integration step, but it is not formal end-to-end
acceptance. The detector's known unripe limitation, perception-controlled
generalization, varied fruit geometry, and hardware transfer remain open.
`pick_authorized` remains false outside the consumed development gate.
