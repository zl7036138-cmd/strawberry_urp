# ADR 0054: Preserve v1 handoff failure and freeze v2

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One measurement-only rerun after the ADR-0053 infrastructure defect

## Context

The single ADR-0053 invocation is preserved at
`results/development/blender_v2_perception_handoff_pregrasp_v1`. Its base
camera stage passed with target 1 in 60/60 frames at confidence
`0.6357963681`, and the selector chose the frozen `lower` wrist preset.

MoveIt then produced a collision-free observation-pose plan with a
`0.034122652 s` planning time and a planned endpoint error of approximately
`0.7 mm` / `0.65 deg`. Execution stopped after `0.014940636 s`. The runtime
log records:

- `Action client not connected to action server`;
- `Failed to send trajectory part 1 of 1`; and
- completed execution status `ABORTED`.

The failure occurred before the controller accepted a trajectory goal. The
runner therefore stopped before wrist perception, handoff, pre-grasp planning,
gripper command, attachment, or pick. This is an observation-motion controller
discovery defect, not detector, localization, collision-planning, grasp
geometry, or fruit-contact evidence. The failed result must not be rewritten
or resumed.

The production backend already creates a separate arm action client and
requires that server to be ready during startup. MoveIt's simple controller
manager instead creates its internal client only at execution time and can
observe a transient not-connected state.

## Decision

1. Preserve the complete v1 directory and failed summary.
2. For the `WRIST_OBSERVATION` stage only, retain MoveIt's collision-checked
   RRT plan and endpoint audit, extract the planned Panda joint states, and
   send that same path through the startup-verified arm action client.
3. Recheck action-server availability immediately before sending. If it is
   unavailable, fail with zero execution time and no goal.
4. Keep every production pick stage on its existing execution path. Do not
   change grasp planning, final approach, closure, attachment, retreat,
   placement, home recovery, collision geometry, detector, threshold, camera
   preset, or synchronization policy.
5. Freeze one new gate named
   `blender_v2_perception_handoff_pregrasp_v2`, using the ADR-0053 parameters
   in ROS domain 231 and a new non-overwriting directory.
6. Execute v2 at most once. A failure is preserved; no further rerun is
   authorized without another decision.

## Boundaries

This decision authorizes only the already-bounded wrist observation motion,
perception handoff audit, and controller-free pre-grasp planning. It does not
authorize a perception-derived grasp trajectory, gripper command, attachment,
pick action, formal acceptance, held-out-test access, or physical hardware.
