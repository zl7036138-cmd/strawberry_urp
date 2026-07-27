# ADR 0047: Accept v2 pre-grasp planning requalification

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Close the Blender-v2 pre-grasp planning subgate without authorizing motion

## Context

ADR 0045 accepted the scene-bound Blender-v2 grasp profile after the frozen
exact-mesh sweep and gripper-only runtime round trip passed. ADR 0046 then
froze one controller-free pre-grasp planning regression using the same
production geometry loader and pre-grasp goal construction.

The consumed result at
`results/development/blender_v2_pregrasp_planning_post_contact_v1/summary.json`
has SHA-256
`8aceb977c956eb11f4d88458d4057576a14a382b50727c6e81222c7eea3a8dc6`
and passed without violations:

- selected profile: `blender_v2_26mm`;
- tool-centre offset: `0.0964 m`;
- pre-grasp stand-off: `0.15 m`;
- planning attempts: one;
- planning time: `0.038638462 s`;
- trajectory waypoints: 24;
- endpoint position/orientation error:
  `0.832448 mm` / `0.009426222 rad`;
- all seven expected collision objects remained present before and after
  planning, including the selected fruit;
- observed arm-joint and MoveIt-state delta:
  `8.47e-21 rad` / `0 rad`; and
- control command count: zero.

The trajectory was generated and explicitly discarded. Perception,
localization, manipulation, attachment, orchestrator, pick action, gripper
commands, robot motion, trajectory execution, and held-out-test access did not
start. `pick_authorized` and `formal_acceptance` remained false.

## Decision

1. Accept the consumed ADR-0046 result as evidence that the qualified
   Blender-v2 tool-centre offset remains reachable and collision-plannable from
   the current ready state in the complete canonical scene.
2. Keep the scene-keyed grasp profile active in both the production action
   server and controller-free pre-grasp Shadow.
3. Close R25 only for exact-mesh geometry, dual-contact round trip,
   attach/detach recovery, and controller-free pre-grasp planning.
4. Require a separately frozen gate before any final approach, target
   collision removal, arm execution, retreat, placement, repeated pick, or
   perception-derived pick is attempted.
5. Preserve the formal held-out test seal and keep
   `pick_authorized=false`.

## Consequences

The project now has a coherent Blender-v2 grasp profile that is qualified
offline, at gripper-only runtime, and through collision-checked pre-grasp
planning. This removes the known v1-to-v2 geometry mismatch without claiming a
complete pick. The next manipulation work is a bounded execution gate for
final approach, closure, retreat, recovery, and repeatability; natural
plant-contact robustness and physical hardware remain later stages.
