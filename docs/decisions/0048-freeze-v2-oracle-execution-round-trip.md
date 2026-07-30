# ADR 0048: Freeze v2 Oracle execution round trip

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One non-acceptance Blender-v2 motion round trip after ADR 0047

## Context

ADRs 0043-0047 qualify the canonical 26 mm Blender-v2 grasp geometry through
an exact-mesh sweep, a gripper-only contact/attachment round trip, and
controller-free pre-grasp planning. Robot motion, final approach, attached
retreat, placement, and recovery remain unqualified.

The production executor already performs a guarded pre-grasp transit, opens
only the selected target collision corridor, descends to the grasp, requires
bilateral contact before attachment, retreats, releases into the collection
bin, verifies the fruit, returns to `ready`, and restores the target collision
sphere. Releasing into the bin is retained in this gate because detaching an
attached fruit over the plant would create an avoidable drop and an unsafe
recovery scene.

The pre-execution audit found that the success path called `move_home()` but
did not check its return value. This could report a successful round trip after
a failed recovery motion.

## Decision

1. Make final home failure convert an otherwise successful action to
   `PLANNING_FAILED`. On an already-failed action, append a recovery-home
   failure to the diagnostic message. Never retry home automatically.
2. Freeze one gate named
   `blender_v2_oracle_execution_round_trip_v1`.
3. Start one fresh canonical Blender-v2 world with both the base and wrist
   cameras. Enable only the production manipulation server and guarded
   attachment backend. Keep perception, localization, Oracle provider,
   orchestrator, and Gazebo pose control stopped.
4. Send target 1 directly from its stable ground-truth pose to the production
   pick-and-place action. Use the scene-bound `blender_v2_26mm` profile:
   `0.0964 m` tool-centre offset, `0.022 m` close, and `0.040 m` open.
5. Require:

   - action success with the exact ordered stages
     `PLAN, APPROACH, GRASP, RETREAT, PLACE, VERIFY, DONE`;
   - planning time no greater than five seconds;
   - processed and raw target contact on both fingers;
   - no unexpected non-finger fruit contact;
   - confirmed detached-to-attached-to-detached state transitions;
   - final detached state;
   - all seven arm joints initially within `0.02 rad` of `ready`;
   - maximum initial-to-final arm-joint delta no greater than `0.02 rad`;
   - both fingers reopened within `0.003 m` of `0.040 m`; and
   - a clean launch shutdown.

6. Execute at most once into a fresh, non-overwriting result directory. No
   retry, alternate target, changed pose, collision relaxation, tolerance
   change, or parameter change is authorized.

## Boundaries

This gate authorizes exactly one Oracle-controlled simulation round trip. A
pass does not authorize repeated execution, perception-derived control,
formal acceptance, held-out-test access, natural-plant contact robustness, or
physical hardware. A separately frozen repeat gate is required before any
repeat run. General `pick_authorized` remains false.
