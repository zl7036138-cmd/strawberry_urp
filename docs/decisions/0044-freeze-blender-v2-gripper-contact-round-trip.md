# ADR 0044: Freeze the Blender-v2 gripper contact round trip

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One gripper-only v2 contact and attachment requalification

## Context

The frozen ADR-0043 mesh sweep passed with six feasible configurations and
showed that the current production pair is not v2-safe. At the old 0.1054 m
tool offset, the predicted contact-pad distance is 32.874 mm, beyond the
29 mm attachment fallback envelope, and the old 0.025 m close command provides
only 1.091 mm of controller over-travel.

ADR 0043's deterministic selection rule recommends a 0.0964 m tool-centre
offset and a 0.022 m per-finger close command. Its exact-mesh prediction is:

- first symmetric contact at `0.025857179 m` per finger;
- `3.857179 mm` close-command over-travel;
- `4.443936 mm` palm clearance;
- `24.953018 mm` wrist-housing clearance;
- `14.113690 mm` minimum open-finger clearance; and
- `0.900290 mm` geometric-fallback margin.

Offline geometry cannot prove Gazebo contact sensors, controller stall,
attachment, detachment, or recovery. A bounded runtime check is required
before changing production defaults.

## Decision

1. Create one development gate named
   `blender_v2_gripper_contact_round_trip_v1`. Bind the exact ADR-0043
   contract and result, canonical v2 scene, Panda xacro, contact manager,
   controller configuration, runtime probe, and runner.
2. Launch one fresh headless canonical world with dual cameras, attachment
   enabled, pose control enabled, and no manipulation, orchestrator,
   localization, perception, planning, or pick-action node.
3. Use only `strawberry_1`. Record its canonical pose and the initial seven arm
   joints. Command the gripper open to `0.04 m` per finger, then use Gazebo
   `set_pose` to place the fruit centre exactly `0.0964 m` along the current
   `panda_hand` local +Z axis. The arm must remain at its initial joint state.
4. Command `0.022 m` per finger with `40 N` effort. Require:

   - a successful or contact-stalled gripper action;
   - symmetric measured finger positions within `2 mm`;
   - measured contact width within `3 mm` of the mesh prediction;
   - raw left and right finger contact with `strawberry_1`;
   - both named pad centres no farther than `29 mm` from the live fruit centre;
   - the fruit centre projected between the pads; and
   - no non-target fruit contact.

5. Call the guarded target-1 attach service and require confirmed attachment.
   Do not move the arm or fruit while attached. Call detach and require
   confirmed release. Restore the fruit to its exact canonical pose, reopen the
   gripper to `0.04 m`, and require both fingers to recover within `3 mm`.
6. Across the entire probe, require maximum seven-arm-joint drift no greater
   than `0.002 rad`, exactly three gripper commands (open, close, reopen), one
   successful attach, one successful detach after attach, no planning/pick
   request, and `pick_authorized=false`.
7. Execute the frozen gate once into a fresh non-overwriting output directory.
   No retry, threshold, pose, parameter, contact source, recovery step, or
   runtime topology change is authorized under this decision.

## Boundaries

This gate authorizes simulated fruit pose motion, three gripper commands, and
one guarded attach/detach round trip only. It does not authorize any arm
trajectory, pre-grasp execution, retreat, placement, complete pick, perception
control, formal acceptance, physical hardware, or held-out-test access. A pass
qualifies the selected v2 hand/contact parameters for a later production
configuration change and planning-only regression; it does not set
`pick_authorized=true`.
