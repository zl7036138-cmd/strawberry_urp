# ADR 0045: Accept v2 gripper contact and scene-bound geometry

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Close R25's geometry/contact subgate without authorizing a pick

## Context

ADR 0043 rejected the legacy 35 mm grasp parameters for the canonical 26 mm
Blender-v2 fruit and selected `0.0964 m` tool-centre offset plus `0.022 m`
per-finger close command from a frozen 24-candidate exact-mesh sweep.

ADR 0044 then authorized one gripper-only runtime trial. The frozen result at
`results/development/blender_v2_gripper_contact_round_trip_v1/summary.json`
has SHA-256
`525adf334468f06fe5d9ae1500335f1ad0c132e000edefe64a6d7a23c1f27b7f`
and passed without violations:

- measured contact width: `0.025853592 m` per finger;
- mesh-prediction error: approximately `0.004 mm`;
- left/right finger asymmetry: `0.317968 mm`;
- left/right pad-centre distance: `28.083653/28.076356 mm`;
- raw and processed target contact observed on both fingers;
- no non-target fruit contact;
- guarded attach and detach both confirmed;
- canonical fruit restore error: `0.009966 mm`;
- gripper open-recovery error: `1.864694 mm`; and
- maximum arm-joint drift: `8.40e-11 rad`.

No perception, localization, planning, arm trajectory, orchestrator, or pick
action ran.

## Decision

1. Accept the ADR-0043 recommendation and ADR-0044 runtime result as the
   Blender-v2 hand/contact parameter qualification.
2. Add one immutable grasp-geometry profile table keyed by the exact
   `(world_name, fruit_collision_radius_m)` pair:

   - canonical 26 mm Blender-v2:
     `tool_center_offset_m=0.0964`,
     `gripper_closed_width_m_per_finger=0.022`;
   - archived 35 mm tabletop-v1:
     `tool_center_offset_m=0.1054`,
     `gripper_closed_width_m_per_finger=0.025`.

   Both profiles retain the 0.04 m per-finger open command.
3. Require exactly one matching profile. Unknown radii, duplicate bindings,
   invalid widths, and non-finite geometry fail closed.
4. Make the production action server and controller-free pre-grasp Shadow load
   the same profile. Keep the historical constants in pure helper defaults for
   old isolated tests, but do not use those defaults when a scene manifest is
   available.
5. Run a separate controller-free pre-grasp planning requalification before
   treating the configuration change as integrated.

## Consequences

The v2 close width, TCP offset, raw dual contact, geometric fallback,
attach/detach, fruit restore, and gripper recovery portions of R25 are now
qualified in simulation. The archived v1 contract is preserved rather than
silently changed. Full approach execution, retreat, placement, repeated picks,
perception-derived control, natural plant contact, and physical hardware
remain unqualified. `pick_authorized` remains false.
