# ADR 0043: Freeze the Blender-v2 gripper geometry sweep

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Non-motion geometric qualification for the 26 mm fruit

## Context

The successful historical pick used a 35 mm spherical fruit, a 0.1054 m
fruit-centre offset from `panda_hand`, and a 0.025 m per-finger close command.
The canonical Blender-v2 fruit collision radius is 26 mm. Planning and
localization now consume that smaller radius, but the hand offset, close
command, contact-pad section, and geometric dual-contact fallback have not been
requalified. Reusing the v1 values would leave risk R25 open.

The first check must be independent of controller and contact-sensor timing.
The upstream Panda hand and finger collision meshes provide enough information
to calculate palm clearance, first finger contact, controller over-travel, and
the named contact-pad distance exactly in the `panda_hand` frame. The dual
camera's wrist housing must also remain clear.

## Decision

1. Create one offline development gate named
   `blender_v2_gripper_geometry_sweep_v1`. It is not a pick, trajectory,
   controller, attachment, formal-acceptance, or physical-hardware test.
2. Bind the canonical v2 scene, local Panda extension xacro, production grasp
   geometry code, attachment fallback code, upstream Jazzy Panda xacro, and
   upstream hand/finger collision meshes by byte size and SHA-256.
3. Model the fruit collision envelope as the canonical 26 mm sphere centred on
   the local tool axis. Use the exact upstream triangle meshes for the hand and
   both fingers. Use the xacro transforms: finger-joint Z `0.0584 m`, contact-pad
   Z `0.027 m` relative to each finger, open width `0.04 m`, wrist housing
   centre `[-0.065, 0, 0.045] m`, size `[0.055, 0.080, 0.040] m`, and wrist
   housing pitch `-pi/2`.
4. Evaluate exactly this Cartesian candidate grid:

   - tool-centre offsets: `0.0944, 0.0954, 0.0964, 0.0974, 0.1004, 0.1054 m`;
   - per-finger close commands: `0.018, 0.020, 0.022, 0.025 m`;
   - order: tool offset, then close command.

   Also report the current production pair `0.1054/0.025 m` explicitly.
5. For each pair, solve the symmetric first-contact finger width against the
   26 mm sphere and freeze these pass conditions:

   - palm-to-fruit clearance at least `3.0 mm`;
   - wrist-camera-housing clearance at least `10.0 mm`;
   - open-finger clearance at least `10.0 mm` on both sides;
   - first contact exists symmetrically on both fingers;
   - commanded close lies `2.0-10.0 mm` beyond first contact;
   - at first contact, both named pad centres are within the attachment
     fallback envelope of radius plus `3.0 mm`, with at least `0.5 mm` margin;
   - fruit centre projects between the two pads;
   - finger contact points remain inside the mesh's axial section; and
   - open-hand pre-grasp clearance at the existing `0.15 m` stand-off is at
     least `100 mm`.

6. A candidate is feasible only if every condition passes. Select one
   recommendation deterministically from the feasible set by:

   - greatest worst normalized threshold margin;
   - then least close-command over-travel;
   - then greatest palm clearance;
   - then lexical `(tool offset, close command)` order.

7. Execute the frozen sweep once into a fresh, non-overwriting result
   directory. Preserve a failure. Do not alter the grid, thresholds, meshes,
   transforms, radius, or selection rule under this decision.

## Boundaries

A pass only identifies a geometrically admissible v2 parameter pair. It does
not prove Gazebo contact sensing, controller stall behavior, dual contact,
attach/detach, recovery, trajectory reachability, plant clearance, picking, or
perception control. Any recommended pair must be frozen in a separate,
gripper-only runtime gate before production defaults may change. Arm motion,
pick actions, simulated attachment, and the formal held-out test remain
unauthorized by this decision.
