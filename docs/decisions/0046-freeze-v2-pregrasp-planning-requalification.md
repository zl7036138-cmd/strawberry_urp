# ADR 0046: Freeze v2 pre-grasp planning requalification

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One controller-free planning regression after ADR 0045

## Context

ADR 0045 replaced the canonical Blender-v2 tool-centre offset with the
qualified 0.0964 m value while preserving the 0.15 m pre-grasp stand-off.
Exact mesh and gripper-only runtime gates passed, but the new hand goal still
needs to be checked against the canonical natural plant, table, planter, bin,
two non-target fruits, and selected-fruit collision sphere.

The existing pre-grasp Shadow constructs the production hand goal, loads the
full MoveIt collision scene, plans without trajectory-execution configuration,
audits the endpoint, and discards the trajectory. An Oracle target can isolate
planning geometry from the known unripe detector limitation without
authorizing motion.

## Decision

1. Create one non-acceptance gate named
   `blender_v2_pregrasp_planning_post_contact_v1`.
2. Launch the canonical world headless with dual cameras and the Oracle target
   provider for target 1. Do not start perception, localization, manipulation,
   orchestrator, attachment, or pose control.
3. Bind the exact ADR-0044 runtime result, ADR-0045 profile configuration,
   production profile loader, pre-grasp Shadow source, frozen Oracle handoff
   receipt, validator, and runner.
4. Collect at least 15 fresh Oracle target samples and 10 joint samples. Plan
   up to three times only until one valid pre-grasp plan is found. The
   trajectory must never execute.
5. Freeze pass conditions:

   - selected profile is `blender_v2_26mm`;
   - reported tool-centre offset is exactly `0.0964 m`;
   - planning succeeds with a valid endpoint;
   - all seven expected collision objects exist before and after planning;
   - the selected fruit collision sphere remains present;
   - observed and MoveIt arm-state deltas are no greater than `0.002 rad`;
   - a generated trajectory is explicitly discarded;
   - no controller configuration, control interface, gripper command, pick
     action, or trajectory execution exists; and
   - `pick_authorized=false`.

6. Execute once into a fresh non-overwriting directory. Preserve failure; no
   retry, target, scene, profile, threshold, collision, or planning relaxation
   is authorized.

## Boundaries

A pass qualifies only reachability and collision-checked pre-grasp planning
from the current ready state with an Oracle target. It does not authorize arm
motion, target-collision removal, final approach, closing, retreat, placement,
full picking, perception control, formal acceptance, held-out-test access, or
physical hardware.
