# ADR 0059: Accept field-v3 collision parity and pre-grasp planning

- Recorded: 2026-07-28
- Status: Accepted
- Scope: Scene-bound MoveIt geometry, read-only handoff audit, and discarded
  field-v3 pre-grasp trajectory

## Context

ADR 0058 accepted field-v3 rendering, dual-camera sensing, target selection,
and no-motion RGB-D localization but blocked motion until MoveIt matched the
new environment.

The first field-v3 pre-grasp shadow loaded exact boxes for the ground, three
ridges, workcell crown, relocated collection bin, and all three fruit.
MoveIt correctly rejected all three planning attempts because the source
ridge-1 collision overlapped `panda_link0` and `panda_link1`. No trajectory was
generated or executed.

The source ridge extended 0.175 m closer to the fixed Panda pedestal than the
field ground. The exporter now clips only that approach-side strip so both
Gazebo and MoveIt start ridge 1 at x=0.20 m, equal to the ground front edge.
The retained collision still covers the workcell plant and fruit volume.
Source tests compare all field boxes, the translated collection bin, and the
workcell crown directly against their Gazebo SDF definitions.

The fresh consolidated result is
`results/development/field_v3_perception_shadow_v6`. Its
`shadow_window.json` has SHA-256
`8894d8f1a39d8f274fbdae95923489c7c581af4ae5a2b6a64b0ea2a2f6e7fad4`.
All 60 frames supplied a target pose and the exact-identity readiness gate
passed. Its 60-sample localization result passed with 4.856 mm median, P95,
and maximum error.

The read-only `collision_scene_audit.json` has SHA-256
`57d4b08a7835ed8a4d80b1bb89ec7b318958c010e748b337362b2e5506d0bcd4`.
All nine expected collision objects were retained, no object was missing, all
frames were accepted, and observed joint-state delta was zero.

The `pregrasp_planning_shadow.json` has SHA-256
`8f83c2bb40795de81c1ac81b548b063f129b48d894ac74d3b8f8b4a6eeecb7d2`.
MoveIt accepted the first planning attempt in 0.0303 s, produced 47 waypoints,
and ended within 0.754 mm / 0.00885 rad of the requested pre-grasp pose. The
trajectory was discarded. Control-command count, execution-request count,
trajectory-executed state, and pick-action calls all remained zero.

## Decision

1. Accept the `field_v3` static-collision profile for the ground, three
   ridges, workcell crown, and relocated collection bin.
2. Select static collision profiles from each scene manifest. Keep
   `blender_v2` as the default for legacy schema-v1 manifests.
3. Preserve the pedestal-overlap planning failure as fail-closed evidence.
4. Accept the clipped ridge-1 box because Gazebo, MoveIt, the exporter
   manifest, and source tests share the same x=0.20 m boundary.
5. Accept the v6 handoff and pre-grasp results as non-formal, controller-free
   development evidence.
6. Keep `motion_authorized: false`; do not infer grasp, attachment, placement,
   or recovery success from a discarded plan.
7. Require one separately frozen Oracle execution gate before considering a
   field-v3 perception-derived pick.

## Consequences

Field-v3 now reaches the project's safe pre-grasp boundary: its natural field,
dual cameras, target identity, depth/TF localization, planning scene, and
collision-aware pre-grasp planner work together. The remaining field-v3 risk
is execution under the new ridge and bin layout, especially contact,
attachment, transport, placement, and recovery. Those behaviors remain
blocked until the next explicit gate passes.
