# ADR 0036: Adopt the Blender plant scene as the canonical simulator scene

- Status: accepted
- Date: 2026-07-25

## Context

The released v1 simulator used floating 35 mm red and green spheres on a work
table. That scene was sufficient for communication, localization, and bounded
pick-and-place integration, but it was not a defensible representation of a
strawberry plant. It also created a large visual domain gap for a detector
trained on real fruit.

The project owner supplied a detailed ripe fruit and a six-leaf-group plant
with four pedicel endpoints. Inspection found that the plant source also
contained imported templates and mounted fruit copies whose own long curved
stems duplicated the plant pedicels.

## Decision

1. The canonical `strawberry_orchard.sdf` now loads `strawberry_plant_v2`.
2. The plant owns crown, leaves, petioles, inflorescences, and pedicels.
3. Graspable fruit owns body, seeds, and calyx only. Its long display stem is
   excluded from runtime export.
4. Each fruit is centred on its body and mounted by aligning the calyx-centre
   point directly with a Blender pedicel endpoint.
5. Three existing identities remain stable: `strawberry_1` and
   `strawberry_3` are ripe; `strawberry_2` is unripe.
6. Fruit collision radius changes from 35 mm to 26 mm to match the new body
   width and prevent overlapping proxies in the natural fruit cluster.
7. A shallow planter raises the plant to preserve gripper clearance. The
   planter and plant crown participate in physics and MoveIt collision
   geometry. Leaves and stems provide natural RGB-D occlusion but do not claim
   flexible contact or damage simulation.
8. The v1 sphere world and manifest remain available as
   `strawberry_tabletop_benchmark_v1.sdf` and `scene_tabletop_v1.yaml`.
9. Runtime OBJ meshes use a deterministic Gazebo-only reduction. The Blender
   sources remain untouched and are the authority for future asset revisions.

## Consequences

- The default scene is now a simplified plant-harvest environment rather than
  a floating-fruit tabletop scene.
- Frozen P3/P4 results remain valid only for the archived v1 benchmark. They
  cannot be compared directly with future v2 measurements.
- Previously generated lighting/blue-box materializations remain historical
  fixtures. Robustness conditions for v2 should use natural leaf and fruit
  occlusion before another formal matrix is frozen.
- Fruit detachment is still abstracted by the existing bounded temporary joint;
  the scene does not model stem cutting, soft leaves, or fruit damage.

## Verification

- Blender 5.2.0 LTS loaded both sources and produced deterministic OBJ/MTL
  exports.
- The runtime export contains 18,150 triangles per fruit and 17,129 for the
  plant, or 71,579 imported visual triangles in the canonical scene.
- SDFormat parsed both fruit models, the plant model, the v2 world, and the
  archived v1 world.
- All dependency-light source suites: 378 run, 377 passed, 1 conditional skip.
- ROS build: `strawberry_interfaces`, `strawberry_sim`, and
  the four dependent pipeline packages completed.
- An 8-second runtime health sample passed with colour, depth, camera info,
  joint states, and all three ground-truth streams. The lightweight export
  advanced 2.915 simulated seconds in 8.054 wall seconds.
- A one-trial Oracle smoke completed the physical pick and place with both raw
  finger contacts, 0.0332 s planning time, no unexpected fruit contact, and
  failure code 0. Evidence is under
  `results/development/blender_scene_v2_oracle_smoke_v4`.
- A separate 60-frame, no-motion Shadow diagnostic detects exactly one ripe
  fruit and publishes its target pose in 60/60 frames, but detects neither the
  second ripe fruit nor the unripe fruit. This is partial improvement, not
  visual acceptance; see `docs/blender-scene-v2-shadow-diagnostic-v1.md`.
- `artifacts/sim/blend_inspection/canonical_plant_scene.png` confirms the
  installed assets render through the fixed RGB-D camera.
