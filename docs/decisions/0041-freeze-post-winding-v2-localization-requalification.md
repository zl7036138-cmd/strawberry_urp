# ADR 0041: Freeze post-winding v2 localization requalification

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One post-defect-fix requalification after ADR 0040

## Context

ADR 0039's single camera-clear Blender-v2 localization gate failed with a
44.062873 mm median error even though all 100 positions produced measurements.
ADR 0040 traced the systematic camera-ray error to inward face winding on the
canonical ripe and unripe fruit-body meshes and authorized one mechanical
repair. The repair reversed only the body face-reference order. Vertex
coordinates, vertex normals, materials, collision geometry, camera geometry,
localization parameters, and the 26 mm surface-to-centre offset are unchanged.

The failed ADR-0039 execution remains immutable and consumed. A distinct
post-fix gate is required to determine whether the corrected camera-facing
surface restores the intended depth measurement.

## Decision

1. Create one new development gate named
   `blender_v2_localization_accuracy_100_post_winding_v1`.
2. Bind the exact repaired ripe mesh with SHA-256
   `25237e08bd74558a124d8dd33d5e5accce845cea5e97fe05cd1a81b4ec121452`
   and the repaired unripe mesh with SHA-256
   `88e6ea10b3629e3dab5a87cb19103ed8cc299543585624b8790b84e36acfd81f`.
   Bind the ADR-0040 repair receipt as precondition evidence.
3. Reuse the exact ADR-0039 camera-clear materialization, target identity,
   identity orientation, 100 Cartesian positions, three settled depth frames,
   three per-position sensor attempts, fixed base camera, production
   localization node, 26 mm radius, and headless runtime.
4. Keep the original pass conditions unchanged:

   - exactly `100/100` valid measurements;
   - median 3-D error no greater than `15 mm`;
   - P95 3-D error no greater than `30 mm`;
   - no unhandled runtime failure; and
   - no robot, gripper, planning, attachment, or pick command.

5. Execute the post-fix contract once into its separately named,
   non-overwriting result directory. No threshold, offset, crop, position,
   retry, orientation, mesh coordinate, or camera change is authorized.

## Boundaries

A pass qualifies only camera-clear v2 depth/TF localization after the winding
repair. It does not erase the failed pre-repair evidence or qualify natural
plant occlusion, detector performance, multi-view fusion, grasp geometry,
trajectory execution, formal acceptance, physical hardware, or sim-to-real
behavior. The real held-out test remains sealed.
