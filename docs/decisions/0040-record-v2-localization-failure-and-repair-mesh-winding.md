# ADR 0040: Record the v2 localization failure and repair mesh winding

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Outcome of ADR 0039 and one bounded asset defect correction

## Evidence

The single ADR-0039 execution completed all 100 frozen positions with no
measurement failure, robot motion, gripper command, attachment, perception
model, or unhandled runtime error. The preserved result at
`results/development/blender_v2_localization_accuracy_100_v1/summary.json`
records:

| Metric | Required | Actual | Passed |
|---|---:|---:|---|
| Valid positions | 100/100 | 100/100 | Yes |
| Median 3-D error | <=15 mm | 44.062873 mm | No |
| P95 3-D error | <=30 mm | 44.537073 mm | No |

The error is systematic rather than noisy. Relative to the fixed camera
origin, the median error component along the camera-to-fruit ray is
`44.058444 mm`; the median perpendicular component is only `0.511948 mm`, and
the median alignment cosine is `0.999933`. Removing the configured 26 mm
surface-to-centre shift implies that the raw median depth lies approximately
`18.058444 mm` behind the model origin. This is consistent with seeing the
rear surface rather than the front surface.

An offline topology audit identifies the asset defect. The
`strawberry_ripe_body_v5` object in both canonical ripe and unripe OBJ files
contains exactly 11,796 triangles, and all 11,796 have inward face winding
relative to the body centroid. The explicit vertex normals point outward, so
the face order and normal direction disagree. Back-face culling can therefore
discard the camera-facing body surface while the depth camera records the rear
surface.

## Decision

1. Record `blender_v2_localization_accuracy_100_v1` as failed. Do not change
   its summary, samples, thresholds, positions, radius, or interpretation.
2. Correct one asset defect only: reverse the vertex/normal reference order of
   every face belonging to `strawberry_ripe_body_v5` in both canonical ripe
   and unripe OBJ files.
3. Do not change vertex coordinates, vertex normals, object names, material
   assignments, collision spheres, SDF files, camera geometry, localization
   code, crop parameters, or the 26 mm offset.
4. Require a mechanical repair tool, before/after hashes, triangle-count and
   winding verification, XML/asset tests, and visual inspection of both
   repaired fruits.
5. ADR 0039's execution claim remains consumed. A post-repair localization
   qualification requires a separately named contract and decision. It must
   retain the same 100 positions and numeric thresholds so the repair is the
   only result-determining intervention.

## Boundaries

This decision authorizes an OBJ face-winding repair, not detector retraining,
threshold changes, localization tuning, a second ADR-0039 execution, robot
motion, gripper tests, formal acceptance, physical hardware, or a sim-to-real
claim. The real held-out test remains sealed.
