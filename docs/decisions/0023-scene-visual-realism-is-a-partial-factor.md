# ADR 0023: Scene visual realism is a partial perception factor

Date: 2026-07-16

Status: Accepted for architecture evidence only

## Context

The real-image detector remains below T30, while simulator behavior varies by
lighting and occlusion. A plausible confound was that the original fruit is a
solid-color 35 mm sphere with a cylindrical calyx rather than a strawberry-like
visual. This decision isolates the visual asset without training, motion, formal
seeds, or a held-out real test.

The first full v1 run is preserved but its C mesh rendered white. Renderer-only
v2 and v3 smokes confirmed that the OBJ texture was not consumed. Those C
results are infrastructure failures and cannot support a model conclusion. The
v4 renderer smoke passed after C was rebuilt from native textured sphere/cone
visuals with a radial calyx and stem. The frozen collision sphere, mass, contact
sensor, camera, lighting, model checkpoint, threshold, positions, and frame
counts did not change. Gazebo's documented texture path is a PBR albedo map, and
mesh-embedded material is an alternative; consumer screenshots remain mandatory
because SDF validity alone does not prove renderer consumption.

## Frozen diagnostic

- Model: `baseline__best`, SHA-256
  `ce6c998ed52ae97c8e8b51880adb60618733fff11be22378baa0044f9a126425`.
- Threshold: 0.31.
- Variants: A legacy solid sphere; B camera-scale albedo on the same sphere;
  C the same albedo plus a native sphere/cone fruit shape, five-leaf calyx and
  stem.
- Targets: ripe and unripe, each at three frozen clear positions.
- Measurement: discard 30 detection frames, then measure exactly 60 frames per
  scenario, for 18 scenarios and 1,080 measured frames.
- Boundary: no training, Oracle, orchestration, attachment, manipulation, robot
  motion, formal matrix, held-out real test, promotion, or perception control.

## Result

| Variant | Target | Correct frames | Wrong-class frames | Mean correct confidence | Target-pose frames |
|---|---|---:|---:|---:|---:|
| A | RIPE | 180/180 | 0/180 | 0.660601 | 176/180 |
| A | UNRIPE | 0/180 | 0/180 | n/a | 0/180 |
| B | RIPE | 180/180 | 0/180 | 0.660598 | 178/180 |
| B | UNRIPE | 0/180 | 0/180 | n/a | 0/180 |
| C | RIPE | 180/180 | 0/180 | 0.758152 | 179/180 |
| C | UNRIPE | 0/180 | 0/180 | n/a | 0/180 |

B minus A changes ripe confidence by only -0.000003 and does not meet the
frozen material-effect threshold. C minus A changes ripe confidence by
+0.097551 and exceeds the frozen +0.05 threshold. All variants fail to detect
the isolated unripe target at the frozen threshold; the absence of a false ripe
box is safe for `NO_PICK`, but it does not satisfy the required unripe detection
metric.

## Decision

1. The simplified scene is a **partial** factor: fruit silhouette and calyx
   structure materially affect ripe confidence.
2. Texture alone at the current 35--45 pixel target scale is not a material
   factor in this bounded test.
3. Scene realism is not the root cause of the unripe-class failure and cannot
   explain the below-gate audited real metric.
4. Variant C v4 is retained as a development asset candidate only. The canonical
   base world is not changed, because doing so would invalidate prior world
   hashes and because C has not passed an Oracle-motion regression.
5. The rejected simulator-adaptation checkpoint remains rejected. No training
   retry, model promotion, perception control, P3/P4 acceptance, or formal test
   is authorized by this diagnostic.

## Consequence

The next perception decision should focus on the unripe class and dual-domain
calibration. If C v4 is proposed for the canonical scene, it first needs a
separate Oracle collision/grasp regression with unchanged physics. The
authoritative evidence is
`results/t70/strawberry_asset_ablation_v4/summary.json` and
`artifacts/t70/strawberry_asset_ablation_handoff_v4.json`.

References:

- https://gazebosim.org/api/sim/10/migrationsdf.html
- https://sdformat.org/spec/1.7/material/
