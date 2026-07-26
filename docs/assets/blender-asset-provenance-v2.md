# Blender plant-scene asset provenance v2

## Sources

The two Blender sources were supplied directly by the project owner on
2026-07-25 and are retained in the repository so the runtime meshes can be
reproduced without the original Desktop paths.

| File | Bytes | SHA-256 |
|---|---:|---|
| `assets/blender_sources/strawberry_ripe_visual_v1.blend` | 502952 | `bd81d003f33f4ae35abc19fdc8192538cb0957f14507a53144126b41f84d3116` |
| `assets/blender_sources/strawberry_plant_v2.blend` | 977180 | `399fbc5b0aab7aa459be1f89d5a004563ca5502c8735a2e43b303dbdaa7c8166` |

Both files load in Blender 5.2.0 LTS, use metres, and contain no external image
textures. Their colours are Blender materials embedded into generated MTL
files.

## Export boundary

`tools/blender/export_gazebo_assets.py` enforces the scene responsibilities:

- fruit: body, seeds, calyx centre, and seven calyx leaves;
- excluded fruit object: `strawberry_curved_stem_v5`;
- plant: direct geometric children of `strawberry_plant_v2_root`;
- excluded plant content: imported fruit templates and mounted fruit copies.
- deterministic Gazebo-only decimation: the fruit body is reduced to 16%,
  calyx meshes to 30%, leaf surfaces to 45%, and plant curves to 25–40%;
  seed geometry is retained at full detail.

The fruit origin is moved to the body bounding-box centre. The mount point is
the `calyx_center_v5` object origin. Fruit positions in the Gazebo world are
computed by connecting that point directly to the Blender pedicel endpoints,
which removes the duplicated-stem gap and fold present in the source scene.

The canonical interface still exposes three independently graspable fruits:
two ripe and one unripe. The fourth Blender pedicel is retained as an empty
natural branch rather than expanding all attachment, bridge, and benchmark
interfaces.

## Generated runtime assets

| File | Bytes | SHA-256 |
|---|---:|---|
| `models/strawberry_ripe/meshes/strawberry_ripe_visual_v2.obj` | 1377293 | `f8cbe53a61ee46b1de6ff1aa6f203291fe1b4f946084ed0617ca8f929dc5b347` |
| `models/strawberry_ripe/meshes/strawberry_ripe_visual_v2.mtl` | 584 | `3e3de17b039060e2a8ecc9d85cdff20b500de982d869365d0b9ecc1cd8efba8b` |
| `models/strawberry_unripe/meshes/strawberry_unripe_visual_v2.obj` | 1377295 | `c8f213bed108c277bedbe910d3215bf73cf25780a90efd99e03cea9a33db0068` |
| `models/strawberry_unripe/meshes/strawberry_unripe_visual_v2.mtl` | 584 | `977dc1b3a16e71edc02ddd99a42d2e2b732730275c37ac7374a3db955d766bbc` |
| `models/strawberry_plant_v2/meshes/strawberry_plant_v2.obj` | 1294962 | `64a97b28b6da157f39a62f3ccba8276aef901184e0b83e0586bab964b5a6cb24` |
| `models/strawberry_plant_v2/meshes/strawberry_plant_v2.mtl` | 926 | `f5fa805ca97c4b171a26634752bf130ae7d9b7c485481aba6f292a9096e55c33` |

Each fruit visual is 18,150 triangles and the plant is 17,129 triangles. The
three-fruit canonical scene therefore uses 71,579 triangles for the imported
plant and fruit visuals, down from 306,316 before the Gazebo-only reduction.
Gazebo uses a 26 mm fruit collision sphere plus compact planter/crown collision
proxies; leaves and stems are visual occluders, not deformable contact geometry.
