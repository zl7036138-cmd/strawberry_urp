# Strawberry texture provenance v2

Date: 2026-07-16

Generation mode: built-in `image_gen` tool through the project image-generation
skill. The v2 files are project-bound diffuse albedo assets. They were generated
at camera scale so seed detail could survive a 35--45 pixel fruit projection.
The earlier dense v1 images remain preserved but are not used by the accepted
v4 asset diagnostic.

## Final ripe prompt

```text
Use case: stylized-concept
Asset type: camera-scale diffuse albedo texture for a small Gazebo ripe strawberry visual, rendered only about 35 to 45 pixels wide
Primary request: create a flat realistic ripe strawberry skin texture optimized so seed features remain visible when the whole texture is wrapped once around a very small 3D fruit
Scene/backdrop: none; strawberry skin fills the canvas edge to edge
Style/medium: photorealistic diffuse albedo map, not a photo of a whole fruit
Composition/framing: use sparse, clearly separated pale yellow achenes in shallow dimples, approximately 10 to 14 seeds across the width of the entire canvas; organic irregular spacing; opposite edges tile cleanly
Lighting/mood: neutral even illumination, no directional shadow and no baked glossy highlight
Color palette: natural ripe red with subtle crimson and scarlet variation
Constraints: no fruit silhouette, no leaf, no calyx, no stem, no background, no border, no text, no watermark; seed details must be large enough to survive downsampling to a 40-pixel-wide fruit; seamless/tileable edges; suitable as a color/albedo map
Avoid: dense tiny seeds, repeated grid, cartoon style, dramatic lighting, black shadows, plate, table, hands
```

Project files share SHA-256
`cf8bbc06dff41db7e803f388f40e3ad7eafebf37c5019f1d338d011080840b57`:

- `ros2_ws/src/strawberry_sim/models/strawberry_ripe_textured_v2/materials/textures/ripe_strawberry_skin_v2.png`
- `ros2_ws/src/strawberry_sim/models/strawberry_ripe_realistic_v4/materials/textures/ripe_strawberry_skin_v2.png`

## Final unripe prompt

```text
Use case: stylized-concept
Asset type: camera-scale diffuse albedo texture for a small Gazebo unripe strawberry visual, rendered only about 35 to 45 pixels wide
Primary request: create a flat realistic unripe strawberry skin texture optimized so seed features remain visible when the whole texture is wrapped once around a very small 3D fruit
Scene/backdrop: none; unripe strawberry skin fills the canvas edge to edge
Style/medium: photorealistic diffuse albedo map, not a photo of a whole fruit
Composition/framing: use sparse, clearly separated pale cream-green achenes in shallow dimples, approximately 10 to 14 seeds across the width of the entire canvas; organic irregular spacing; opposite edges tile cleanly
Lighting/mood: neutral even illumination, no directional shadow and no baked glossy highlight
Color palette: pale natural yellow-green with subtle lime, ivory-green, and light green variation; absolutely no red ripe patches
Constraints: no fruit silhouette, no leaf, no calyx, no stem, no background, no border, no text, no watermark; seed details must be large enough to survive downsampling to a 40-pixel-wide fruit; seamless/tileable edges; suitable as a color/albedo map
Avoid: dense tiny seeds, repeated grid, cartoon style, dramatic lighting, red areas, black shadows, plate, table, hands
```

Project files share SHA-256
`401fdcb1f9a08257d16ed41046f0b03481991be87a96bbb88c7aab603123a4db`:

- `ros2_ws/src/strawberry_sim/models/strawberry_unripe_textured_v2/materials/textures/unripe_strawberry_skin_v2.png`
- `ros2_ws/src/strawberry_sim/models/strawberry_unripe_realistic_v4/materials/textures/unripe_strawberry_skin_v2.png`
