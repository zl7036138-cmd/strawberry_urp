# Blender to Gazebo asset pipeline

The committed sources live in `assets/blender_sources`. Use Blender 5.2.0 LTS
or a compatible later version:

```powershell
$repo = "C:\Users\12753\Documents\New project\strawberry_urp"
$blender = "C:\path\to\blender.exe"

& $blender --background `
  --python "$repo\tools\blender\export_gazebo_assets.py" -- `
  "$repo\assets\blender_sources\strawberry_ripe_visual_v1.blend" `
  "$repo\assets\blender_sources\strawberry_plant_v2.blend" `
  "$repo\ros2_ws\src\strawberry_sim\models" `
  "$repo\artifacts\sim\blend_inspection\gazebo_asset_export_v2.json"
```

The script writes metres/Z-up OBJ and MTL files without relying on Blender's
optional export extensions. It also applies deterministic per-part decimation
to the generated Gazebo copies; the retained `.blend` sources are never
modified. Do not export the whole plant `.blend` directly: it contains imported
templates, mounted fruit copies, and duplicated long fruit stems that must stay
outside the canonical Gazebo plant shell.
