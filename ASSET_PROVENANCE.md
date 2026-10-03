# Asset provenance index

## User-supplied Blender sources

The following source files were supplied by the project owner and are retained
as project inputs. Their inclusion does not imply that third-party elements
inside a `.blend` file have been relicensed:

- `assets/blender_sources/strawberry_ripe_visual_v1.blend`
- `assets/blender_sources/strawberry_plant_v2.blend`
- `assets/blender_sources/strawberry_field_v3.blend` (`st1.blend` source)

Detailed conversion decisions, object ownership, hashes, mesh reduction, and
texture provenance are recorded in:

- `docs/assets/blender-asset-provenance-v2.md`
- `docs/assets/strawberry-texture-provenance-v2.md`
- `docs/field-v3-integration.md`

## Generated simulation assets

OBJ/MTL/SDF derivatives are generated for Gazebo by the scripts under
`tools/blender/`. Their scientific role is simulation input; they are not proof
of real-plant geometry, material response, or flexible vegetation dynamics.

## Dataset and model files

- Real-image data source: Zenodo record 6126677, recorded as CC BY 4.0 in the
  project manifests and documentation.
- Ultralytics base weights and locally trained `.pt` files remain subject to
  their source terms and are excluded from Git by default.
- A final delivery must include hashes and acquisition instructions rather than
  silently treating third-party data or weights as project-authored material.

## Final-review rule

Before public redistribution, verify every included binary asset against this
index and its detailed provenance record. Unknown or unverified assets must be
removed from the public package or listed as externally supplied prerequisites.
