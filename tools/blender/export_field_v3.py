"""Export the supplied strawberry field as a reusable Gazebo visual model.

The Blender file contains a complete 5 x 12 metre field, 24 shared plant
meshes, and 103 placed plant instances.  This exporter preserves that sharing:
each plant variant is exported once and the generated SDF places lightweight
visual instances.  Only the ground and planting ridges receive box collision
geometry.

Two background plants are intentionally omitted around the robot workcell.
The runtime world fills that bay with the project's already-qualified v2 plant
and independently graspable fruit models.
"""

from __future__ import annotations

import importlib.util
import json
import math
import re
import sys
from pathlib import Path

import bpy
from mathutils import Vector


MODEL_NAME = "strawberry_field_v3"
PLANT_NAME_PATTERN = re.compile(r"^草莓植株_\d+$")
WORKCELL_SOURCE_ORIGIN_M = Vector((-2.70, -0.15, 0.0))
OMITTED_WORKCELL_PLANTS = {"草莓植株_010", "草莓植株_026"}
COLLISION_SOURCE_OBJECTS = (
    "5×12米地面",
    "种植垄_1",
    "种植垄_2",
    "种植垄_3",
)
PLANT_DECIMATION_RATIO = 0.45
# The source ridge reaches 17.5 cm farther toward panda_link0 than the ground
# and intersects the fixed robot pedestal. Clip only that approach-side strip;
# the retained ridge still covers the workcell plant and fruit volume.
WORKCELL_RIDGE_COLLISION_MIN_X_M = 0.20


def _argv() -> list[str]:
    if "--" not in sys.argv:
        return []
    return sys.argv[sys.argv.index("--") + 1 :]


def _load_common_exporter():
    module_path = Path(__file__).with_name("export_gazebo_assets.py")
    spec = importlib.util.spec_from_file_location(
        "strawberry_gazebo_asset_exporter", module_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load common exporter: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _round_values(values, digits: int = 9) -> list[float]:
    return [round(float(value), digits) for value in values]


def _format_values(values) -> str:
    return " ".join(f"{float(value):.9f}" for value in values)


def _world_bounds_in_model(
    obj: bpy.types.Object,
) -> tuple[Vector, Vector]:
    if not getattr(obj, "bound_box", None):
        raise RuntimeError(f"object has no bounds: {obj.name}")
    lower = Vector((math.inf, math.inf, math.inf))
    upper = Vector((-math.inf, -math.inf, -math.inf))
    for corner in obj.bound_box:
        point = obj.matrix_world @ Vector(corner) - WORKCELL_SOURCE_ORIGIN_M
        for axis in range(3):
            lower[axis] = min(lower[axis], point[axis])
            upper[axis] = max(upper[axis], point[axis])
    return lower, upper


def _box_collision_xml(obj: bpy.types.Object, index: int) -> tuple[str, dict]:
    lower, upper = _world_bounds_in_model(obj)
    adjustments = {}
    if index == 2:
        lower.x = max(lower.x, WORKCELL_RIDGE_COLLISION_MIN_X_M)
        adjustments["minimum_x_m"] = WORKCELL_RIDGE_COLLISION_MIN_X_M
        adjustments["reason"] = "fixed Panda pedestal clearance"
    if any(lower[axis] >= upper[axis] for axis in range(3)):
        raise RuntimeError(f"invalid clipped collision bounds: {obj.name}")
    center = (lower + upper) * 0.5
    size = upper - lower
    collision_name = f"field_collision_{index:02d}"
    xml = f"""      <collision name="{collision_name}">
        <pose>{_format_values((*center, 0.0, 0.0, 0.0))}</pose>
        <geometry>
          <box><size>{_format_values(size)}</size></box>
        </geometry>
      </collision>"""
    return (
        xml,
        {
            "name": collision_name,
            "source_object": obj.name,
            "center_m": _round_values(center),
            "size_m": _round_values(size),
            "adjustments": adjustments,
        },
    )


def _variant_proxy(
    representative: bpy.types.Object,
    name: str,
) -> bpy.types.Object:
    proxy = bpy.data.objects.new(name, representative.data)
    bpy.context.scene.collection.objects.link(proxy)
    proxy.matrix_world.identity()
    return proxy


def _rename_materials_for_export() -> list[dict]:
    """Give Unicode source materials deterministic OBJ-safe aliases.

    The shared exporter deliberately strips non-ASCII characters.  Every
    source material in this field is named in Chinese, so exporting without
    aliases would collapse unrelated leaf, fruit, soil, and sign materials
    onto the same ``.001`` token.
    """

    materials = sorted(bpy.data.materials, key=lambda material: material.name)
    original_names = [material.name for material in materials]
    for index, material in enumerate(materials, start=1):
        material.name = f"__field_material_tmp_{index:02d}"
    rows = []
    for index, (material, original_name) in enumerate(
        zip(materials, original_names), start=1
    ):
        alias = f"field_material_{index:02d}"
        material.name = alias
        rows.append(
            {
                "source_name": original_name,
                "export_name": alias,
                "diffuse_rgba": _round_values(material.diffuse_color),
            }
        )
    return rows


def _model_sdf(
    structure_mesh_name: str,
    plant_rows: list[dict],
    collision_xml: list[str],
) -> str:
    visuals = [
        f"""      <visual name="field_structure_visual">
        <geometry>
          <mesh>
            <uri>model://{MODEL_NAME}/meshes/{structure_mesh_name}</uri>
          </mesh>
        </geometry>
      </visual>"""
    ]
    for index, row in enumerate(plant_rows, start=1):
        pose = (*row["location_m"], *row["rotation_rpy_rad"])
        visuals.append(
            f"""      <visual name="field_plant_{index:03d}">
        <pose>{_format_values(pose)}</pose>
        <geometry>
          <mesh>
            <uri>model://{MODEL_NAME}/meshes/{row["mesh_file"]}</uri>
            <scale>{_format_values(row["scale"])}</scale>
          </mesh>
        </geometry>
      </visual>"""
        )
    body = "\n".join([*collision_xml, *visuals])
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{MODEL_NAME}">
    <static>true</static>
    <link name="field_link">
{body}
    </link>
  </model>
</sdf>
"""


def _model_config() -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{MODEL_NAME}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>Strawberry URP Team</name>
  </author>
  <description>
    Blender-derived static strawberry field for the opt-in field-v3 scene.
  </description>
</model>
"""


def export_field(source: Path, model_dir: Path) -> dict:
    common = _load_common_exporter()
    bpy.ops.wm.open_mainfile(filepath=str(source))
    units = bpy.context.scene.unit_settings
    if units.system != "METRIC" or abs(float(units.scale_length) - 1.0) > 1.0e-9:
        raise RuntimeError(
            "field source must use metric units with scale_length equal to 1.0"
        )
    material_aliases = _rename_materials_for_export()

    visible_meshes = sorted(
        (
            obj
            for obj in bpy.data.objects
            if obj.type == "MESH" and not obj.hide_render
        ),
        key=lambda item: item.name,
    )
    plant_objects = [
        obj for obj in visible_meshes if PLANT_NAME_PATTERN.fullmatch(obj.name)
    ]
    structure_objects = [
        obj for obj in visible_meshes if not PLANT_NAME_PATTERN.fullmatch(obj.name)
    ]
    missing_collision_objects = [
        name for name in COLLISION_SOURCE_OBJECTS if bpy.data.objects.get(name) is None
    ]
    if missing_collision_objects:
        raise RuntimeError(
            "field source is missing collision objects: "
            + ", ".join(missing_collision_objects)
        )
    missing_omissions = sorted(
        OMITTED_WORKCELL_PLANTS.difference(obj.name for obj in plant_objects)
    )
    if missing_omissions:
        raise RuntimeError(
            "field source is missing workcell plants: " + ", ".join(missing_omissions)
        )

    meshes_dir = model_dir / "meshes"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    structure_mesh_name = "field_structure_v3.obj"
    structure_report = common._write_obj(
        meshes_dir / structure_mesh_name,
        structure_objects,
        origin=WORKCELL_SOURCE_ORIGIN_M,
    )

    retained_plants = [
        obj for obj in plant_objects if obj.name not in OMITTED_WORKCELL_PLANTS
    ]
    data_names = sorted({obj.data.name for obj in retained_plants})
    variants: dict[str, dict] = {}
    for index, data_name in enumerate(data_names, start=1):
        representative = next(
            obj for obj in retained_plants if obj.data.name == data_name
        )
        variant_name = f"plant_variant_{index:02d}"
        mesh_name = f"{variant_name}.obj"
        proxy = _variant_proxy(representative, f"__field_export_{variant_name}")
        try:
            export_report = common._write_obj(
                meshes_dir / mesh_name,
                [proxy],
                origin=Vector((0.0, 0.0, 0.0)),
                decimate_ratio=lambda _obj: PLANT_DECIMATION_RATIO,
            )
        finally:
            bpy.data.objects.remove(proxy, do_unlink=True)
        variants[data_name] = {
            "variant": variant_name,
            "mesh_file": mesh_name,
            "source_mesh_data": data_name,
            "source_representative": representative.name,
            "vertices": export_report["vertices"],
            "triangles": export_report["triangles"],
        }

    plant_rows = []
    for obj in retained_plants:
        location = obj.matrix_world.translation - WORKCELL_SOURCE_ORIGIN_M
        rotation = obj.matrix_world.to_euler("XYZ")
        scale = obj.matrix_world.to_scale()
        variant = variants[obj.data.name]
        plant_rows.append(
            {
                "source_object": obj.name,
                "variant": variant["variant"],
                "mesh_file": variant["mesh_file"],
                "location_m": _round_values(location),
                "rotation_rpy_rad": _round_values(rotation),
                "scale": _round_values(scale),
            }
        )

    collision_xml = []
    collision_rows = []
    for index, name in enumerate(COLLISION_SOURCE_OBJECTS, start=1):
        xml, record = _box_collision_xml(bpy.data.objects[name], index)
        collision_xml.append(xml)
        collision_rows.append(record)

    model_dir.mkdir(parents=True, exist_ok=True)
    (model_dir / "model.sdf").write_text(
        _model_sdf(structure_mesh_name, plant_rows, collision_xml),
        encoding="utf-8",
        newline="\n",
    )
    (model_dir / "model.config").write_text(
        _model_config(),
        encoding="utf-8",
        newline="\n",
    )

    return {
        "source": str(source),
        "source_blender_version": bpy.app.version_string,
        "model_name": MODEL_NAME,
        "coordinate_contract": "metres, right-handed, Z-up",
        "source_origin_m": _round_values(WORKCELL_SOURCE_ORIGIN_M),
        "workcell_rule": (
            "source outer-row bay is shifted beside panda_link0; two combined "
            "background plants are replaced by the qualified v2 plant and fruit"
        ),
        "omitted_source_plants": sorted(OMITTED_WORKCELL_PLANTS),
        "visible_source_meshes": len(visible_meshes),
        "structure_source_objects": [obj.name for obj in structure_objects],
        "source_plant_instances": len(plant_objects),
        "exported_plant_instances": len(plant_rows),
        "unique_plant_variants": len(variants),
        "plant_decimation_ratio": PLANT_DECIMATION_RATIO,
        "material_aliases": material_aliases,
        "structure": structure_report,
        "variants": list(variants.values()),
        "plant_instances": plant_rows,
        "collisions": collision_rows,
    }


def main() -> None:
    args = _argv()
    if len(args) != 3:
        raise SystemExit(
            "usage: blender --background --python export_field_v3.py -- "
            "<field.blend> <model-output-dir> <metadata.json>"
        )
    source, model_dir, metadata_path = map(Path, args)
    report = {
        "schema_version": 1,
        "field": export_field(source, model_dir),
    }
    serialized = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    (model_dir / "export_manifest.json").write_text(
        serialized,
        encoding="utf-8",
        newline="\n",
    )
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(
        serialized,
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
