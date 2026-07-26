"""Print a machine-readable summary of the currently opened Blender scene."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def _argv() -> list[str]:
    if "--" not in sys.argv:
        return []
    return sys.argv[sys.argv.index("--") + 1 :]


def _rounded(values, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values]


def _world_bounds(obj: bpy.types.Object) -> tuple[list[float], list[float]] | None:
    if not getattr(obj, "bound_box", None):
        return None
    corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    return (
        _rounded(min(corner[index] for corner in corners) for index in range(3)),
        _rounded(max(corner[index] for corner in corners) for index in range(3)),
    )


def _material_summary(material: bpy.types.Material) -> dict:
    summary = {
        "name": material.name,
        "use_nodes": bool(material.use_nodes),
        "diffuse_color": _rounded(material.diffuse_color),
        "blend_method": getattr(material, "surface_render_method", None),
        "images": [],
    }
    if material.use_nodes and material.node_tree:
        image_names = []
        for node in material.node_tree.nodes:
            image = getattr(node, "image", None)
            if image is not None:
                image_names.append(image.name)
        summary["images"] = sorted(set(image_names))
    return summary


def main() -> None:
    args = _argv()
    output_path = Path(args[0]) if args else None
    depsgraph = bpy.context.evaluated_depsgraph_get()
    objects = []
    scene_min = [float("inf")] * 3
    scene_max = [float("-inf")] * 3

    for obj in sorted(bpy.data.objects, key=lambda item: item.name):
        bounds = _world_bounds(obj)
        if bounds:
            for index in range(3):
                scene_min[index] = min(scene_min[index], bounds[0][index])
                scene_max[index] = max(scene_max[index], bounds[1][index])

        mesh_stats = None
        if obj.type == "MESH":
            evaluated = obj.evaluated_get(depsgraph)
            mesh = evaluated.to_mesh()
            mesh.calc_loop_triangles()
            mesh_stats = {
                "vertices": len(mesh.vertices),
                "polygons": len(mesh.polygons),
                "triangles": len(mesh.loop_triangles),
            }
            evaluated.to_mesh_clear()

        objects.append(
            {
                "name": obj.name,
                "type": obj.type,
                "parent": obj.parent.name if obj.parent else None,
                "collections": sorted(collection.name for collection in obj.users_collection),
                "visible_render": not bool(obj.hide_render),
                "visible_viewport": not bool(obj.hide_get()),
                "location": _rounded(obj.location),
                "rotation_euler": _rounded(obj.rotation_euler),
                "scale": _rounded(obj.scale),
                "world_bounds": bounds,
                "mesh": mesh_stats,
                "materials": [
                    slot.material.name
                    for slot in obj.material_slots
                    if slot.material is not None
                ],
                "custom_properties": {
                    key: obj[key]
                    for key in obj.keys()
                    if key != "_RNA_UI"
                    and isinstance(obj[key], (str, int, float, bool))
                },
            }
        )

    finite_bounds = all(value != float("inf") for value in scene_min) and all(
        value != float("-inf") for value in scene_max
    )
    report = {
        "blend_file": bpy.data.filepath,
        "blender_version": bpy.app.version_string,
        "unit_settings": {
            "system": bpy.context.scene.unit_settings.system,
            "scale_length": bpy.context.scene.unit_settings.scale_length,
            "length_unit": bpy.context.scene.unit_settings.length_unit,
        },
        "scene_bounds": (
            {"min": _rounded(scene_min), "max": _rounded(scene_max)}
            if finite_bounds
            else None
        ),
        "collections": [
            {
                "name": collection.name,
                "objects": sorted(obj.name for obj in collection.objects),
            }
            for collection in sorted(bpy.data.collections, key=lambda item: item.name)
        ],
        "objects": objects,
        "materials": [
            _material_summary(material)
            for material in sorted(bpy.data.materials, key=lambda item: item.name)
        ],
        "images": [
            {
                "name": image.name,
                "filepath": image.filepath,
                "packed": image.packed_file is not None,
                "size": list(image.size),
            }
            for image in sorted(bpy.data.images, key=lambda item: item.name)
        ],
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2, default=str)
    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
