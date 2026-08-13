"""Deterministic multi-plant scene generation for fixed-base harvesting.

The generator owns simulation truth, but the generated truth is intentionally
kept outside the runtime perception/control interfaces.  A seed produces both
the Gazebo world and the matching scene manifest used by the simulator's
ground-truth and collision bookkeeping nodes.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import random
import re
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET

import yaml


PLANT_COUNT_RANGE = (1, 3)
FRUIT_COUNT_RANGE = (2, 3)
PLANT_X_RANGE_M = (0.35, 0.68)
PLANT_Y_RANGE_M = (-0.21, 0.21)
PLANT_Z_M = 0.47
MIN_PLANT_SEPARATION_M = 0.25
# Blender-v2 uses a 26 mm fruit collision radius.  A 54 mm centre distance
# preserves a small initial gap without rejecting adjacent authored pedicels.
MIN_FRUIT_SEPARATION_M = 0.054
MAX_FRUIT_SCALE = 1.08
MIN_INITIAL_STATIC_COLLISION_GAP_M = 0.002
CONSERVATIVE_REACH_BOUNDS_M = {
    "min_x": 0.30,
    "max_x": 0.72,
    "min_y": -0.34,
    "max_y": 0.34,
    "min_z": 0.48,
    "max_z": 0.66,
}
# Keep the construction-time reachability label aligned with the runtime
# selector: 50 mm of static-obstacle padding plus the unchanged 20 mm target
# clearance gate.  A fruit may still be generated near the bin as an explicit
# unreachable target, but it must never be counted as safely reachable.
MIN_STATIC_OBSTACLE_CENTER_CLEARANCE_M = 0.070
BIN_INTERIOR_TO_OUTER_PADDING_M = (0.05, 0.05, 0.05, 0.05, 0.03, 0.0)

# Authored pedicel endpoints and fruit orientations, expressed relative to the
# canonical Blender-v2 plant origin at [0.50, 0.0, 0.47].
FRUIT_SLOTS = (
    (
        (-0.080935247, -0.053479813, 0.076111838),
        (-0.069813050, -0.087266445, 0.733038306),
    ),
    (
        (0.050333202, 0.118588343, 0.062157422),
        (-0.104719654, 0.087266445, -0.418878973),
    ),
    ((0.078895651, 0.068192676, 0.070231110), (0.087266490, -0.122173049, 0.314159304)),
)


def _distance_xy(left: Sequence[float], right: Sequence[float]) -> float:
    return math.hypot(
        float(left[0]) - float(right[0]), float(left[1]) - float(right[1])
    )


def _distance_xyz(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


def _inside_reach(position: Sequence[float]) -> bool:
    x, y, z = (float(value) for value in position)
    bounds = CONSERVATIVE_REACH_BOUNDS_M
    return (
        bounds["min_x"] <= x <= bounds["max_x"]
        and bounds["min_y"] <= y <= bounds["max_y"]
        and bounds["min_z"] <= z <= bounds["max_z"]
    )


def _axis_aligned_box_clearance(
    position: Sequence[float], bounds: Sequence[float]
) -> float:
    if len(position) != 3 or len(bounds) != 6:
        raise ValueError("position and bounds must contain three and six values")
    min_x, max_x, min_y, max_y, min_z, max_z = (float(value) for value in bounds)
    if min_x >= max_x or min_y >= max_y or min_z >= max_z:
        raise ValueError("axis-aligned bounds are invalid")
    x, y, z = (float(value) for value in position)
    deltas = (
        max(min_x - x, 0.0, x - max_x),
        max(min_y - y, 0.0, y - max_y),
        max(min_z - z, 0.0, z - max_z),
    )
    return math.sqrt(sum(value * value for value in deltas))


def _bin_exclusion_bounds(scene: Mapping[str, Any]) -> tuple[float, ...]:
    try:
        interior = scene["bin"]["interior_bounds_m"]
        values = (
            float(interior["min_x"]),
            float(interior["max_x"]),
            float(interior["min_y"]),
            float(interior["max_y"]),
            float(interior["min_z"]),
            float(interior["max_z"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("scene bin interior bounds are incomplete") from exc
    padding = BIN_INTERIOR_TO_OUTER_PADDING_M
    return (
        values[0] - padding[0],
        values[1] + padding[1],
        values[2] - padding[2],
        values[3] + padding[3],
        values[4] - padding[4],
        values[5] + padding[5],
    )


def _reachable_by_construction(
    position: Sequence[float], bin_exclusion_bounds: Sequence[float]
) -> bool:
    return (
        _inside_reach(position)
        and _axis_aligned_box_clearance(position, bin_exclusion_bounds)
        >= MIN_STATIC_OBSTACLE_CENTER_CLEARANCE_M
    )


def _sample_plant_centres(
    rng: random.Random, count: int
) -> list[tuple[float, float, float]]:
    # A jittered triangular layout guarantees space for the authored fruit
    # slots while still varying every plant transform between seeds.
    # The bin's back wall occupies y=-0.24..-0.22.  Centring the nearest plant
    # at y=-0.10 leaves every authored fruit slot outside the wall even after
    # the bounded pose jitter and maximum fruit scale are applied.
    anchors = [(0.40, -0.09), (0.67, 0.055), (0.40, 0.20)]
    if rng.random() < 0.5:
        # Reflect about y=0.055 instead of y=0 so the randomized layout keeps
        # the same bin clearance on both variants.
        anchors = [(x, 0.11 - y) for x, y in anchors]
    rng.shuffle(anchors)
    centres = [
        (
            min(
                max(x + rng.uniform(-0.006, 0.006), PLANT_X_RANGE_M[0]),
                PLANT_X_RANGE_M[1],
            ),
            min(
                max(y + rng.uniform(-0.006, 0.006), PLANT_Y_RANGE_M[0]),
                PLANT_Y_RANGE_M[1],
            ),
            PLANT_Z_M,
        )
        for x, y in anchors[:count]
    ]
    if any(
        _distance_xy(centre, previous) < MIN_PLANT_SEPARATION_M
        for index, centre in enumerate(centres)
        for previous in centres[:index]
    ):
        raise RuntimeError("unable to place non-overlapping plant instances")
    return centres


def _maturities(rng: random.Random, total: int, profile: str) -> list[str]:
    if profile == "all_unripe":
        return ["UNRIPE"] * total
    values = ["RIPE"] * min(2, total) + ["UNRIPE"]
    while len(values) < total:
        values.append("RIPE" if rng.random() < 0.60 else "UNRIPE")
    rng.shuffle(values)
    return values[:total]


def generate_scene(
    base_scene: Mapping[str, Any],
    *,
    seed: int,
    profile: str = "mixed",
    plant_count: int | None = None,
    occlusion: str | None = None,
    position_band: str = "middle",
) -> dict[str, Any]:
    """Return one validated, deterministic scene manifest.

    ``profile`` is one of ``mixed``, ``all_unripe``, or ``unsafe``.  Unsafe
    scenes contain a ripe fruit deliberately outside the conservative control
    envelope so the fail-closed selector can be exercised.
    """

    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a non-negative integer")
    if profile not in {"mixed", "all_unripe", "unsafe"}:
        raise ValueError("profile must be mixed, all_unripe, or unsafe")
    if (
        plant_count is not None
        and not PLANT_COUNT_RANGE[0] <= plant_count <= PLANT_COUNT_RANGE[1]
    ):
        raise ValueError("plant_count must be between 1 and 3")
    if occlusion not in {None, "none", "partial", "heavy"}:
        raise ValueError("occlusion must be none, partial, or heavy")
    if position_band not in {"near", "middle", "far"}:
        raise ValueError("position_band must be near, middle, or far")

    rng = random.Random(seed)
    bin_exclusion_bounds = _bin_exclusion_bounds(base_scene)
    fruit_collision_radius_m = float(base_scene.get("fruit_collision_radius_m", 0.0))
    if not math.isfinite(fruit_collision_radius_m) or fruit_collision_radius_m <= 0.0:
        raise ValueError("fruit collision radius must be positive")
    count = plant_count if plant_count is not None else rng.randint(*PLANT_COUNT_RANGE)
    centres = _sample_plant_centres(rng, count)
    x_shift = {"near": -0.035, "middle": 0.0, "far": 0.035}[position_band]
    centres = [
        (
            min(max(center[0] + x_shift, PLANT_X_RANGE_M[0]), PLANT_X_RANGE_M[1]),
            center[1],
            center[2],
        )
        for center in centres
    ]
    plants: list[dict[str, Any]] = []
    fruit_drafts: list[dict[str, Any]] = []
    positions: list[tuple[float, float, float]] = []

    for plant_index, centre in enumerate(centres, start=1):
        yaw = rng.uniform(-0.45, 0.45)
        plants.append(
            {
                "plant_id": plant_index,
                "model_name": f"strawberry_plant_{plant_index}",
                "asset": "strawberry_plant_v2",
                "pose_in_robot_base": [*centre, 0.0, 0.0, yaw],
            }
        )
        fruit_count = (
            FRUIT_COUNT_RANGE[1]
            if profile == "mixed" and count == 1
            else rng.randint(*FRUIT_COUNT_RANGE)
        )
        slot_indices = list(range(len(FRUIT_SLOTS)))
        rng.shuffle(slot_indices)
        for slot_index in slot_indices[:fruit_count]:
            offset, orientation = FRUIT_SLOTS[slot_index]
            rotated_x, rotated_y = _rotate_xy(offset[0], offset[1], yaw)
            scale = round(rng.uniform(0.92, MAX_FRUIT_SCALE), 6)
            required_bin_clearance = (
                fruit_collision_radius_m * scale + MIN_INITIAL_STATIC_COLLISION_GAP_M
            )
            # Small perturbations exercise localization without detaching the
            # visual fruit from its authored pedicel neighbourhood.
            position = None
            for _attempt in range(50):
                candidate = (
                    centre[0] + rotated_x + rng.uniform(-0.008, 0.008),
                    centre[1] + rotated_y + rng.uniform(-0.008, 0.008),
                    centre[2] + offset[2] + rng.uniform(-0.006, 0.006),
                )
                if (
                    all(
                        _distance_xyz(candidate, previous) >= MIN_FRUIT_SEPARATION_M
                        for previous in positions
                    )
                    and _axis_aligned_box_clearance(candidate, bin_exclusion_bounds)
                    >= required_bin_clearance
                ):
                    position = candidate
                    break
            if position is None:
                # Reusing another authored slot is safer than accepting an
                # initial overlap.  The bounded generator fails if no slot is
                # geometrically valid for this plant placement.
                continue
            positions.append(position)
            fruit_drafts.append(
                {
                    "plant_id": plant_index,
                    "slot_id": slot_index + 1,
                    "position": position,
                    "orientation": (
                        orientation[0],
                        orientation[1],
                        orientation[2] + yaw,
                    ),
                    "scale": scale,
                }
            )

    if len(fruit_drafts) < count * FRUIT_COUNT_RANGE[0]:
        raise RuntimeError(
            "generated plants do not have enough collision-free fruit slots"
        )

    maturity_values = _maturities(rng, len(fruit_drafts), profile)
    fruits: list[dict[str, Any]] = []
    for target_id, (draft, maturity) in enumerate(
        zip(fruit_drafts, maturity_values), start=1
    ):
        position = list(draft["position"])
        if profile == "unsafe" and target_id == 1:
            maturity = "RIPE"
            position[0] = 0.74 + rng.uniform(0.0, 0.03)
            position[1] = rng.choice((-1.0, 1.0)) * rng.uniform(0.32, 0.38)
        reachable = _reachable_by_construction(position, bin_exclusion_bounds)
        asset = (
            "strawberry_ripe_generalized"
            if maturity == "RIPE"
            else "strawberry_unripe_generalized"
        )
        fruits.append(
            {
                "target_id": target_id,
                "plant_id": int(draft["plant_id"]),
                "slot_id": int(draft["slot_id"]),
                "model_name": f"strawberry_{target_id}",
                "maturity": maturity,
                "asset": asset,
                "scale": float(draft["scale"]),
                "initial_pose_m": [float(value) for value in position],
                "initial_rpy_rad": [float(value) for value in draft["orientation"]],
                "reachable_by_construction": reachable,
            }
        )

    result = deepcopy(dict(base_scene))
    selected_occlusion = occlusion or rng.choice(("none", "partial", "heavy"))
    light_level = rng.choice(("nominal", "dim"))
    result["schema_version"] = 1
    result["generator"] = {
        "schema_version": 1,
        "seed": seed,
        "profile": profile,
        "position_band": position_band,
        "truth_for_runtime_control": False,
        "released_fruit_physics": "gravity",
        "plant_count_range": list(PLANT_COUNT_RANGE),
        "fruit_count_per_plant_range": list(FRUIT_COUNT_RANGE),
        "conservative_reach_bounds_m": deepcopy(CONSERVATIVE_REACH_BOUNDS_M),
        "bin_exclusion_bounds_m": list(bin_exclusion_bounds),
        "minimum_static_obstacle_center_clearance_m": (
            MIN_STATIC_OBSTACLE_CENTER_CLEARANCE_M
        ),
        "minimum_initial_static_collision_gap_m": (MIN_INITIAL_STATIC_COLLISION_GAP_M),
    }
    result["condition"] = {"occlusion": selected_occlusion, "lighting": light_level}
    result["plants"] = plants
    result["plant"] = plants[0]
    result["fruits"] = fruits
    result["evaluation"] = {
        "ground_truth_only": True,
        "ripe_target_ids": [
            row["target_id"] for row in fruits if row["maturity"] == "RIPE"
        ],
        "reachable_ripe_target_ids": [
            row["target_id"]
            for row in fruits
            if row["maturity"] == "RIPE" and row["reachable_by_construction"]
        ],
        "unreachable_ripe_target_ids": [
            row["target_id"]
            for row in fruits
            if row["maturity"] == "RIPE" and not row["reachable_by_construction"]
        ],
    }
    validate_generated_scene(result)
    return result


def validate_generated_scene(scene: Mapping[str, Any]) -> None:
    """Fail closed on naming, count, overlap, or profile contract drift."""

    generator = scene.get("generator")
    plants = scene.get("plants")
    fruits = scene.get("fruits")
    if (
        not isinstance(generator, Mapping)
        or not isinstance(plants, list)
        or not isinstance(fruits, list)
    ):
        raise ValueError("generated scene is missing generator, plants, or fruits")
    if not PLANT_COUNT_RANGE[0] <= len(plants) <= PLANT_COUNT_RANGE[1]:
        raise ValueError("generated scene plant count is outside the frozen range")
    plant_ids = [int(row["plant_id"]) for row in plants]
    if len(plant_ids) != len(set(plant_ids)):
        raise ValueError("plant IDs must be unique")
    centres = [row["pose_in_robot_base"][:3] for row in plants]
    for index, centre in enumerate(centres):
        if any(
            _distance_xy(centre, other) < MIN_PLANT_SEPARATION_M
            for other in centres[:index]
        ):
            raise ValueError("plant instances overlap")
    target_ids = [int(row["target_id"]) for row in fruits]
    if target_ids != list(range(1, len(fruits) + 1)):
        raise ValueError("target IDs must be contiguous and deterministic")
    model_names = [str(row["model_name"]) for row in fruits]
    if len(model_names) != len(set(model_names)):
        raise ValueError("fruit model names must be unique")
    counts = {plant_id: 0 for plant_id in plant_ids}
    fruit_positions: list[Sequence[float]] = []
    for row in fruits:
        plant_id = int(row["plant_id"])
        if plant_id not in counts:
            raise ValueError("fruit references an absent plant")
        counts[plant_id] += 1
        if row["maturity"] not in {"RIPE", "UNRIPE"}:
            raise ValueError("fruit maturity must be RIPE or UNRIPE")
        expected_asset = (
            "strawberry_ripe_generalized"
            if row["maturity"] == "RIPE"
            else "strawberry_unripe_generalized"
        )
        if row.get("asset") != expected_asset:
            raise ValueError("generalized fruit asset must enable release gravity")
        position = row["initial_pose_m"]
        if any(
            _distance_xyz(position, previous) < MIN_FRUIT_SEPARATION_M
            for previous in fruit_positions
        ):
            raise ValueError("fruit collision spheres overlap")
        fruit_positions.append(position)
    if any(
        not FRUIT_COUNT_RANGE[0] <= value <= FRUIT_COUNT_RANGE[1]
        for value in counts.values()
    ):
        raise ValueError("fruit count per plant is outside the frozen range")
    profile = str(generator.get("profile"))
    if generator.get("released_fruit_physics") != "gravity":
        raise ValueError("generalized release physics must use gravity")
    expected_bin_bounds = _bin_exclusion_bounds(scene)
    recorded_bin_bounds = generator.get("bin_exclusion_bounds_m")
    if not isinstance(recorded_bin_bounds, list) or len(recorded_bin_bounds) != 6:
        raise ValueError("generator bin exclusion bounds are missing")
    if any(
        not math.isclose(float(actual), expected, abs_tol=1e-9)
        for actual, expected in zip(recorded_bin_bounds, expected_bin_bounds)
    ):
        raise ValueError("generator bin exclusion bounds drifted from the scene")
    recorded_clearance = float(
        generator.get("minimum_static_obstacle_center_clearance_m", -1.0)
    )
    if not math.isclose(
        recorded_clearance,
        MIN_STATIC_OBSTACLE_CENTER_CLEARANCE_M,
        abs_tol=1e-9,
    ):
        raise ValueError("generator static-obstacle clearance threshold drifted")
    for row in fruits:
        required_bin_clearance = (
            float(scene.get("fruit_collision_radius_m", 0.0))
            * float(row.get("scale", 1.0))
            + MIN_INITIAL_STATIC_COLLISION_GAP_M
        )
        if (
            _axis_aligned_box_clearance(row["initial_pose_m"], expected_bin_bounds)
            < required_bin_clearance
        ):
            raise ValueError("fruit initially collides with the collection bin")
        expected_reachable = _reachable_by_construction(
            row["initial_pose_m"], expected_bin_bounds
        )
        if bool(row.get("reachable_by_construction")) != expected_reachable:
            raise ValueError(
                "fruit reachability label disagrees with reach and bin clearance"
            )
    if profile == "mixed":
        if sum(row["maturity"] == "RIPE" for row in fruits) < 2 or not any(
            row["maturity"] == "UNRIPE" for row in fruits
        ):
            raise ValueError(
                "mixed scenes require multiple ripe fruit and one unripe fruit"
            )
    elif profile == "all_unripe" and any(row["maturity"] != "UNRIPE" for row in fruits):
        raise ValueError("all_unripe scene contains a ripe fruit")
    elif profile == "unsafe" and not any(
        row["maturity"] == "RIPE" and not bool(row["reachable_by_construction"])
        for row in fruits
    ):
        raise ValueError("unsafe scene lacks an unreachable ripe fruit")


def _include(
    name: str, uri: str, pose: Sequence[float], scale: float | None = None
) -> ET.Element:
    include = ET.Element("include")
    ET.SubElement(include, "uri").text = f"model://{uri}"
    ET.SubElement(include, "name").text = name
    ET.SubElement(include, "pose").text = " ".join(
        f"{float(value):.9f}" for value in pose
    )
    if scale is not None:
        ET.SubElement(include, "scale").text = f"{scale:.6f} {scale:.6f} {scale:.6f}"
    return include


def _occluder(name: str, pose: Sequence[float], size: Sequence[float]) -> ET.Element:
    model = ET.Element("model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = " ".join(
        f"{float(value):.9f}" for value in pose
    )
    link = ET.SubElement(model, "link", {"name": "visual_only_link"})
    visual = ET.SubElement(link, "visual", {"name": "foliage_occluder"})
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = " ".join(f"{float(value):.6f}" for value in size)
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0.08 0.30 0.06 0.90"
    ET.SubElement(material, "diffuse").text = "0.10 0.42 0.08 0.90"
    return model


def materialize_world(
    base_world: str | Path, scene: Mapping[str, Any]
) -> ET.ElementTree:
    """Materialize plants, fruit, light, and visual-only occlusion in SDF."""

    validate_generated_scene(scene)
    tree = ET.parse(base_world)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("base SDF has no world element")
    for include in list(world.findall("include")):
        name = (include.findtext("name") or "").strip()
        if name == "strawberry_plant" or re.fullmatch(r"strawberry_\d+", name):
            world.remove(include)
    for model in list(world.findall("model")):
        if str(model.get("name", "")).startswith("generalization_occluder_"):
            world.remove(model)

    for plant in scene["plants"]:
        world.append(
            _include(plant["model_name"], plant["asset"], plant["pose_in_robot_base"])
        )
    for fruit in scene["fruits"]:
        pose = [*fruit["initial_pose_m"], *fruit["initial_rpy_rad"]]
        world.append(
            _include(fruit["model_name"], fruit["asset"], pose, float(fruit["scale"]))
        )

    condition = scene["condition"]
    sun = world.find("./light[@name='sun']/diffuse")
    if sun is not None:
        sun.text = (
            "0.48 0.48 0.48 1" if condition["lighting"] == "dim" else "0.90 0.90 0.90 1"
        )
    if condition["occlusion"] != "none":
        first = scene["plants"][0]["pose_in_robot_base"]
        heavy = condition["occlusion"] == "heavy"
        world.append(
            _occluder(
                "generalization_occluder_1",
                (first[0] - 0.04, first[1] - 0.08, 0.61, 0.0, 0.0, 0.2),
                (0.12 if heavy else 0.07, 0.025, 0.18 if heavy else 0.11),
            )
        )
    ET.indent(tree, space="  ")
    return tree


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_new(path: Path, content: str, *, force: bool) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def main(args: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", type=Path, required=True)
    parser.add_argument("--base-world", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument(
        "--profile", choices=("mixed", "all_unripe", "unsafe"), default="mixed"
    )
    parser.add_argument("--plant-count", type=int)
    parser.add_argument("--occlusion", choices=("none", "partial", "heavy"))
    parser.add_argument(
        "--position-band", choices=("near", "middle", "far"), default="middle"
    )
    parser.add_argument("--force", action="store_true")
    options = parser.parse_args(args)

    base_scene = yaml.safe_load(options.base_scene.read_text(encoding="utf-8"))
    scene = generate_scene(
        base_scene,
        seed=options.seed,
        profile=options.profile,
        plant_count=options.plant_count,
        occlusion=options.occlusion,
        position_band=options.position_band,
    )
    stem = f"generalized_seed_{options.seed:06d}"
    scene_path = options.output_dir / f"{stem}.yaml"
    world_path = options.output_dir / f"{stem}.sdf"
    receipt_path = options.output_dir / f"{stem}.receipt.json"
    _write_new(
        scene_path,
        yaml.safe_dump(scene, sort_keys=False, allow_unicode=True),
        force=options.force,
    )
    tree = materialize_world(options.base_world, scene)
    if world_path.exists() and not options.force:
        raise FileExistsError(f"refusing to overwrite {world_path}")
    world_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(world_path, encoding="utf-8", xml_declaration=True)
    receipt = {
        "schema_version": 1,
        "seed": options.seed,
        "profile": options.profile,
        "truth_for_runtime_control": False,
        "scene": {"path": str(scene_path), "sha256": _sha256(scene_path)},
        "world": {"path": str(world_path), "sha256": _sha256(world_path)},
    }
    _write_new(
        receipt_path,
        json.dumps(receipt, indent=2, sort_keys=True) + "\n",
        force=options.force,
    )
    print(json.dumps(receipt, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
