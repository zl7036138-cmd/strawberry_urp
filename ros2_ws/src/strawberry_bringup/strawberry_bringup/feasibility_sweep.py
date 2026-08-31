"""Pure schedule and receipt logic for generalized feasibility sweeps."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence


def validate_development_matrix(
    config: Mapping[str, object], *, formal_seeds: Sequence[int]
) -> None:
    schema_version = int(config.get("schema_version", 0))
    if schema_version not in {1, 2}:
        raise ValueError("unsupported development matrix schema")
    if config.get("formal_acceptance") is not False or config.get(
        "formal_results_consumed"
    ) is not False:
        raise ValueError("development matrix must not claim formal results")
    discovery = config.get("discovery")
    qualification = config.get("qualification")
    if not isinstance(discovery, Mapping) or not isinstance(qualification, Mapping):
        raise ValueError("development matrix splits are missing")
    discovery_count = int(
        discovery.get(
            "scenario_count_per_batch", discovery.get("scenario_count", 0)
        )
    )
    if discovery_count != 18 or int(
        qualification.get("scenario_count_per_batch", 0)
    ) != 18:
        raise ValueError("each feasibility sweep must contain 18 scenarios")
    if int(qualification.get("selected_runtime_scenarios", 0)) != 5:
        raise ValueError("qualification must select exactly five runtime scenes")
    if schema_version == 2:
        schedule = config.get("schedule")
        generation = config.get("generation_contract")
        if not isinstance(schedule, Mapping) or schedule.get(
            "layout_contract"
        ) != "multi_pick_v2":
            raise ValueError("schema v2 requires the multi_pick_v2 layout contract")
        if not isinstance(generation, Mapping) or int(
            generation.get("minimum_primary_ripe_by_construction", 0)
        ) != 2:
            raise ValueError("schema v2 must require two primary ripe spawn candidates")
        if generation.get("moveit_feasibility_guaranteed_by_generator") is not False:
            raise ValueError("scene generation may not claim MoveIt feasibility")
    all_formal = {int(seed) for seed in formal_seeds}
    for split in ("discovery", "qualification"):
        for row in scenario_rows(config, split=split, batch_index=0):
            if row["seed"] in all_formal:
                raise ValueError("development seed overlaps the formal matrix")


def scenario_rows(
    config: Mapping[str, object], *, split: str, batch_index: int = 0
) -> tuple[dict[str, object], ...]:
    if split not in {"discovery", "qualification"}:
        raise ValueError("split must be discovery or qualification")
    if isinstance(batch_index, bool) or int(batch_index) < 0:
        raise ValueError("batch index must be non-negative")
    schedule = config["schedule"]
    bands = tuple(schedule["position_bands"])
    occlusions = tuple(schedule["occlusions"])
    plant_counts = tuple(int(value) for value in schedule["plant_counts_by_repeat"])
    if bands != ("near", "middle", "far") or occlusions != (
        "none",
        "partial",
        "heavy",
    ) or plant_counts != (2, 3):
        raise ValueError("development schedule must remain the balanced 18-scene grid")
    schema_version = int(config.get("schema_version", 0))
    if split == "discovery":
        discovery = config["discovery"]
        if schema_version == 1:
            if int(batch_index) != 0:
                raise ValueError("schema v1 discovery has one fixed seed block")
            start = int(discovery["seed_start"])
        else:
            start = int(discovery["initial_seed_start"]) + int(batch_index) * int(
                discovery["seed_block_stride"]
            )
    else:
        qualification = config["qualification"]
        start = int(qualification["initial_seed_start"]) + int(batch_index) * int(
            qualification["seed_block_stride"]
        )
    rows = []
    index = 0
    for plant_count in plant_counts:
        for band in bands:
            for occlusion in occlusions:
                seed = start + index
                rows.append(
                    {
                        "scenario_id": f"{split}_b{int(batch_index):02d}_{seed}",
                        "seed": seed,
                        "profile": str(schedule["profile"]),
                        "plant_count": plant_count,
                        "position_band": band,
                        "occlusion": occlusion,
                        "layout_contract": str(
                            schedule.get("layout_contract", "legacy_random_v1")
                        ),
                    }
                )
                index += 1
    return tuple(rows)


def select_runtime_scenarios(
    rows: Sequence[Mapping[str, object]],
    receipts: Mapping[str, Mapping[str, object]],
    *,
    count: int = 5,
) -> tuple[str, ...]:
    """Select the first eligible receipts in frozen matrix order."""

    if count <= 0:
        raise ValueError("selection count must be positive")
    selected = []
    for row in rows:
        scenario_id = str(row["scenario_id"])
        receipt = receipts.get(scenario_id)
        if receipt is None:
            continue
        if (
            receipt.get("eligible_for_multi_fruit_runtime") is True
            and receipt.get("runtime_truth_use") is False
            and receipt.get("trajectory_execution_allowed") is False
        ):
            selected.append(scenario_id)
            if len(selected) == count:
                break
    return tuple(selected)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def receipt_file(path: Path, root: Path) -> dict[str, object]:
    return {
        "path": path.resolve().relative_to(root.resolve()).as_posix(),
        "sha256": sha256_file(path),
        "size_bytes": path.stat().st_size,
    }


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload
