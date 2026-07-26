"""Deterministic generation of the fixed robustness matrix."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

from .models import Maturity, Scenario, ScenarioKind
from .yaml_subset import load_yaml_mapping


EXPECTED_OCCLUSION_LEVELS = 3
EXPECTED_LIGHTING_LEVELS = 3
EXPECTED_POSITIONS = 5
EXPECTED_SEEDS = 3
EXPECTED_POSITIVE_TRIALS = 135
EXPECTED_NEGATIVE_TRIALS = 30


def _validated_unique_strings(
    values: Sequence[Any], *, name: str, expected_count: int
) -> tuple[str, ...]:
    converted = tuple(str(value).strip() for value in values)
    if len(converted) != expected_count:
        raise ValueError(f"{name} must contain exactly {expected_count} values")
    if any(not value for value in converted):
        raise ValueError(f"{name} values must be non-empty")
    if len(set(converted)) != len(converted):
        raise ValueError(f"{name} values must be unique")
    return converted


@dataclass(frozen=True)
class BenchmarkSpec:
    occlusion_levels: tuple[str, ...]
    lighting_levels: tuple[str, ...]
    positions: tuple[str, ...]
    seeds: tuple[int, ...]
    negative_trials: int = EXPECTED_NEGATIVE_TRIALS
    schema_version: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "occlusion_levels",
            _validated_unique_strings(
                self.occlusion_levels,
                name="occlusion_levels",
                expected_count=EXPECTED_OCCLUSION_LEVELS,
            ),
        )
        object.__setattr__(
            self,
            "lighting_levels",
            _validated_unique_strings(
                self.lighting_levels,
                name="lighting_levels",
                expected_count=EXPECTED_LIGHTING_LEVELS,
            ),
        )
        object.__setattr__(
            self,
            "positions",
            _validated_unique_strings(
                self.positions,
                name="positions",
                expected_count=EXPECTED_POSITIONS,
            ),
        )
        converted_seeds = tuple(int(seed) for seed in self.seeds)
        if len(converted_seeds) != EXPECTED_SEEDS:
            raise ValueError(f"seeds must contain exactly {EXPECTED_SEEDS} values")
        if len(set(converted_seeds)) != len(converted_seeds):
            raise ValueError("seeds must be unique")
        object.__setattr__(self, "seeds", converted_seeds)
        if self.negative_trials != EXPECTED_NEGATIVE_TRIALS:
            raise ValueError(
                f"negative_trials must be exactly {EXPECTED_NEGATIVE_TRIALS}"
            )
        if self.schema_version != 1:
            raise ValueError(f"unsupported benchmark schema_version: {self.schema_version}")

    @property
    def positive_trials(self) -> int:
        return (
            len(self.occlusion_levels)
            * len(self.lighting_levels)
            * len(self.positions)
            * len(self.seeds)
        )


DEFAULT_SPEC = BenchmarkSpec(
    occlusion_levels=("none", "partial", "heavy"),
    lighting_levels=("dim", "nominal", "bright"),
    positions=("near_left", "near_right", "center", "far_left", "far_right"),
    seeds=(20260710, 20260711, 20260712),
)


def _slug(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.strip().lower()).strip("-")
    if not slug:
        raise ValueError(f"cannot create trial id from value: {value!r}")
    return slug


def load_benchmark_spec(path: str | Path) -> BenchmarkSpec:
    """Load and validate ``config/benchmark.yaml``."""

    raw = load_yaml_mapping(path)
    if not isinstance(raw, Mapping) or not isinstance(raw.get("benchmark"), Mapping):
        raise ValueError("benchmark config must contain a 'benchmark' mapping")
    config = raw["benchmark"]
    required = ("occlusion_levels", "lighting_levels", "positions", "seeds")
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError(f"benchmark config is missing: {', '.join(missing)}")
    return BenchmarkSpec(
        schema_version=int(config.get("schema_version", 1)),
        occlusion_levels=tuple(config["occlusion_levels"]),
        lighting_levels=tuple(config["lighting_levels"]),
        positions=tuple(config["positions"]),
        seeds=tuple(config["seeds"]),
        negative_trials=int(
            config.get("negative_trials", EXPECTED_NEGATIVE_TRIALS)
        ),
    )


def generate_scenarios(spec: BenchmarkSpec = DEFAULT_SPEC) -> list[Scenario]:
    """Return 135 positive and 30 balanced negative scenarios.

    Positive scenarios are the full Cartesian product.  Negative scenarios use
    a deterministic cyclic design with balanced one-dimensional marginals:
    each occlusion, lighting, and seed occurs ten times, and each position six
    times.  Every negative world contains only unripe fruit.
    """

    scenarios: list[Scenario] = []
    for occlusion, lighting, position, seed in product(
        spec.occlusion_levels,
        spec.lighting_levels,
        spec.positions,
        spec.seeds,
    ):
        trial_id = (
            f"positive__occ-{_slug(occlusion)}__light-{_slug(lighting)}"
            f"__pos-{_slug(position)}__seed-{seed}"
        )
        scenarios.append(
            Scenario(
                trial_id=trial_id,
                kind=ScenarioKind.POSITIVE,
                occlusion=occlusion,
                lighting=lighting,
                position=position,
                seed=seed,
                expected_maturity=Maturity.RIPE,
            )
        )

    for index in range(spec.negative_trials):
        group, within_group = divmod(index, len(spec.occlusion_levels))
        occlusion = spec.occlusion_levels[within_group]
        lighting = spec.lighting_levels[
            (group + 2 * within_group) % len(spec.lighting_levels)
        ]
        position = spec.positions[index % len(spec.positions)]
        seed = spec.seeds[(group + within_group) % len(spec.seeds)]
        trial_id = (
            f"negative-{index + 1:03d}__occ-{_slug(occlusion)}"
            f"__light-{_slug(lighting)}__pos-{_slug(position)}__seed-{seed}"
        )
        scenarios.append(
            Scenario(
                trial_id=trial_id,
                kind=ScenarioKind.NEGATIVE,
                occlusion=occlusion,
                lighting=lighting,
                position=position,
                seed=seed,
                expected_maturity=Maturity.UNRIPE,
            )
        )

    if spec.positive_trials != EXPECTED_POSITIVE_TRIALS:
        raise AssertionError("validated benchmark spec produced the wrong matrix size")
    if len(scenarios) != EXPECTED_POSITIVE_TRIALS + EXPECTED_NEGATIVE_TRIALS:
        raise AssertionError("benchmark generator produced the wrong trial count")
    if len({scenario.trial_id for scenario in scenarios}) != len(scenarios):
        raise ValueError("benchmark values produce duplicate trial ids")
    return scenarios
