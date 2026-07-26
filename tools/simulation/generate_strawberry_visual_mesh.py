#!/usr/bin/env python3
"""Generate one deterministic UV-mapped strawberry-shaped visual OBJ."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def _radius_profile(t: float) -> float:
    """Return a crown-heavy, pointed strawberry profile for t in [0, 1]."""

    if t <= 0.0 or t >= 1.0:
        return 0.0
    roundness = math.sin(math.pi * t) ** 0.72
    crown_bias = 0.82 + 0.34 * (1.0 - t)
    return roundness * crown_bias


def generate_obj(*, rings: int = 24, segments: int = 48) -> str:
    if rings < 4 or segments < 8:
        raise ValueError("mesh requires at least 4 rings and 8 segments")

    raw_profiles = [_radius_profile(index / rings) for index in range(1, rings)]
    profile_scale = 0.034 / max(raw_profiles)
    lines = [
        "# Deterministic visual-only strawberry mesh.",
        "# Collision remains the frozen 0.035 m sphere in the SDF model.",
        "o strawberry_body",
        "mtllib strawberry_body_v1.mtl",
        "usemtl strawberry_skin",
    ]

    # Top pole.
    lines.extend(("v 0.000000000 0.000000000 0.035000000", "vt 0.500000000 0.000000000"))
    top_index = 1
    vertex_count = 1

    ring_starts: list[int] = []
    for ring in range(1, rings):
        t = ring / rings
        z = 0.035 * math.cos(math.pi * t)
        radius = raw_profiles[ring - 1] * profile_scale
        ring_starts.append(vertex_count + 1)
        for segment in range(segments + 1):
            u = segment / segments
            angle = 2.0 * math.pi * u
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            lines.append(f"v {x:.9f} {y:.9f} {z:.9f}")
            lines.append(f"vt {u:.9f} {t:.9f}")
            vertex_count += 1

    # Bottom pole.
    bottom_index = vertex_count + 1
    lines.extend(("v 0.000000000 0.000000000 -0.035000000", "vt 0.500000000 1.000000000"))

    # Vertex and texture coordinate indices are intentionally identical.
    first_ring = ring_starts[0]
    for segment in range(segments):
        left = first_ring + segment
        right = first_ring + segment + 1
        lines.append(f"f {top_index}/{top_index} {right}/{right} {left}/{left}")

    for upper_start, lower_start in zip(ring_starts, ring_starts[1:]):
        for segment in range(segments):
            upper_left = upper_start + segment
            upper_right = upper_left + 1
            lower_left = lower_start + segment
            lower_right = lower_left + 1
            lines.append(
                f"f {upper_left}/{upper_left} {upper_right}/{upper_right} "
                f"{lower_right}/{lower_right} {lower_left}/{lower_left}"
            )

    last_ring = ring_starts[-1]
    for segment in range(segments):
        left = last_ring + segment
        right = last_ring + segment + 1
        lines.append(f"f {left}/{left} {right}/{right} {bottom_index}/{bottom_index}")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rings", type=int, default=24)
    parser.add_argument("--segments", type=int, default=48)
    options = parser.parse_args()
    if options.output.exists():
        raise SystemExit(f"refusing to overwrite {options.output}")
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(
        generate_obj(rings=options.rings, segments=options.segments),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
