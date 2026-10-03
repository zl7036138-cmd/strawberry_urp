"""Dependency-light OBJ face-winding inspection and repair."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class WindingStats:
    object_name: str
    triangle_count: int
    outward_triangles: int
    inward_triangles: int
    degenerate_triangles: int
    signed_sum: float


def _subtract(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float]:
    return tuple(float(left[index]) - float(right[index]) for index in range(3))


def _cross(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(float(left[index]) * float(right[index]) for index in range(3))


def _vertex_index(token: str, vertex_count: int) -> int:
    raw = int(token.split("/", 1)[0])
    if raw == 0:
        raise ValueError("OBJ vertex indices are one-based")
    index = raw - 1 if raw > 0 else vertex_count + raw
    if not 0 <= index < vertex_count:
        raise ValueError(f"OBJ vertex index is out of range: {raw}")
    return index


def _load_object(
    path: Path,
    object_name: str,
) -> tuple[
    list[tuple[float, float, float]],
    list[tuple[int, int, int]],
]:
    vertices: list[tuple[float, float, float]] = []
    triangles: list[tuple[int, int, int]] = []
    current_object = ""
    for line_number, raw in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        1,
    ):
        line = raw.strip()
        if line.startswith("o "):
            current_object = line[2:].strip()
        elif line.startswith("v "):
            fields = line.split()
            if len(fields) < 4:
                raise ValueError(f"{path}:{line_number}: invalid vertex")
            vertex = tuple(float(value) for value in fields[1:4])
            if not all(math.isfinite(value) for value in vertex):
                raise ValueError(f"{path}:{line_number}: non-finite vertex")
            vertices.append(vertex)
        elif line.startswith("f ") and current_object == object_name:
            fields = line.split()[1:]
            if len(fields) < 3:
                raise ValueError(f"{path}:{line_number}: invalid face")
            indices = [
                _vertex_index(field, len(vertices)) for field in fields
            ]
            for index in range(1, len(indices) - 1):
                triangles.append(
                    (indices[0], indices[index], indices[index + 1])
                )
    if not triangles:
        raise ValueError(f"OBJ object has no faces: {object_name}")
    return vertices, triangles


def winding_stats(path: Path, object_name: str) -> WindingStats:
    vertices, triangles = _load_object(path, object_name)
    used_indices = sorted({index for triangle in triangles for index in triangle})
    centroid = tuple(
        sum(vertices[index][axis] for index in used_indices) / len(used_indices)
        for axis in range(3)
    )
    signed_values = []
    for first, second, third in triangles:
        point_0 = vertices[first]
        point_1 = vertices[second]
        point_2 = vertices[third]
        normal = _cross(
            _subtract(point_1, point_0),
            _subtract(point_2, point_0),
        )
        face_center = tuple(
            (point_0[axis] + point_1[axis] + point_2[axis]) / 3.0
            for axis in range(3)
        )
        signed_values.append(_dot(normal, _subtract(face_center, centroid)))
    tolerance = 1.0e-20
    outward = sum(value > tolerance for value in signed_values)
    inward = sum(value < -tolerance for value in signed_values)
    degenerate = len(signed_values) - outward - inward
    return WindingStats(
        object_name=object_name,
        triangle_count=len(signed_values),
        outward_triangles=outward,
        inward_triangles=inward,
        degenerate_triangles=degenerate,
        signed_sum=sum(signed_values),
    )


def reverse_object_faces(
    source: Path,
    destination: Path,
    object_name: str,
) -> int:
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite {destination}")
    current_object = ""
    reversed_faces = 0
    output_lines = []
    for raw in source.read_text(encoding="utf-8").splitlines():
        stripped = raw.strip()
        if stripped.startswith("o "):
            current_object = stripped[2:].strip()
        if stripped.startswith("f ") and current_object == object_name:
            indentation = raw[: len(raw) - len(raw.lstrip())]
            fields = stripped.split()[1:]
            raw = indentation + "f " + " ".join(reversed(fields))
            reversed_faces += 1
        output_lines.append(raw)
    if reversed_faces <= 0:
        raise ValueError(f"OBJ object has no faces: {object_name}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        "\n".join(output_lines) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return reversed_faces
