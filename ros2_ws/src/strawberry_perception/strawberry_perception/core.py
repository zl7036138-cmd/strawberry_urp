"""Dependency-free YOLO post-processing for strawberry detections."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
import math
import operator
from pathlib import Path
import re
from typing import Any, Iterable, Mapping, Optional, Sequence, Tuple, Union


class Maturity(IntEnum):
    """Stable values from ``docs/architecture.md``."""

    UNKNOWN = 0
    RIPE = 1
    UNRIPE = 2


@dataclass(frozen=True)
class RuntimeParameters:
    """Validated detector settings used by the ROS adapter."""

    model_path: str
    confidence_threshold: float
    image_size: int
    nms_iou_threshold: float


_MODEL_CLASS_CONTRACT = ("ripe", "unripe")


def _finite_float(name: str, value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        converted = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be a finite number")
    return converted


def validate_runtime_parameters(
    model_path: Any,
    confidence_threshold: Any,
    image_size: Any,
    nms_iou_threshold: Any,
) -> RuntimeParameters:
    """Validate startup parameters before loading the detector.

    The returned model path is absolute, which makes the path recorded by
    Ultralytics and startup diagnostics independent of the caller's cwd after
    node construction.
    """

    if isinstance(model_path, bool):
        raise ValueError("model_path must identify an existing regular file")
    try:
        candidate = Path(model_path).expanduser()
        resolved_model_path = candidate.resolve(strict=True)
    except (OSError, TypeError, ValueError) as error:
        raise ValueError("model_path must identify an existing regular file") from error
    if not resolved_model_path.is_file():
        raise ValueError("model_path must identify an existing regular file")

    confidence = _finite_float("confidence_threshold", confidence_threshold)
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence_threshold must be in [0, 1]")

    if isinstance(image_size, bool):
        raise ValueError("image_size must be a positive integer")
    try:
        size = operator.index(image_size)
    except TypeError as error:
        raise ValueError("image_size must be a positive integer") from error
    if size <= 0:
        raise ValueError("image_size must be a positive integer")

    nms_iou = _finite_float("nms_iou_threshold", nms_iou_threshold)
    if not 0.0 < nms_iou <= 1.0:
        raise ValueError("nms_iou_threshold must be in (0, 1]")

    return RuntimeParameters(
        model_path=str(resolved_model_path),
        confidence_threshold=confidence,
        image_size=size,
        nms_iou_threshold=nms_iou,
    )


def _class_id(raw_class_id: Any) -> int:
    if isinstance(raw_class_id, bool):
        raise ValueError("model class ids must be integers")
    if isinstance(raw_class_id, str):
        if raw_class_id not in {"0", "1"}:
            raise ValueError(f"invalid model class id {raw_class_id!r}")
        return int(raw_class_id)
    try:
        return operator.index(raw_class_id)
    except TypeError as error:
        raise ValueError("model class ids must be integers") from error


def validate_model_class_contract(
    class_names: Union[Sequence[str], Mapping[Union[int, str], str]],
) -> Tuple[str, str]:
    """Require exactly ``0=ripe`` and ``1=unripe`` from a loaded model.

    Ultralytics exposes class names as either a sequence or an id-to-name
    mapping depending on the model/export path.  String mapping keys are
    accepted for serialized metadata, but aliases, missing classes, swapped
    ids, duplicate ids, and extra classes are rejected.
    """

    if isinstance(class_names, Mapping):
        by_id = {}
        for raw_class_id, class_name in class_names.items():
            class_id = _class_id(raw_class_id)
            if class_id in by_id:
                raise ValueError(f"model class id {class_id} is duplicated")
            by_id[class_id] = class_name
    elif isinstance(class_names, Sequence) and not isinstance(
        class_names, (str, bytes, bytearray)
    ):
        by_id = dict(enumerate(class_names))
    else:
        raise ValueError("model class names must be a mapping or sequence")

    expected_ids = set(range(len(_MODEL_CLASS_CONTRACT)))
    if set(by_id) != expected_ids:
        raise ValueError("model classes must be exactly {0: 'ripe', 1: 'unripe'}")
    for class_id, expected_name in enumerate(_MODEL_CLASS_CONTRACT):
        actual_name = by_id[class_id]
        if not isinstance(actual_name, str) or actual_name != expected_name:
            raise ValueError(
                "model classes must be exactly {0: 'ripe', 1: 'unripe'}"
            )
    return _MODEL_CLASS_CONTRACT


@dataclass(frozen=True)
class PixelRoi:
    """Integer half-open pixel rectangle suitable for RegionOfInterest."""

    x_offset: int
    y_offset: int
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.x_offset < 0 or self.y_offset < 0:
            raise ValueError("ROI offsets must be non-negative")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("ROI width and height must be positive")

    @property
    def x_max(self) -> int:
        return self.x_offset + self.width

    @property
    def y_max(self) -> int:
        return self.y_offset + self.height


@dataclass(frozen=True)
class Detection:
    """ROS-independent representation of one accepted detector result."""

    target_id: int
    maturity: Maturity
    confidence: float
    roi: PixelRoi
    source_class_id: int
    source_class_name: str

    def __post_init__(self) -> None:
        if not isinstance(self.target_id, int) or not 0 <= self.target_id <= 0xFFFFFFFF:
            raise ValueError("target_id must fit the ROS uint32 contract")
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be finite and in [0, 1]")
        if self.source_class_id < 0:
            raise ValueError("source_class_id must be non-negative")


_LABEL_ALIASES = {
    "ripe": Maturity.RIPE,
    "mature": Maturity.RIPE,
    "red": Maturity.RIPE,
    "ripe_strawberry": Maturity.RIPE,
    "strawberry_ripe": Maturity.RIPE,
    "unripe": Maturity.UNRIPE,
    "immature": Maturity.UNRIPE,
    "green": Maturity.UNRIPE,
    "unripe_strawberry": Maturity.UNRIPE,
    "strawberry_unripe": Maturity.UNRIPE,
}


def _normalise_label(label: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", label.strip().lower()).strip("_")


def maturity_from_label(label: str) -> Maturity:
    """Map common model labels to the stable maturity enum.

    ``peduncle`` and any unrecognised class intentionally map to ``UNKNOWN``;
    the default post-processing policy removes them from the v1 output.
    """

    if not isinstance(label, str):
        raise TypeError("label must be a string")
    return _LABEL_ALIASES.get(_normalise_label(label), Maturity.UNKNOWN)


def _class_name(
    class_names: Union[Sequence[str], Mapping[Union[int, str], str]],
    class_id: int,
) -> str:
    if isinstance(class_names, Mapping):
        if class_id in class_names:
            value = class_names[class_id]
        elif str(class_id) in class_names:
            value = class_names[str(class_id)]
        else:
            raise ValueError(f"class id {class_id} is missing from class_names")
    else:
        if class_id >= len(class_names):
            raise ValueError(f"class id {class_id} is outside class_names")
        value = class_names[class_id]
    if not isinstance(value, str):
        raise ValueError(f"class name for id {class_id} is not a string")
    return value


def _unpack_row(row: object) -> Tuple[float, float, float, float, float, int]:
    if isinstance(row, Mapping):
        try:
            if "xyxy" in row:
                x1, y1, x2, y2 = row["xyxy"]  # type: ignore[misc]
            else:
                x1, y1, x2, y2 = (row[key] for key in ("x1", "y1", "x2", "y2"))
            confidence = row.get("confidence", row.get("conf"))
            class_id_value = row.get("class_id", row.get("cls"))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("invalid mapping YOLO row") from error
    else:
        try:
            values = list(row)  # type: ignore[arg-type]
        except TypeError as error:
            raise ValueError("YOLO row must be a mapping or sequence") from error
        if len(values) != 6:
            raise ValueError("YOLO sequence row must be [x1, y1, x2, y2, conf, class_id]")
        x1, y1, x2, y2, confidence, class_id_value = values

    try:
        coordinates = tuple(float(value) for value in (x1, y1, x2, y2))
        confidence_float = float(confidence)  # type: ignore[arg-type]
        class_id_float = float(class_id_value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise ValueError("YOLO row contains a non-numeric value") from error
    if not all(math.isfinite(value) for value in (*coordinates, confidence_float, class_id_float)):
        raise ValueError("YOLO row contains a non-finite value")
    if not class_id_float.is_integer() or class_id_float < 0:
        raise ValueError("class_id must be a non-negative integer")
    if not 0.0 <= confidence_float <= 1.0:
        raise ValueError("confidence must be in [0, 1]")
    return (*coordinates, confidence_float, int(class_id_float))


def _pixel_roi(
    xyxy: Tuple[float, float, float, float],
    image_size: Optional[Tuple[int, int]],
) -> PixelRoi:
    x1, y1, x2, y2 = xyxy
    if x2 <= x1 or y2 <= y1:
        raise ValueError("YOLO box must have positive area")
    if image_size is not None:
        image_width, image_height = image_size
        if image_width <= 0 or image_height <= 0:
            raise ValueError("image_size must contain positive width and height")
        x1 = min(max(x1, 0.0), float(image_width))
        x2 = min(max(x2, 0.0), float(image_width))
        y1 = min(max(y1, 0.0), float(image_height))
        y2 = min(max(y2, 0.0), float(image_height))
    left, top = math.floor(x1), math.floor(y1)
    right, bottom = math.ceil(x2), math.ceil(y2)
    if right <= left or bottom <= top:
        raise ValueError("YOLO box is empty after clipping")
    return PixelRoi(left, top, right - left, bottom - top)


def _sort_key(detection: Detection) -> Tuple[float, int, int, int, int, int, int, int]:
    return (
        -detection.confidence,
        int(detection.maturity),
        detection.roi.x_offset,
        detection.roi.y_offset,
        detection.roi.width,
        detection.roi.height,
        detection.source_class_id,
        detection.target_id,
    )


def filter_and_sort(
    detections: Iterable[Detection],
    confidence_threshold: float = 0.60,
    allowed_maturities: Iterable[Maturity] = (Maturity.RIPE, Maturity.UNRIPE),
) -> Tuple[Detection, ...]:
    """Filter and deterministically order an existing set of detections."""

    if not math.isfinite(confidence_threshold) or not 0.0 <= confidence_threshold <= 1.0:
        raise ValueError("confidence_threshold must be finite and in [0, 1]")
    allowed = frozenset(Maturity(value) for value in allowed_maturities)
    accepted = (
        detection
        for detection in detections
        if detection.confidence >= confidence_threshold and detection.maturity in allowed
    )
    return tuple(sorted(accepted, key=_sort_key))


def parse_yolo_detections(
    rows: Iterable[object],
    class_names: Union[Sequence[str], Mapping[Union[int, str], str]],
    confidence_threshold: float = 0.60,
    image_size: Optional[Tuple[int, int]] = None,
    target_id_offset: int = 0,
    strict: bool = True,
) -> Tuple[Detection, ...]:
    """Parse Ultralytics-style ``xyxy/conf/class`` rows without Ultralytics.

    Invalid rows raise ``ValueError`` in strict mode.  Runtime nodes may select
    ``strict=False`` to discard a malformed prediction while continuing to
    publish the remaining valid predictions.  Target IDs are assigned *after*
    the deterministic sort, so equal inputs always produce equal IDs.
    """

    if not isinstance(target_id_offset, int) or not 0 <= target_id_offset <= 0xFFFFFFFF:
        raise ValueError("target_id_offset must fit uint32")
    parsed = []
    for source_index, row in enumerate(rows):
        try:
            x1, y1, x2, y2, confidence, class_id = _unpack_row(row)
            source_name = _class_name(class_names, class_id)
            maturity = maturity_from_label(source_name)
            roi = _pixel_roi((x1, y1, x2, y2), image_size)
            parsed.append(
                Detection(
                    target_id=source_index,
                    maturity=maturity,
                    confidence=confidence,
                    roi=roi,
                    source_class_id=class_id,
                    source_class_name=source_name,
                )
            )
        except (TypeError, ValueError):
            if strict:
                raise

    ordered = filter_and_sort(parsed, confidence_threshold=confidence_threshold)
    if len(ordered) and target_id_offset + len(ordered) - 1 > 0xFFFFFFFF:
        raise ValueError("target IDs overflow uint32")
    return tuple(
        replace(detection, target_id=target_id_offset + rank)
        for rank, detection in enumerate(ordered)
    )
