"""Pure-Python perception contracts.

Importing this package intentionally does not import ROS 2 or Ultralytics.  This
keeps the post-processing contract testable on development hosts that have
neither dependency installed.
"""

from .core import (
    Detection,
    Maturity,
    PixelRoi,
    RuntimeParameters,
    filter_and_sort,
    maturity_from_label,
    parse_yolo_detections,
    validate_model_class_contract,
    validate_runtime_parameters,
)

__all__ = [
    "Detection",
    "Maturity",
    "PixelRoi",
    "RuntimeParameters",
    "filter_and_sort",
    "maturity_from_label",
    "parse_yolo_detections",
    "validate_model_class_contract",
    "validate_runtime_parameters",
]
