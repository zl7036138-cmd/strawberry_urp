import math
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from strawberry_perception import (  # noqa: E402
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


class MaturityMappingTest(unittest.TestCase):
    def test_maps_dataset_and_common_aliases(self):
        self.assertEqual(maturity_from_label("ripe"), Maturity.RIPE)
        self.assertEqual(maturity_from_label("RIPE strawberry"), Maturity.RIPE)
        self.assertEqual(maturity_from_label("green"), Maturity.UNRIPE)
        self.assertEqual(maturity_from_label("strawberry-unripe"), Maturity.UNRIPE)
        self.assertEqual(maturity_from_label("peduncle"), Maturity.UNKNOWN)


class RuntimeParameterValidationTest(unittest.TestCase):
    def test_accepts_existing_model_and_valid_numeric_boundaries(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            model_path = Path(temporary_directory) / "best.pt"
            model_path.write_bytes(b"test weight")
            parameters = validate_runtime_parameters(model_path, 0.0, 1, 1.0)

        self.assertIsInstance(parameters, RuntimeParameters)
        self.assertEqual(parameters.model_path, str(model_path.resolve()))
        self.assertEqual(parameters.confidence_threshold, 0.0)
        self.assertEqual(parameters.image_size, 1)
        self.assertEqual(parameters.nms_iou_threshold, 1.0)

    def test_rejects_missing_model_and_directory(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            with self.assertRaisesRegex(ValueError, "existing regular file"):
                validate_runtime_parameters(directory / "missing.pt", 0.6, 640, 0.7)
            with self.assertRaisesRegex(ValueError, "existing regular file"):
                validate_runtime_parameters(directory, 0.6, 640, 0.7)

    def test_rejects_invalid_confidence_threshold(self):
        with tempfile.NamedTemporaryFile() as model:
            for invalid in (math.nan, math.inf, -0.01, 1.01, True):
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    validate_runtime_parameters(model.name, invalid, 640, 0.7)

    def test_rejects_non_positive_or_non_integer_image_size(self):
        with tempfile.NamedTemporaryFile() as model:
            for invalid in (0, -1, 640.0, "640", True):
                with self.subTest(invalid=invalid), self.assertRaisesRegex(
                    ValueError, "positive integer"
                ):
                    validate_runtime_parameters(model.name, 0.6, invalid, 0.7)

    def test_rejects_invalid_nms_iou_threshold(self):
        with tempfile.NamedTemporaryFile() as model:
            for invalid in (math.nan, math.inf, 0.0, -0.01, 1.01, True):
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    validate_runtime_parameters(model.name, 0.6, 640, invalid)


class ModelClassContractTest(unittest.TestCase):
    def test_accepts_exact_list_and_integer_or_string_key_mappings(self):
        expected = ("ripe", "unripe")
        self.assertEqual(validate_model_class_contract(["ripe", "unripe"]), expected)
        self.assertEqual(
            validate_model_class_contract({0: "ripe", 1: "unripe"}), expected
        )
        self.assertEqual(
            validate_model_class_contract({"0": "ripe", "1": "unripe"}), expected
        )

    def test_rejects_missing_extra_swapped_or_aliased_classes(self):
        invalid_contracts = (
            ["ripe"],
            ["ripe", "unripe", "peduncle"],
            ["unripe", "ripe"],
            ["mature", "unripe"],
            {0: "ripe", 2: "unripe"},
            {0: "ripe", "0": "ripe", 1: "unripe"},
        )
        for invalid in invalid_contracts:
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_model_class_contract(invalid)

    def test_rejects_non_sequence_metadata_and_non_string_name(self):
        for invalid in (None, "ripe,unripe", {0: "ripe", 1: 1}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_model_class_contract(invalid)


class YoloParsingTest(unittest.TestCase):
    def test_filters_unknown_and_low_confidence_then_sorts(self):
        rows = [
            [20, 10, 30, 20, 0.75, 1],
            [8, 10, 18, 20, 0.80, 2],
            [30, 10, 40, 20, 0.59, 0],
            [10, 10, 20, 20, 0.75, 0],
        ]
        detections = parse_yolo_detections(
            rows,
            {0: "ripe", 1: "unripe", 2: "peduncle"},
            confidence_threshold=0.60,
            image_size=(100, 80),
        )
        self.assertEqual([item.target_id for item in detections], [0, 1])
        self.assertEqual([item.maturity for item in detections], [Maturity.RIPE, Maturity.UNRIPE])
        self.assertEqual([item.roi.x_offset for item in detections], [10, 20])

    def test_clips_and_rounds_to_half_open_pixel_roi(self):
        detection = parse_yolo_detections(
            [[-1.2, 2.1, 101.8, 79.2, 0.9, 0]],
            ["ripe"],
            image_size=(100, 80),
        )[0]
        self.assertEqual(detection.roi, PixelRoi(0, 2, 100, 78))

    def test_mapping_row_and_string_class_keys(self):
        detections = parse_yolo_detections(
            [{"xyxy": (1, 2, 4, 6), "conf": 0.8, "cls": 1}],
            {"0": "ripe", "1": "unripe"},
        )
        self.assertEqual(detections[0].maturity, Maturity.UNRIPE)

    def test_strict_mode_rejects_bad_rows_and_lenient_mode_skips(self):
        bad = [[0, 0, 10, 10, math.nan, 0]]
        with self.assertRaises(ValueError):
            parse_yolo_detections(bad, ["ripe"])
        self.assertEqual(parse_yolo_detections(bad, ["ripe"], strict=False), ())

    def test_existing_detections_have_deterministic_tie_breakers(self):
        right = Detection(20, Maturity.RIPE, 0.8, PixelRoi(20, 0, 5, 5), 0, "ripe")
        left = Detection(10, Maturity.RIPE, 0.8, PixelRoi(10, 0, 5, 5), 0, "ripe")
        self.assertEqual([item.target_id for item in filter_and_sort([right, left])], [10, 20])

    def test_target_id_matches_ros_uint32_contract(self):
        detection = parse_yolo_detections(
            [[0, 0, 10, 10, 0.8, 0]], ["ripe"], target_id_offset=0xFFFFFFFF
        )[0]
        self.assertEqual(detection.target_id, 0xFFFFFFFF)
        with self.assertRaises(ValueError):
            parse_yolo_detections(
                [[0, 0, 10, 10, 0.9, 0], [1, 1, 9, 9, 0.8, 0]],
                ["ripe"],
                target_id_offset=0xFFFFFFFF,
            )


if __name__ == "__main__":
    unittest.main()
