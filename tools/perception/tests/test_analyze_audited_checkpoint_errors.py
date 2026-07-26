import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "analyze_audited_checkpoint_errors.py"
SPEC = importlib.util.spec_from_file_location("analyze_audited_checkpoint_errors", MODULE_PATH)
analysis = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(analysis)


class ErrorAttributionTests(unittest.TestCase):
    def setUp(self):
        self.truth = {
            "class_id": 1,
            "bbox_xyxy_normalized": [0.2, 0.2, 0.4, 0.4],
        }

    def prediction(self, class_id, confidence, box):
        return {
            "class_id": class_id,
            "confidence": confidence,
            "bbox_xyxy_normalized": box,
        }

    def test_wrong_class_has_priority(self):
        predictions = [self.prediction(0, 0.9, [0.2, 0.2, 0.4, 0.4])]
        self.assertEqual(
            analysis.classify_false_negative(self.truth, predictions, 0.3),
            "WRONG_CLASS",
        )

    def test_low_confidence_requires_same_class_iou_match(self):
        predictions = [self.prediction(1, 0.2, [0.2, 0.2, 0.4, 0.4])]
        self.assertEqual(
            analysis.classify_false_negative(self.truth, predictions, 0.3),
            "LOW_CONFIDENCE",
        )

    def test_localization_and_no_detection_are_separate(self):
        localized = [self.prediction(1, 0.9, [0.28, 0.28, 0.48, 0.48])]
        self.assertEqual(
            analysis.classify_false_negative(self.truth, localized, 0.3),
            "LOCALIZATION",
        )
        self.assertEqual(
            analysis.classify_false_negative(self.truth, [], 0.3), "NO_DETECTION"
        )


if __name__ == "__main__":
    unittest.main()
