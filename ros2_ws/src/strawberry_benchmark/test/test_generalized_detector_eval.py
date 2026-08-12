import unittest

from strawberry_benchmark.generalized_detector_eval import (
    DetectionBox,
    _prediction_rows,
    box_iou,
    evaluate_threshold,
    select_validation_threshold,
)


class _FakeTensor:
    def __init__(self, value):
        self._value = value

    def cpu(self):
        return self

    def tolist(self):
        return self._value


class _FakeBoxes:
    xyxy = _FakeTensor([[64.0, 48.0, 128.0, 96.0]])
    conf = _FakeTensor([0.75])
    cls = _FakeTensor([0.0])


class _FakeResult:
    path = "image0.jpg"
    orig_shape = (480, 640)
    boxes = _FakeBoxes()


class GeneralizedDetectorEvaluationTests(unittest.TestCase):
    def test_prediction_rows_preserve_explicit_source_identity(self):
        rows = _prediction_rows([_FakeResult()], ["validation_seed_42001"])
        self.assertEqual(rows[0].image_id, "validation_seed_42001")
        self.assertEqual(rows[0].xyxy, (0.1, 0.1, 0.2, 0.2))

    def test_iou_and_greedy_matching(self):
        self.assertAlmostEqual(box_iou((0, 0, 1, 1), (0, 0, 1, 1)), 1.0)
        self.assertEqual(box_iou((0, 0, 1, 1), (1, 1, 2, 2)), 0.0)
        truth = [DetectionBox("a", 0, (0, 0, 1, 1))]
        predictions = [DetectionBox("a", 0, (0, 0, 1, 1), 0.8)]
        result = evaluate_threshold(truth, predictions, threshold=0.5)
        self.assertEqual(result["ripe"]["tp"], 1)
        self.assertEqual(result["ripe"]["fp"], 0)

    def test_threshold_selection_requires_both_ripe_metrics(self):
        truth = [
            DetectionBox("a", 0, (0, 0, 1, 1)),
            DetectionBox("b", 0, (0, 0, 1, 1)),
        ]
        predictions = [
            DetectionBox("a", 0, (0, 0, 1, 1), 0.9),
            DetectionBox("b", 0, (0, 0, 1, 1), 0.8),
        ]
        result = select_validation_threshold(truth, predictions)
        self.assertTrue(result["gate_pass"])
        self.assertGreaterEqual(result["selected"]["ripe"]["precision"], 0.95)
        self.assertGreaterEqual(result["selected"]["ripe"]["recall"], 0.90)

    def test_false_positive_prevents_gate_at_high_recall(self):
        truth = [DetectionBox("a", 0, (0, 0, 1, 1))]
        predictions = [
            DetectionBox("a", 0, (0, 0, 1, 1), 0.8),
            DetectionBox("b", 0, (0, 0, 1, 1), 0.9),
        ]
        result = select_validation_threshold(truth, predictions)
        self.assertFalse(result["gate_pass"])


if __name__ == "__main__":
    unittest.main()
