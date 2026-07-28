import importlib.util
import pathlib
import unittest


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[4]
SCRIPT_PATH = REPOSITORY_ROOT / "scripts" / "capture_target_pose_accuracy.py"
SPEC = importlib.util.spec_from_file_location("target_pose_accuracy", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TargetPoseAccuracyTests(unittest.TestCase):
    def test_nearest_rank_percentile_is_deterministic(self):
        self.assertEqual(MODULE.percentile([5, 1, 4, 2, 3], 0.95), 5.0)
        self.assertEqual(MODULE.percentile([5, 1, 4, 2, 3], 0.50), 3.0)

    def test_summary_reports_millimetres(self):
        summary = MODULE.summarize_errors([0.001, 0.003, 0.002])
        self.assertEqual(summary["sample_count"], 3)
        self.assertEqual(summary["minimum_mm"], 1.0)
        self.assertEqual(summary["median_mm"], 2.0)
        self.assertEqual(summary["p95_mm"], 3.0)
        self.assertEqual(summary["maximum_mm"], 3.0)

    def test_empty_inputs_fail_closed(self):
        with self.assertRaises(ValueError):
            MODULE.percentile([], 0.95)
        with self.assertRaises(ValueError):
            MODULE.summarize_errors([])


if __name__ == "__main__":
    unittest.main()
