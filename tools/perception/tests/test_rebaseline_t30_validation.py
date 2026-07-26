from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "rebaseline_t30_validation.py"
SPEC = importlib.util.spec_from_file_location("rebaseline_t30_validation", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
rebaseline = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rebaseline)


class IdealRepairTest(unittest.TestCase):
    def test_current_unripe_gap_requires_thirteen_ideal_fn_recoveries(self):
        result = rebaseline._minimum_ideal_repairs(
            tp=80,
            fp=27,
            fn=30,
            required_f1=0.8056910569105691,
        )
        self.assertEqual(13, result["false_negatives_recovered_with_no_new_fp"])
        self.assertEqual(19, result["false_positives_removed_with_no_new_fn"])

    def test_impossible_target_returns_none(self):
        result = rebaseline._minimum_ideal_repairs(
            tp=0,
            fp=2,
            fn=3,
            required_f1=1.1,
        )
        self.assertIsNone(result["false_negatives_recovered_with_no_new_fp"])
        self.assertIsNone(result["false_positives_removed_with_no_new_fn"])


class DistributionTest(unittest.TestCase):
    def test_empty_distribution_is_explicit(self):
        self.assertEqual(
            {"count": 0, "median": None, "mean": None, "min": None, "max": None},
            rebaseline._distribution([]),
        )

    def test_distribution_reports_median_and_mean(self):
        result = rebaseline._distribution([1.0, 2.0, 9.0])
        self.assertEqual(3, result["count"])
        self.assertEqual(2.0, result["median"])
        self.assertEqual(4.0, result["mean"])


if __name__ == "__main__":
    unittest.main()
