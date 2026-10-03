import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("contact_analysis", ROOT / "contact_analysis.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


class ContactAnalysisTests(unittest.TestCase):
    def contact(self, stamp=10, reverse=False, depth=0.001):
        first, second = {"id": "1", "name": "finger"}, {"id": "2", "name": "fruit"}
        if reverse:
            first, second = second, first
        return {"phase": "CONTACTS", "sim_time_sec": stamp, "source_entity": 3,
            "contacts": {"contact": [{"collision1": first, "collision2": second, "depth": [depth]}]}}

    def test_pair_order_is_symmetric_and_sources_deduplicated(self):
        result = analysis.summarize([self.contact(), self.contact(10.01, True, .002)], 10)
        self.assertEqual(len(result["pairs"]), 1)
        self.assertEqual(result["pairs"][0]["sample_count"], 2)
        self.assertEqual(result["pairs"][0]["max_depth_m"], .002)
        self.assertEqual(result["pairs"][0]["source_entities"], [3])

    def test_empty_data_never_proves_collision_freedom(self):
        result = analysis.summarize([], 10)
        self.assertFalse(result["collision_free_proven"])
        self.assertIsNone(result["minimum_observed_source_count"])

    def test_window_and_coverage(self):
        rows = [self.contact(9), {"phase": "CONTACT_COVERAGE", "sim_time_sec": 9.8, "source_count": 0},
                self.contact(), {"phase": "CONTACT_COVERAGE", "sim_time_sec": 10.1, "source_count": 5}]
        result = analysis.summarize(rows, 10)
        self.assertEqual(result["coverage_sample_count"], 2)
        self.assertEqual(result["minimum_observed_source_count"], 0)
        self.assertEqual(result["pairs"][0]["sample_count"], 1)

    def test_invalid_timestamp_depth_and_window_rejected(self):
        for rows in ([self.contact(float("nan"))], [self.contact(10), self.contact(9)],
                     [self.contact(depth=float("inf"))]):
            with self.assertRaises(ValueError): analysis.summarize(rows, 10)
        with self.assertRaises(ValueError): analysis.summarize([], 10, before=-1)

    def test_observer_is_opt_in_and_serializes_original_contacts(self):
        text = (ROOT / "CommandAudit.cc").read_text()
        self.assertIn('Get<bool>("contact_audit_enabled", false)', text)
        self.assertIn('MessageToJsonString(data->Data(), &raw)', text)
        for mutation in ("CreateComponent(", "SetData(", "RemoveComponent("):
            self.assertNotIn(mutation, text)


if __name__ == "__main__": unittest.main()
