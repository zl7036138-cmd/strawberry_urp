import json
from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.p4_qualification import (
    evaluate_qualification_records,
    generate_qualification_schedule,
    load_qualification_contract,
)


ROOT = Path(__file__).resolve().parents[4]
MANIFEST = ROOT / "config/p4_sim_adapt_qualification_v1.json"


def passing_records(contract):
    records = []
    for scenario in generate_qualification_schedule(contract):
        records.append({
            "scenario_id": scenario.scenario_id,
            "maturity": scenario.maturity,
            "position_label": scenario.position_label,
            "condition_label": scenario.condition_label,
            "occlusion": scenario.occlusion,
            "frame_count": 60,
            "ripe_frames": 60 if scenario.maturity == "RIPE" else 0,
            "unripe_frames": 60 if scenario.maturity == "UNRIPE" else 0,
            "target_pose_frames": 60 if scenario.maturity == "RIPE" else 0,
            "infrastructure_valid": True,
            "robot_motion_started": False,
        })
    return records


class P4QualificationTests(unittest.TestCase):
    def setUp(self):
        self.contract = load_qualification_contract(MANIFEST, ROOT)

    def test_contract_emits_balanced_interleaved_schedule(self):
        schedule = generate_qualification_schedule(self.contract)
        self.assertEqual(30, len(schedule))
        self.assertEqual(15, sum(item.maturity == "RIPE" for item in schedule))
        self.assertEqual(15, sum(item.maturity == "UNRIPE" for item in schedule))
        self.assertEqual(list(range(200, 230)), [item.ros_domain_id for item in schedule])
        self.assertEqual({"RIPE", "UNRIPE"}, {item.maturity for item in schedule[:6]})

    def test_contract_rejects_threshold_or_motion_relaxation(self):
        for section, key, value, message in (
            ("intervention", "confidence_threshold", 0.3, "threshold"),
            ("runtime", "start_manipulation", True, "motion-capable"),
        ):
            raw = json.loads(MANIFEST.read_text(encoding="utf-8"))
            raw[section][key] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "contract.json"
                path.write_text(json.dumps(raw), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    load_qualification_contract(path, ROOT)

    def test_passing_records_pass(self):
        result = evaluate_qualification_records(
            self.contract, passing_records(self.contract)
        )
        self.assertTrue(result["qualification_passed"])
        self.assertEqual(1.0, result["heavy_overall"]["ripe_frame_rate"])
        self.assertEqual(0.0, result["unripe"]["false_ripe_frame_rate"])

    def test_heavy_or_unripe_failure_fails_closed(self):
        records = passing_records(self.contract)
        for item in records:
            if item["maturity"] == "RIPE" and item["occlusion"] == "heavy":
                item["ripe_frames"] = 0
                item["target_pose_frames"] = 0
            if item["maturity"] == "UNRIPE":
                item["ripe_frames"] = 10
        result = evaluate_qualification_records(self.contract, records)
        self.assertFalse(result["qualification_passed"])
        self.assertFalse(result["checks"]["heavy_ripe_overall"])
        self.assertFalse(result["checks"]["unripe_false_ripe_rate"])


if __name__ == "__main__":
    unittest.main()
