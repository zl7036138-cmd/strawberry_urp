import json
from collections import Counter
from pathlib import Path
import tempfile
import unittest

from strawberry_benchmark.formal_matrix import (
    generate_formal_schedule,
    load_formal_matrix_contract,
)


ROOT = Path(__file__).resolve().parents[4]
MANIFEST = ROOT / "config/p3_formal_matrix_v1.json"


class FormalMatrixContractTests(unittest.TestCase):
    def setUp(self):
        self.contract = load_formal_matrix_contract(MANIFEST)
        self.schedule = generate_formal_schedule(
            self.contract, ROOT / "config/benchmark.yaml"
        )

    def test_frozen_counts_order_domains_and_true_seed_binding(self):
        self.assertEqual(165, len(self.schedule))
        self.assertEqual(135, sum(item.kind == "POSITIVE" for item in self.schedule))
        self.assertEqual(30, sum(item.kind == "NEGATIVE" for item in self.schedule))
        self.assertEqual(list(range(32, 197)), [item.ros_domain_id for item in self.schedule])
        self.assertEqual(
            {20260710, 20260711, 20260712},
            {item.simulation_seed for item in self.schedule},
        )
        self.assertEqual(20260710, self.contract.materialization_id)

    def test_schedule_is_deterministic_and_not_positive_first(self):
        repeated = generate_formal_schedule(
            self.contract, ROOT / "config/benchmark.yaml"
        )
        self.assertEqual(
            [item.to_dict() for item in self.schedule],
            [item.to_dict() for item in repeated],
        )
        first_ten = {item.kind for item in self.schedule[:10]}
        self.assertEqual({"POSITIVE", "NEGATIVE"}, first_ten)

    def test_negative_marginals_remain_balanced(self):
        negative = [item for item in self.schedule if item.kind == "NEGATIVE"]
        self.assertEqual(
            {"dim": 10, "nominal": 10, "bright": 10},
            Counter(item.lighting for item in negative),
        )
        self.assertEqual(
            {"none": 10, "partial": 10, "heavy": 10},
            Counter(item.occlusion for item in negative),
        )
        self.assertEqual(
            {"near_left": 6, "near_right": 6, "center": 6, "far_left": 6, "far_right": 6},
            Counter(item.position.label for item in negative),
        )

    def test_contract_rejects_a_fake_seed_or_relaxed_threshold(self):
        raw = json.loads(MANIFEST.read_text(encoding="utf-8"))
        for mutation, message in (
            (("matrix", "simulation_seeds", [20260710, 20260711, 7]), "simulation seeds"),
            (("acceptance", "minimum_positive_end_to_end_success_rate", 0.5), "thresholds"),
        ):
            altered = json.loads(json.dumps(raw))
            section, key, value = mutation
            altered[section][key] = value
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "manifest.json"
                path.write_text(json.dumps(altered), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, message):
                    load_formal_matrix_contract(path)


if __name__ == "__main__":
    unittest.main()
