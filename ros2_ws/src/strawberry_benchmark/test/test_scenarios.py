from collections import Counter
from itertools import product
from pathlib import Path
import unittest

from strawberry_benchmark.models import Maturity, ScenarioKind
from strawberry_benchmark.scenarios import (
    EXPECTED_NEGATIVE_TRIALS,
    EXPECTED_POSITIVE_TRIALS,
    BenchmarkSpec,
    generate_scenarios,
    load_benchmark_spec,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


class ScenarioGenerationTests(unittest.TestCase):
    def setUp(self):
        self.spec = load_benchmark_spec(REPOSITORY_ROOT / "config" / "benchmark.yaml")
        self.scenarios = generate_scenarios(self.spec)

    def test_fixed_counts_and_unique_ids(self):
        positive = [
            scenario
            for scenario in self.scenarios
            if scenario.kind is ScenarioKind.POSITIVE
        ]
        negative = [
            scenario
            for scenario in self.scenarios
            if scenario.kind is ScenarioKind.NEGATIVE
        ]
        self.assertEqual(EXPECTED_POSITIVE_TRIALS, len(positive))
        self.assertEqual(EXPECTED_NEGATIVE_TRIALS, len(negative))
        self.assertEqual(len(self.scenarios), len({item.trial_id for item in self.scenarios}))
        self.assertTrue(all(item.expected_maturity is Maturity.RIPE for item in positive))
        self.assertTrue(
            all(item.expected_maturity is Maturity.UNRIPE for item in negative)
        )

    def test_positive_trials_are_complete_cartesian_product(self):
        expected = set(
            product(
                self.spec.occlusion_levels,
                self.spec.lighting_levels,
                self.spec.positions,
                self.spec.seeds,
            )
        )
        actual = {
            (item.occlusion, item.lighting, item.position, item.seed)
            for item in self.scenarios
            if item.kind is ScenarioKind.POSITIVE
        }
        self.assertEqual(expected, actual)

    def test_negative_marginals_are_balanced(self):
        negative = [
            item for item in self.scenarios if item.kind is ScenarioKind.NEGATIVE
        ]
        self.assertEqual(
            {value: 10 for value in self.spec.occlusion_levels},
            Counter(item.occlusion for item in negative),
        )
        self.assertEqual(
            {value: 10 for value in self.spec.lighting_levels},
            Counter(item.lighting for item in negative),
        )
        self.assertEqual(
            {value: 6 for value in self.spec.positions},
            Counter(item.position for item in negative),
        )
        self.assertEqual(
            {value: 10 for value in self.spec.seeds},
            Counter(item.seed for item in negative),
        )

    def test_spec_rejects_a_non_frozen_matrix(self):
        with self.assertRaisesRegex(ValueError, "exactly 3"):
            BenchmarkSpec(
                occlusion_levels=("none", "heavy"),
                lighting_levels=("dim", "nominal", "bright"),
                positions=("p1", "p2", "p3", "p4", "p5"),
                seeds=(1, 2, 3),
            )


if __name__ == "__main__":
    unittest.main()
