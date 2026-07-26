import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "sweep_audited_checkpoints.py"
SPEC = importlib.util.spec_from_file_location("sweep_audited_checkpoints", MODULE_PATH)
sweep = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(sweep)


class CheckpointSweepCoreTests(unittest.TestCase):
    def test_inventory_requires_exact_frozen_names_for_both_families(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            families = {"baseline": root / "base", "opt1": root / "opt"}
            for family in families.values():
                family.mkdir()
                for name in sweep.EXPECTED_WEIGHT_NAMES:
                    (family / name).write_bytes(f"{family.name}-{name}".encode())
            inventory = sweep.build_inventory(families)
            self.assertEqual(len(inventory), 26)
            self.assertEqual(len({item["sha256"] for item in inventory}), 26)

    def test_inventory_rejects_missing_or_extra_weights(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            families = {"baseline": root / "base", "opt1": root / "opt"}
            for family in families.values():
                family.mkdir()
                for name in sweep.EXPECTED_WEIGHT_NAMES:
                    (family / name).write_bytes(name.encode())
            (families["baseline"] / "epoch100.pt").unlink()
            (families["opt1"] / "unexpected.pt").write_bytes(b"extra")
            with self.assertRaisesRegex(ValueError, "checkpoint set mismatch"):
                sweep.build_inventory(families)

    def test_winner_selection_uses_frozen_tie_breaks(self):
        def candidate(checkpoint_id, family, macro, ripe, unripe, threshold):
            return {
                "checkpoint_id": checkpoint_id,
                "family": family,
                "validation_metrics": {
                    "macro_f1": macro,
                    "confidence_threshold": threshold,
                    "per_class": {"0": {"f1": ripe}, "1": {"f1": unripe}},
                },
            }

        candidates = [
            candidate("opt1__a", "opt1", 0.86, 0.92, 0.80, 0.4),
            candidate("baseline__b", "baseline", 0.86, 0.90, 0.82, 0.4),
            candidate("baseline__a", "baseline", 0.86, 0.90, 0.82, 0.4),
        ]
        self.assertEqual(sweep.select_winner(candidates)["checkpoint_id"], "baseline__a")


if __name__ == "__main__":
    unittest.main()
