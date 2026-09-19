from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

from recompute_p0_evidence import assert_allowed_input  # noqa: E402


class HistoricalInputBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.root = SCRIPTS.parent

    def test_declared_exhausted_input_is_allowed(self):
        path = self.root / "results/p3/formal_matrix_v1/summary.json"
        self.assertEqual(
            assert_allowed_input(path, self.root),
            "results/p3/formal_matrix_v1/summary.json",
        )

    def test_formal_30_materialization_is_rejected(self):
        path = self.root / (
            ".codex_tmp/generalized_formal_matrix_v1_final/"
            "materialization_manifest.json"
        )
        with self.assertRaisesRegex(PermissionError, "protected resource"):
            assert_allowed_input(path, self.root)

    def test_real_test_split_is_rejected(self):
        path = self.root / "data/processed/zenodo_6126677/test/images/example.jpg"
        with self.assertRaisesRegex(PermissionError, "protected resource"):
            assert_allowed_input(path, self.root)

    def test_unlisted_input_is_rejected(self):
        with self.assertRaisesRegex(PermissionError, "outside.*allowlist"):
            assert_allowed_input(self.root / "README.md", self.root)


if __name__ == "__main__":
    unittest.main()
