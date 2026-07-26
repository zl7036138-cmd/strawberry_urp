import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "build_label_audit_packet.py"
SPEC = importlib.util.spec_from_file_location("build_label_audit_packet", MODULE_PATH)
audit = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(audit)


class ContextBoxTests(unittest.TestCase):
    def test_context_expands_and_clips_edge_candidate(self):
        crop = audit._context_box(
            (0.95, 0.90, 1.0, 1.0), image_width=1000, image_height=500
        )
        self.assertEqual(crop[2:], (1000, 500))
        self.assertGreater(crop[0], 0)
        self.assertGreater(crop[1], 0)

    def test_context_rejects_invalid_inputs(self):
        with self.assertRaises(ValueError):
            audit._context_box((0, 0, 0, 1), image_width=10, image_height=10)
        with self.assertRaises(ValueError):
            audit._context_box((0, 0, 1, 1), image_width=10, image_height=10, expansion=0.5)


class LabelReaderTests(unittest.TestCase):
    def test_loads_only_v1_classes(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "labels.txt"
            path.write_text("0 0.5 0.5 0.2 0.4\n1 0.2 0.3 0.1 0.1\n", encoding="utf-8")
            labels = audit._load_labels(path)
            self.assertEqual([item["class_name"] for item in labels], ["ripe", "unripe"])
            path.write_text("2 0.5 0.5 0.2 0.4\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                audit._load_labels(path)

    def test_decision_vocabulary_is_closed(self):
        self.assertEqual(len(audit.ALLOWED_REVIEW_DECISIONS), 5)
        self.assertIn("AMBIGUOUS_EXCLUDE", audit.ALLOWED_REVIEW_DECISIONS)


if __name__ == "__main__":
    unittest.main()
