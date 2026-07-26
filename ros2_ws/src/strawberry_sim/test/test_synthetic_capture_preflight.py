import hashlib
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts"))

from summarize_t70_synthetic_capture_preflight import (  # noqa: E402
    canonical_dataset_digest,
    png_size,
    validate_hash_partitions,
)
from validate_t70_synthetic_capture_preflight import (  # noqa: E402
    iter_groups,
    load_capture_manifest,
)


MANIFEST = ROOT / "config" / "t70_synthetic_capture_preflight.json"


def _record(sample_id, split, image_hash, source_hash):
    return {
        "sample_id": sample_id,
        "split": split,
        "maturity": "RIPE",
        "class_id": 0,
        "condition_id": "nominal_none",
        "position_id": sample_id,
        "target_position_m": [0.4, 0.0, 0.5],
        "image": {"sha256": image_hash},
        "label": {"sha256": hashlib.sha256(sample_id.encode()).hexdigest()},
        "source_image_sha256": source_hash,
    }


class SyntheticCaptureManifestTests(unittest.TestCase):
    def test_manifest_freezes_balanced_nonformal_capture(self):
        manifest = load_capture_manifest(MANIFEST)
        self.assertEqual(36, len(list(iter_groups(manifest))))
        self.assertEqual(216, manifest["capture"]["expected_train_images"])
        self.assertEqual(72, manifest["capture"]["expected_heldout_images"])
        self.assertFalse(manifest["training_started"])
        self.assertTrue(
            {20260710, 20260711, 20260712}.isdisjoint(
                {
                    manifest["splits"]["train"]["materialization_id"],
                    manifest["splits"]["heldout"]["materialization_id"],
                }
            )
        )

    def test_hash_partition_rejects_within_and_cross_split_duplicates(self):
        valid = [
            _record("train-a", "train", "a", "sa"),
            _record("heldout-b", "heldout", "b", "sb"),
        ]
        self.assertEqual(
            0, validate_hash_partitions(valid)["cross_split_encoded_image_hash_overlap"]
        )
        with self.assertRaisesRegex(ValueError, "inside train"):
            validate_hash_partitions(
                [valid[0], _record("train-c", "train", "a", "sc"), valid[1]]
            )
        with self.assertRaisesRegex(ValueError, "overlap"):
            validate_hash_partitions(
                [valid[0], _record("heldout-c", "heldout", "a", "sc")]
            )

    def test_canonical_digest_is_order_independent(self):
        first = _record("train-a", "train", "a", "sa")
        second = _record("heldout-b", "heldout", "b", "sb")
        self.assertEqual(
            canonical_dataset_digest([first, second]),
            canonical_dataset_digest([second, first]),
        )

    def test_png_header_dimensions_are_read_without_decoder(self):
        header = (
            b"\x89PNG\r\n\x1a\n"
            + b"\x00\x00\x00\x0dIHDR"
            + (640).to_bytes(4, "big")
            + (480).to_bytes(4, "big")
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "image.png"
            path.write_bytes(header)
            self.assertEqual((640, 480), png_size(path))


if __name__ == "__main__":
    unittest.main()
