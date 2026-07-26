from contextlib import redirect_stdout
import hashlib
from io import StringIO
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import sys
import tempfile
import unittest
import zipfile


TOOLS_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(TOOLS_ROOT))

from dataset_tools import (  # noqa: E402
    DatasetVerificationError,
    Sample,
    SplitValidationError,
    discover_samples,
    file_digest,
    group_stratified_split,
    load_manifest,
    make_splits,
    materialize_splits,
    validate_splits,
    verify_file,
    verify_materialized_split_manifest,
)
from split_dataset import main as split_main  # noqa: E402
from restore_extracted_from_archive import restore_changed_images  # noqa: E402


def write_sample(directory: Path, name: str, labels: str = "0 0.5 0.5 0.2 0.2\n") -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / f"{name}.jpg").write_bytes(b"not-a-real-image\xff\xd9")
    (directory / f"{name}.txt").write_text(labels, encoding="utf-8")


class ManifestAndChecksumTest(unittest.TestCase):
    def test_manifest_pins_zenodo_artifact(self):
        manifest_path = REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677.json"
        manifest = load_manifest(manifest_path)
        self.assertEqual(manifest["record"]["doi"], "10.5281/zenodo.6126677")
        self.assertEqual(manifest["license"]["spdx"], "CC-BY-4.0")
        self.assertEqual(
            manifest["files"][0]["checksum"]["value"],
            "db8d5dcb4b8adebf1621788373fd3031",
        )
        self.assertEqual(manifest["files"][0]["size_bytes"], 1485730857)

    def test_digest_and_verify_are_machine_readable(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "artifact.bin"
            path.write_bytes(b"strawberry")
            expected = hashlib.md5(b"strawberry").hexdigest()
            self.assertEqual(file_digest(path, "md5"), expected)
            status = verify_file(path, "md5", expected)
            self.assertTrue(status["valid"])
            json.dumps(status)

    def test_changed_extraction_is_restored_only_after_all_hash_gates(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_root = root / "extracted" / "strawberries"
            (source_root / "training").mkdir(parents=True)
            (source_root / "validation").mkdir(parents=True)
            expected = {
                "training/a.jpg": b"original-a",
                "validation/b.jpg": b"original-b",
            }
            (source_root / "training" / "a.jpg").write_bytes(b"changed-a")
            (source_root / "validation" / "b.jpg").write_bytes(expected["validation/b.jpg"])

            archive = root / "strawberries.zip"
            with zipfile.ZipFile(archive, "w") as bundle:
                for relative, payload in expected.items():
                    bundle.writestr(f"strawberries/{relative}", payload)
            dataset_manifest = root / "dataset_manifest.json"
            dataset_manifest.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "dataset_id": "zenodo_6126677",
                        "files": [
                            {
                                "name": archive.name,
                                "size_bytes": archive.stat().st_size,
                                "checksum": {
                                    "algorithm": "md5",
                                    "value": file_digest(archive, "md5"),
                                },
                            }
                        ],
                        "archive": {"root_directory": "strawberries"},
                    }
                ),
                encoding="utf-8",
            )
            audit = root / "audit.json"
            audit.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "dataset_id": "zenodo_6126677",
                        "input": {
                            "image_count": len(expected),
                            "images": [
                                {
                                    "path": relative,
                                    "sha256": hashlib.sha256(payload).hexdigest(),
                                }
                                for relative, payload in expected.items()
                            ],
                        },
                    }
                ),
                encoding="utf-8",
            )
            audit_sha256 = file_digest(audit, "sha256")

            with self.assertRaises(ValueError):
                restore_changed_images(
                    archive=archive,
                    source_root=source_root,
                    dataset_manifest_path=dataset_manifest,
                    audit_path=audit,
                    expected_audit_sha256=audit_sha256,
                    expected_mismatches=0,
                )
            self.assertEqual(
                (source_root / "training" / "a.jpg").read_bytes(), b"changed-a"
            )

            report = restore_changed_images(
                archive=archive,
                source_root=source_root,
                dataset_manifest_path=dataset_manifest,
                audit_path=audit,
                expected_audit_sha256=audit_sha256,
                expected_mismatches=1,
            )
            self.assertEqual(report["restored_count"], 1)
            self.assertEqual(report["post_restore_mismatch_count"], 0)
            for relative, payload in expected.items():
                self.assertEqual(
                    source_root.joinpath(*PurePosixPath(relative).parts).read_bytes(),
                    payload,
                )


class SplitTest(unittest.TestCase):
    @staticmethod
    def _numeric_official_two_way_fixture(root: Path):
        """Create overlapping numeric names and scenes spanning both source dirs."""

        source = root / "strawberries"
        rows = ["image,group_id"]
        labels = "0 0.5 0.5 0.2 0.2\n1 0.4 0.4 0.1 0.1\n"
        for directory_name in ("training", "validation"):
            for scene in range(12):
                for frame in range(2):
                    image_id = scene * 2 + frame
                    name = f"{image_id:04d}"
                    write_sample(source / directory_name, name, labels)
                    rows.append(
                        f"{directory_name}/{name}.jpg,scene_{scene:02d}"
                    )
        groups_csv = root / "groups.csv"
        groups_csv.write_text("\n".join(rows) + "\n", encoding="utf-8")
        return source, groups_csv

    @staticmethod
    def _content_bound_fixture(root: Path):
        source = root / "raw"
        splits = {}
        for split_name in ("train", "val", "test"):
            directory = source / split_name
            write_sample(
                directory,
                "fruit",
                "0 0.5 0.5 0.2 0.2\n1 0.4 0.4 0.1 0.1\n",
            )
            splits[split_name] = [
                Sample(
                    image=directory / "fruit.jpg",
                    label=directory / "fruit.txt",
                    relative_image=f"{split_name}/fruit.jpg",
                    group_id=f"{split_name}_scene",
                    class_counts=((0, 1), (1, 1)),
                )
            ]
        return source, splits

    def test_force_refuses_to_clear_an_unmarked_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "raw"
            output = root / "processed"
            source.mkdir()
            output.mkdir()
            user_file = output / "keep.txt"
            user_file.write_text("user data", encoding="utf-8")
            splits = {
                split_name: [
                    Sample(
                        image=source / f"{split_name}.jpg",
                        label=None,
                        relative_image=f"{split_name}.jpg",
                        group_id=f"{split_name}_scene",
                        class_counts=((0, 1), (1, 1)),
                    )
                ]
                for split_name in ("train", "val", "test")
            }
            with self.assertRaises(ValueError):
                materialize_splits(
                    source,
                    splits,
                    output,
                    "test",
                    force=True,
                )
            self.assertEqual(user_file.read_text(encoding="utf-8"), "user data")

    def test_group_allocator_tracks_requested_ratios(self):
        samples = [
            Sample(
                image=Path(f"{index}.jpg"),
                label=None,
                relative_image=f"{index}.jpg",
                group_id=str(index),
                class_counts=((index % 2, 1),),
            )
            for index in range(100)
        ]
        splits = group_stratified_split(
            samples, {"train": 0.70, "val": 0.15, "test": 0.15}, 20260710
        )
        self.assertLessEqual(abs(len(splits["train"]) - 70), 1)
        self.assertLessEqual(abs(len(splits["val"]) - 15), 1)
        self.assertLessEqual(abs(len(splits["test"]) - 15), 1)
        for split_samples in splits.values():
            covered = {
                class_id
                for sample in split_samples
                for class_id, count in sample.class_counts
                if count > 0
            }
            self.assertEqual(covered, {0, 1})

    def test_official_heldout_becomes_test_and_peduncle_is_filtered(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "source" / "strawberries"
            for index in range(12):
                write_sample(
                    source / "training",
                    f"scene{index}_frame000",
                    f"{index % 2} 0.5 0.5 0.2 0.2\n2 0.4 0.4 0.1 0.1\n",
                )
            for index in range(4):
                write_sample(
                    source / "validation",
                    f"heldout_scene{index}_frame000",
                    f"{index % 2} 0.5 0.5 0.2 0.2\n",
                )

            dataset_root, splits, strategy = make_splits(source.parent, seed=20260710)
            self.assertEqual(strategy, "official_heldout_plus_seeded_validation")
            self.assertEqual(len(splits["test"]), 4)
            self.assertTrue(
                all("validation" in sample.relative_image for sample in splits["test"])
            )
            self.assertTrue(splits["train"])
            self.assertTrue(splits["val"])

            output = Path(temporary) / "processed"
            report = materialize_splits(dataset_root, splits, output, strategy)
            self.assertEqual(sum(report["counts"].values()), 16)
            self.assertTrue(report["validation"]["valid"])
            self.assertEqual(report["validation"]["violations"], [])
            output_labels = list((output / "labels").rglob("*.txt"))
            self.assertTrue(output_labels)
            output_lines = [
                line
                for path in output_labels
                for line in path.read_text(encoding="utf-8").splitlines()
            ]
            self.assertFalse(any(line.startswith("2 ") for line in output_lines))
            source_label = source / "training" / "scene0_frame000.txt"
            self.assertIn("2 0.4", source_label.read_text(encoding="utf-8"))
            self.assertTrue((output / "dataset.yaml").is_file())
            self.assertTrue((output / "split_manifest.json").is_file())
            dataset_yaml = (output / "dataset.yaml").read_text(encoding="utf-8")
            self.assertNotIn("path:", dataset_yaml)
            self.assertIn("train: images/train", dataset_yaml)
            persisted = json.loads(
                (output / "split_manifest.json").read_text(encoding="utf-8")
            )
            self.assertTrue(persisted["validation"]["valid"])
            self.assertRegex(
                persisted["content_binding"]["canonical_dataset_sha256"],
                r"^[0-9a-f]{64}$",
            )
            for split_samples in persisted["splits"].values():
                for sample in split_samples:
                    self.assertRegex(sample["source_image_sha256"], r"^[0-9a-f]{64}$")
                    self.assertRegex(sample["output_image_sha256"], r"^[0-9a-f]{64}$")
                    self.assertRegex(sample["source_label_sha256"], r"^[0-9a-f]{64}$")
                    self.assertRegex(sample["output_label_sha256"], r"^[0-9a-f]{64}$")

    def test_zero_area_v1_boxes_are_dropped_from_counts_and_materialized_labels(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "raw"
            splits = {}
            for split_name in ("train", "val", "test"):
                directory = source / split_name
                write_sample(
                    directory,
                    "fruit",
                    "0 0.5 0.5 0.2 0.2\n"
                    "1 0.6 0.6 0.1 0.1\n"
                    "1 0.4 0.4 0.0 0.1\n"
                    "2 0.3 0.3 0.1 0.1\n",
                )
                splits[split_name] = discover_samples(
                    directory,
                    source,
                    {
                        f"{split_name}/fruit.jpg": f"{split_name}_scene",
                    },
                )
                self.assertEqual(dict(splits[split_name][0].counts), {0: 1, 1: 1})

            output = root / "processed"
            materialize_splits(source, splits, output, "zero_area_filter")
            for label in (output / "labels").rglob("*.txt"):
                self.assertEqual(
                    label.read_text(encoding="utf-8").strip(),
                    "0 0.5 0.5 0.2 0.2\n1 0.6 0.6 0.1 0.1",
                )

    def test_resplit_all_combines_official_directories_at_group_boundaries(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, groups_csv = self._numeric_official_two_way_fixture(
                Path(temporary)
            )

            _, splits, strategy = make_splits(
                source,
                seed=20260710,
                groups_csv=groups_csv,
                resplit_all=True,
            )

            self.assertEqual(strategy, "seeded_group_stratified_full_resplit")
            self.assertEqual(sum(len(samples) for samples in splits.values()), 48)
            targets = {"train": 33.6, "val": 7.2, "test": 7.2}
            for split_name, split_samples in splits.items():
                # Each four-image scene is indivisible, so a single-group
                # tolerance is the tightest generally valid ratio assertion.
                self.assertLessEqual(
                    abs(len(split_samples) - targets[split_name]),
                    4,
                )
                source_directories = {
                    Path(sample.relative_image).parts[0]
                    for sample in split_samples
                }
                self.assertEqual(source_directories, {"training", "validation"})

            validation = validate_splits(splits)
            self.assertTrue(validation["valid"])
            self.assertEqual(validation["group_leakage"], [])
            self.assertEqual(validation["source_overlap"], [])

    def test_resplit_all_is_deterministic_and_group_disjoint(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, groups_csv = self._numeric_official_two_way_fixture(
                Path(temporary)
            )

            first = make_splits(
                source,
                seed=20260710,
                groups_csv=groups_csv,
                resplit_all=True,
            )[1]
            second = make_splits(
                source,
                seed=20260710,
                groups_csv=groups_csv,
                resplit_all=True,
            )[1]
            membership = lambda splits: {
                sample.relative_image: split_name
                for split_name, split_samples in splits.items()
                for sample in split_samples
            }
            self.assertEqual(membership(first), membership(second))

            group_destinations = {}
            for split_name, split_samples in first.items():
                covered = {
                    class_id
                    for sample in split_samples
                    for class_id, count in sample.class_counts
                    if count > 0
                }
                self.assertEqual(covered, {0, 1})
                for sample in split_samples:
                    previous = group_destinations.setdefault(
                        sample.group_id, split_name
                    )
                    self.assertEqual(previous, split_name)

    def test_resplit_all_requires_complete_mapping_across_official_directories(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, groups_csv = self._numeric_official_two_way_fixture(root)
            missing_path = "validation/0023.jpg"
            rows = [
                row
                for row in groups_csv.read_text(encoding="utf-8").splitlines()
                if not row.startswith(f"{missing_path},")
            ]
            incomplete_csv = root / "incomplete_groups.csv"
            incomplete_csv.write_text("\n".join(rows) + "\n", encoding="utf-8")

            with self.assertRaisesRegex(
                ValueError,
                r"groups CSV has no scene assignment for validation/0023\.jpg",
            ):
                make_splits(
                    source,
                    seed=20260710,
                    groups_csv=incomplete_csv,
                    resplit_all=True,
                )

    def test_resplit_all_rejects_group_rows_outside_source_pool(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, groups_csv = self._numeric_official_two_way_fixture(root)
            with groups_csv.open("a", encoding="utf-8") as stream:
                stream.write("training/not_present.jpg,stale_scene\n")

            with self.assertRaisesRegex(
                ValueError,
                r"assignments for images outside the source pool",
            ):
                make_splits(
                    source,
                    seed=20260710,
                    groups_csv=groups_csv,
                    resplit_all=True,
                )

    def test_resplit_all_requires_authoritative_groups_csv(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, _ = self._numeric_official_two_way_fixture(Path(temporary))
            with self.assertRaisesRegex(
                ValueError, r"--resplit-all requires.*--groups-csv"
            ):
                make_splits(source, seed=20260710, resplit_all=True)

    def test_resplit_all_applies_audited_exclusions_and_binds_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, groups_csv = self._numeric_official_two_way_fixture(root)
            exclusions_csv = root / "exclusions.csv"
            exclusions_csv.write_text(
                "image,reason,representative\n"
                "training/0000.jpg,near_duplicate,validation/0000.jpg\n"
                "training/0002.jpg,near_duplicate,validation/0002.jpg\n",
                encoding="utf-8",
            )

            dataset_root, splits, strategy = make_splits(
                source,
                seed=20260710,
                groups_csv=groups_csv,
                resplit_all=True,
                exclusions_csv=exclusions_csv,
            )
            retained = {
                sample.relative_image
                for split_samples in splits.values()
                for sample in split_samples
            }
            self.assertEqual(len(retained), 46)
            self.assertNotIn("training/0000.jpg", retained)
            self.assertNotIn("training/0002.jpg", retained)
            self.assertIn("validation/0000.jpg", retained)
            self.assertIn("validation/0002.jpg", retained)

            first_output = root / "processed_first"
            first = materialize_splits(
                dataset_root,
                splits,
                first_output,
                strategy,
                image_mode="copy",
                exclusions_csv=exclusions_csv,
            )
            self.assertEqual(first["exclusions_csv"], "exclusions.csv")
            self.assertEqual(
                first["exclusions_csv_sha256"],
                file_digest(exclusions_csv, "sha256"),
            )
            self.assertEqual(first["source_image_count"], 48)
            self.assertEqual(first["excluded_count"], 2)
            self.assertEqual(first["retained_count"], 46)
            verification = verify_materialized_split_manifest(
                first_output / "split_manifest.json"
            )
            self.assertEqual(verification["source_image_count"], 48)
            self.assertEqual(verification["excluded_count"], 2)
            self.assertEqual(verification["retained_count"], 46)

            # The reason is provenance even when membership is unchanged, so
            # its CSV hash must affect the canonical dataset identity.
            exclusions_csv.write_text(
                exclusions_csv.read_text(encoding="utf-8").replace(
                    "near_duplicate", "duplicate_after_manual_review"
                ),
                encoding="utf-8",
            )
            second_output = root / "processed_second"
            second = materialize_splits(
                dataset_root,
                splits,
                second_output,
                strategy,
                image_mode="copy",
                exclusions_csv=exclusions_csv,
            )
            self.assertNotEqual(
                first["content_binding"]["canonical_dataset_sha256"],
                second["content_binding"]["canonical_dataset_sha256"],
            )

    def test_exclusion_without_representative_is_a_valid_whole_image_drop(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, groups_csv = self._numeric_official_two_way_fixture(root)
            exclusions_csv = root / "missing_annotation.csv"
            exclusions_csv.write_text(
                "image,reason,representative\n"
                "training/0000.jpg,missing_annotation,\n",
                encoding="utf-8",
            )
            dataset_root, splits, strategy = make_splits(
                source,
                groups_csv=groups_csv,
                resplit_all=True,
                exclusions_csv=exclusions_csv,
            )
            retained = {
                sample.relative_image
                for split_samples in splits.values()
                for sample in split_samples
            }
            self.assertNotIn("training/0000.jpg", retained)
            self.assertEqual(len(retained), 47)
            report = materialize_splits(
                dataset_root,
                splits,
                root / "processed",
                strategy,
                image_mode="copy",
                exclusions_csv=exclusions_csv,
            )
            self.assertEqual(report["excluded_count"], 1)
            self.assertEqual(report["retained_count"], 47)

    def test_exclusion_manifest_rejects_orphans_cross_group_chains_and_cycles(self):
        cases = (
            (
                "orphan_image",
                "training/9999.jpg,duplicate,training/0000.jpg\n",
                r"images outside the source pool",
            ),
            (
                "orphan_representative",
                "training/0000.jpg,duplicate,training/9999.jpg\n",
                r"representatives outside the source pool",
            ),
            (
                "self_representative",
                "training/0000.jpg,duplicate,training/0000.jpg\n",
                r"must differ",
            ),
            (
                "cross_group",
                "training/0000.jpg,duplicate,training/0002.jpg\n",
                r"same group",
            ),
            (
                "chain",
                "training/0000.jpg,duplicate,validation/0000.jpg\n"
                "validation/0000.jpg,duplicate,training/0001.jpg\n",
                r"chains are forbidden",
            ),
            (
                "cycle",
                "training/0000.jpg,duplicate,validation/0000.jpg\n"
                "validation/0000.jpg,duplicate,training/0000.jpg\n",
                r"representative cycle",
            ),
        )
        for name, rows, message in cases:
            with self.subTest(case=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source, groups_csv = self._numeric_official_two_way_fixture(root)
                exclusions_csv = root / "exclusions.csv"
                exclusions_csv.write_text(
                    "image,reason,representative\n" + rows,
                    encoding="utf-8",
                )
                with self.assertRaisesRegex(ValueError, message):
                    make_splits(
                        source,
                        groups_csv=groups_csv,
                        resplit_all=True,
                        exclusions_csv=exclusions_csv,
                    )

    def test_exclusions_are_rejected_outside_full_resplit(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, groups_csv = self._numeric_official_two_way_fixture(root)
            exclusions_csv = root / "exclusions.csv"
            exclusions_csv.write_text(
                "image,reason,representative\n"
                "training/0000.jpg,duplicate,validation/0000.jpg\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                ValueError, r"--exclusions-csv requires --resplit-all"
            ):
                make_splits(
                    source,
                    groups_csv=groups_csv,
                    exclusions_csv=exclusions_csv,
                )

    def test_resplit_all_cli_forwards_policy_and_records_strategy(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, groups_csv = self._numeric_official_two_way_fixture(root)
            processed = root / "processed"
            output = StringIO()
            with redirect_stdout(output):
                return_code = split_main(
                    [
                        "--source",
                        str(source),
                        "--output-dir",
                        str(processed),
                        "--groups-csv",
                        str(groups_csv),
                        "--resplit-all",
                    ]
                )

            payload = json.loads(output.getvalue())
            self.assertEqual(return_code, 0)
            self.assertEqual(
                payload["strategy"], "seeded_group_stratified_full_resplit"
            )
            self.assertEqual(sum(payload["counts"].values()), 48)
            self.assertTrue(payload["validation"]["valid"])
            persisted = json.loads(
                (processed / "split_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(persisted["strategy"], payload["strategy"])
            self.assertEqual(persisted["counts"], payload["counts"])
            self.assertEqual(persisted["image_placement"], {"copy": 48})

    def test_resplit_all_cli_records_exclusion_provenance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, groups_csv = self._numeric_official_two_way_fixture(root)
            exclusions_csv = root / "exclusions.csv"
            exclusions_csv.write_text(
                "image,reason,representative\n"
                "training/0000.jpg,duplicate,validation/0000.jpg\n",
                encoding="utf-8",
            )
            processed = root / "processed"
            output = StringIO()
            with redirect_stdout(output):
                return_code = split_main(
                    [
                        "--source",
                        str(source),
                        "--output-dir",
                        str(processed),
                        "--groups-csv",
                        str(groups_csv),
                        "--exclusions-csv",
                        str(exclusions_csv),
                        "--resplit-all",
                        "--image-mode",
                        "copy",
                    ]
                )
            payload = json.loads(output.getvalue())
            self.assertEqual(return_code, 0)
            self.assertEqual(payload["source_image_count"], 48)
            self.assertEqual(payload["excluded_count"], 1)
            self.assertEqual(payload["retained_count"], 47)
            self.assertEqual(sum(payload["counts"].values()), 47)
            self.assertEqual(payload["exclusions_csv"], "exclusions.csv")
            self.assertEqual(
                payload["exclusions_csv_sha256"],
                file_digest(exclusions_csv, "sha256"),
            )

    def test_canonical_digest_is_independent_of_absolute_roots(self):
        with tempfile.TemporaryDirectory() as temporary:
            digests = []
            split_digests = []
            for machine_name in ("machine_a", "machine_b"):
                machine_root = Path(temporary) / machine_name
                source, splits = self._content_bound_fixture(machine_root)
                output = machine_root / "processed"
                report = materialize_splits(
                    source, splits, output, "root_independence", image_mode="copy"
                )
                digests.append(
                    report["content_binding"]["canonical_dataset_sha256"]
                )
                split_digests.append(
                    report["content_binding"]["canonical_split_sha256"]
                )
                verified = verify_materialized_split_manifest(
                    output / "split_manifest.json"
                )
                self.assertEqual(
                    verified["canonical_dataset_sha256"], digests[-1]
                )
            self.assertEqual(digests[0], digests[1])
            self.assertEqual(split_digests[0], split_digests[1])

    def test_relocated_dataset_ignores_recorded_absolute_roots(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root / "original")
            output = root / "original" / "processed"
            materialize_splits(source, splits, output, "relocation", image_mode="copy")
            expected = verify_materialized_split_manifest(
                output / "split_manifest.json"
            )["canonical_dataset_sha256"]
            relocated = root / "relocated" / "processed"
            shutil.copytree(output, relocated)
            manifest_path = relocated / "split_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["source_root"] = "/different/machine/raw"
            manifest["output_root"] = "/different/machine/processed"
            manifest_path.write_text(
                json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
            )
            verified = verify_materialized_split_manifest(manifest_path)
            self.assertEqual(verified["canonical_dataset_sha256"], expected)
            self.assertEqual(verified["dataset_root"], str(relocated.resolve()))

    def test_hardlink_source_mutation_is_detected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root)
            output = root / "processed"
            report = materialize_splits(
                source, splits, output, "hardlink_guard", image_mode="hardlink"
            )
            self.assertEqual(report["image_placement"], {"hardlink": 3})
            train_source = source / "train" / "fruit.jpg"
            train_output = output / report["splits"]["train"][0]["output_image"]
            self.assertTrue(os.path.samefile(train_source, train_output))
            train_source.write_bytes(b"hardlink-content-was-mutated")
            with self.assertRaises(DatasetVerificationError) as caught:
                verify_materialized_split_manifest(output / "split_manifest.json")
            codes = {item["code"] for item in caught.exception.report["violations"]}
            self.assertIn("CONTENT_HASH_MISMATCH", codes)

    def test_missing_jpeg_eoi_is_normalized_only_in_processed_copy(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root)
            train_source = source / "train" / "fruit.jpg"
            original = b"incomplete-jpeg-source"
            train_source.write_bytes(original)
            output = root / "processed"

            report = materialize_splits(source, splits, output, "jpeg_eoi_guard")
            record = report["splits"]["train"][0]
            train_output = output / record["output_image"]

            self.assertEqual(train_source.read_bytes(), original)
            self.assertEqual(train_output.read_bytes(), original + b"\xff\xd9")
            self.assertFalse(os.path.samefile(train_source, train_output))
            self.assertEqual(record["image_transform"], "append_jpeg_eoi")
            self.assertEqual(
                report["image_normalization"]["policy"],
                "append_jpeg_eoi_if_missing_v1",
            )
            self.assertEqual(
                report["image_transforms"],
                {"append_jpeg_eoi": 1, "identity": 2},
            )
            self.assertEqual(report["image_placement"], {"copy": 3})
            verification = verify_materialized_split_manifest(
                output / "split_manifest.json"
            )
            self.assertTrue(verification["valid"])

    def test_hardlink_mode_still_copies_an_incomplete_jpeg(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root)
            train_source = source / "train" / "fruit.jpg"
            train_source.write_bytes(b"incomplete-jpeg")
            output = root / "processed"

            report = materialize_splits(
                source,
                splits,
                output,
                "hardlink_jpeg_eoi_guard",
                image_mode="hardlink",
            )
            train_record = report["splits"]["train"][0]
            train_output = output / train_record["output_image"]
            val_record = report["splits"]["val"][0]
            val_output = output / val_record["output_image"]

            self.assertEqual(report["image_placement"], {"copy": 1, "hardlink": 2})
            self.assertEqual(train_record["image_transform"], "append_jpeg_eoi")
            self.assertFalse(os.path.samefile(train_source, train_output))
            self.assertTrue(
                os.path.samefile(source / "val" / "fruit.jpg", val_output)
            )
            self.assertTrue(
                verify_materialized_split_manifest(
                    output / "split_manifest.json"
                )["valid"]
            )

    def test_image_normalization_metadata_tampering_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root)
            output = root / "processed"
            materialize_splits(source, splits, output, "normalization_guard")
            manifest_path = output / "split_manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["image_normalization"]["marker_hex"] = "0000"
            manifest["image_transforms"]["identity"] = 999
            manifest_path.write_text(
                json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
            )

            with self.assertRaises(DatasetVerificationError) as caught:
                verify_materialized_split_manifest(manifest_path)
            codes = {item["code"] for item in caught.exception.report["violations"]}
            self.assertIn("MANIFEST_SCHEMA_ERROR", codes)
            self.assertIn("IMAGE_TRANSFORM_COUNT_MISMATCH", codes)

    def test_materialized_label_mutation_is_detected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root)
            output = root / "processed"
            report = materialize_splits(
                source, splits, output, "label_guard", image_mode="copy"
            )
            label_path = output / report["splits"]["val"][0]["output_label"]
            label_path.write_text("0 0.1 0.1 0.1 0.1\n", encoding="utf-8")
            with self.assertRaises(DatasetVerificationError) as caught:
                verify_materialized_split_manifest(output / "split_manifest.json")
            mismatches = [
                item
                for item in caught.exception.report["violations"]
                if item["code"] == "CONTENT_HASH_MISMATCH"
            ]
            self.assertEqual(len(mismatches), 1)
            self.assertEqual(mismatches[0]["path"], label_path.relative_to(output).as_posix())

    def test_unregistered_yolo_inputs_are_rejected(self):
        cases = (
            ("image_and_label", True, True, {"images", "labels"}),
            ("image_only", True, False, {"images"}),
            ("label_only", False, True, {"labels"}),
        )
        for case_name, add_image, add_label, expected_kinds in cases:
            with self.subTest(case=case_name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source, splits = self._content_bound_fixture(root)
                output = root / "processed"
                materialize_splits(
                    source, splits, output, "unregistered_guard", image_mode="copy"
                )
                if add_image:
                    (output / "images" / "train" / "extra.jpg").write_bytes(
                        b"unregistered-image"
                    )
                if add_label:
                    (output / "labels" / "train" / "extra.txt").write_text(
                        "0 0.5 0.5 0.2 0.2\n", encoding="utf-8"
                    )
                with self.assertRaises(DatasetVerificationError) as caught:
                    verify_materialized_split_manifest(output / "split_manifest.json")
                unregistered = [
                    item
                    for item in caught.exception.report["violations"]
                    if item["code"] == "UNREGISTERED_INPUT_FILE"
                ]
                self.assertEqual({item["kind"] for item in unregistered}, expected_kinds)
                self.assertTrue(
                    all("extra" in item["paths"][0] for item in unregistered)
                )

    def test_yolo_cache_files_do_not_invalidate_dataset(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root)
            output = root / "processed"
            materialize_splits(source, splits, output, "cache_boundary", image_mode="copy")
            for split_name in ("train", "val", "test"):
                (output / "labels" / f"{split_name}.cache").write_bytes(
                    b"ultralytics-cache-metadata"
                )
            verification = verify_materialized_split_manifest(
                output / "split_manifest.json"
            )
            self.assertTrue(verification["valid"])

    def test_wrong_suffix_and_nested_directory_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, splits = self._content_bound_fixture(root)
            output = root / "processed"
            materialize_splits(source, splits, output, "layout_guard", image_mode="copy")
            (output / "images" / "train" / "not_an_image.txt").write_text(
                "unexpected", encoding="utf-8"
            )
            (output / "labels" / "test" / "nested").mkdir()
            with self.assertRaises(DatasetVerificationError) as caught:
                verify_materialized_split_manifest(output / "split_manifest.json")
            codes = {item["code"] for item in caught.exception.report["violations"]}
            self.assertIn("WRONG_INPUT_SUFFIX", codes)
            self.assertIn("UNEXPECTED_DATASET_DIRECTORY", codes)

    def test_numeric_frame_sequence_requires_authoritative_groups_csv(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "numeric"
            rows = ["image,group_id"]
            for index in range(12):
                write_sample(
                    source,
                    f"{index:04d}",
                    f"{index % 2} 0.5 0.5 0.2 0.2\n",
                )
                rows.append(f"{index:04d}.jpg,scene_{index // 2}")

            with self.assertRaisesRegex(ValueError, "--groups-csv"):
                make_splits(source, seed=20260710)

            groups_csv = Path(temporary) / "groups.csv"
            groups_csv.write_text("\n".join(rows) + "\n", encoding="utf-8")
            _, splits, strategy = make_splits(
                source, seed=20260710, groups_csv=groups_csv
            )
            self.assertEqual(strategy, "seeded_group_stratified_fallback")
            self.assertTrue(validate_splits(splits)["valid"])

    def test_arbitrary_identifiers_cannot_imply_independent_scenes(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "ambiguous"
            for index, name in enumerate(("alpha", "bravo", "charlie", "delta")):
                write_sample(
                    source,
                    name,
                    f"{index % 2} 0.5 0.5 0.2 0.2\n",
                )
            with self.assertRaisesRegex(ValueError, "--groups-csv"):
                make_splits(source, seed=20260710)

    def test_validation_reports_class_coverage_and_group_leakage(self):
        both = ((0, 1), (1, 1))
        splits = {
            "train": [Sample(Path("a.jpg"), None, "a.jpg", "shared", both)],
            "val": [Sample(Path("b.jpg"), None, "b.jpg", "shared", ((0, 1),))],
            "test": [Sample(Path("c.jpg"), None, "c.jpg", "test", both)],
        }
        report = validate_splits(splits)
        self.assertFalse(report["valid"])
        codes = {item["code"] for item in report["violations"]}
        self.assertIn("GROUP_LEAKAGE", codes)
        self.assertIn("MISSING_CLASS_COVERAGE", codes)
        json.dumps(report)
        with self.assertRaises(SplitValidationError):
            materialize_splits(Path("raw"), splits, Path("processed"), "test")

    def test_split_cli_reports_failure_as_json(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "numeric"
            for index in range(6):
                write_sample(
                    source,
                    f"{index:04d}",
                    f"{index % 2} 0.5 0.5 0.2 0.2\n",
                )
            output = StringIO()
            with redirect_stdout(output):
                return_code = split_main(
                    [
                        "--source",
                        str(source),
                        "--output-dir",
                        str(Path(temporary) / "processed"),
                    ]
                )
            payload = json.loads(output.getvalue())
            self.assertEqual(return_code, 2)
            self.assertFalse(payload["validation"]["valid"])
            self.assertEqual(
                payload["validation"]["violations"][0]["code"],
                "SPLIT_GENERATION_ERROR",
            )

    def test_fallback_is_scene_grouped_and_deterministic(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "flat"
            for scene in range(6):
                for frame in range(2):
                    label = f"{scene % 2} 0.5 0.5 0.2 0.2\n"
                    write_sample(source, f"scene{scene}_frame{frame:03d}", label)

            samples = discover_samples(source, source)
            first = group_stratified_split(
                samples, {"train": 0.70, "val": 0.15, "test": 0.15}, 20260710
            )
            second = group_stratified_split(
                list(reversed(samples)), {"train": 0.70, "val": 0.15, "test": 0.15}, 20260710
            )
            first_paths = {
                split: [sample.relative_image for sample in first[split]]
                for split in ("train", "val", "test")
            }
            second_paths = {
                split: [sample.relative_image for sample in second[split]]
                for split in ("train", "val", "test")
            }
            self.assertEqual(first_paths, second_paths)
            group_to_split = {}
            for split, split_samples in first.items():
                covered = {
                    class_id
                    for sample in split_samples
                    for class_id, count in sample.class_counts
                    if count > 0
                }
                self.assertEqual(covered, {0, 1})
                for sample in split_samples:
                    previous = group_to_split.setdefault(sample.group_id, split)
                    self.assertEqual(previous, split)
            self.assertTrue(all(first[split] for split in ("train", "val", "test")))


if __name__ == "__main__":
    unittest.main()
