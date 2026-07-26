"""Standard-library data utilities for the strawberry perception pipeline."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
import csv
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
from typing import Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple
from urllib.request import urlopen
import zipfile


IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"})
JPEG_SUFFIXES = frozenset({".jpg", ".jpeg"})
JPEG_EOI = b"\xff\xd9"
IMAGE_NORMALIZATION = {
    "policy": "append_jpeg_eoi_if_missing_v1",
    "jpeg_suffixes": sorted(JPEG_SUFFIXES),
    "marker_hex": JPEG_EOI.hex(),
}
V1_CLASS_IDS = frozenset({0, 1})
SPLIT_NAMES = ("train", "val", "test")
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


class SplitValidationError(ValueError):
    """Raised with a machine-readable report when a split contract is invalid."""

    def __init__(self, report: Mapping[str, object]):
        self.report = report
        violations = report.get("violations", [])
        codes = [
            str(item.get("code", "UNKNOWN"))
            for item in violations
            if isinstance(item, Mapping)
        ]
        detail = ", ".join(codes) if codes else "UNKNOWN"
        super().__init__(f"dataset split validation failed: {detail}")


class DatasetVerificationError(ValueError):
    """Raised when a materialized dataset no longer matches its manifest."""

    def __init__(self, report: Mapping[str, object]):
        self.report = report
        violations = report.get("violations", [])
        codes = [
            str(item.get("code", "UNKNOWN"))
            for item in violations
            if isinstance(item, Mapping)
        ]
        detail = ", ".join(codes) if codes else "UNKNOWN"
        super().__init__(f"materialized dataset verification failed: {detail}")


@dataclass(frozen=True)
class Sample:
    image: Path
    label: Optional[Path]
    relative_image: str
    group_id: str
    class_counts: Tuple[Tuple[int, int], ...]

    @property
    def counts(self) -> Counter:
        return Counter(dict(self.class_counts))


@dataclass(frozen=True)
class Exclusion:
    """One audited duplicate exclusion and its retained representative."""

    image: str
    reason: str
    representative: Optional[str]


def load_manifest(path: Path) -> Mapping[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("schema_version") != 1:
        raise ValueError("unsupported or missing dataset manifest schema_version")
    if not manifest.get("files"):
        raise ValueError("dataset manifest must contain at least one file")
    return manifest


def file_digest(path: Path, algorithm: str, chunk_size: int = 8 * 1024 * 1024) -> str:
    try:
        digest = hashlib.new(algorithm.lower())
    except ValueError as error:
        raise ValueError(f"unsupported checksum algorithm: {algorithm}") from error
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def verify_file(path: Path, algorithm: str, expected: str) -> Mapping[str, object]:
    if not path.is_file():
        return {
            "path": str(path),
            "exists": False,
            "algorithm": algorithm.lower(),
            "expected": expected.lower(),
            "actual": None,
            "valid": False,
        }
    actual = file_digest(path, algorithm)
    return {
        "path": str(path),
        "exists": True,
        "size_bytes": path.stat().st_size,
        "algorithm": algorithm.lower(),
        "expected": expected.lower(),
        "actual": actual.lower(),
        "valid": actual.lower() == expected.lower(),
    }


def download_file(
    url: str,
    destination: Path,
    algorithm: str,
    expected: str,
    force: bool = False,
) -> Mapping[str, object]:
    """Stream an artifact to a temporary file and atomically publish it."""

    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() and not force:
        status = dict(verify_file(destination, algorithm, expected))
        if status["valid"]:
            status["downloaded"] = False
            return status
        raise FileExistsError(
            f"{destination} exists but its checksum is invalid; pass --force to replace it"
        )

    partial = destination.with_name(destination.name + ".part")
    if partial.exists():
        partial.unlink()
    try:
        # The URL comes from the repository-pinned manifest, not user input.
        with urlopen(url) as response, partial.open("wb") as output:  # nosec B310
            shutil.copyfileobj(response, output, length=8 * 1024 * 1024)
        status = dict(verify_file(partial, algorithm, expected))
        if not status["valid"]:
            raise ValueError(
                f"checksum mismatch for {destination.name}: expected {expected}, "
                f"got {status['actual']}"
            )
        os.replace(partial, destination)
    finally:
        if partial.exists():
            partial.unlink()

    final_status = dict(verify_file(destination, algorithm, expected))
    final_status["downloaded"] = True
    return final_status


def extract_zip_safely(archive: Path, destination: Path, force: bool = False) -> None:
    """Extract a verified ZIP while rejecting absolute and traversal paths."""

    destination.mkdir(parents=True, exist_ok=True)
    destination_root = destination.resolve()
    with zipfile.ZipFile(archive, "r") as bundle:
        for member in bundle.infolist():
            member_path = (destination / member.filename).resolve()
            try:
                member_path.relative_to(destination_root)
            except ValueError as error:
                raise ValueError(f"unsafe ZIP member path: {member.filename}") from error
            if member_path.exists() and not force:
                raise FileExistsError(
                    f"{member_path} already exists; use --force only after checking "
                    "the target directory"
                )
        bundle.extractall(destination)


def _normalise_relative(path: str) -> str:
    normalised = path.replace("\\", "/").lstrip("./")
    return str(Path(normalised).as_posix())


def read_groups_csv(path: Optional[Path]) -> Mapping[str, str]:
    if path is None:
        return {}
    groups: Dict[str, str] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or not {"image", "group_id"}.issubset(reader.fieldnames):
            raise ValueError("groups CSV must have image and group_id columns")
        for row in reader:
            image = _normalise_relative(row["image"].strip())
            group_id = row["group_id"].strip()
            if not image or not group_id:
                raise ValueError("groups CSV contains an empty image or group_id")
            if image in groups and groups[image] != group_id:
                raise ValueError(f"conflicting group assignments for {image}")
            groups[image] = group_id
    return groups


def _normalise_csv_image(value: str, field_name: str) -> str:
    raw = value.strip().replace("\\", "/")
    relative = PurePosixPath(raw)
    if (
        not raw
        or relative.is_absolute()
        or re.match(r"^[A-Za-z]:", raw)
        or ".." in relative.parts
        or str(relative) in {"", "."}
    ):
        raise ValueError(f"exclusions CSV contains an unsafe {field_name}: {value!r}")
    return relative.as_posix()


def read_exclusions_csv(path: Optional[Path]) -> Mapping[str, Exclusion]:
    """Read the audited duplicate-exclusion manifest without applying it."""

    if path is None:
        return {}
    exclusions: Dict[str, Exclusion] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"image", "reason", "representative"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(
                "exclusions CSV must have image, reason, and representative columns"
            )
        for line_number, row in enumerate(reader, start=2):
            image = _normalise_csv_image(row["image"], "image")
            reason = row["reason"].strip()
            representative_value = row["representative"].strip()
            representative = (
                _normalise_csv_image(representative_value, "representative")
                if representative_value
                else None
            )
            if not reason:
                raise ValueError(
                    f"exclusions CSV contains an empty reason at line {line_number}"
                )
            if image in exclusions:
                raise ValueError(f"exclusions CSV contains duplicate image {image}")
            exclusions[image] = Exclusion(image, reason, representative)
    return exclusions


def _apply_exclusions(
    samples: Sequence[Sample], exclusions: Mapping[str, Exclusion]
) -> List[Sample]:
    """Validate an exclusion graph and return only its retained source samples."""

    if not exclusions:
        return list(samples)
    by_image = {sample.relative_image: sample for sample in samples}
    if len(by_image) != len(samples):
        raise ValueError("source pool contains duplicate relative image paths")

    missing_images = sorted(set(exclusions) - set(by_image), key=str.lower)
    if missing_images:
        examples = ", ".join(missing_images[:3])
        raise ValueError(
            "exclusions CSV contains images outside the source pool "
            f"(examples: {examples})"
        )
    missing_representatives = sorted(
        {
            item.representative
            for item in exclusions.values()
            if item.representative is not None
            and item.representative not in by_image
        },
        key=str.lower,
    )
    if missing_representatives:
        examples = ", ".join(missing_representatives[:3])
        raise ValueError(
            "exclusions CSV contains representatives outside the source pool "
            f"(examples: {examples})"
        )

    for item in exclusions.values():
        if item.representative is None:
            continue
        if item.image == item.representative:
            raise ValueError(
                f"excluded image must differ from its representative: {item.image}"
            )
        image_group = by_image[item.image].group_id
        representative_group = by_image[item.representative].group_id
        if image_group != representative_group:
            raise ValueError(
                "excluded image and representative must belong to the same group: "
                f"{item.image} ({image_group}) != "
                f"{item.representative} ({representative_group})"
            )

    # Detect a cycle before the simpler chain rejection so a two-way duplicate
    # mapping is diagnosed precisely instead of depending on CSV row order.
    edges = {
        item.image: item.representative
        for item in exclusions.values()
        if item.representative is not None and item.representative in exclusions
    }
    visited = set()
    for start in sorted(edges, key=str.lower):
        if start in visited:
            continue
        path_positions: Dict[str, int] = {}
        current = start
        while current in edges:
            if current in path_positions:
                cycle = list(path_positions)[path_positions[current] :] + [current]
                raise ValueError(
                    "exclusions CSV contains a representative cycle: "
                    + " -> ".join(cycle)
                )
            if current in visited:
                break
            path_positions[current] = len(path_positions)
            current = edges[current]
        visited.update(path_positions)

    chained = sorted(
        (
            (item.image, item.representative)
            for item in exclusions.values()
            if item.representative is not None and item.representative in exclusions
        ),
        key=lambda pair: pair[0].lower(),
    )
    if chained:
        image, representative = chained[0]
        raise ValueError(
            "exclusion chains are forbidden because every representative must be "
            f"retained: {image} -> {representative}"
        )

    excluded = set(exclusions)
    return [sample for sample in samples if sample.relative_image not in excluded]


def _inferred_group(relative_image: str) -> str:
    path = Path(relative_image)
    # Only an explicit frame token is treated as scene evidence.  A suffix such
    # as ``_0001`` alone could just be an unrelated image identifier and must
    # not silently authorize a derived train/validation/test split.
    match = re.match(
        r"^(.*?)[_-](?:frame|img|image)[_-]?\d+$",
        path.stem,
        flags=re.IGNORECASE,
    )
    if match and match.group(1):
        return (path.parent / match.group(1)).as_posix()
    # Numeric-only Zenodo names do not carry scene identity. Treat them as
    # independent unless a groups CSV supplies the missing scene metadata.
    return path.as_posix()


def _unknown_sequence_samples(samples: Sequence[Sample]) -> List[str]:
    """Return samples whose scene identity cannot be proven from their names.

    Only names with an explicit non-empty scene prefix and frame token, such as
    ``greenhouse_a_frame0001.jpg``, provide usable built-in grouping evidence.
    Numeric names, generic ``image_0001`` names, and arbitrary one-off names are
    ambiguous.  They require an authoritative groups CSV before any split is
    derived; treating them as independent groups would risk adjacent-frame
    leakage.
    """

    return sorted(
        (
            sample.relative_image
            for sample in samples
            if _inferred_group(sample.relative_image) == sample.relative_image
        ),
        key=str.lower,
    )


def _require_scene_metadata_for_derived_split(
    samples: Sequence[Sample],
    groups: Mapping[str, str],
) -> None:
    if groups:
        return
    unknown = _unknown_sequence_samples(samples)
    if unknown:
        examples = ", ".join(unknown[:3])
        raise ValueError(
            "cannot prove scene identity for every sample; provide "
            f"--groups-csv with image,group_id assignments (examples: {examples})"
        )


def _require_no_unused_group_assignments(
    samples: Sequence[Sample],
    groups: Mapping[str, str],
) -> None:
    """Reject stale or misspelled rows in an authoritative group manifest."""

    if not groups:
        return
    discovered = {sample.relative_image for sample in samples}
    unused = sorted(set(groups) - discovered, key=str.lower)
    if unused:
        examples = ", ".join(unused[:3])
        raise ValueError(
            "groups CSV contains assignments for images outside the source pool "
            f"(examples: {examples})"
        )


def _label_candidate(image: Path) -> Optional[Path]:
    same_directory = image.with_suffix(".txt")
    if same_directory.is_file():
        return same_directory
    parts = list(image.parts)
    lowered = [part.lower() for part in parts]
    if "images" in lowered:
        index = len(lowered) - 1 - lowered[::-1].index("images")
        parts[index] = "labels"
        beside_labels = Path(*parts).with_suffix(".txt")
        if beside_labels.is_file():
            return beside_labels
    return None


def _read_class_counts(label: Optional[Path]) -> Counter:
    counts: Counter = Counter()
    if label is None:
        return counts
    with label.open("r", encoding="utf-8-sig") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != 5:
                raise ValueError(f"{label}:{line_number}: expected five YOLO fields")
            try:
                class_value = float(fields[0])
                coordinates = [float(field) for field in fields[1:]]
            except ValueError as error:
                raise ValueError(f"{label}:{line_number}: non-numeric YOLO field") from error
            if not class_value.is_integer() or class_value < 0:
                raise ValueError(f"{label}:{line_number}: invalid class id")
            if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in coordinates):
                raise ValueError(
                    f"{label}:{line_number}: coordinates must be finite and normalised"
                )
            class_id = int(class_value)
            if class_id in V1_CLASS_IDS:
                # A zero-area target can never reach positive IoU and creates
                # contradictory supervision.  Raw labels remain untouched;
                # the curated materialized view drops such rows, and class
                # counts must use the identical rule for split stratification.
                width, height = coordinates[2], coordinates[3]
                if width <= 0.0 or height <= 0.0:
                    continue
                counts[class_id] += 1
    return counts


def discover_samples(
    search_root: Path,
    dataset_root: Path,
    groups: Optional[Mapping[str, str]] = None,
) -> List[Sample]:
    groups = groups or {}
    samples: List[Sample] = []
    for image in sorted(
        (
            path
            for path in search_root.rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        ),
        key=lambda path: path.as_posix().lower(),
    ):
        relative_image = image.relative_to(dataset_root).as_posix()
        if groups:
            if relative_image not in groups:
                raise ValueError(f"groups CSV has no scene assignment for {relative_image}")
            group_id = groups[relative_image]
        else:
            group_id = _inferred_group(relative_image)
        label = _label_candidate(image)
        counts = _read_class_counts(label)
        samples.append(
            Sample(
                image=image,
                label=label,
                relative_image=relative_image,
                group_id=group_id,
                class_counts=tuple(sorted(counts.items())),
            )
        )
    return samples


def find_dataset_root(source: Path) -> Path:
    source = source.resolve()
    children = [path for path in source.iterdir()] if source.is_dir() else []
    directories = [path for path in children if path.is_dir()]
    files = [
        path
        for path in children
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    ]
    if len(directories) == 1 and not files:
        nested = directories[0]
        nested_names = {path.name.lower() for path in nested.iterdir() if path.is_dir()}
        layout_names = {"training", "train", "validation", "evaluation", "test", "images"}
        if nested_names.intersection(layout_names):
            return nested
    return source


def _official_directories(dataset_root: Path) -> Mapping[str, Path]:
    candidates = [dataset_root]
    images_root = dataset_root / "images"
    if images_root.is_dir():
        candidates.append(images_root)
    aliases = {
        "train": ("training", "train"),
        "val": ("validation", "valid", "val"),
        "test": ("evaluation", "test", "testing"),
    }
    for base in candidates:
        by_name = {path.name.lower(): path for path in base.iterdir() if path.is_dir()}
        found: Dict[str, Path] = {}
        for split_name, split_aliases in aliases.items():
            for alias in split_aliases:
                if alias in by_name:
                    found[split_name] = by_name[alias]
                    break
        if "train" in found:
            return found
    return {}


def _seed_key(seed: int, value: str) -> str:
    return hashlib.sha256(f"{seed}:{value}".encode("utf-8")).hexdigest()


def group_stratified_split(
    samples: Sequence[Sample],
    ratios: Mapping[str, float],
    seed: int,
) -> Mapping[str, List[Sample]]:
    if not samples:
        raise ValueError("cannot split an empty dataset")
    if set(ratios) != set(SPLIT_NAMES):
        raise ValueError("ratios must define train, val, and test")
    ratio_sum = sum(float(ratios[name]) for name in SPLIT_NAMES)
    has_invalid_ratio = any(float(ratios[name]) <= 0.0 for name in SPLIT_NAMES)
    if has_invalid_ratio or not math.isclose(ratio_sum, 1.0):
        raise ValueError("split ratios must be positive and sum to 1")

    grouped: MutableMapping[str, List[Sample]] = defaultdict(list)
    for sample in samples:
        grouped[sample.group_id].append(sample)
    if len(grouped) < 3:
        raise ValueError("at least three scene groups are required for train/val/test")

    total_classes: Counter = Counter()
    for sample in samples:
        total_classes.update(sample.counts)
    missing_classes = sorted(class_id for class_id in V1_CLASS_IDS if total_classes[class_id] <= 0)
    if missing_classes:
        raise ValueError(f"dataset has no instances for required v1 classes: {missing_classes}")
    groups_per_class = {
        class_id: sum(
            1
            for group_samples in grouped.values()
            if any(sample.counts[class_id] > 0 for sample in group_samples)
        )
        for class_id in V1_CLASS_IDS
    }
    insufficient_groups = {
        class_id: count
        for class_id, count in groups_per_class.items()
        if count < len(SPLIT_NAMES)
    }
    if insufficient_groups:
        raise ValueError(
            "each required class must occur in at least three scene groups; "
            f"got {insufficient_groups}"
        )
    target_sizes = {name: len(samples) * float(ratios[name]) for name in SPLIT_NAMES}
    target_classes = {
        name: {
            class_id: total * float(ratios[name])
            for class_id, total in total_classes.items()
        }
        for name in SPLIT_NAMES
    }
    assignments: Dict[str, List[Sample]] = {name: [] for name in SPLIT_NAMES}
    assigned_classes: Dict[str, Counter] = {name: Counter() for name in SPLIT_NAMES}

    ordered_groups = sorted(
        grouped.items(),
        key=lambda item: (
            -len(item[1]),
            -sum(sum(sample.counts.values()) for sample in item[1]),
            _seed_key(seed, item[0]),
        ),
    )
    for group_id, group_samples in ordered_groups:
        group_classes: Counter = Counter()
        for sample in group_samples:
            group_classes.update(sample.counts)

        def score(split_name: str) -> Tuple[float, str]:
            current_size = len(assignments[split_name])
            projected_size = len(assignments[split_name]) + len(group_samples)
            size_target = target_sizes[split_name]
            # Scale by global totals rather than the split target. Scaling by a
            # small validation target would otherwise make early assignments
            # appear disproportionately valuable and overfill that split.
            error = (
                (projected_size - size_target) ** 2
                - (current_size - size_target) ** 2
            ) / (len(samples) ** 2)
            for class_id, total in total_classes.items():
                class_target = target_classes[split_name][class_id]
                current = assigned_classes[split_name][class_id]
                projected = assigned_classes[split_name][class_id] + group_classes[class_id]
                error += (
                    (projected - class_target) ** 2 - (current - class_target) ** 2
                ) / (max(total, 1) ** 2)
            return error, _seed_key(seed, f"{group_id}:{split_name}")

        destination = min(SPLIT_NAMES, key=score)
        assignments[destination].extend(group_samples)
        assigned_classes[destination].update(group_classes)

    # The objective normally fills every split. For very uneven groups, move a
    # smallest group from the largest donor so all three outputs remain usable.
    for empty_name in (name for name in SPLIT_NAMES if not assignments[name]):
        donor = max(SPLIT_NAMES, key=lambda name: len(assignments[name]))
        donor_groups: MutableMapping[str, List[Sample]] = defaultdict(list)
        for sample in assignments[donor]:
            donor_groups[sample.group_id].append(sample)
        movable = [item for item in donor_groups.items() if len(donor_groups) > 1]
        if not movable:
            raise ValueError(
                "unable to produce three non-empty splits without breaking a scene group"
            )
        _, moved = min(movable, key=lambda item: (len(item[1]), _seed_key(seed, item[0])))
        moved_ids = {sample.relative_image for sample in moved}
        assignments[donor] = [
            sample for sample in assignments[donor] if sample.relative_image not in moved_ids
        ]
        assignments[empty_name].extend(moved)

    def split_counts(split_name: str) -> Counter:
        counts: Counter = Counter()
        for assigned_sample in assignments[split_name]:
            counts.update(assigned_sample.counts)
        return counts

    def move_objective(
        donor_name: str,
        destination_name: str,
        moved_samples: Sequence[Sample],
    ) -> float:
        moved_counts: Counter = Counter()
        for moved_sample in moved_samples:
            moved_counts.update(moved_sample.counts)
        current_counts = {name: split_counts(name) for name in SPLIT_NAMES}
        error = 0.0
        for split_name in SPLIT_NAMES:
            size = len(assignments[split_name])
            counts = current_counts[split_name].copy()
            if split_name == donor_name:
                size -= len(moved_samples)
                counts.subtract(moved_counts)
            elif split_name == destination_name:
                size += len(moved_samples)
                counts.update(moved_counts)
            error += ((size - target_sizes[split_name]) / len(samples)) ** 2
            for class_id, total in total_classes.items():
                error += (
                    (counts[class_id] - target_classes[split_name][class_id])
                    / max(total, 1)
                ) ** 2
        return error

    # The ratio objective alone can place a rare class in only one of the two
    # small splits. Repair coverage by moving whole groups while preserving all
    # required classes in the donor. This keeps the no-scene-leakage contract.
    for _ in range(len(SPLIT_NAMES) * len(V1_CLASS_IDS)):
        missing = [
            (split_name, class_id)
            for split_name in SPLIT_NAMES
            for class_id in sorted(V1_CLASS_IDS)
            if split_counts(split_name)[class_id] <= 0
        ]
        if not missing:
            break
        destination, missing_class = missing[0]
        candidates = []
        for donor in SPLIT_NAMES:
            if donor == destination:
                continue
            donor_groups: MutableMapping[str, List[Sample]] = defaultdict(list)
            for assigned_sample in assignments[donor]:
                donor_groups[assigned_sample.group_id].append(assigned_sample)
            for group_id, moved_samples in donor_groups.items():
                if not any(sample.counts[missing_class] > 0 for sample in moved_samples):
                    continue
                moved_ids = {sample.relative_image for sample in moved_samples}
                remaining = [
                    sample
                    for sample in assignments[donor]
                    if sample.relative_image not in moved_ids
                ]
                remaining_counts: Counter = Counter()
                for remaining_sample in remaining:
                    remaining_counts.update(remaining_sample.counts)
                if not remaining or any(
                    remaining_counts[class_id] <= 0 for class_id in V1_CLASS_IDS
                ):
                    continue
                candidates.append(
                    (
                        move_objective(donor, destination, moved_samples),
                        len(moved_samples),
                        _seed_key(seed, f"coverage:{donor}:{destination}:{group_id}"),
                        donor,
                        moved_samples,
                    )
                )
        if not candidates:
            raise ValueError(
                "unable to give every split both maturity classes without breaking "
                "a scene group"
            )
        _, _, _, donor, moved_samples = min(candidates, key=lambda item: item[:3])
        moved_ids = {sample.relative_image for sample in moved_samples}
        assignments[donor] = [
            sample
            for sample in assignments[donor]
            if sample.relative_image not in moved_ids
        ]
        assignments[destination].extend(moved_samples)

    if any(
        split_counts(split_name)[class_id] <= 0
        for split_name in SPLIT_NAMES
        for class_id in V1_CLASS_IDS
    ):
        raise ValueError("failed to establish required class coverage in every split")

    return {
        name: sorted(assignments[name], key=lambda sample: sample.relative_image.lower())
        for name in SPLIT_NAMES
    }


def validate_splits(
    splits: Mapping[str, Sequence[Sample]],
    required_class_ids: Iterable[int] = V1_CLASS_IDS,
) -> Mapping[str, object]:
    """Return a deterministic, JSON-serialisable split validation report."""

    required_classes = tuple(sorted({int(class_id) for class_id in required_class_ids}))
    violations: List[Mapping[str, object]] = []
    missing_split_names = sorted(set(SPLIT_NAMES) - set(splits))
    unexpected_split_names = sorted(set(splits) - set(SPLIT_NAMES))
    if missing_split_names:
        violations.append({"code": "MISSING_SPLIT", "splits": missing_split_names})
    if unexpected_split_names:
        violations.append(
            {"code": "UNEXPECTED_SPLIT", "splits": unexpected_split_names}
        )

    group_to_splits: MutableMapping[str, set] = defaultdict(set)
    source_to_splits: MutableMapping[str, set] = defaultdict(set)
    source_occurrences: Counter = Counter()
    split_reports: Dict[str, Mapping[str, object]] = {}
    for split_name in SPLIT_NAMES:
        split_samples = list(splits.get(split_name, ()))
        class_instances: Counter = Counter()
        images_with_class: Counter = Counter()
        groups = set()
        for sample in split_samples:
            class_instances.update(sample.counts)
            images_with_class.update(
                class_id for class_id in required_classes if sample.counts[class_id] > 0
            )
            groups.add(sample.group_id)
            group_to_splits[sample.group_id].add(split_name)
            source_to_splits[sample.relative_image].add(split_name)
            source_occurrences[sample.relative_image] += 1
        coverage = {
            str(class_id): class_instances[class_id] > 0
            for class_id in required_classes
        }
        split_reports[split_name] = {
            "sample_count": len(split_samples),
            "group_count": len(groups),
            "class_instance_counts": {
                str(class_id): class_instances[class_id]
                for class_id in required_classes
            },
            "images_with_class": {
                str(class_id): images_with_class[class_id]
                for class_id in required_classes
            },
            "class_coverage": coverage,
        }
        if not split_samples:
            violations.append({"code": "EMPTY_SPLIT", "split": split_name})
        missing_classes = [
            class_id for class_id in required_classes if not coverage[str(class_id)]
        ]
        if missing_classes:
            violations.append(
                {
                    "code": "MISSING_CLASS_COVERAGE",
                    "split": split_name,
                    "class_ids": missing_classes,
                }
            )

    group_leakage = [
        {"group_id": group_id, "splits": sorted(split_names)}
        for group_id, split_names in sorted(group_to_splits.items())
        if len(split_names) > 1
    ]
    if group_leakage:
        violations.append(
            {"code": "GROUP_LEAKAGE", "count": len(group_leakage)}
        )
    source_overlap = [
        {"source_image": source_image, "splits": sorted(split_names)}
        for source_image, split_names in sorted(source_to_splits.items())
        if len(split_names) > 1
    ]
    if source_overlap:
        violations.append(
            {"code": "SOURCE_IMAGE_OVERLAP", "count": len(source_overlap)}
        )
    duplicate_sources = sorted(
        source_image
        for source_image, count in source_occurrences.items()
        if count > 1
    )
    if duplicate_sources:
        violations.append(
            {"code": "DUPLICATE_SOURCE_IMAGE", "count": len(duplicate_sources)}
        )

    return {
        "schema_version": 1,
        "valid": not violations,
        "required_class_ids": list(required_classes),
        "splits": split_reports,
        "group_leakage": group_leakage,
        "source_overlap": source_overlap,
        "duplicate_source_images": duplicate_sources,
        "violations": violations,
    }


def _require_valid_splits(
    splits: Mapping[str, Sequence[Sample]],
) -> Mapping[str, object]:
    report = validate_splits(splits)
    if not report["valid"]:
        raise SplitValidationError(report)
    return report


def make_splits(
    source: Path,
    seed: int = 20260710,
    groups_csv: Optional[Path] = None,
    *,
    resplit_all: bool = False,
    exclusions_csv: Optional[Path] = None,
) -> Tuple[Path, Mapping[str, List[Sample]], str]:
    dataset_root = find_dataset_root(source)
    groups = read_groups_csv(groups_csv)

    if exclusions_csv is not None and not resplit_all:
        raise ValueError("--exclusions-csv requires --resplit-all")

    # Some published two-way layouts are not scene-disjoint.  The explicit
    # full-resplit policy combines every source directory before allocating
    # complete scene groups to 70/15/15.  Keep this opt-in so genuinely
    # independent official splits retain their existing default semantics.
    if resplit_all:
        if groups_csv is None:
            raise ValueError("--resplit-all requires an authoritative --groups-csv")
        all_samples = discover_samples(dataset_root, dataset_root, groups)
        _require_no_unused_group_assignments(all_samples, groups)
        exclusions = read_exclusions_csv(exclusions_csv)
        retained_samples = _apply_exclusions(all_samples, exclusions)
        splits = group_stratified_split(
            retained_samples,
            {"train": 0.70, "val": 0.15, "test": 0.15},
            seed,
        )
        _require_valid_splits(splits)
        return dataset_root, splits, "seeded_group_stratified_full_resplit"

    official = _official_directories(dataset_root)
    if {"train", "val", "test"}.issubset(official):
        splits = {
            name: discover_samples(official[name], dataset_root, groups)
            for name in SPLIT_NAMES
        }
        if any(not splits[name] for name in SPLIT_NAMES):
            raise ValueError("an official split directory contains no images")
        _require_valid_splits(splits)
        return dataset_root, splits, "official_three_way"

    if "train" in official and ("test" in official or "val" in official):
        held_out_name = "test" if "test" in official else "val"
        official_train = discover_samples(official["train"], dataset_root, groups)
        held_out = discover_samples(official[held_out_name], dataset_root, groups)
        if not official_train or not held_out:
            raise ValueError("the official training or held-out directory contains no images")
        _require_scene_metadata_for_derived_split(official_train, groups)
        derived = group_stratified_split(
            official_train,
            {"train": 0.85, "val": 0.15, "test": 1e-12},
            seed,
        )
        # group_stratified_split always produces three outputs; merge its tiny
        # internal test bucket back into train because the official held-out set
        # is the only final test set.
        train_samples = sorted(
            derived["train"] + derived["test"],
            key=lambda sample: sample.relative_image.lower(),
        )
        splits = {"train": train_samples, "val": derived["val"], "test": held_out}
        _require_valid_splits(splits)
        return dataset_root, splits, "official_heldout_plus_seeded_validation"

    all_samples = discover_samples(dataset_root, dataset_root, groups)
    _require_scene_metadata_for_derived_split(all_samples, groups)
    splits = group_stratified_split(
        all_samples,
        {"train": 0.70, "val": 0.15, "test": 0.15},
        seed,
    )
    _require_valid_splits(splits)
    return dataset_root, splits, "seeded_group_stratified_fallback"


def _filtered_label_lines(label: Optional[Path]) -> List[str]:
    if label is None:
        return []
    retained: List[str] = []
    with label.open("r", encoding="utf-8-sig") as stream:
        for raw_line in stream:
            fields = raw_line.strip().split()
            if not fields or int(float(fields[0])) not in V1_CLASS_IDS:
                continue
            if len(fields) != 5:
                raise ValueError(f"{label}: expected five YOLO fields")
            width, height = float(fields[3]), float(fields[4])
            if width <= 0.0 or height <= 0.0:
                continue
            retained.append(" ".join(fields))
    return retained


def _has_jpeg_eoi(path: Path) -> bool:
    if path.suffix.lower() not in JPEG_SUFFIXES or path.stat().st_size < len(JPEG_EOI):
        return False
    with path.open("rb") as stream:
        stream.seek(-len(JPEG_EOI), os.SEEK_END)
        return stream.read(len(JPEG_EOI)) == JPEG_EOI


def _place_image(source: Path, destination: Path, image_mode: str) -> Tuple[str, str]:
    """Place an image while isolating incomplete JPEGs from the raw source.

    Ultralytics rewrites JPEG files that do not end in the EOI marker.  A
    hard-linked processed image would therefore also rewrite the immutable raw
    extraction.  Missing markers are appended only to a processed copy; all
    other image bytes remain unchanged.
    """

    if image_mode not in {"hardlink", "copy"}:
        raise ValueError("image_mode must be hardlink or copy")
    if source.suffix.lower() in JPEG_SUFFIXES and not _has_jpeg_eoi(source):
        shutil.copy2(source, destination)
        with destination.open("ab") as stream:
            stream.write(JPEG_EOI)
        return "copy", "append_jpeg_eoi"
    if image_mode == "hardlink":
        try:
            os.link(source, destination)
            return "hardlink", "identity"
        except OSError:
            shutil.copy2(source, destination)
            return "copy_fallback", "identity"
    shutil.copy2(source, destination)
    return "copy", "identity"


def _canonical_json_sha256(value: object) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _canonical_relative_path(value: object) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("manifest paths must be non-empty strings")
    relative = PurePosixPath(value.replace("\\", "/"))
    if (
        relative.is_absolute()
        or ".." in relative.parts
        or not relative.parts
        or relative.parts[0].endswith(":")
    ):
        raise ValueError(f"unsafe relative path in split manifest: {value}")
    return relative.as_posix()


def _normalise_sha256(value: object, field: str) -> str:
    digest = str(value).lower()
    if re.fullmatch(r"[0-9a-f]{64}", digest) is None:
        raise ValueError(f"{field} must be a lowercase-compatible SHA-256 digest")
    return digest


def _canonical_sample_record(item: Mapping[str, object]) -> Mapping[str, object]:
    source_label_value = item.get("source_label")
    source_label = (
        None
        if source_label_value is None
        else _canonical_relative_path(source_label_value)
    )
    source_label_sha = item.get("source_label_sha256")
    if source_label is None:
        if source_label_sha is not None:
            raise ValueError("source_label_sha256 must be null when source_label is null")
        canonical_source_label_sha = None
    else:
        canonical_source_label_sha = _normalise_sha256(
            source_label_sha, "source_label_sha256"
        )
    counts = item.get("v1_class_counts")
    if not isinstance(counts, Mapping):
        raise ValueError("v1_class_counts must be an object")
    canonical_counts = {
        str(int(class_id)): int(count)
        for class_id, count in sorted(counts.items(), key=lambda pair: int(pair[0]))
    }
    if any(count < 0 for count in canonical_counts.values()):
        raise ValueError("v1_class_counts cannot contain negative values")
    record = {
        "source_image": _canonical_relative_path(item.get("source_image")),
        "source_label": source_label,
        "group_id": str(item.get("group_id", "")),
        "output_image": _canonical_relative_path(item.get("output_image")),
        "output_label": _canonical_relative_path(item.get("output_label")),
        "image_transform": str(item.get("image_transform", "")),
        "v1_class_counts": canonical_counts,
        "source_image_sha256": _normalise_sha256(
            item.get("source_image_sha256"), "source_image_sha256"
        ),
        "source_label_sha256": canonical_source_label_sha,
        "output_image_sha256": _normalise_sha256(
            item.get("output_image_sha256"), "output_image_sha256"
        ),
        "output_label_sha256": _normalise_sha256(
            item.get("output_label_sha256"), "output_label_sha256"
        ),
        "source_image_size_bytes": int(item.get("source_image_size_bytes", -1)),
        "source_label_size_bytes": (
            None
            if item.get("source_label_size_bytes") is None
            else int(item["source_label_size_bytes"])
        ),
        "output_image_size_bytes": int(item.get("output_image_size_bytes", -1)),
        "output_label_size_bytes": int(item.get("output_label_size_bytes", -1)),
    }
    if not record["group_id"]:
        raise ValueError("group_id must be non-empty")
    if record["image_transform"] not in {"identity", "append_jpeg_eoi"}:
        raise ValueError("image_transform must be identity or append_jpeg_eoi")
    size_fields = (
        "source_image_size_bytes",
        "output_image_size_bytes",
        "output_label_size_bytes",
    )
    if any(record[field] < 0 for field in size_fields):
        raise ValueError("materialized file sizes must be non-negative")
    if source_label is not None and record["source_label_size_bytes"] is None:
        raise ValueError("source_label_size_bytes is required for a source label")
    if record["source_label_size_bytes"] is not None and record["source_label_size_bytes"] < 0:
        raise ValueError("source label size must be non-negative")
    if record["image_transform"] == "identity":
        if record["source_image_sha256"] != record["output_image_sha256"]:
            raise ValueError("identity image transform requires equal source/output hashes")
        if record["source_image_size_bytes"] != record["output_image_size_bytes"]:
            raise ValueError("identity image transform requires equal source/output sizes")
    else:
        if PurePosixPath(record["source_image"]).suffix.lower() not in JPEG_SUFFIXES:
            raise ValueError("append_jpeg_eoi is valid only for JPEG source images")
        if (
            record["output_image_size_bytes"]
            != record["source_image_size_bytes"] + len(JPEG_EOI)
        ):
            raise ValueError("append_jpeg_eoi must add exactly two output bytes")
    return record


def _portable_relative_artifact_path(path: Path) -> str:
    """Record an artifact without binding a manifest to one machine root."""

    original = Path(path)
    if not original.is_absolute():
        relative = PurePosixPath(original.as_posix())
        if ".." not in relative.parts and str(relative) not in {"", "."}:
            return relative.as_posix()
    resolved = original.resolve()
    try:
        return resolved.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        # Unit-test and external callers may keep the CSV beside a temporary
        # source tree.  Its content hash is the identity; the basename remains
        # a portable, relative human-readable locator.
        return resolved.name


def _canonical_exclusion_metadata(report: Mapping[str, object]) -> Mapping[str, object]:
    path_value = report.get("exclusions_csv")
    hash_value = report.get("exclusions_csv_sha256")
    source_count = int(report.get("source_image_count", -1))
    excluded_count = int(report.get("excluded_count", -1))
    retained_count = int(report.get("retained_count", -1))
    if min(source_count, excluded_count, retained_count) < 0:
        raise ValueError("source/excluded/retained image counts must be non-negative")
    if source_count != excluded_count + retained_count:
        raise ValueError(
            "source_image_count must equal excluded_count plus retained_count"
        )

    if path_value is None:
        if hash_value is not None or excluded_count != 0:
            raise ValueError(
                "exclusion metadata requires both a relative CSV path and SHA-256"
            )
        canonical_path = None
        canonical_hash = None
    else:
        canonical_path = _canonical_relative_path(path_value)
        canonical_hash = _normalise_sha256(
            hash_value, "exclusions_csv_sha256"
        )
    return {
        "exclusions_csv": canonical_path,
        "exclusions_csv_sha256": canonical_hash,
        "source_image_count": source_count,
        "excluded_count": excluded_count,
        "retained_count": retained_count,
    }


def _build_content_binding(
    report: Mapping[str, object], dataset_yaml_sha256: str
) -> Mapping[str, object]:
    """Build the absolute-path-independent content identity for a split report."""

    splits = report.get("splits")
    if not isinstance(splits, Mapping) or set(splits) != set(SPLIT_NAMES):
        raise ValueError("split manifest must contain exactly train, val, and test")
    split_digests: Dict[str, str] = {}
    for split_name in SPLIT_NAMES:
        items = splits[split_name]
        if not isinstance(items, list):
            raise ValueError(f"split {split_name} must be a list")
        records = [_canonical_sample_record(item) for item in items]
        records.sort(key=lambda item: (item["source_image"], item["output_image"]))
        split_digests[split_name] = _canonical_json_sha256(
            {
                "canonical_schema": "strawberry_materialized_split_v2",
                "dataset_id": str(report.get("dataset_id", "")),
                "split": split_name,
                "samples": records,
            }
        )
    classes = report.get("v1_classes")
    if not isinstance(classes, Mapping):
        raise ValueError("v1_classes must be an object")
    canonical_classes = {
        str(int(class_id)): str(name)
        for class_id, name in sorted(classes.items(), key=lambda pair: int(pair[0]))
    }
    normalization = report.get("image_normalization")
    if normalization != IMAGE_NORMALIZATION:
        raise ValueError(
            "image_normalization must use append_jpeg_eoi_if_missing_v1"
        )
    exclusion_metadata = _canonical_exclusion_metadata(report)
    canonical_dataset_sha256 = _canonical_json_sha256(
        {
            "canonical_schema": "strawberry_materialized_dataset_v3",
            "dataset_id": str(report.get("dataset_id", "")),
            "strategy": str(report.get("strategy", "")),
            "seed": int(report.get("seed", -1)),
            "v1_classes": canonical_classes,
            "image_normalization": normalization,
            "exclusions": exclusion_metadata,
            "dataset_yaml_sha256": _normalise_sha256(
                dataset_yaml_sha256, "dataset_yaml_sha256"
            ),
            "canonical_split_sha256": split_digests,
        }
    )
    return {
        "schema_version": 1,
        "algorithm": "sha256",
        "canonicalization": "json-sort-keys-utf8-v1",
        "dataset_yaml_sha256": dataset_yaml_sha256,
        "canonical_split_sha256": split_digests,
        "canonical_dataset_sha256": canonical_dataset_sha256,
    }


def _resolve_materialized_path(root: Path, value: object) -> Path:
    relative = PurePosixPath(_canonical_relative_path(value))
    resolved = root.joinpath(*relative.parts).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"path escapes processed dataset root: {value}") from error
    return resolved


def _dataset_yaml_split_paths(path: Path) -> Mapping[str, str]:
    """Read the three top-level split paths from the generated minimal YAML."""

    values: Dict[str, str] = {}
    with path.open("r", encoding="utf-8-sig") as stream:
        for raw_line in stream:
            line = raw_line.split("#", 1)[0].rstrip()
            if not line or line[0].isspace() or ":" not in line:
                continue
            key, raw_value = line.split(":", 1)
            key = key.strip()
            if key == "path":
                raise ValueError(
                    "dataset.yaml must not override the manifest root with a path key"
                )
            if key not in SPLIT_NAMES:
                continue
            if key in values:
                raise ValueError(f"dataset.yaml repeats the {key} path")
            value = raw_value.strip()
            if (
                len(value) >= 2
                and value[0] == value[-1]
                and value[0] in {"'", '"'}
            ):
                value = value[1:-1]
            if not value:
                raise ValueError(f"dataset.yaml has an empty {key} path")
            values[key] = _canonical_relative_path(value)
    if set(values) != set(SPLIT_NAMES):
        missing = sorted(set(SPLIT_NAMES) - set(values))
        raise ValueError(f"dataset.yaml is missing split paths: {missing}")
    return values


def _enumerate_materialized_inputs(
    root: Path,
    expected: Mapping[str, Mapping[str, set]],
    violations: List[Mapping[str, object]],
) -> Mapping[str, Mapping[str, set]]:
    """Enumerate exactly the files that can act as YOLO image/label inputs."""

    actual: Dict[str, Dict[str, set]] = {
        "images": {split_name: set() for split_name in SPLIT_NAMES},
        "labels": {split_name: set() for split_name in SPLIT_NAMES},
    }
    cache_names = {f"{split_name}.cache" for split_name in SPLIT_NAMES}
    for kind in ("images", "labels"):
        base = root / kind
        if base.is_symlink() or not base.is_dir():
            violations.append(
                {
                    "code": "DATASET_LAYOUT_ERROR",
                    "path": kind,
                    "message": "must be a real directory, not a symlink or file",
                }
            )
            continue
        for child in base.iterdir():
            relative_child = child.relative_to(root).as_posix()
            if child.name in SPLIT_NAMES:
                if child.is_symlink() or not child.is_dir():
                    violations.append(
                        {
                            "code": "DATASET_LAYOUT_ERROR",
                            "path": relative_child,
                            "message": "split entry must be a real directory",
                        }
                    )
                continue
            # Ultralytics creates labels/{split}.cache during normal use.  It is
            # provenance/cache metadata, not an image or YOLO text label input.
            if child.is_file() and child.name in cache_names:
                continue
            violations.append(
                {
                    "code": (
                        "UNEXPECTED_DATASET_DIRECTORY"
                        if child.is_dir()
                        else "UNEXPECTED_DATASET_ENTRY"
                    ),
                    "path": relative_child,
                }
            )

        for split_name in SPLIT_NAMES:
            directory = base / split_name
            if directory.is_symlink() or not directory.is_dir():
                # The base-level check already reports wrong existing entities;
                # this also reports a completely missing split directory.
                if not any(
                    item.get("path") == directory.relative_to(root).as_posix()
                    for item in violations
                ):
                    violations.append(
                        {
                            "code": "DATASET_LAYOUT_ERROR",
                            "path": directory.relative_to(root).as_posix(),
                            "message": "missing real split directory",
                        }
                    )
                continue
            for entry in directory.rglob("*"):
                relative_entry = entry.relative_to(root).as_posix()
                if entry.is_symlink():
                    violations.append(
                        {"code": "UNSAFE_DATASET_ENTRY", "path": relative_entry}
                    )
                    continue
                if entry.is_dir():
                    violations.append(
                        {
                            "code": "UNEXPECTED_DATASET_DIRECTORY",
                            "path": relative_entry,
                        }
                    )
                    continue
                if not entry.is_file():
                    violations.append(
                        {"code": "UNEXPECTED_DATASET_ENTRY", "path": relative_entry}
                    )
                    continue
                suffix = entry.suffix.lower()
                is_input = (
                    suffix in IMAGE_SUFFIXES if kind == "images" else suffix == ".txt"
                )
                if is_input:
                    actual[kind][split_name].add(relative_entry)
                elif suffix != ".cache":
                    violations.append(
                        {
                            "code": "WRONG_INPUT_SUFFIX",
                            "path": relative_entry,
                            "kind": kind,
                        }
                    )

            expected_paths = expected[kind][split_name]
            unregistered = sorted(actual[kind][split_name] - expected_paths)
            if unregistered:
                violations.append(
                    {
                        "code": "UNREGISTERED_INPUT_FILE",
                        "kind": kind,
                        "split": split_name,
                        "count": len(unregistered),
                        "paths": unregistered,
                    }
                )
            unscanned = sorted(expected_paths - actual[kind][split_name])
            if unscanned:
                violations.append(
                    {
                        "code": "MANIFEST_INPUT_NOT_IN_SCAN_SET",
                        "kind": kind,
                        "split": split_name,
                        "count": len(unscanned),
                        "paths": unscanned,
                    }
                )
    return actual


def verify_materialized_split_manifest(manifest_path: Path) -> Mapping[str, object]:
    """Verify every training input and return its canonical content identity.

    The dataset root is always the manifest's current parent directory.  Stored
    ``source_root`` and ``output_root`` values are informational only, so moving
    an intact processed dataset to another machine does not change its digest.
    Image bytes are re-hashed even when materialization used hard links.
    """

    manifest_path = Path(manifest_path).resolve()
    root = manifest_path.parent
    violations: List[Mapping[str, object]] = []
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        report = {
            "schema_version": 1,
            "valid": False,
            "manifest_path": str(manifest_path),
            "dataset_root": str(root),
            "violations": [
                {"code": "MANIFEST_UNREADABLE", "message": str(error)}
            ],
        }
        raise DatasetVerificationError(report) from error
    if not isinstance(manifest, Mapping):
        violations.append({"code": "MANIFEST_SCHEMA_ERROR", "message": "root must be an object"})
    elif manifest.get("schema_version") != 1:
        violations.append(
            {"code": "MANIFEST_SCHEMA_ERROR", "message": "unsupported schema_version"}
        )
    binding = manifest.get("content_binding") if isinstance(manifest, Mapping) else None
    if not isinstance(binding, Mapping):
        violations.append(
            {"code": "CONTENT_BINDING_MISSING", "message": "content_binding is required"}
        )

    dataset_yaml = root / "dataset.yaml"
    actual_yaml_sha = None
    if not dataset_yaml.is_file():
        violations.append(
            {"code": "MISSING_MATERIALIZED_FILE", "path": "dataset.yaml"}
        )
    else:
        actual_yaml_sha = file_digest(dataset_yaml, "sha256")
        try:
            yaml_split_paths = _dataset_yaml_split_paths(dataset_yaml)
        except (OSError, ValueError) as error:
            violations.append(
                {"code": "DATASET_YAML_SCHEMA_ERROR", "message": str(error)}
            )
        else:
            required_yaml_paths = {
                split_name: f"images/{split_name}" for split_name in SPLIT_NAMES
            }
            if yaml_split_paths != required_yaml_paths:
                violations.append(
                    {
                        "code": "DATASET_YAML_SPLIT_PATH_MISMATCH",
                        "expected": required_yaml_paths,
                        "actual": dict(yaml_split_paths),
                    }
                )
        if isinstance(binding, Mapping):
            expected_yaml_sha = str(binding.get("dataset_yaml_sha256", "")).lower()
            if actual_yaml_sha != expected_yaml_sha:
                violations.append(
                    {
                        "code": "CONTENT_HASH_MISMATCH",
                        "path": "dataset.yaml",
                        "expected": expected_yaml_sha,
                        "actual": actual_yaml_sha,
                    }
                )

    sample_count = 0
    verified_file_count = 1 if actual_yaml_sha is not None else 0
    split_sample_count: Dict[str, int] = {}
    observed_transform_counts: Counter = Counter()
    seen_output_paths = set()
    expected_inputs: Dict[str, Dict[str, set]] = {
        "images": {split_name: set() for split_name in SPLIT_NAMES},
        "labels": {split_name: set() for split_name in SPLIT_NAMES},
    }
    splits = manifest.get("splits") if isinstance(manifest, Mapping) else None
    if not isinstance(splits, Mapping) or set(splits) != set(SPLIT_NAMES):
        violations.append(
            {
                "code": "MANIFEST_SCHEMA_ERROR",
                "message": "splits must contain exactly train, val, and test",
            }
        )
    else:
        for split_name in SPLIT_NAMES:
            items = splits[split_name]
            if not isinstance(items, list):
                violations.append(
                    {
                        "code": "MANIFEST_SCHEMA_ERROR",
                        "message": f"split {split_name} must be a list",
                    }
                )
                continue
            split_sample_count[split_name] = len(items)
            sample_count += len(items)
            for index, item in enumerate(items):
                if not isinstance(item, Mapping):
                    violations.append(
                        {
                            "code": "MANIFEST_SCHEMA_ERROR",
                            "split": split_name,
                            "index": index,
                            "message": "sample must be an object",
                        }
                    )
                    continue
                try:
                    record = _canonical_sample_record(item)
                except (KeyError, TypeError, ValueError) as error:
                    violations.append(
                        {
                            "code": "MANIFEST_SCHEMA_ERROR",
                            "split": split_name,
                            "index": index,
                            "message": str(error),
                        }
                    )
                    continue
                observed_transform_counts[record["image_transform"]] += 1
                image_relative = record["output_image"]
                label_relative = record["output_label"]
                expected_inputs["images"][split_name].add(image_relative)
                expected_inputs["labels"][split_name].add(label_relative)
                image_path = PurePosixPath(image_relative)
                label_path = PurePosixPath(label_relative)
                image_layout_valid = (
                    image_path.parent == PurePosixPath("images", split_name)
                    and image_path.suffix.lower() in IMAGE_SUFFIXES
                )
                label_layout_valid = (
                    label_path.parent == PurePosixPath("labels", split_name)
                    and label_path.suffix.lower() == ".txt"
                )
                if not image_layout_valid or not label_layout_valid:
                    violations.append(
                        {
                            "code": "MANIFEST_OUTPUT_LAYOUT_ERROR",
                            "split": split_name,
                            "index": index,
                            "output_image": image_relative,
                            "output_label": label_relative,
                        }
                    )
                elif image_path.stem != label_path.stem:
                    violations.append(
                        {
                            "code": "MANIFEST_IMAGE_LABEL_PAIR_MISMATCH",
                            "split": split_name,
                            "index": index,
                            "output_image": image_relative,
                            "output_label": label_relative,
                        }
                    )
                for path_field, hash_field, size_field in (
                    ("output_image", "output_image_sha256", "output_image_size_bytes"),
                    ("output_label", "output_label_sha256", "output_label_size_bytes"),
                ):
                    relative_path = record[path_field]
                    if relative_path in seen_output_paths:
                        violations.append(
                            {"code": "DUPLICATE_OUTPUT_PATH", "path": relative_path}
                        )
                        continue
                    seen_output_paths.add(relative_path)
                    try:
                        materialized_path = _resolve_materialized_path(root, relative_path)
                    except ValueError as error:
                        violations.append(
                            {
                                "code": "UNSAFE_OUTPUT_PATH",
                                "path": relative_path,
                                "message": str(error),
                            }
                        )
                        continue
                    if not materialized_path.is_file():
                        violations.append(
                            {"code": "MISSING_MATERIALIZED_FILE", "path": relative_path}
                        )
                        continue
                    actual_sha = file_digest(materialized_path, "sha256")
                    actual_size = materialized_path.stat().st_size
                    verified_file_count += 1
                    if (
                        path_field == "output_image"
                        and materialized_path.suffix.lower() in JPEG_SUFFIXES
                        and not _has_jpeg_eoi(materialized_path)
                    ):
                        violations.append(
                            {
                                "code": "JPEG_EOI_MISSING",
                                "path": relative_path,
                            }
                        )
                    if actual_sha != record[hash_field]:
                        violations.append(
                            {
                                "code": "CONTENT_HASH_MISMATCH",
                                "path": relative_path,
                                "expected": record[hash_field],
                                "actual": actual_sha,
                            }
                        )
                    if actual_size != record[size_field]:
                        violations.append(
                            {
                                "code": "CONTENT_SIZE_MISMATCH",
                                "path": relative_path,
                                "expected": record[size_field],
                                "actual": actual_size,
                            }
                        )

    reported_transform_counts = (
        manifest.get("image_transforms") if isinstance(manifest, Mapping) else None
    )
    if not isinstance(reported_transform_counts, Mapping):
        violations.append(
            {
                "code": "IMAGE_TRANSFORM_COUNT_MISMATCH",
                "message": "image_transforms must be an object",
            }
        )
    else:
        try:
            normalized_transform_counts = {
                str(name): int(count)
                for name, count in sorted(reported_transform_counts.items())
            }
        except (TypeError, ValueError):
            normalized_transform_counts = {}
        expected_transform_counts = dict(sorted(observed_transform_counts.items()))
        if normalized_transform_counts != expected_transform_counts:
            violations.append(
                {
                    "code": "IMAGE_TRANSFORM_COUNT_MISMATCH",
                    "expected": expected_transform_counts,
                    "actual": normalized_transform_counts,
                }
            )

    _enumerate_materialized_inputs(root, expected_inputs, violations)

    exclusion_metadata = None
    if isinstance(manifest, Mapping):
        try:
            exclusion_metadata = _canonical_exclusion_metadata(manifest)
        except (TypeError, ValueError) as error:
            violations.append(
                {"code": "EXCLUSION_METADATA_ERROR", "message": str(error)}
            )
        else:
            if exclusion_metadata["retained_count"] != sample_count:
                violations.append(
                    {
                        "code": "EXCLUSION_COUNT_MISMATCH",
                        "field": "retained_count",
                        "expected": sample_count,
                        "actual": exclusion_metadata["retained_count"],
                    }
                )

    calculated_binding = None
    if actual_yaml_sha is not None and isinstance(splits, Mapping):
        try:
            calculated_binding = _build_content_binding(manifest, actual_yaml_sha)
        except (KeyError, TypeError, ValueError) as error:
            violations.append(
                {"code": "MANIFEST_SCHEMA_ERROR", "message": str(error)}
            )
    if calculated_binding is not None and isinstance(binding, Mapping):
        for field in ("canonical_split_sha256", "canonical_dataset_sha256"):
            if binding.get(field) != calculated_binding[field]:
                violations.append(
                    {
                        "code": "CANONICAL_DIGEST_MISMATCH",
                        "field": field,
                        "expected": binding.get(field),
                        "actual": calculated_binding[field],
                    }
                )
        if binding.get("schema_version") != 1 or binding.get("algorithm") != "sha256":
            violations.append(
                {
                    "code": "CONTENT_BINDING_SCHEMA_ERROR",
                    "message": "content binding must use schema 1 and SHA-256",
                }
            )

    result: Dict[str, object] = {
        "schema_version": 1,
        "valid": not violations,
        "dataset_id": manifest.get("dataset_id") if isinstance(manifest, Mapping) else None,
        "manifest_path": str(manifest_path),
        "dataset_root": str(root),
        "sample_count": sample_count,
        "split_sample_count": split_sample_count,
        "strategy": (
            manifest.get("strategy") if isinstance(manifest, Mapping) else None
        ),
        "seed": manifest.get("seed") if isinstance(manifest, Mapping) else None,
        "exclusions_csv": (
            exclusion_metadata["exclusions_csv"]
            if exclusion_metadata is not None
            else None
        ),
        "exclusions_csv_sha256": (
            exclusion_metadata["exclusions_csv_sha256"]
            if exclusion_metadata is not None
            else None
        ),
        "source_image_count": (
            exclusion_metadata["source_image_count"]
            if exclusion_metadata is not None
            else None
        ),
        "excluded_count": (
            exclusion_metadata["excluded_count"]
            if exclusion_metadata is not None
            else None
        ),
        "retained_count": (
            exclusion_metadata["retained_count"]
            if exclusion_metadata is not None
            else None
        ),
        "verified_file_count": verified_file_count,
        "dataset_yaml_sha256": actual_yaml_sha,
        "canonical_dataset_sha256": (
            calculated_binding["canonical_dataset_sha256"]
            if calculated_binding is not None
            else None
        ),
        "canonical_split_sha256": (
            calculated_binding["canonical_split_sha256"]
            if calculated_binding is not None
            else None
        ),
        "violations": violations,
    }
    if violations:
        raise DatasetVerificationError(result)
    return result


def _clear_generated_output(output: Path) -> None:
    """Remove only an output previously created by this module."""

    marker = output / "split_manifest.json"
    if not marker.is_file():
        raise ValueError(
            "--force refuses to clear a non-empty directory without split_manifest.json"
        )
    try:
        previous = json.loads(marker.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError("--force found an unreadable split_manifest.json") from error
    if previous.get("dataset_id") != "zenodo_6126677":
        raise ValueError("--force found a split manifest for a different dataset")
    recorded_output = Path(str(previous.get("output_root", ""))).resolve()
    if recorded_output != output:
        raise ValueError("--force output path does not match the existing split manifest")

    allowed = {"images", "labels", "dataset.yaml", "split_manifest.json"}
    unexpected = {child.name for child in output.iterdir()} - allowed
    if unexpected:
        raise ValueError(f"--force refuses to remove unexpected entries: {sorted(unexpected)}")
    for name in allowed:
        target = (output / name).resolve()
        try:
            target.relative_to(output)
        except ValueError as error:
            raise ValueError(f"unsafe generated output path: {target}") from error
        if target.is_dir():
            shutil.rmtree(target)
        elif target.exists():
            target.unlink()


def materialize_splits(
    dataset_root: Path,
    splits: Mapping[str, Sequence[Sample]],
    output: Path,
    strategy: str,
    seed: int = 20260710,
    image_mode: str = "copy",
    force: bool = False,
    exclusions_csv: Optional[Path] = None,
) -> Mapping[str, object]:
    output = output.resolve()
    dataset_root = dataset_root.resolve()
    validation = _require_valid_splits(splits)
    if output == dataset_root or output in dataset_root.parents or dataset_root in output.parents:
        raise ValueError("processed output and raw dataset roots must not contain one another")

    retained_paths = {
        sample.relative_image
        for split_samples in splits.values()
        for sample in split_samples
    }
    exclusions = read_exclusions_csv(exclusions_csv)
    excluded_paths = set(exclusions)
    if exclusions_csv is not None and strategy != "seeded_group_stratified_full_resplit":
        raise ValueError(
            "an exclusions CSV may only be materialized with the full-resplit strategy"
        )
    if strategy == "seeded_group_stratified_full_resplit":
        source_paths = {
            path.relative_to(dataset_root).as_posix()
            for path in dataset_root.rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        }
        missing_excluded = sorted(excluded_paths - source_paths, key=str.lower)
        if missing_excluded:
            raise ValueError(
                "exclusions CSV contains images outside the source pool "
                f"(examples: {', '.join(missing_excluded[:3])})"
            )
        missing_representatives = sorted(
            {
                item.representative
                for item in exclusions.values()
                if item.representative is not None
                and item.representative not in source_paths
            },
            key=str.lower,
        )
        if missing_representatives:
            raise ValueError(
                "exclusions CSV contains representatives outside the source pool "
                f"(examples: {', '.join(missing_representatives[:3])})"
            )
        excluded_representatives = sorted(
            {
                item.representative
                for item in exclusions.values()
                if item.representative is not None
                and item.representative in excluded_paths
            },
            key=str.lower,
        )
        if excluded_representatives:
            raise ValueError(
                "every exclusion representative must be retained; excluded "
                f"representatives: {excluded_representatives[:3]}"
            )
        expected_retained = source_paths - excluded_paths
        if retained_paths != expected_retained:
            missing = sorted(expected_retained - retained_paths, key=str.lower)
            unexpected = sorted(retained_paths - expected_retained, key=str.lower)
            raise ValueError(
                "materialized retained samples do not exactly cover the audited "
                f"source pool (missing: {missing[:3]}, unexpected: {unexpected[:3]})"
            )
    else:
        source_paths = set(retained_paths)

    exclusions_recorded_path = (
        _portable_relative_artifact_path(exclusions_csv)
        if exclusions_csv is not None
        else None
    )
    exclusions_sha256 = (
        file_digest(exclusions_csv, "sha256")
        if exclusions_csv is not None
        else None
    )
    if output.exists() and any(output.iterdir()):
        if not force:
            raise FileExistsError(f"output directory is not empty: {output}")
        _clear_generated_output(output)
    output.mkdir(parents=True, exist_ok=True)

    report_splits: Dict[str, List[Mapping[str, object]]] = {}
    placement_counts: Counter = Counter()
    transform_counts: Counter = Counter()
    seen_sources = set()
    seen_groups: Dict[str, str] = {}
    for split_name in SPLIT_NAMES:
        image_directory = output / "images" / split_name
        label_directory = output / "labels" / split_name
        image_directory.mkdir(parents=True, exist_ok=True)
        label_directory.mkdir(parents=True, exist_ok=True)
        report_samples: List[Mapping[str, object]] = []
        for sample in splits[split_name]:
            if sample.relative_image in seen_sources:
                raise ValueError(
                    f"source image appears in more than one split: {sample.relative_image}"
                )
            seen_sources.add(sample.relative_image)
            previous_split = seen_groups.setdefault(sample.group_id, split_name)
            if previous_split != split_name:
                raise ValueError(
                    f"scene group {sample.group_id!r} crosses {previous_split} "
                    f"and {split_name}"
                )
            prefix = hashlib.sha256(sample.relative_image.encode("utf-8")).hexdigest()[:12]
            output_name = f"{prefix}_{sample.image.name}"
            output_image = image_directory / output_name
            output_label = label_directory / Path(output_name).with_suffix(".txt").name
            source_image_sha256 = file_digest(sample.image, "sha256")
            source_image_size_bytes = sample.image.stat().st_size
            source_label_sha256 = (
                file_digest(sample.label, "sha256") if sample.label is not None else None
            )
            source_label_size_bytes = (
                sample.label.stat().st_size if sample.label is not None else None
            )
            placement, image_transform = _place_image(
                sample.image, output_image, image_mode
            )
            placement_counts[placement] += 1
            transform_counts[image_transform] += 1
            lines = _filtered_label_lines(sample.label)
            label_text = ("\n".join(lines) + "\n") if lines else ""
            output_label.write_text(label_text, encoding="utf-8")
            output_image_sha256 = file_digest(output_image, "sha256")
            if (
                image_transform == "identity"
                and output_image_sha256 != source_image_sha256
            ):
                raise ValueError(
                    f"materialized image changed while being placed: {sample.relative_image}"
                )
            if image_transform == "append_jpeg_eoi":
                expected_digest = hashlib.sha256()
                with sample.image.open("rb") as stream:
                    for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                        expected_digest.update(chunk)
                expected_digest.update(JPEG_EOI)
                if output_image_sha256 != expected_digest.hexdigest():
                    raise ValueError(
                        "JPEG EOI normalization changed bytes other than the appended "
                        f"marker: {sample.relative_image}"
                    )
            report_samples.append(
                {
                    "source_image": sample.relative_image,
                    "source_label": (
                        sample.label.relative_to(dataset_root).as_posix()
                        if sample.label
                        else None
                    ),
                    "group_id": sample.group_id,
                    "output_image": output_image.relative_to(output).as_posix(),
                    "output_label": output_label.relative_to(output).as_posix(),
                    "image_transform": image_transform,
                    "v1_class_counts": {
                        str(class_id): count for class_id, count in sample.class_counts
                    },
                    "source_image_sha256": source_image_sha256,
                    "source_label_sha256": source_label_sha256,
                    "output_image_sha256": output_image_sha256,
                    "output_label_sha256": file_digest(output_label, "sha256"),
                    "source_image_size_bytes": source_image_size_bytes,
                    "source_label_size_bytes": source_label_size_bytes,
                    "output_image_size_bytes": output_image.stat().st_size,
                    "output_label_size_bytes": output_label.stat().st_size,
                }
            )
        report_splits[split_name] = report_samples

    dataset_yaml = f"""# Generated by tools/data/split_dataset.py; source labels remain untouched.
train: images/train
val: images/val
test: images/test
names:
  0: ripe
  1: unripe
"""
    dataset_yaml_path = output / "dataset.yaml"
    dataset_yaml_path.write_text(dataset_yaml, encoding="utf-8")
    report: Dict[str, object] = {
        "schema_version": 1,
        "dataset_id": "zenodo_6126677",
        "strategy": strategy,
        "seed": seed,
        "source_root": str(dataset_root),
        "output_root": str(output),
        "exclusions_csv": exclusions_recorded_path,
        "exclusions_csv_sha256": exclusions_sha256,
        "source_image_count": len(source_paths),
        "excluded_count": len(excluded_paths),
        "retained_count": len(retained_paths),
        "image_placement": dict(sorted(placement_counts.items())),
        "image_transforms": dict(sorted(transform_counts.items())),
        "image_normalization": IMAGE_NORMALIZATION,
        "v1_classes": {"0": "ripe", "1": "unripe"},
        "validation": validation,
        "splits": report_splits,
        "counts": {name: len(report_splits[name]) for name in SPLIT_NAMES},
    }
    report["content_binding"] = _build_content_binding(
        report, file_digest(dataset_yaml_path, "sha256")
    )
    (output / "split_manifest.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    verify_materialized_split_manifest(output / "split_manifest.json")
    return report
