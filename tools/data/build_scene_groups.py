#!/usr/bin/env python3
"""Build an auditable scene-group manifest for the Zenodo strawberry data.

The published archive contains shuffled numeric filenames and near-duplicate
views on both sides of its original training/validation boundary.  This tool
therefore treats every source image as an unsplit curation pool.  It proposes
candidate pairs using perceptual hashes and optional YOLO embeddings, verifies
the candidates with local-feature geometry, and emits deterministic connected
components for use by ``split_dataset.py --resplit-all``.

The geometric "conservative" band is merged by default.  A false positive only
makes a scene group larger, whereas a false negative can leak a scene across
train/validation/test.  Every decision and threshold is recorded in JSON.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Iterable, Mapping, Sequence

import cv2
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SOURCE = (
    REPOSITORY_ROOT
    / "data"
    / "raw"
    / "zenodo_6126677"
    / "extracted"
    / "strawberries"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "data" / "manifests" / "zenodo_6126677_groups.csv"
)
DEFAULT_AUDIT = (
    REPOSITORY_ROOT / "artifacts" / "data" / "zenodo_6126677_group_audit.json"
)
DEFAULT_WEIGHTS = REPOSITORY_ROOT / "weights" / "yolo11n.pt"
IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"})


@dataclass(frozen=True)
class ImageFeatures:
    relative_path: str
    path: Path
    file_sha256: str
    decoded_sha256: str
    width: int
    height: int
    feature_width: int
    feature_height: int
    phash: np.ndarray
    dhash: np.ndarray
    keypoints: np.ndarray
    descriptors: np.ndarray | None


@dataclass(frozen=True)
class PairMetrics:
    image_a: str
    image_b: str
    reasons: tuple[str, ...]
    phash_distance: int
    dhash_distance: int
    embedding_similarity: float | None
    good_matches: int
    inliers: int
    inlier_ratio: float
    homography_valid: bool
    projected_area_ratio: float | None
    condition_number: float | None
    source_inlier_coverage: float
    target_inlier_coverage: float
    decision: str


class UnionFind:
    def __init__(self, values: Iterable[str]):
        self.parent = {value: value for value in values}
        self.rank = {value: 0 for value in values}

    def find(self, value: str) -> str:
        parent = self.parent[value]
        if parent != value:
            self.parent[value] = self.find(parent)
        return self.parent[value]

    def union(self, left: str, right: str) -> None:
        root_left = self.find(left)
        root_right = self.find(right)
        if root_left == root_right:
            return
        if self.rank[root_left] < self.rank[root_right]:
            root_left, root_right = root_right, root_left
        self.parent[root_right] = root_left
        if self.rank[root_left] == self.rank[root_right]:
            self.rank[root_left] += 1


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_image(path: Path) -> tuple[np.ndarray, bytes]:
    encoded = path.read_bytes()
    image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"OpenCV could not decode {path}")
    return image, encoded


def _phash(gray: np.ndarray) -> np.ndarray:
    resized = cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA)
    transformed = cv2.dct(np.float32(resized))[:8, :8].reshape(-1)
    # The DC coefficient describes global brightness rather than scene content.
    values = transformed[1:]
    return values > np.median(values)


def _dhash(gray: np.ndarray) -> np.ndarray:
    resized = cv2.resize(gray, (17, 16), interpolation=cv2.INTER_AREA)
    return (resized[:, 1:] > resized[:, :-1]).reshape(-1)


def _scaled_gray(image: np.ndarray, max_dimension: int) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(1.0, max_dimension / float(max(height, width)))
    if scale < 1.0:
        image = cv2.resize(
            image,
            (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def _extract_features(
    source: Path,
    relative_paths: Sequence[str],
    *,
    max_dimension: int,
    orb_features: int,
    orb_fast_threshold: int,
) -> list[ImageFeatures]:
    orb = cv2.ORB_create(
        nfeatures=orb_features,
        fastThreshold=orb_fast_threshold,
    )
    records: list[ImageFeatures] = []
    for index, relative_path in enumerate(relative_paths, start=1):
        path = source / relative_path
        image, encoded = _read_image(path)
        height, width = image.shape[:2]
        gray = _scaled_gray(image, max_dimension)
        feature_height, feature_width = gray.shape[:2]
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        points = np.asarray(
            [keypoint.pt for keypoint in keypoints],
            dtype=np.float32,
        ).reshape(-1, 2)
        decoded_digest = hashlib.sha256()
        decoded_digest.update(np.asarray(image.shape, dtype=np.int64).tobytes())
        decoded_digest.update(image.tobytes())
        records.append(
            ImageFeatures(
                relative_path=relative_path,
                path=path,
                file_sha256=_sha256_bytes(encoded),
                decoded_sha256=decoded_digest.hexdigest(),
                width=width,
                height=height,
                feature_width=feature_width,
                feature_height=feature_height,
                phash=_phash(gray),
                dhash=_dhash(gray),
                keypoints=points,
                descriptors=descriptors,
            )
        )
        if index % 50 == 0 or index == len(relative_paths):
            print(f"features: {index}/{len(relative_paths)}", flush=True)
    return records


def _embedding_candidates(
    paths: Sequence[Path],
    weights: Path | None,
    *,
    top_k: int,
    imgsz: int,
    batch: int,
    device: str,
) -> tuple[set[tuple[int, int]], np.ndarray | None, Mapping[str, object]]:
    if weights is None or top_k == 0:
        return set(), None, {"enabled": False}
    if not weights.is_file():
        raise FileNotFoundError(f"embedding weights not found: {weights}")
    try:
        from ultralytics import YOLO
        import ultralytics
    except ImportError as error:
        raise RuntimeError(
            "Ultralytics is required when --embedding-top-k is nonzero"
        ) from error

    model = YOLO(str(weights))
    vectors = []
    embedding_dimension: int | None = None
    # Ultralytics treats a list source as an in-memory batch and eagerly opens
    # every image in that list.  Chunk at the caller boundary so ``batch`` is a
    # real RAM/VRAM limit even for multi-megapixel source photographs.
    for start in range(0, len(paths), batch):
        chunk = paths[start : start + batch]
        outputs = model.embed(
            source=[str(path) for path in chunk],
            imgsz=imgsz,
            batch=len(chunk),
            device=device,
            verbose=False,
        )
        if len(outputs) != len(chunk):
            raise RuntimeError(
                "embedding count mismatch in chunk "
                f"{start // batch}: expected {len(chunk)}, got {len(outputs)}"
            )
        for output in outputs:
            if hasattr(output, "detach"):
                output = output.detach().cpu().numpy()
            vector = np.asarray(output, dtype=np.float32).reshape(-1)
            if embedding_dimension is None:
                embedding_dimension = len(vector)
            elif len(vector) != embedding_dimension:
                raise RuntimeError("model returned inconsistent embedding dimensions")
            norm = float(np.linalg.norm(vector))
            if not math.isfinite(norm) or norm <= 0.0:
                raise ValueError("model returned a non-finite or zero embedding")
            vectors.append(vector / norm)
        print(
            f"embeddings: {min(start + batch, len(paths))}/{len(paths)}",
            flush=True,
        )
    embeddings = np.stack(vectors)
    similarities = embeddings @ embeddings.T
    np.fill_diagonal(similarities, -np.inf)
    count = min(top_k, max(0, len(paths) - 1))
    pairs: set[tuple[int, int]] = set()
    if count:
        # mergesort gives deterministic ordering when similarities tie.
        for index in range(len(paths)):
            neighbours = np.argsort(-similarities[index], kind="mergesort")[:count]
            for neighbour in neighbours:
                pairs.add(tuple(sorted((index, int(neighbour)))))
    metadata = {
        "enabled": True,
        "weights": str(weights.resolve()),
        "weights_sha256": _file_sha256(weights),
        "ultralytics_version": ultralytics.__version__,
        "top_k": top_k,
        "imgsz": imgsz,
        "batch": batch,
        "device": device,
        "embedding_dimension": int(embeddings.shape[1]),
    }
    return pairs, similarities, metadata


def _hash_candidates(
    features: Sequence[ImageFeatures],
    *,
    phash_max: int,
    dhash_max: int,
) -> tuple[dict[tuple[int, int], set[str]], Mapping[str, int]]:
    candidates: dict[tuple[int, int], set[str]] = defaultdict(set)
    phash_histogram: Counter[int] = Counter()
    decoded: dict[str, list[int]] = defaultdict(list)
    for index, record in enumerate(features):
        decoded[record.decoded_sha256].append(index)

    for indices in decoded.values():
        if len(indices) > 1:
            for offset, left in enumerate(indices):
                for right in indices[offset + 1 :]:
                    candidates[(left, right)].add("decoded_pixel_duplicate")

    for left in range(len(features)):
        for right in range(left + 1, len(features)):
            phash_distance = int(
                np.count_nonzero(features[left].phash != features[right].phash)
            )
            phash_histogram[phash_distance] += 1
            dhash_distance = int(
                np.count_nonzero(features[left].dhash != features[right].dhash)
            )
            pair = (left, right)
            if phash_distance <= phash_max:
                candidates[pair].add("phash")
            if dhash_distance <= dhash_max:
                candidates[pair].add("dhash")
    histogram = {str(key): value for key, value in sorted(phash_histogram.items())}
    return candidates, histogram


def _convex_hull_coverage(points: np.ndarray, width: int, height: int) -> float:
    if len(points) < 3 or width <= 0 or height <= 0:
        return 0.0
    hull = cv2.convexHull(np.asarray(points, dtype=np.float32))
    return float(abs(cv2.contourArea(hull))) / float(width * height)


def _match_pair(
    left: ImageFeatures,
    right: ImageFeatures,
    *,
    reasons: Iterable[str],
    embedding_similarity: float | None,
    ratio_threshold: float,
    ransac_threshold: float,
    min_area_ratio: float,
    max_area_ratio: float,
    max_condition_number: float,
    min_inlier_coverage: float,
    strict_good: int,
    strict_inliers: int,
    strict_inlier_ratio: float,
    conservative_good: int,
    conservative_inliers: int,
    conservative_inlier_ratio: float,
    merge_conservative: bool,
) -> PairMetrics:
    phash_distance = int(np.count_nonzero(left.phash != right.phash))
    dhash_distance = int(np.count_nonzero(left.dhash != right.dhash))
    good = []
    if (
        left.descriptors is not None
        and right.descriptors is not None
        and len(left.descriptors) >= 2
        and len(right.descriptors) >= 2
    ):
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        for matches in matcher.knnMatch(left.descriptors, right.descriptors, k=2):
            if len(matches) == 2 and matches[0].distance < ratio_threshold * matches[1].distance:
                good.append(matches[0])

    inliers = 0
    inlier_ratio = 0.0
    homography_valid = False
    projected_area_ratio: float | None = None
    condition_number: float | None = None
    source_coverage = 0.0
    target_coverage = 0.0
    if len(good) >= 4:
        source_points = np.float32(
            [left.keypoints[match.queryIdx] for match in good]
        ).reshape(-1, 1, 2)
        target_points = np.float32(
            [right.keypoints[match.trainIdx] for match in good]
        ).reshape(-1, 1, 2)
        homography, mask = cv2.findHomography(
            source_points,
            target_points,
            cv2.RANSAC,
            ransac_threshold,
        )
        if homography is not None and mask is not None and np.all(np.isfinite(homography)):
            inlier_mask = mask.reshape(-1).astype(bool)
            inliers = int(np.count_nonzero(inlier_mask))
            inlier_ratio = inliers / float(len(good))
            if abs(float(homography[2, 2])) > 1e-12:
                homography = homography / homography[2, 2]
                raw_condition_number = float(np.linalg.cond(homography))
                condition_number = (
                    raw_condition_number
                    if math.isfinite(raw_condition_number)
                    else None
                )
            corners = np.float32(
                [
                    [0.0, 0.0],
                    [float(left.feature_width), 0.0],
                    [float(left.feature_width), float(left.feature_height)],
                    [0.0, float(left.feature_height)],
                ]
            ).reshape(-1, 1, 2)
            projected = cv2.perspectiveTransform(corners, homography).reshape(-1, 2)
            if np.all(np.isfinite(projected)):
                projected_contour = np.asarray(projected, dtype=np.float32).reshape(-1, 1, 2)
                projected_area_ratio = float(abs(cv2.contourArea(projected_contour))) / float(
                    right.feature_width * right.feature_height
                )
                source_inliers = source_points.reshape(-1, 2)[inlier_mask]
                target_inliers = target_points.reshape(-1, 2)[inlier_mask]
                source_coverage = _convex_hull_coverage(
                    source_inliers, left.feature_width, left.feature_height
                )
                target_coverage = _convex_hull_coverage(
                    target_inliers, right.feature_width, right.feature_height
                )
                homography_valid = bool(
                    cv2.isContourConvex(projected_contour)
                    and condition_number is not None
                    and condition_number <= max_condition_number
                    and min_area_ratio <= projected_area_ratio <= max_area_ratio
                    and source_coverage >= min_inlier_coverage
                    and target_coverage >= min_inlier_coverage
                )

    if "decoded_pixel_duplicate" in reasons:
        decision = "merge_decoded_duplicate"
    elif (
        homography_valid
        and len(good) >= strict_good
        and inliers >= strict_inliers
        and inlier_ratio >= strict_inlier_ratio
    ):
        decision = "merge_strict_geometry"
    elif (
        homography_valid
        and len(good) >= conservative_good
        and inliers >= conservative_inliers
        and inlier_ratio >= conservative_inlier_ratio
    ):
        decision = (
            "merge_conservative_geometry"
            if merge_conservative
            else "review_conservative_geometry"
        )
    else:
        decision = "separate_no_geometric_evidence"

    return PairMetrics(
        image_a=left.relative_path,
        image_b=right.relative_path,
        reasons=tuple(sorted(reasons)),
        phash_distance=phash_distance,
        dhash_distance=dhash_distance,
        embedding_similarity=embedding_similarity,
        good_matches=len(good),
        inliers=inliers,
        inlier_ratio=inlier_ratio,
        homography_valid=homography_valid,
        projected_area_ratio=projected_area_ratio,
        condition_number=condition_number,
        source_inlier_coverage=source_coverage,
        target_inlier_coverage=target_coverage,
        decision=decision,
    )


def _load_overrides(
    path: Path | None,
    known_paths: set[str],
) -> dict[tuple[str, str], tuple[str, str]]:
    if path is None:
        return {}
    decisions: dict[tuple[str, str], tuple[str, str]] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"image_a", "image_b", "decision", "reason"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(
                "override CSV must have image_a,image_b,decision,reason columns"
            )
        for row in reader:
            left = row["image_a"].strip().replace("\\", "/")
            right = row["image_b"].strip().replace("\\", "/")
            decision = row["decision"].strip().lower()
            reason = row["reason"].strip()
            if left == right or left not in known_paths or right not in known_paths:
                raise ValueError(f"invalid override pair: {left!r}, {right!r}")
            if decision not in {"merge", "separate"}:
                raise ValueError("override decision must be merge or separate")
            pair = tuple(sorted((left, right)))
            if pair in decisions and decisions[pair] != (decision, reason):
                raise ValueError(f"conflicting overrides for {pair}")
            decisions[pair] = (decision, reason)
    return decisions


def _write_groups(path: Path, assignments: Mapping[str, str]) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("image", "group_id"))
        for image_path in sorted(assignments, key=str.casefold):
            writer.writerow((image_path, assignments[image_path]))
    return _file_sha256(path)


def _json_pair(metrics: PairMetrics) -> Mapping[str, object]:
    return {
        "image_a": metrics.image_a,
        "image_b": metrics.image_b,
        "candidate_reasons": list(metrics.reasons),
        "phash_distance": metrics.phash_distance,
        "dhash_distance": metrics.dhash_distance,
        "embedding_similarity": metrics.embedding_similarity,
        "good_matches": metrics.good_matches,
        "inliers": metrics.inliers,
        "inlier_ratio": metrics.inlier_ratio,
        "homography_valid": metrics.homography_valid,
        "projected_area_ratio": metrics.projected_area_ratio,
        "condition_number": metrics.condition_number,
        "source_inlier_coverage": metrics.source_inlier_coverage,
        "target_inlier_coverage": metrics.target_inlier_coverage,
        "decision": metrics.decision,
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--audit-json", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--overrides", type=Path)
    parser.add_argument("--embedding-weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--embedding-top-k", type=int, default=20)
    parser.add_argument("--embedding-imgsz", type=int, default=320)
    parser.add_argument("--embedding-batch", type=int, default=8)
    parser.add_argument("--embedding-device", default="0")
    parser.add_argument("--phash-max", type=int, default=24)
    parser.add_argument("--dhash-max", type=int, default=48)
    parser.add_argument("--max-dimension", type=int, default=800)
    parser.add_argument("--orb-features", type=int, default=1200)
    parser.add_argument("--orb-fast-threshold", type=int, default=12)
    parser.add_argument("--lowe-ratio", type=float, default=0.75)
    parser.add_argument("--ransac-threshold", type=float, default=5.0)
    parser.add_argument("--min-area-ratio", type=float, default=0.05)
    parser.add_argument("--max-area-ratio", type=float, default=20.0)
    parser.add_argument("--max-condition-number", type=float, default=1e8)
    parser.add_argument("--min-inlier-coverage", type=float, default=0.005)
    parser.add_argument("--strict-good", type=int, default=50)
    parser.add_argument("--strict-inliers", type=int, default=40)
    parser.add_argument("--strict-inlier-ratio", type=float, default=0.50)
    parser.add_argument("--conservative-good", type=int, default=25)
    parser.add_argument("--conservative-inliers", type=int, default=20)
    parser.add_argument("--conservative-inlier-ratio", type=float, default=0.35)
    parser.add_argument(
        "--merge-conservative",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--force", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    integer_minimums = {
        "embedding_top_k": (args.embedding_top_k, 0),
        "embedding_imgsz": (args.embedding_imgsz, 32),
        "embedding_batch": (args.embedding_batch, 1),
        "phash_max": (args.phash_max, 0),
        "dhash_max": (args.dhash_max, 0),
        "max_dimension": (args.max_dimension, 64),
        "orb_features": (args.orb_features, 4),
        "orb_fast_threshold": (args.orb_fast_threshold, 0),
        "strict_good": (args.strict_good, 4),
        "strict_inliers": (args.strict_inliers, 4),
        "conservative_good": (args.conservative_good, 4),
        "conservative_inliers": (args.conservative_inliers, 4),
    }
    for name, (value, minimum) in integer_minimums.items():
        if value < minimum:
            raise ValueError(f"--{name.replace('_', '-')} must be >= {minimum}")
    if args.phash_max > 63 or args.dhash_max > 256:
        raise ValueError("perceptual hash thresholds exceed their bit lengths")
    if not 0.0 < args.lowe_ratio < 1.0:
        raise ValueError("--lowe-ratio must be between 0 and 1")
    for name in ("strict_inlier_ratio", "conservative_inlier_ratio"):
        if not 0.0 <= getattr(args, name) <= 1.0:
            raise ValueError(f"--{name.replace('_', '-')} must be between 0 and 1")
    if args.ransac_threshold <= 0.0 or args.min_inlier_coverage < 0.0:
        raise ValueError("RANSAC and coverage thresholds must be non-negative")
    if not 0.0 < args.min_area_ratio <= args.max_area_ratio:
        raise ValueError("homography area-ratio bounds are invalid")
    if args.max_condition_number <= 0.0:
        raise ValueError("--max-condition-number must be positive")
    cv2.setRNGSeed(20260710)
    source = args.source.resolve()
    if not source.is_dir():
        raise FileNotFoundError(f"source dataset directory not found: {source}")
    if args.output.exists() and not args.force:
        raise FileExistsError(f"{args.output} already exists; pass --force after review")
    if args.audit_json.exists() and not args.force:
        raise FileExistsError(
            f"{args.audit_json} already exists; pass --force after review"
        )
    relative_paths = sorted(
        (
            path.relative_to(source).as_posix()
            for path in source.rglob("*")
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        ),
        key=str.casefold,
    )
    if len(relative_paths) < 3:
        raise ValueError("at least three source images are required")

    features = _extract_features(
        source,
        relative_paths,
        max_dimension=args.max_dimension,
        orb_features=args.orb_features,
        orb_fast_threshold=args.orb_fast_threshold,
    )
    candidates, phash_histogram = _hash_candidates(
        features,
        phash_max=args.phash_max,
        dhash_max=args.dhash_max,
    )
    embedding_pairs, similarities, embedding_metadata = _embedding_candidates(
        [record.path for record in features],
        args.embedding_weights,
        top_k=args.embedding_top_k,
        imgsz=args.embedding_imgsz,
        batch=args.embedding_batch,
        device=args.embedding_device,
    )
    for pair in embedding_pairs:
        candidates[pair].add("embedding_neighbour")
    print(f"candidate pairs: {len(candidates)}", flush=True)

    metrics: list[PairMetrics] = []
    for index, ((left_index, right_index), reasons) in enumerate(
        sorted(candidates.items()), start=1
    ):
        similarity = (
            None
            if similarities is None
            else float(similarities[left_index, right_index])
        )
        metrics.append(
            _match_pair(
                features[left_index],
                features[right_index],
                reasons=reasons,
                embedding_similarity=similarity,
                ratio_threshold=args.lowe_ratio,
                ransac_threshold=args.ransac_threshold,
                min_area_ratio=args.min_area_ratio,
                max_area_ratio=args.max_area_ratio,
                max_condition_number=args.max_condition_number,
                min_inlier_coverage=args.min_inlier_coverage,
                strict_good=args.strict_good,
                strict_inliers=args.strict_inliers,
                strict_inlier_ratio=args.strict_inlier_ratio,
                conservative_good=args.conservative_good,
                conservative_inliers=args.conservative_inliers,
                conservative_inlier_ratio=args.conservative_inlier_ratio,
                merge_conservative=args.merge_conservative,
            )
        )
        if index % 500 == 0 or index == len(candidates):
            print(f"geometry: {index}/{len(candidates)}", flush=True)

    overrides = _load_overrides(args.overrides, set(relative_paths))
    pair_to_metrics = {
        tuple(sorted((item.image_a, item.image_b))): item for item in metrics
    }
    union_find = UnionFind(relative_paths)
    for item in metrics:
        pair = tuple(sorted((item.image_a, item.image_b)))
        override = overrides.get(pair)
        should_merge = item.decision.startswith("merge_")
        if override is not None:
            should_merge = override[0] == "merge"
        if should_merge:
            union_find.union(*pair)
    for pair, (decision, _reason) in overrides.items():
        if pair not in pair_to_metrics and decision == "merge":
            union_find.union(*pair)
    violated_separations = [
        pair
        for pair, (decision, _reason) in overrides.items()
        if decision == "separate" and union_find.find(pair[0]) == union_find.find(pair[1])
    ]
    if violated_separations:
        examples = ", ".join(f"{left}<->{right}" for left, right in violated_separations[:3])
        raise ValueError(
            "manual separate overrides conflict with transitive merge edges "
            f"(examples: {examples})"
        )

    components: dict[str, list[str]] = defaultdict(list)
    for relative_path in relative_paths:
        components[union_find.find(relative_path)].append(relative_path)
    ordered_components = sorted(
        (sorted(members, key=str.casefold) for members in components.values()),
        key=lambda members: members[0].casefold(),
    )
    assignments: dict[str, str] = {}
    for members in ordered_components:
        stable_key = "\n".join(members).encode("utf-8")
        group_id = f"scene_{hashlib.sha256(stable_key).hexdigest()[:12]}"
        for member in members:
            assignments[member] = group_id

    decision_counts = Counter(item.decision for item in metrics)
    group_sizes = Counter(len(members) for members in ordered_components)
    cross_directory_groups = [
        members
        for members in ordered_components
        if len({Path(member).parts[0] for member in members}) > 1
    ]
    audit = {
        "schema_version": 1,
        "dataset_id": "zenodo_6126677",
        "source_root": str(source),
        "algorithm": {
            "name": "phash_dhash_yolo_orb_scene_grouping",
            "curation_scope": (
                "conservative visual grouping; original capture-session IDs "
                "are unavailable in the source archive"
            ),
            "opencv_version": cv2.__version__,
            "rng_seed": 20260710,
            "phash_bits": 63,
            "phash_max": args.phash_max,
            "dhash_bits": 256,
            "dhash_max": args.dhash_max,
            "max_dimension": args.max_dimension,
            "orb_features": args.orb_features,
            "orb_fast_threshold": args.orb_fast_threshold,
            "lowe_ratio": args.lowe_ratio,
            "ransac_threshold_px": args.ransac_threshold,
            "homography": {
                "min_area_ratio": args.min_area_ratio,
                "max_area_ratio": args.max_area_ratio,
                "max_condition_number": args.max_condition_number,
                "min_inlier_coverage": args.min_inlier_coverage,
            },
            "strict_geometry": {
                "good_matches_min": args.strict_good,
                "inliers_min": args.strict_inliers,
                "inlier_ratio_min": args.strict_inlier_ratio,
            },
            "conservative_geometry": {
                "good_matches_min": args.conservative_good,
                "inliers_min": args.conservative_inliers,
                "inlier_ratio_min": args.conservative_inlier_ratio,
                "merged": args.merge_conservative,
            },
            "embedding": embedding_metadata,
        },
        "input": {
            "image_count": len(features),
            "images": [
                {
                    "path": record.relative_path,
                    "sha256": record.file_sha256,
                    "decoded_sha256": record.decoded_sha256,
                    "width": record.width,
                    "height": record.height,
                    "feature_width": record.feature_width,
                    "feature_height": record.feature_height,
                    "orb_keypoints": len(record.keypoints),
                }
                for record in features
            ],
        },
        "candidate_summary": {
            "pair_count": len(metrics),
            "decision_counts": dict(sorted(decision_counts.items())),
            "phash_distance_histogram_all_pairs": phash_histogram,
        },
        "pairs": [_json_pair(item) for item in metrics],
        "manual_overrides": [
            {
                "image_a": pair[0],
                "image_b": pair[1],
                "decision": decision,
                "reason": reason,
            }
            for pair, (decision, reason) in sorted(overrides.items())
        ],
        "output": {
            "groups_csv": str(args.output.resolve()),
            "groups_csv_sha256": None,
            "image_count": len(assignments),
            "group_count": len(ordered_components),
            "group_size_histogram": {
                str(size): count for size, count in sorted(group_sizes.items())
            },
            "cross_original_directory_group_count": len(cross_directory_groups),
            "cross_original_directory_groups": cross_directory_groups,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.audit_json.parent.mkdir(parents=True, exist_ok=True)
    groups_temporary = args.output.with_name(args.output.name + ".part")
    audit_temporary = args.audit_json.with_name(args.audit_json.name + ".part")
    for temporary in (groups_temporary, audit_temporary):
        if temporary.exists():
            temporary.unlink()
    try:
        groups_sha256 = _write_groups(groups_temporary, assignments)
        audit["output"]["groups_csv_sha256"] = groups_sha256
        audit_temporary.write_text(
            json.dumps(
                audit,
                indent=2,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        # Publish the audit first and the formal splitter input last.  A crash
        # can therefore never leave a groups CSV without its matching audit.
        os.replace(audit_temporary, args.audit_json)
        os.replace(groups_temporary, args.output)
    finally:
        for temporary in (groups_temporary, audit_temporary):
            if temporary.exists():
                temporary.unlink()
    print(
        json.dumps(
            {
                "images": len(assignments),
                "groups": len(ordered_components),
                "candidate_pairs": len(metrics),
                "decisions": dict(sorted(decision_counts.items())),
                "groups_csv": str(args.output),
                "groups_csv_sha256": groups_sha256,
                "audit_json": str(args.audit_json),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"scene grouping failed: {error}", file=sys.stderr)
        raise SystemExit(2) from error
