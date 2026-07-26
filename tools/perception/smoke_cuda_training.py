#!/usr/bin/env python3
"""Run a non-formal, train-split-only CUDA/backpropagation smoke test.

This command intentionally bypasses the formal training receipt and writes only
under ``.codex_tmp``.  Both YAML split paths alias the processed train split, so
even Ultralytics' end-of-epoch validator cannot read formal validation or test
images.  Its purpose is to prove the frozen batch size, image size, workers,
CUDA stack, and YOLO training path before a one-time formal job is claimed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys
import time
from typing import Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DATASET_ROOT = REPOSITORY_ROOT / "data" / "processed" / "zenodo_6126677"
SPLIT_MANIFEST = DATASET_ROOT / "split_manifest.json"
EXPECTED_WEIGHTS = {
    "yolo11n": (
        REPOSITORY_ROOT / "weights" / "yolo11n.pt",
        "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1",
    ),
    "yolo11s": (
        REPOSITORY_ROOT / "weights" / "yolo11s.pt",
        "85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5",
    ),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=tuple(EXPECTED_WEIGHTS), default="yolo11s")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--fraction", type=float, default=0.10)
    parser.add_argument(
        "--amp",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "enable Ultralytics AMP; disabled by default because 8.4.92 tries "
            "to download an unrelated YOLO26n probe model"
        ),
    )
    parser.add_argument("--name", default="yolo11s_640_batch8")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.imgsz < 32 or args.batch < 1 or args.workers < 0:
        raise ValueError("invalid smoke-test resource arguments")
    if not 0.0 < args.fraction <= 1.0:
        raise ValueError("--fraction must be in (0, 1]")
    output_project = REPOSITORY_ROOT / ".codex_tmp" / "perception_smoke"
    output_directory = output_project / args.name
    summary_path = output_project / f"{args.name}_summary.json"
    data_yaml = output_project / f"{args.name}_train_only.yaml"
    if output_directory.exists() or summary_path.exists() or data_yaml.exists():
        raise FileExistsError(
            "smoke-test output already exists; choose a new --name instead of "
            "overwriting diagnostic evidence"
        )

    sys.path.insert(0, str(REPOSITORY_ROOT / "tools" / "data"))
    from dataset_tools import verify_materialized_split_manifest

    verification = verify_materialized_split_manifest(SPLIT_MANIFEST)
    if not verification["valid"]:
        raise ValueError("materialized dataset verification failed")
    weights, expected_hash = EXPECTED_WEIGHTS[args.model]
    actual_hash = _sha256(weights)
    if actual_hash != expected_hash:
        raise ValueError(
            f"{args.model} weight hash mismatch: expected {expected_hash}, got {actual_hash}"
        )

    output_project.mkdir(parents=True, exist_ok=True)
    # Both fields intentionally reference train.  Ultralytics may still build
    # an end-of-epoch validator with ``val=False``; the alias guarantees that
    # it remains train-only.
    data_yaml.write_text(
        "\n".join(
            (
                f"path: {DATASET_ROOT.as_posix()}",
                "train: images/train",
                "val: images/train",
                "names:",
                "  0: ripe",
                "  1: unripe",
                "",
            )
        ),
        encoding="utf-8",
    )

    import torch
    import ultralytics
    from ultralytics import YOLO

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is not available")
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    started = time.monotonic()
    status = "failed"
    error_text = None
    try:
        model = YOLO(str(weights))
        model.train(
            data=str(data_yaml),
            epochs=1,
            fraction=args.fraction,
            imgsz=args.imgsz,
            batch=args.batch,
            device=0,
            workers=args.workers,
            cache=False,
            val=False,
            plots=False,
            save=False,
            project=str(output_project),
            name=args.name,
            exist_ok=False,
            seed=20260710,
            deterministic=True,
            amp=args.amp,
            verbose=False,
        )
        status = "passed"
    except BaseException as error:
        error_text = f"{type(error).__name__}: {error}"
        raise
    finally:
        duration = time.monotonic() - started
        summary = {
            "schema_version": 1,
            "formal": False,
            "scope": (
                "processed train split only; YAML val aliases train and no formal "
                "validation/test images are addressable"
            ),
            "status": status,
            "error": error_text,
            "model": args.model,
            "weights": str(weights),
            "weights_sha256": actual_hash,
            "canonical_dataset_sha256": verification[
                "canonical_dataset_sha256"
            ],
            "train_sample_count": verification["split_sample_count"]["train"],
            "parameters": {
                "epochs": 1,
                "fraction": args.fraction,
                "imgsz": args.imgsz,
                "batch": args.batch,
                "workers": args.workers,
                "seed": 20260710,
                "deterministic": True,
                "amp": args.amp,
            },
            "environment": {
                "python": platform.python_version(),
                "torch": torch.__version__,
                "ultralytics": ultralytics.__version__,
                "cuda_runtime": torch.version.cuda,
                "gpu": torch.cuda.get_device_name(0),
            },
            "duration_seconds": duration,
            "peak_cuda_memory_bytes": torch.cuda.max_memory_allocated(),
            "output_directory": str(output_directory),
        }
        summary_path.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )
    print(summary_path.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
