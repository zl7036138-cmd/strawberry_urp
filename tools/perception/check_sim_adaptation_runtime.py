#!/usr/bin/env python3
"""Read-only Ultralytics/CUDA readiness check before the ADR-0021 claim."""

from __future__ import annotations

import json
from pathlib import Path

import torch
from ultralytics import YOLO
from ultralytics.data.utils import check_det_dataset


ROOT = Path(__file__).resolve().parents[2]
DATASET = ROOT / "data/processed/yolo11s_640_sim_adapt_v1/dataset.yaml"
MODEL = ROOT / "outputs/perception/yolo11s_640/weights/best.pt"
CLAIM = ROOT / "artifacts/perception/training/yolo11s_640_sim_adapt_v1.claim.json"
OUTPUT = ROOT / "outputs/perception/yolo11s_640_sim_adapt_v1"


def main() -> int:
    if CLAIM.exists() or OUTPUT.exists():
        raise FileExistsError("training claim or output already exists")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    gpu = torch.cuda.get_device_name(0)
    if "RTX 4060" not in gpu:
        raise RuntimeError(f"unexpected training GPU: {gpu}")
    dataset = check_det_dataset(str(DATASET), autodownload=False)
    train = ROOT / "data/processed/yolo11s_640_sim_adapt_v1/images/train"
    val = ROOT / "data/processed/yolo11s_640_sim_adapt_v1/images/val"
    train_count = sum(path.is_file() for path in train.iterdir())
    val_count = sum(path.is_file() for path in val.iterdir())
    if train_count != 717 or val_count != 72:
        raise ValueError("Ultralytics dataset image count mismatch")
    model = YOLO(str(MODEL))
    names = {int(key): str(value) for key, value in model.names.items()}
    if names != {0: "ripe", 1: "unripe"}:
        raise ValueError(f"baseline model class contract changed: {names}")
    print(
        json.dumps(
            {
                "runtime_ready": True,
                "cuda": True,
                "gpu": gpu,
                "train_images": train_count,
                "validation_images": val_count,
                "resolved_train": dataset["train"],
                "resolved_val": dataset["val"],
                "model_names": names,
                "claim_exists": False,
                "output_exists": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
