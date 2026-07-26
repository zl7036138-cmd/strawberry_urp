#!/usr/bin/env python3
"""Fail-fast verification for the ROS, OpenCV, YOLO, and CUDA environment."""

from __future__ import annotations

import json

import cv2
import numpy as np
import torch
import torchvision
import ultralytics
from cv_bridge import CvBridge


def main() -> None:
    if not torch.cuda.is_available():
        raise SystemExit("CUDA is not visible to PyTorch")

    left = torch.tensor([[1.0, 2.0], [3.0, 4.0]], device="cuda")
    right = torch.tensor([[2.0], [1.0]], device="cuda")
    product = left @ right
    torch.cuda.synchronize()
    if product.cpu().tolist() != [[4.0], [10.0]]:
        raise SystemExit("CUDA matrix multiplication returned an unexpected result")

    bridge = CvBridge()
    source = np.zeros((4, 6, 3), dtype=np.uint8)
    source[1, 2] = (11, 22, 33)
    message = bridge.cv2_to_imgmsg(source, encoding="bgr8")
    restored = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
    if not np.array_equal(source, restored):
        raise SystemExit("cv_bridge image round trip changed pixel data")

    report = {
        "cuda_available": True,
        "cuda_device": torch.cuda.get_device_name(0),
        "cv2": cv2.__version__,
        "cv_bridge_round_trip": True,
        "numpy": np.__version__,
        "torch": torch.__version__,
        "torch_cuda": torch.version.cuda,
        "torchvision": torchvision.__version__,
        "ultralytics": ultralytics.__version__,
    }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
