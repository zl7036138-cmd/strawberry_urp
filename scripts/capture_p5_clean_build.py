#!/usr/bin/env python3
"""Capture and validate the clean-distribution P5 build/test receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess

import cv2
import torch
import torchvision
import ultralytics


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _source_binding(relative_path: str) -> dict:
    path = REPOSITORY_ROOT / relative_path
    return {
        "path": relative_path,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _os_release() -> dict[str, str]:
    result = {}
    for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value.strip().strip('"')
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--artifact-root",
        type=Path,
        default=Path.home() / ".cache" / "strawberry_urp" / "colcon",
    )
    arguments = parser.parse_args()
    build_root = arguments.artifact_root.resolve()
    test_root = build_root / "build"
    install_setup = build_root / "install" / "setup.bash"
    if not install_setup.is_file():
        raise SystemExit(f"clean install setup is missing: {install_setup}")

    command = [
        "colcon",
        "test-result",
        "--test-result-base",
        str(test_root),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    rendered = (completed.stdout + completed.stderr).strip()
    match = re.search(
        r"Summary:\s+(\d+)\s+tests,\s+(\d+)\s+errors,\s+"
        r"(\d+)\s+failures,\s+(\d+)\s+skipped",
        rendered,
    )
    if completed.returncode != 0 or not match:
        raise SystemExit(f"cannot validate colcon results:\n{rendered}")
    tests, errors, failures, skipped = (int(value) for value in match.groups())
    result_files = sorted(test_root.glob("*/test_results/**/*.xml"))
    digest_rows = [
        (
            path.relative_to(test_root).as_posix(),
            path.stat().st_size,
            _sha256(path),
        )
        for path in result_files
    ]
    canonical = hashlib.sha256(
        json.dumps(digest_rows, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    package_names = sorted(
        path.parent.name
        for path in (build_root / "install").glob("*/share/*/package.xml")
    )
    release = _os_release()
    distro = os.environ.get("WSL_DISTRO_NAME")
    user = os.environ.get("USER")
    passed = (
        distro == "Ubuntu-24.04-URP-Repro"
        and user == "lzl"
        and release.get("ID") == "ubuntu"
        and release.get("VERSION_ID") == "24.04"
        and os.environ.get("ROS_DISTRO") == "jazzy"
        and tests == 246
        and errors == failures == skipped == 0
        and len(package_names) == 7
        and torch.cuda.is_available()
    )
    receipt = {
        "schema_version": 1,
        "kind": "p5_clean_build_receipt",
        "status": "PASS" if passed else "FAIL",
        "environment": {
            "wsl_distribution": distro,
            "user": user,
            "os_id": release.get("ID"),
            "os_version_id": release.get("VERSION_ID"),
            "os_pretty_name": release.get("PRETTY_NAME"),
            "kernel": platform.release(),
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "python": platform.python_version(),
            "cuda_device": (
                torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
            ),
        },
        "software": {
            "opencv": cv2.__version__,
            "torch": torch.__version__,
            "torchvision": torchvision.__version__,
            "ultralytics": ultralytics.__version__,
        },
        "build": {
            "artifact_root": str(build_root),
            "fresh_distribution_local_storage": True,
            "packages": package_names,
            "package_count": len(package_names),
        },
        "tests": {
            "tests": tests,
            "errors": errors,
            "failures": failures,
            "skipped": skipped,
            "result_file_count": len(result_files),
            "result_files_canonical_sha256": canonical,
            "colcon_summary": rendered,
        },
        "source_bindings": [
            _source_binding("scripts/bootstrap_ubuntu_2404.sh"),
            _source_binding("scripts/build_and_test.sh"),
            _source_binding("requirements/perception.txt"),
            _source_binding(
                "requirements/perception-linux-cp312-wheelhouse.txt"
            ),
            _source_binding("scripts/cache_perception_wheels.ps1"),
            _source_binding(".cache/wheels/wheelhouse-manifest.json"),
        ],
        "scope": {
            "held_out_real_test_consumed": False,
            "formal_p3_rerun": False,
            "p4_intervention_used": False,
            "physical_robot_evidence": False,
            "sim_to_real_claim": False,
        },
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
