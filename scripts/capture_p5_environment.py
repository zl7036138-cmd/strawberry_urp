#!/usr/bin/env python3
"""Capture a compact machine-readable environment receipt for P5 smoke runs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import subprocess


def _command(*command: str) -> str | None:
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=20,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    text = (result.stdout or result.stderr).strip()
    return text if text else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=("reference", "clean"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    os_release = {}
    release_path = Path("/etc/os-release")
    if release_path.is_file():
        for line in release_path.read_text(encoding="utf-8").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                os_release[key] = value.strip().strip('"')
    receipt = {
        "schema_version": 1,
        "kind": "p5_reproduction_environment",
        "declared_role": arguments.role,
        "clean_install_is_user_attested": arguments.role == "clean",
        "wsl_distribution": os.environ.get("WSL_DISTRO_NAME"),
        "os": {
            "id": os_release.get("ID"),
            "version_id": os_release.get("VERSION_ID"),
            "pretty_name": os_release.get("PRETTY_NAME"),
            "kernel": platform.release(),
            "machine": platform.machine(),
        },
        "software": {
            "python": platform.python_version(),
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "gz_version": _command("gz", "sim", "--versions"),
            "nvidia_smi": _command(
                "nvidia-smi",
                "--query-gpu=name,driver_version,memory.total",
                "--format=csv,noheader",
            ),
        },
        "interpretation": (
            "The clean role records operator attestation, not cryptographic proof of "
            "a newly provisioned distribution. Preserve the distro provisioning log."
        ),
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
