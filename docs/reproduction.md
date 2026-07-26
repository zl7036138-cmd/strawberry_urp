# Release reproduction guide

This guide targets a clean Ubuntu 24.04 WSL2 distribution. It reproduces the
engineering environment and launches the simulator; it does not authorize the
sealed real-image test, another formal P3 matrix, or another P4 intervention.

## Required local artifacts

Copy these files into the repository before bootstrap:

| Artifact | Repository destination |
|---|---|
| Zenodo `strawberries.zip` | `data/raw/zenodo_6126677/strawberries.zip` |
| Ultralytics `yolo11s.pt` | `weights/yolo11s.pt` |
| Ultralytics `yolo11n.pt` | `weights/yolo11n.pt` |

The project bootstrap and perception manifests verify the expected sizes and
digests. Trained project checkpoints under `outputs/perception` must also be
included in a release archive because they are generated artifacts rather than
Git-tracked source.

For unreliable or offline Python package installation, prepare the exact
CPython 3.12 Linux wheelhouse from Windows PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/cache_perception_wheels.ps1
```

The script downloads the versions in
`requirements/perception-linux-cp312-wheelhouse.txt`, verifies every wheel
against its PyPI SHA-256 digest, and writes
`.cache/wheels/wheelhouse-manifest.json`. The bootstrap consumes this
wheelhouse when it is present.

## Three deployment commands

Run the environment step once as `root` inside the clean distribution:

```bash
bash scripts/bootstrap_ubuntu_2404.sh
```

Then run these commands as the normal WSL user from the repository root:

```bash
bash scripts/build_and_test.sh
bash scripts/run_system.sh headless:=false
```

The first command is the single build-and-test entry point. The second is the
single system launch entry point; use `headless:=true` for unattended runs.

## Verification commands

Confirm CUDA, OpenCV, cv_bridge and YOLO compatibility:

```bash
bash scripts/verify_environment.sh
```

Regenerate and verify the frozen P5 tables and SVG figures without ROS:

```bash
python3 scripts/generate_p5_release.py
python3 scripts/verify_p5_release.py
```

The current environment reference is a separate ten-trial smoke:

```bash
bash scripts/run_p5_release_smoke.sh \
  results/p5/release_smoke_reference_v1 reference 220
```

It is already frozen at 10/10 correct behaviours. In the new clean
distribution, use a new output directory and compare against that summary:

```bash
bash scripts/run_p5_release_smoke.sh \
  results/p5/release_smoke_clean_v1 clean 220 \
  results/p5/release_smoke_reference_v1/summary.json
```

The composite ten-trial behaviour rate must differ by no more than 0.05. With
ten trials, this effectively requires the same number of correct behaviours as
the reference. The clean run also requires all infrastructure and shutdown
checks to pass.

The accepted clean reproduction used the separate
`Ubuntu-24.04-URP-Repro` distribution. It passed all 246 colcon tests and all
10 smoke behaviours; the absolute difference from the reference was 0.0.
The frozen handoff is
`artifacts/p5/p5_clean_reproduction_handoff_v1.json`.

The final P5 status is `FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4`. ADR 0034
binds the verified headed demonstrations and 288.7-second H.264 video into the
release. Run `python3 scripts/verify_p5_release.py`; the expected verifier
status is `VERIFIED_FINAL_RELEASE`.

## Acceptance boundaries

- Do not open `data/processed/.../test` or create the held-out-test receipt.
- Do not rerun or selectively replace any trial in the consumed 135+30 P3
  matrix.
- Do not run the rejected P4 checkpoint as a control source.
- Do not claim real-robot, fruit-damage, stem-cutting, or sim-to-real evidence.
- Preserve the failed P3 and P4 outcomes in all reports.
