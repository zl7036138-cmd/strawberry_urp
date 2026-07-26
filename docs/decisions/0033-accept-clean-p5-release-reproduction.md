# ADR 0033: Accept the clean P5 release reproduction

- Recorded: 2026-07-24
- Status: Accepted
- Scope: Clean Ubuntu 24.04 WSL2 build and release-smoke comparison

## Context

P5 requires a clean-environment reproduction independent of the development
distribution. The reproduction must preserve the failed formal P3 result, the
rejected P4 intervention, and the sealed real-image test. It is not permission
to repeat either consumed experiment.

A separate WSL2 distribution named `Ubuntu-24.04-URP-Repro` was created without
changing the existing Ubuntu 22.04 or project Ubuntu 24.04 distributions. The
ROS and system packages were installed into that new distribution. Cached
signed Debian packages and a hash-verified Python wheelhouse were used only as
installation bytes because direct downloads were unreliable; the build,
installation, user home, ROS workspace artifacts, and experiment processes
were all new-distribution local.

## Evidence

`results/p5/clean_build_v1/summary.json` records:

- Ubuntu 24.04.4 LTS, ROS 2 Jazzy, normal user `lzl`;
- seven project packages built into
  `/home/lzl/.cache/strawberry_urp/colcon`;
- 246 tests, zero errors, zero failures, and zero skips;
- Ultralytics 8.4.92, OpenCV 4.11.0, NumPy-compatible project bindings,
  PyTorch 2.13.0+cu130 and torchvision 0.28.0+cu130; and
- an NVIDIA GeForce RTX 4060 Laptop GPU visible through CUDA.

`results/p5/release_smoke_clean_v1/summary.json` records:

- 5/5 clear ripe positive trials completed successfully;
- 5/5 only-unripe negative trials safely returned `NO_PICK`;
- zero negative control attempts and zero false picks;
- all infrastructure and shutdown checks passed;
- a composite behaviour pass rate of 1.0; and
- an absolute difference of 0.0 from the frozen reference rate, within the
  maximum allowed difference of 0.05.

Both receipts state that the held-out real-image test was not consumed, formal
P3 was not rerun, the rejected P4 intervention was not used, and no physical
robot or sim-to-real claim is made.

## Decision

Accept the P5 clean-environment reproduction subgate. The project now has a
verified clean Ubuntu 24.04 WSL2 build and a matching ten-trial release smoke.

This decision does not change the scientific outcomes: T30 remains below its
numeric F1 gate under an engineering waiver, formal P3 remains failed at
39/135 positive successes, and the sole P4 intervention remains rejected.
P5 stays in progress until the headed demonstration recording is captured,
assembled, checked, and bound into the final release manifest.

## Boundaries

Do not call the ten-trial smoke a replacement for the formal 135+30 P3 matrix.
Do not use it as real-image, physical-robot, fruit-damage, stem-cutting, or
sim-to-real evidence. Keep the separate reproduction distribution available
until the release is frozen so the receipt can be independently inspected.
