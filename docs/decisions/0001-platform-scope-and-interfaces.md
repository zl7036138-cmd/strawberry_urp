# ADR-0001: Platform, scope, and interface baseline

- Status: Accepted
- Date: 2026-07-10

## Decision

Use Ubuntu 24.04 WSL2, ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, a fixed
eye-to-hand RGB-D camera, and a Panda arm. Perception is a binary ripe/unripe
YOLO11s detector. The v1 task ends when a rigid ripe fruit is grasped, carried,
released into the bin, and remains there for one simulated second.

The ROS API and retry semantics in `docs/architecture.md` are frozen for v1.

## Consequences

- Existing Ubuntu 22.04 remains untouched.
- All batch benchmarks must run headlessly; WSLg is a development convenience.
- Ground-truth target pose remains available as an oracle path until the
  perception/localization chain passes its own gate.
- Hardware control, stem cutting, deformable fruit, and sim-to-real claims are
  outside the project boundary.

