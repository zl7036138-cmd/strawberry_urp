# Permanent final evidence

This directory is the clone-safe evidence subset for the 2026-10 closure pass.
`manifest.json` records every file's size and SHA-256 and verifies fixed hashes
for seven critical ADR 0086/0087 runtime artifacts.

The directory contains:

- ADR 0086 positive and deliberately blocked-transport runtime receipts,
  truth-isolation reports, and cleanup reports;
- ADR 0087 plan-only and controlled positive receipts, including the exact
  controlled-challenge world;
- canonical Ubuntu/WSL dependency-light and full ROS/colcon test receipts;
- a noncanonical Windows test receipt retained to expose platform assumptions.

The runtime receipts prove only the bounded claims stated in
`docs/FINAL_ARCHITECTURE.md`. They do not open the formal 30-seed matrix or
establish a general adaptive-grasp success-rate improvement.
