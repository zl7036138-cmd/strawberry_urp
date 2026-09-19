# ADR 0065: Qualify the generalized detector and keep formal runtime sealed

## Status

Accepted on 2026-08-12.

## Context

The generalized fixed-arm path needed a detector trained only on randomized
development scenes, an independent qualification pass with frozen inference
parameters, and runtime evidence before the 30 one-attempt held-out behavioral
matrix could be consumed.

## Decision

Accept candidate v2 for generalized development runtime with image size 800,
confidence 0.20524564385414124, and NMS IoU 0.50. On the 24-image validation
split it reached ripe precision 96.15% and recall 90.91%. The same frozen
settings were then used once on the independent 24-image qualification split,
where ripe precision was 95.65% and recall was 97.78%.

Do not authorize the formal 30-scene behavior matrix yet. Fresh runtime probes
showed that the limiting layer is now RGB-D localization and safe manipulation,
not detector qualification. Seed 44003 contained four scoring-truth ripe fruit
but formed only one stable ripe runtime track and safely ended `NO_PICK`.

A probe also found that a skipped fruit could return under a new tracking ID
after the old track aged out. Completed tracks are therefore retained as
stationary geometric tombstones with a narrower 40 mm suppression gate, and
the batch state machine now has a hard terminal target limit for both success
and skip paths.

## Consequences

- The accepted detector settings are shared by both cameras, both localizers,
  the target selector, and wrist confirmation in the generalized launch.
- Gazebo truth remains scoring-only and is not used by runtime target identity,
  maturity, ranking, pose correction, or planning.
- The fixed-scene baseline and all historical formal outcomes remain unchanged.
- The next milestone is a seed-disjoint RGB-D localization/runtime gate, not
  additional detector tuning or consumption of formal seeds.

The machine-readable authority is
`config/generalized_development_outcome_v2.json`.
