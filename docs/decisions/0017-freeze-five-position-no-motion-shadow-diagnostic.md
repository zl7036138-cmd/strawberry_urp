# ADR 0017: Freeze a five-position no-motion Shadow diagnostic

- Status: Accepted
- Date: 2026-07-16

## Context

The ADR 0016 fixed-window pilot separates scene setup from measurement and
shows 60/60 ripe frames at `nominal+none`, 60/60 at `dim+none`, and 0/60 at
`nominal+heavy`. That evidence uses only one target position. It is therefore
insufficient to distinguish a position-specific projection artifact from a
condition-wide detection failure, and it does not show where detection exists
but localization fails.

The formal 135-positive plus 30-negative matrix must not begin while T30 and
full P3 remain unaccepted. Reusing three seed labels would also be misleading:
the current deterministic materializer has one frozen seed and does not yet
implement independent scene randomization.

## Decision

Run one bounded diagnostic, `t70_shadow_multiposition_fixed_window_v1`, over
five previously verified single-target reachability candidates. Rename them
only as diagnostic camera-relative labels: `near_left`, `near_right`, `center`,
`far_left`, and `far_right`; retain each original `camera_clear_*` source label
in the manifest.

At every position run exactly three one-factor conditions in the same order:
`nominal+none`, `dim+none`, and `nominal+heavy`. Collect five scene-condition
probe frames, then exactly 60 Shadow detection frames and wait one wall-clock
second for timestamp-associated target poses. Do not start Oracle,
orchestration, attachment, manipulation, or robot motion. Use the existing
below-gate model at confidence 0.31 strictly as Shadow observation.

For heavy occlusion, move the visual-only occluder from its accepted anchor by
70% of the target's Cartesian displacement. This preserves the accepted anchor
geometry while avoiding the known error of leaving a fixed occluder at one
world coordinate for all five positions. Reject target offsets greater than
0.10 m. The occluder remains collision-free.

The single seed `20260710` has only the role
`single_diagnostic_materialization_only`; this diagnostic claims zero
independent random seeds. Completion requires 15 valid scenarios and 900 fixed
Shadow frames plus valid light/occlusion injection. No detection, localization,
or success-rate threshold is introduced after observing results. Results are
reported descriptively as `DETECTION_ABSENT`,
`LOCALIZATION_OR_PIPELINE_GAP`, or `OBSERVATION_COMPLETE`.

## Consequences

The diagnostic can reveal whether heavy-occlusion failure and dim-light
localization loss repeat across positions, and can justify a later frozen
simulator-specific intervention. It cannot close T30, P2, P3, or P4; cannot
replace the held-out real-image test; and cannot be reported as a three-seed or
formal robustness result.
