# ADR 0016: Freeze a post-setup fixed-frame Shadow window

- Status: Accepted
- Date: 2026-07-15

## Context

The ADR 0015 v1 pilot proves Shadow topic liveness, but its client counts from
launch until trial completion. Those totals include frames before the two
non-target fruit are parked and span unequal execution times. Comparing them
would confound scene setup, duration, and the intended light/occlusion factor.

## Decision

Keep the v1 manifest and result immutable. Add a schema-v2 pilot manifest that
requires an exact 60-detection-frame observation window for every condition.
The window begins only after the condition probe has acknowledged the target
and both parked fruit, and ends before the robot trial begins. Wait one
additional wall-clock second to associate localization messages by acquisition
timestamp.

Record per frame the ripe/unripe box counts and whether a Shadow target pose
with the same image stamp arrived. Empty detection arrays are valid evidence;
missing frames, a wrong boundary, or a mismatched condition invalidates the v2
pilot. These measurements remain diagnostic and do not set an acceptance
threshold after seeing the result.

## Consequences

The v2 window permits fair descriptive comparison across the three pilot
conditions. It still uses a simplistic simulator, one position, one seed, and
the below-gate model, so it cannot establish P3/P4 robustness or replace the
sealed real-image test.

The completed v2 run collects all 180 required frames and retains 3/3 Oracle
motion success. At `nominal+none`, ripe detection and target-pose availability
are both 60/60 frames. At `dim+none`, ripe detection remains 60/60 while target
pose availability is 42/60. At `nominal+heavy`, both are 0/60. This identifies
a severe Shadow perception sensitivity to the frozen heavy visual occluder, but
only at one simulator position and seed; it is not a formal robustness rate.
