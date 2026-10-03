# ADR 0081: Preserve same-view systematic uncertainty

## Status

Accepted as a safety correction. Five previously MoveIt-eligible development
scenes now fail closed before motion. No trajectory was executed and the
qualification and formal matrices remain untouched.

## Context

ADR 0080 added a depth-observability gate before multi-fruit behavior. A fresh
18-scene discovery sweep over seeds 45201–45218 then exposed a complete split:
six scenes passed the observability gate, five different scenes passed the old
MoveIt gate, and no scene passed both. All 18 scenes contained at least two
depth-visible ripe fruits, so projection or total occlusion was not the main
cause.

Across the batch, 64 ripe fruits were depth-visible and 35 were localized
within 30 mm. The localizer accepted 47 ripe positions, but only 35 were true
ripe positions within 30 mm. Ten of the 12 false positions were 52–90 mm away
from a ripe fruit and two were associated with an unripe fruit. Far scenes had
50% visible-ripe localization recall and 56% accepted-position precision,
while five of six far scenes appeared MoveIt eligible.

The apparently feasible far tracks were unsafe. Repeated frames from one fixed
RGB-D viewpoint were fused as if their uncertainty were independent. Stable
but biased depth layers therefore collapsed to the 5 mm tracker floor after
roughly 80 frames. In addition, the base launch multiplied the geometry-size
residual by 0.5, while the offline diagnostic used the configuration default
of 1.0. On seed 45207, a position with 71.8 mm truth error entered MoveIt with
5 mm reported uncertainty.

## Decision

- Treat repeated observations from one unchanged viewpoint as correlated for
  safety. Until an explicit tracker reset, retain the maximum of prior sigma,
  current sigma, and position innovation; frame count alone cannot reduce a
  systematic bound, while small jitter cannot accumulate without limit.
- Keep the existing tracker reset on a new wrist target hint, so a genuinely
  new settled viewpoint starts a fresh uncertainty history.
- Make the base-camera geometry residual weight explicitly 1.0 in the shared
  generalized localization configuration. Remove the unsafe base launch
  override of 0.5.
- Preserve the wrist-camera override at 0.5 because its 640×480 close-range
  confirmation path has separate validation evidence.
- Do not change the 15 mm control uncertainty threshold, maturity threshold,
  collision limits, retry count, or target correction limit.
- Keep every comparison no-motion and development-only. Runtime control still
  has no truth input.

## Evidence

The frozen pre-fix discovery sweep completed 18/18 materializations, graph
audits, and clean shutdowns with zero trajectory execution. It reported six
observability-eligible scenes, five MoveIt-eligible scenes, zero jointly
eligible scenes, and has SHA-256
`b7ef82e19e3c2183a253995061ac43e99123f60270dac37e0db7b3ac6f1ce94d`.

Seeds 45207, 45208, 45209, 45216, and 45217 were the five old MoveIt-eligible
scenes. After the correction, all five returned `eligible: false`; all five
truth-isolation audits passed and all five process groups exited `CLEAN`.
MoveIt evaluations fell from 13 to 5; eight candidates were explicitly
rejected as `HIGH_UNCERTAINTY`, and one replay produced no evaluable ripe
candidate in its bounded sample window. The corrected probe hashes are:

- 45207: `5495ffad7dfb87bb144ee4924e4b85e1e47cdd47e0a5b7ac116c159031822fba`
- 45208: `d02cc99b69fdec21880b424f86aeff5bf801f2bb2a9d67542aa3ce9251e4098c`
- 45209: `211db981188a64aaa9a21237c6462f665263846d36890993ad5be7f228873c59`
- 45216: `a216a2abadfee2b5d3f24d74b45b32212a7bac0927d4798f2f48c0dd425d7236`
- 45217: `a44ef0be9e10a842008ffc98da9fdbf3912e5acee8568bda33ba974a31455da8`

The dependency-light suite passes 752 tests with zero failures or errors and
two skips. The full ROS 2/colcon suite passes 634 tests with zero failures,
errors, or skips.

## Consequences

The project no longer treats repeated confirmation of the same biased depth
surface as evidence of precision. This removes five false-positive behavior
candidates and strengthens the zero unsafe-motion requirement, but it also
leaves zero multi-fruit development scenes eligible. This is expected: a
safety correction must expose missing measurement quality rather than hide it.

The next mainline task is to improve the fixed base camera's far-workspace
measurement quality across a development batch. The current base stream is
320×240 and far fruit boxes are typically only 12–15 pixels wide. Resolution,
depth support, and global hardware layout should be compared as fixed
cross-seed configurations; no per-seed pose tuning or threshold relaxation is
allowed. Because behavior changed after batch 45201–45218, the next fresh
discovery block is 45301–45318.
