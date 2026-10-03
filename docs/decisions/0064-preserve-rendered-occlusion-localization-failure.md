# ADR 0064: Preserve the rendered-occlusion localization failure

- Recorded: 2026-08-05
- Status: Accepted; diagnostic failed and runtime promotion blocked
- Scope: Non-formal simulator localization evidence, no robot motion

## Context

ADR 0063 authorized exactly one 5-position by 3-occlusion rendered matrix.
Pre-execution checks repaired infrastructure and froze all hashes before the
full run. The resulting execution completed 15/15 worlds and 300/300 paired
frames with no infrastructure or safety violation.

The frozen summarizer returned `FAIL`.

## Decision

1. Preserve the full result and do not rerun, retune, or weaken the v1
   thresholds.
2. Keep `geometry_layer` disconnected from planning, manipulation, and
   control. Its runtime promotion status remains `BLOCKED`.
3. Record both failed checks without waivers:
   - the near-left no-occluder ROI has 0.153846 blue coverage despite an
     `occluder: null` receipt, so the colour-only condition binding fails;
   - the far-left clear geometry path accepts 18/20 rather than the required
     19/20 frames because its first two counted frames have no pose.
4. Retain the positive accuracy finding only as diagnostic evidence. Geometry
   P95 errors pass at 15.432, 18.755, and 26.962 mm for none, partial, and
   heavy. Heavy legacy P95 is 361.727 mm.
5. Do not reinterpret low legacy sigma under heavy occlusion as safety: it is
   confidently wrong by hundreds of millimetres.
6. Require a new v2 protocol before any additional rendered claim. It must
   bind occlusion by instance or paired-render difference and verify that both
   localizer sensor caches are warm before recording.
7. Preserve the separate remaining gates for detector boxes, natural leaves,
   field-v3 wrist viewpoints, uncertainty calibration, and motion.

## Consequences

The project now has multi-position rendered evidence that geometry-layer
localization substantially reduces heavy-occlusion error, but it does not have
a passing rendered-occlusion diagnostic or authority to use the estimator for
pre-grasp control. The failure also identifies two protocol defects that can be
fixed without changing the estimator or consuming formal P3/P4 evidence.

The authoritative result is documented in
`docs/rendered-occlusion-localization-matrix-v1.md` and bound by summary SHA-256
`395e80cc76da74cc75deb78f7dd96dfcf702f75aadfec29c46601e15157c8c68`.
