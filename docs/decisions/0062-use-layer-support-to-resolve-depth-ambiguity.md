# ADR 0062: Use depth-layer support to resolve near ties

- Recorded: 2026-08-05
- Status: Accepted for opt-in Shadow development
- Scope: Paired recorded-depth evaluation; no ROS runtime or robot motion

## Context

ADR 0061 introduced an opt-in geometry-layer estimator and kept runtime
promotion blocked. Its first identical-frame matrix used 40 synchronized
wrist RGB-D observations from two prior captures. The matrix perturbed each
detection box by plus or minus 10% and injected deterministic centre
occluders.

The first run failed only when boxes were shrunk by 10%. All 40 geometry-layer
observations were rejected as ambiguous even though the likely fruit layer had
roughly 16--18 times the pixel support of the competing layer. The two layers'
geometric residuals differed by less than the existing 10 mm ambiguity margin,
so the previous rule discarded the support evidence.

## Decision

1. Preserve the failed matrix receipt without changing its result.
2. Add `geometry_ambiguity_min_support_ratio`, defaulting to `0.50` in the
   opt-in estimator and development configuration.
3. Reject a near tie only when the runner-up layer contains at least half as
   many pixels as the selected layer. A low-support background sliver no longer
   vetoes a strongly dominant fruit layer.
4. Keep the existing geometric tolerance, layer gap, minimum support, and
   ambiguity distance unchanged.
5. Preserve the equal-support two-layer rejection test and add a dominant-layer
   regression test.
6. Rerun only the non-formal, offline paired matrix on the exact same bundle.
   Do not alter frozen P3/P4 evidence, consume the sealed real test, or enable
   control.

## Evidence

The compact 40-sample input bundle has SHA-256
`bcc45a26e47f182192f7de3900d9f7f2d533f81aea79471e518d76a3feb524bf`.
The preserved failing receipt has SHA-256
`ea2c247666478ea3e3ce16e62cd2ed62ca8de7a7804030c3751ad6bc29201069`.

After the support-aware change, all eight diagnostic checks pass. The repaired
matrix accepts all 40 smaller-box observations at 4.206 mm P95 error, recovers
the fruit layer in all 80 centre-occlusion observations, and rejects all 40
fruit-absent observations. A clean second invocation produced the identical
receipt SHA-256
`bc8a4526b5a21bb3c62ad548212eb6d1ef8e24b8f2621c9eb3f98448915e08d4`.
The four-case synthetic regression remains `PASS`; its repaired-core receipt
SHA-256 is
`5ac3f21c3c3c324f4e6fc36ff61110900031b5e477060581d30f0393b4032726`.
All seven ROS 2 packages subsequently built and all 410 colcon tests passed
with zero errors, failures, or skips.

## Consequences

Detection-box under-coverage no longer creates a false ambiguity when the
selected layer has overwhelming support. True near-equal depth-layer cases
still fail closed.

This does not promote the estimator. The input represents only two slightly
different executions of one requested wrist pose, and its occlusions are
offline depth interventions. Reported P95 sigma remains above the 15 mm
planning limit in several conditions. At least five meaningfully different
viewpoints and rendered occlusion are required before a new promotion decision.
