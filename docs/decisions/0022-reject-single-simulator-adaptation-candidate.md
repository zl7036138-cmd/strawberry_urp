# ADR 0022: Reject the single simulator-adaptation candidate

- Status: Accepted
- Date: 2026-07-16

## Context

ADR 0021 froze one 30-epoch claim from `baseline__best`, using 501 real and
216 synthetic training images. Only the 72 synthetic held-out images could
select a checkpoint and confidence threshold. The resulting selected candidate
then had to pass the audited real-validation non-regression screen before any
D2, negative-scene, motion, or formal evaluation could support promotion.

## Outcome

The one-time claim completed successfully in 507.14 seconds and preserved
eight contracted checkpoints. `best.pt` has SHA-256
`2806968b8b4157169302948a75c99427de41b287b9c77dd2cfd4a436782c7e17`.

Synthetic held-out selection evaluated all eight checkpoints. `best.pt` won at
confidence 0.80 with macro-F1 1.000, so the synthetic gate passed. The same
checkpoint and threshold were then applied once to all 116 audited real
validation images without threshold search. The result failed every
non-regression requirement:

| Metric | Required | Actual | Passed |
|---|---:|---:|---|
| Macro-F1 | 0.795818 | 0.625537 | No |
| RIPE F1 | 0.874309 | 0.754430 | No |
| UNRIPE F1 | 0.717327 | 0.496644 | No |

An offline post-rejection sweep of the already saved predictions is diagnostic
only and has no decision effect. Its best real threshold is 0.47, where
macro-F1 is 0.800035 and RIPE F1 is 0.896552, but UNRIPE F1 is still only
0.703518 and remains below the 0.717327 minimum. Thus the failure combines a
large cross-domain threshold-transfer penalty with a smaller underlying
UNRIPE regression in the checkpoint itself.

## Decision

Reject `yolo11s_640_sim_adapt_v1`. Do not promote it to runtime Shadow, do not
authorize perception-controlled motion, and do not use it for T30, P2, P3, or
P4 acceptance. The single training claim is consumed. No retry, parameter
search, threshold relaxation, alternate real-selected checkpoint, or second
synthetic training run is authorized.

Apply the frozen fail-fast rule: because the audited real non-regression screen
already failed, do not run the remaining D2 multiposition or simulator-negative
promotion screens. They cannot reverse the aggregate decision and are recorded
as not run, not as passes or failures. The formal 135+30 matrix and the real
held-out test remain unconsumed.

## Consequences

Oracle remains the only permitted arm-control source and the historical
`baseline__best` remains the runtime Shadow reference. T30, P2, full P3, T70,
and P4 remain unaccepted. Any new intervention—such as changing simulator
visual assets without further training—requires a separate architecture
decision and cannot silently reuse or reset this consumed claim.
