# ADR 0038: Reject the Blender-v2 unripe-adaptation candidate

- Recorded: 2026-07-26
- Status: Accepted
- Scope: Outcome of the ADR-0037 non-formal research claim

## Evidence

The frozen capture completed with 72 training and 24 synthetic held-out images.
It retained the Blender-v2 plant, used a 26 mm truth-projection radius, covered
three lighting levels, and produced disjoint train/held-out image hashes.
Representative ripe and unripe samples passed visual review. No arm motion,
perception control, formal matrix, or held-out real-test access occurred.

The one authorized training claim completed all 12 epochs in 193.81 seconds.
The selected `last.pt` is 19,160,026 bytes with SHA-256
`6e847b548d186c052e56d358328d8d286d985c43c32eb0914d7b26b86217afb4`.
Selection and evaluation kept the frozen confidence threshold `0.58`.

`artifacts/perception/optimization/yolo11s_640_blender_v2_research_v1_synthetic_heldout_v1/summary.json`
records:

| Metric | Required | Actual | Passed |
|---|---:|---:|---|
| Synthetic ripe F1 | 0.85 | 0.857143 | Yes |
| Synthetic unripe F1 | 0.85 | 0.000000 | No |

At raw confidence `0.001`, only three of the 12 unripe images contain any
correct-class prediction overlapping the truth box at IoU 0.50, and their
maximum confidence is `0.002541`. Four images instead contain an overlapping
RIPE prediction, also at very low confidence. The failure is therefore not
merely a `0.58` threshold-transfer effect; the fixed fine-tuning did not learn
a stable unripe representation at the rendered scale.

## Decision

Reject `yolo11s_640_blender_v2_research_v1`. Do not replace the active
engineering checkpoint, do not use the candidate for runtime Shadow, and do
not start audited real-validation, live no-motion qualification, control-side
handoff, or manipulation tests. Those downstream steps cannot reverse the
failed first promotion gate.

The single ADR-0037 training claim is consumed. Do not switch to `best.pt`,
lower the confidence threshold, add epochs, change sampling, or rerun training
under this decision.

## Interpretation and next research direction

The conservative mix added only 36 unripe Blender-v2 instances to a 573-image
training set whose total instance counts remain 1,407 ripe and 547 unripe.
The rendered unripe target is also small (roughly 22--25 pixels) and visually
similar to plant leaves. A future, separately authorized research phase should
first choose one explicit hypothesis rather than combining fixes:

1. increase target information through closer camera crops or higher effective
   object scale while preserving the real plant background; or
2. use a frozen class-aware sampling or loss strategy with a new real
   non-regression budget.

Neither option is authorized here. The current working system remains the
geometry-corrected Blender-v2 scene with the established base-plus-wrist
observation path and the prior engineering checkpoint.

## Boundaries

The audited real validation was not accessed for this candidate. The sealed
real held-out test remains unconsumed. No formal acceptance, real-world
generalization, physical-robot behavior, or sim-to-real claim is supported.
