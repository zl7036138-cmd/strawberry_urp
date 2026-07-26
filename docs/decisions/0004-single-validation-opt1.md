# ADR-0004: Single validation-only T30 optimization

- Status: Accepted
- Date: 2026-07-14

## Context

The historical `yolo11s_640` run completed without opening the held-out test.
On the unchanged 116-image validation split, threshold 0.31 maximized the fixed
metric at macro-F1 0.7871205585. Ripe F1 was 0.8852459016; unripe F1 was
0.6889952153. Training labels contain 1,364 ripe and 510 unripe boxes, and
validation attribution identifies unripe classification/detection as the main
controlled bottleneck. The exact former experiment contract is archived with
SHA-256 `529dc43bddec4a1ba8061020aedb31640184552387ae78441f21c934a4a25f5e`.

## Decision

Run exactly one candidate, `yolo11s_640_cls_pw05_opt1`. Relative to the
historical primary, only the output name and Ultralytics 8.4.92 `cls_pw` change;
`cls_pw` moves from its 0.0 default to 0.5. Data, split, model, seed, input size,
schedule, patience, augmentation, and all other settings remain fixed.

The candidate is promoted as an improvement only if the same validation split
meets all of these conditions:

- unripe F1 >= 0.7090;
- macro-F1 >= 0.7971;
- ripe F1 >= 0.8752.

Promotion is distinct from permission to open the held-out test. Test access
also requires validation macro-F1 >= 0.85. A machine-readable completed
decision may document a failed or partially improved candidate, but only
`authorize_held_out_test: true` unlocks formal inference. The independent test
still uses its frozen validation threshold and must itself reach macro-F1 0.85.

## Consequences

- The historical YOLO11s artifacts remain auditable but are ineligible for the
  new formal contract.
- The Opt1 decision cannot be tuned from test outcomes; the dataset-level test
  receipt remains unconsumed unless validation authorizes it.
- If Opt1 misses validation authorization, T30 reports the metric shortfall
  honestly and no second optimization is introduced under this decision.
- Likely validation-label omissions are recorded as a measurement risk; they do
  not authorize relabeling or another hyperparameter change in Opt1.

## Outcome

Opt1 completed 109 epochs and selected epoch 59. On the unchanged 116-image
validation split, threshold 0.44 maximized the fixed metric at macro-F1
0.7809123649. Ripe F1 was 0.8781512605 and unripe F1 was 0.6836734694. The
candidate-minus-baseline deltas were -0.0062081935, -0.0070946411, and
-0.0053217459, respectively.

The candidate passed only the ripe-F1 floor. It failed the macro-F1 and
unripe-F1 promotion checks and the 0.85 validation authorization threshold.
It was not promoted; formal test inference was not run, the dataset-level test
receipt is absent, and no formal test score exists. T30 and P2 are therefore
not accepted. The immutable result binding is
`artifacts/perception/t30_validation_gate_summary.json`.

The primary measured bottleneck remains unripe detection/recall. A diagnostic
screen also flags 21 possible missing-label candidates across 18 validation
images; this is an annotation-risk signal, not confirmation of omissions and
not authorization for relabeling.
