# ADR 0009 outcome: Unripe-exposure intervention rejected

- Recorded: 2026-07-15
- Frozen decision: `0009-single-unripe-exposure-training.md` (left immutable)
- Outcome: Completed, validation rejected

The authorized run completed 78 epochs in 1402.94 seconds and stopped normally
after 50 epochs without Ultralytics fitness improvement. Its training-time best
was epoch 28. The completed claim preserves 10 weights: `best.pt`, `last.pt`,
and `epoch0.pt` through `epoch70.pt` at 10-epoch intervals.

All 10 weights were evaluated under the frozen audited-validation contract.
`best.pt` was also the macro-F1 winner, at confidence 0.54:

- macro-F1: `0.7956957611` (required `0.85`);
- ripe F1: `0.8861283644` (required `0.8843089431`, passed);
- unripe F1: `0.7052631579` (required `0.7573271889`, failed).

Relative to the audited existing-checkpoint baseline, macro-F1 changes by
`-0.0201223049`, ripe F1 by `-0.0081805787`, and unripe F1 by
`-0.0320640310`. The selected result contains 67 unripe true positives, 13
false positives, and 43 false negatives.

Error attribution shows the unripe false negatives are 29 low-confidence, 9
no-detection, 4 wrong-class, and 1 localization. The independently optimal
unripe threshold is 0.43 but yields only F1 `0.707547`, so the failure is not
caused solely by the shared 0.54 threshold. Repeating unripe-containing images
made the detector more conservative and did not solve unripe recall.

The candidate is not promoted. T30/P2 remain unaccepted. No test image or label
was accessed, the held-out-test receipt remains absent, and neither additional
training nor formal-test execution is authorized by this outcome.
