# ADR 0025 outcome: Consensus training-label retraining rejected

- Recorded: 2026-07-17
- Frozen decision: `0025-authorize-consensus-train-label-retraining.md`
- Outcome: Completed, validation rejected

The two-reviewer resolution retained 18 agreements and conservatively excluded
all 16 disagreements. A versioned derivative applied exactly 13 consensus
changes to copies of the training labels: five relabels and eight additions.
The original dataset remained unchanged, the 116-image audited validation
derivative was reused read-only, and no test split was present. The corrected
training counts are 1,371 ripe and 511 unripe instances.

The single authorized training claim stopped normally after 115 epochs and
1,456.47 seconds. It preserved 14 checkpoints: `best.pt`, `last.pt`, and
`epoch0.pt` through `epoch110.pt` at ten-epoch intervals. All 14 were evaluated
on audited validation with the frozen threshold grid.

`best.pt` is the selected checkpoint at confidence 0.58:

- macro-F1: `0.8006746847` (required `0.85`, failed);
- ripe F1: `0.8870636550` (required `0.8843089431`, passed);
- unripe F1: `0.7142857143` (required `0.7573271889`, failed).

Relative to the accepted existing-checkpoint baseline, macro/ripe/unripe F1
change by `-0.0151433814/-0.0072452881/-0.0230414747`. The selected result has
216/26/29 ripe TP/FP/FN and 70/16/40 unripe TP/FP/FN.

Unripe false negatives comprise 21 low-confidence detections, 13 absent
detections, five wrong-class detections, and one localization failure. Ripe
false negatives comprise 23 low-confidence detections, two absent detections,
one wrong-class detection, and three localization failures. The intervention
therefore does not support training-label quality as the primary remaining
cause of the T30 gap.

The candidate is rejected and remains Shadow-only. T30/P2 and full P3 remain
unaccepted. The held-out test was not accessed, no formal test score exists,
and no retry or new training intervention is authorized by this outcome.
