# ADR 0025: Authorize consensus training-label derivative and one clean-label retraining

- Status: Accepted
- Date: 2026-07-17
- Scope: P2/T30 recovery only

## Context

ADR 0024 authorized a read-only screen of the frozen 501-image training split.
The resulting packet contains 34 candidates and does not access validation or
test data during screening. The user completed the second review and explicitly
authorized the proposed corrections and continued project work.

After correcting nine semantic no-op entries to `MODEL_ERROR_KEEP_LABELS`, the
two independent review files agree on 18 candidates and disagree on 16. The
frozen conservative policy maps every disagreement to `AMBIGUOUS_EXCLUDE`.
The 18 agreements contain 13 actual label changes, four keep-label decisions,
and one ambiguous exclusion. The 13 changes comprise five class relabels and
eight missing-box additions. No source label has changed yet.

The accepted changes alter the training-instance totals from 1,364 ripe and
510 unripe to 1,371 ripe and 511 unripe. This is a small, review-driven
correction, not class rebalancing and not a new evaluation set.

## Decision

1. Materialize one versioned derivative containing exactly the 501 original
   training images, consensus-corrected training labels, and the existing
   116-image audited validation derivative. It must contain no test split.
2. Apply only the 13 double-review consensus changes. Preserve all 16 reviewer
   disagreements, the agreed ambiguous item, and all keep-label items without
   alteration. Never modify the original dataset in place.
3. Bind every relabel to the opposite-class source box at IoU at least 0.50.
   Bind additions to the reviewed normalized box. Record before/after label
   lines, hashes, counts, and a canonical derivative digest.
4. Authorize one training claim named `yolo11s_640_train_audit_v1`, initialized
   from the pinned upstream `yolo11s.pt`. Relative to the historical baseline,
   only the dataset path and run name may change. Keep 640 px, 200 epochs,
   batch 8, seed `20260710`, deterministic mode, AMP disabled, patience 50,
   and the existing augmentation defaults.
5. Evaluate every preserved checkpoint on the audited validation derivative
   only, using raw confidence 0.001, NMS IoU 0.70, IoU 0.50, and the frozen
   0.05--0.95 threshold grid. Select by macro-F1, then minimum class F1, then
   threshold, with the existing deterministic tie-breaks.
6. Promotion requires macro-F1 at least 0.85, ripe F1 at least
   0.8843089431, and unripe F1 at least 0.7573271889. The selected model remains
   Shadow-only unless all checks pass.
7. This is a single-use claim. If it fails, do not retry with new
   hyperparameters, reopen disagreements based on metrics, or access the test
   split without another architecture decision.
8. The held-out test remains sealed. Training-label corrections are not
   independent evaluation evidence, and no real-world or sim-to-real
   generalization claim is authorized.

## Consequences

- The intervention directly tests whether consensus-confirmed training-label
  defects explain the remaining unripe-recall gap.
- Model-screened training corrections can reinforce model bias; the independent
  second review and conservative disagreement rule limit but do not remove that
  risk.
- Passing audited validation makes the candidate eligible for a separate
  formal-test authorization decision; it does not itself authorize the test.
- Failure leaves T30/P2 blocked while preserving all accepted T40 and
  Oracle-only T50/T60 engineering evidence.
