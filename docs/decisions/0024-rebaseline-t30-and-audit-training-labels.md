# ADR 0024: Rebaseline T30 and audit training labels before another model intervention

- Status: Accepted
- Date: 2026-07-17
- Scope: P2/T30 recovery only

## Context

The project has progressed beyond the accepted P2 boundary through Oracle-only
integration and several T70 diagnostics, but T30 remains the controlling stage
gate.  The selected audited-validation checkpoint is still `baseline__best` at
confidence 0.31, with macro/ripe/unripe F1
`0.815818/0.894309/0.737327`.  The formal requirement remains macro-F1
`>= 0.85`; the held-out test remains sealed.

The validation audit added 13 human-approved boxes and materially raised the
measured macro-F1.  All 26 existing checkpoints have already been swept.
Independent per-class threshold selection raises macro-F1 only to `0.817223`.
The single unripe-image duplication intervention regressed to `0.795696`, and
the completed simulator-mixed candidate regressed real validation to
`0.625537`.  Only 8 of the 30 selected-baseline unripe false negatives have a
model-input minimum side below 32 pixels; low-confidence misses have a target
scale similar to true positives.  These results do not support more threshold
tuning, blind retraining, simulator work, or input resolution as the next
primary action.

The immutable quantitative rebaseline is
`artifacts/perception/rebaseline/t30_validation_rebaseline_v1.json`.

## Decision

1. Return the project to the P2/T30 critical path.  Pause new T70 feature,
   scene-realism, robustness, and perception-controlled-motion work.
2. Retain `outputs/perception/yolo11s_640/weights/best.pt` only as the audit
   screening model.  It remains a below-gate Shadow model and is not promoted.
3. Run one read-only label-quality screen over exactly the registered 501-image
   training split.  Do not traverse the validation or test split during this
   new inference run.
4. Freeze inference at image size 640, raw confidence 0.001, NMS IoU 0.70,
   batch 8, device 0, and the selected-baseline weight SHA-256
   `ce6c998ed52ae97c8e8b51880adb60618733fff11be22378baa0044f9a126425`.
5. Generate human-review candidates from two deterministic screens:
   - a prediction at confidence `>= 0.75` whose maximum IoU with any current
     training annotation is `< 0.10`;
   - a prediction at confidence `>= 0.50` whose IoU with the opposite-class
     annotation is `>= 0.50` and whose same-class IoU is `< 0.50`.
6. Rank cross-class candidates before unmatched candidates, then by descending
   confidence and stable image/box order.  Limit the first packet to 60 items.
   The pre-limit count and exclusion count must remain visible.
7. Treat every model box as a review aid, not ground truth.  No label may change
   until the user reviews the packet.  Ambiguity or reviewer disagreement is
   conservatively excluded.
8. This decision authorizes prediction export and packet generation only.  It
   does not authorize a training derivative, a new training claim, threshold
   promotion, held-out test access, perception control, or formal simulator
   trials.  Any confirmed training-label changes and one subsequent model
   intervention require a new decision.

## Consequences

- The next work item tests the remaining data-quality hypothesis without
  spending another training claim.
- A packet with no credible label defects would reject annotation quality as
  the primary remedy and force a separate capacity/augmentation decision.
- A packet with credible defects still does not guarantee a higher F1; it only
  supports preparing a versioned training-label derivative after explicit
  review.
- Prior T60/T70 results remain preserved as engineering evidence but do not
  advance the active acceptance gate.
