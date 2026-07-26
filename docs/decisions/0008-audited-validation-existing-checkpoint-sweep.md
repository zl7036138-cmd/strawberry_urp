# ADR 0008: Audited-validation sweep of existing checkpoints

- Status: Accepted
- Date: 2026-07-15
- Scope: T30 validation-only checkpoint selection

## Context

The user-approved validation derivative adds 5 ripe and 8 unripe boxes while
preserving all original labels and the held-out test seal. With the historical
YOLO11s `best.pt` predictions, macro-F1 increases from 0.787121 to 0.816939;
ripe/unripe F1 becomes 0.896552/0.737327. The optimal threshold remains 0.31,
so label correction alone does not pass the frozen 0.85 T30 gate.

The two already completed training runs retain periodic checkpoints. Starting
another training intervention before checking these immutable checkpoints would
spend GPU time and expand the optimization search unnecessarily. Reselecting a
checkpoint on corrected validation labels is a normal consequence of changing
validation ground truth, provided the candidate set and selection rule are
frozen before inference.

Bound audited evidence:

- derivative manifest SHA-256:
  `f2760fbf2836d0fe2c75f789da42aaf2c5218309e3da26782ddcf1a863eec7b3`;
- label-only metric report SHA-256:
  `a7cb961d5e1e786087422ab0e2b079634b1179d8ad5dd4fdf5f77570243efeca`;
- canonical validation derivative SHA-256:
  `f4e2b735ff50ef42fa5bd7d4fd4b8552d390403327c1c8dd3968566bcd45e7bf`.

## Decision

- Do not start a new training run yet.
- Evaluate exactly the 26 pre-existing weights in the historical
  `yolo11s_640` and `yolo11s_640_cls_pw05_opt1` directories: `best.pt`,
  `last.pt`, and `epoch0.pt` through `epoch100.pt` at 10-epoch intervals for
  each family. Missing, extra, changed, or duplicate candidates fail closed.
- Hash and inventory all candidates before the first model is evaluated.
- Use only the 116 audited validation labels and their original registered
  validation images. Do not open test images or labels.
- Keep inference fixed at 640 px, confidence floor 0.001, NMS IoU 0.70, batch
  8, and device 0. For each checkpoint, select confidence from the frozen
  0.05--0.95 grid in 0.01 steps with IoU 0.50.
- Rank candidates by highest macro-F1, then highest minimum per-class F1, then
  highest selected confidence threshold. Remaining ties prefer the historical
  baseline family and finally the lexicographically smallest checkpoint ID.
- Preserve predictions and a metric receipt for every candidate. Do not delete
  unfavorable results.
- A checkpoint reaching macro-F1 0.85 is only validation-eligible. This sweep
  does not itself authorize the held-out test or perception-controlled motion.
- If no checkpoint reaches 0.85, report the shortfall before authorizing any
  new training or label intervention.

## Consequences

The sweep is a bounded reuse of existing work, not another hyperparameter
search or training claim. Because the audited labels were model-screened, its
metrics remain validation evidence rather than an independent generalization
result. All eight conservatively excluded disagreements remain untouched.

## Outcome

The frozen inventory contained all 26 required, hash-distinct checkpoints. The
sweep completed without accessing the held-out test or starting a training
run. `baseline__best` remained the winner at threshold 0.31:

- macro-F1: `0.8158180660` (required: `0.85`);
- ripe F1: `0.8943089431`;
- unripe F1: `0.7373271889`.

The gate therefore failed by `0.0341819340`. The inventory and summary hashes
are `2219ac89e2266afc74c3b032bd5f3831288ac8b87e0dbae29c590d8c180d226a`
and `7fd8447d0afee0eb3bc68617277002196a4e14bee1df57cdac6c9e2ac1efe274`.

Follow-on error attribution found 30 unripe false negatives: 15
`LOW_CONFIDENCE`, 9 `NO_DETECTION`, 5 `WRONG_CLASS`, and 1 `LOCALIZATION`.
This rejects further threshold/checkpoint tuning as the primary remedy and
identifies unripe confidence/recall as the next training decision target. T30
remains unaccepted; training, formal-test access, and perception-controlled
motion remain unauthorized.
