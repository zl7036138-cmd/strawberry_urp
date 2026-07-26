# ADR 0021: Freeze one simulator-adaptation training claim

- Status: Accepted
- Date: 2026-07-16

## Context

ADR 0020 accepts a leakage-safe synthetic capture with 216 training and 72
synthetic held-out images. The historical `baseline__best` detector remains
below the real-image T30 gate and fails the simulator unripe/heavy-occlusion
diagnostics. ADR 0019 permits at most one simulator-specific fine-tuning claim
after the capture and all hyperparameters are frozen.

Using the audited real validation images during early stopping or checkpoint
selection would invalidate their non-regression role. Reusing the Opt2
oversampled derivative would also change two factors at once. The formal real
test and the formal 135+30 simulator matrix remain out of scope.

## Decision

Authorize exactly one run named `yolo11s_640_sim_adapt_v1` under the frozen
contract `tools/perception/sim_adaptation_v1_contract.json`.

- Initialize from the hash-bound real-image `baseline__best` checkpoint, not
  the upstream `yolo11s.pt` asset and not the rejected Opt2 checkpoint.
- Train on exactly 501 original registered real training images plus 216
  synthetic training images. Do not duplicate or oversample either source.
- Use exactly 72 disjoint synthetic held-out images as Ultralytics validation.
  Do not expose the 116 audited real validation images to training, early
  stopping, checkpoint selection, or threshold selection.
- Fine-tune for at most 30 epochs at 640 pixels, batch 8, SGD learning rate
  0.001, deterministic seed `20260603`, AMP disabled, and the complete frozen
  augmentation/configuration values in the training YAML.
- Preserve `best.pt`, `last.pt`, and every configured five-epoch checkpoint.
  Select one candidate and one confidence threshold only from synthetic
  held-out macro-F1, using the predeclared tie breaks in the contract.
- Apply that same selected checkpoint and threshold to the audited real
  validation non-regression screen. Do not retune after seeing those results.

Promotion to a simulator-specific Shadow candidate requires every ADR 0019
screen: synthetic held-out macro-F1 at least 0.85; D2 heavy ripe-frame rate at
least 0.60 overall and 0.50 at each position; heavy target-pose rate at least
0.50; clear/dim ripe rates at least 0.95 and target-pose rates at least 0.90;
false-ripe negative rate at most 0.05; unripe observation rate at least 0.50;
and audited real-validation macro, ripe, and unripe F1 each no more than 0.02
below the baseline.

## Consequences

The exact user authorization receipt, materialized mixed-data manifest, config,
base checkpoint, runner, and one-time claim must all hash-validate before the
GPU process starts. Acquiring the claim consumes the only authorized attempt
even if training or a promotion screen fails; a failure triggers analysis, not
a retry or parameter relaxation.

This intervention may only produce a simulator-specific Shadow candidate. It
cannot close T30 or P2, cannot authorize perception-controlled motion, cannot
consume the formal real test, and cannot start or score the formal simulator
matrix. No sim-to-real generalization claim is permitted.
