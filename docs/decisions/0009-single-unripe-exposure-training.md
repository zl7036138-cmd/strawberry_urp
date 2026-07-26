# ADR 0009: Single unripe-exposure training intervention

- Status: Accepted and user-authorized
- Date: 2026-07-15
- Scope: T30 validation-only model improvement

## Context

The approved validation-label derivative and ADR 0008 checkpoint sweep leave
the best existing checkpoint at macro-F1 0.815818, below the frozen 0.85 gate.
Its ripe/unripe F1 values are 0.894309/0.737327. Of 30 unripe false negatives,
15 are low-confidence, 9 have no detection, 5 are classified ripe, and only 1
is a localization failure. The previous `cls_pw=0.5` loss-weight intervention
regressed all principal metrics, so another loss-weight or threshold search is
not justified.

The original 501-image training split contains 1364 ripe and 510 unripe boxes.
Exactly 188 images contain at least one unripe box; 186 of those are mixed-class
and 2 contain only unripe fruit. A bounded image-exposure intervention can
increase the effective unripe signal while retaining normal augmentation and
without inventing labels or accessing the test split.

The user authorized this intervention with the exact message: “可以，按你的计划执行”.

## Decision

- Run exactly one new variant, `yolo11s_640_unripe_x2_opt2`.
- Materialize a separate training derivative. Include every one of the 501
  registered training images once, then include each of the 188 images whose
  registered label contains class 1 exactly one additional time. Use distinct
  derivative filenames; duplicate content remains hash-bound to its source.
- The effective training set is therefore 689 image entries. Effective box
  exposure is 1971 ripe and 1020 unripe, changing the ratio from 2.6745:1 to
  1.9324:1. No image is selected from model predictions.
- Use the 116 original registered validation images with only the approved
  audited validation labels. The derivative contains no test directory and its
  dataset YAML contains no test key.
- Initialize from the pinned upstream `weights/yolo11s.pt`, not from a prior
  trained checkpoint. Relative to the historical baseline, keep model, seed,
  input size, epochs, batch, device, workers, determinism, AMP, patience,
  checkpoint schedule, cache, and augmentations unchanged. Only the registered
  data derivative and run name differ.
- Preserve every generated `best.pt`, `last.pt`, and `epochN.pt` checkpoint.
  Evaluate all of them on the audited validation set at 640 px, raw confidence
  0.001, NMS IoU 0.70, batch 8, and device 0. Select confidence on the frozen
  0.05--0.95 grid at metric IoU 0.50.
- Select the checkpoint by maximum macro-F1, then maximum minimum class F1,
  then maximum confidence threshold, then prefer `best.pt`, then lexical ID.
- Validation promotion requires all three: macro-F1 at least 0.85, ripe F1 at
  least 0.884309 (no more than 0.01 below the audited baseline), and unripe F1
  at least 0.757327 (at least 0.02 above the audited baseline).
- The machine-readable contract is
  `tools/perception/audited_opt2_contract.json`. Training must fail closed on
  any path, count, content hash, configuration, authorization, or output-path
  mismatch.
- Do not read a test image or label and do not create the held-out-test receipt.
  Even if validation passes, this ADR produces validation eligibility only;
  formal-test execution requires a separately bound contract update.

## Consequences

This is one data-exposure intervention, not a hyperparameter sweep. Repeated
images receive independent stochastic augmentations during training, but no
new semantic annotation is claimed. A failed run or failed validation gate is
preserved as evidence and cannot be silently retried under another name or
ratio. The original training and validation labels remain immutable.
