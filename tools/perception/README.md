# Perception training and evaluation workflow

Run every command from the repository root inside the Ubuntu 24.04 WSL
environment.  The workflow is deliberately fail-closed: it verifies every
materialized image and label, refuses Ultralytics output suffixes, binds
artifacts to a path-independent canonical dataset digest, freezes the
confidence threshold from validation only, and claims the held-out test before
any test image is read.

## Required local artifacts

Place the three pinned files without renaming them:

| File | Repository path | Integrity |
|---|---|---|
| `strawberries.zip` | `data/raw/zenodo_6126677/strawberries.zip` | MD5 `db8d5dcb4b8adebf1621788373fd3031` |
| `yolo11n.pt` | `weights/yolo11n.pt` | SHA-256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |
| `yolo11s.pt` | `weights/yolo11s.pt` | SHA-256 `85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5` |

The URLs and expected byte sizes are pinned in `experiments.json` and
`data/manifests/zenodo_6126677.json`.

## Prepare the dataset

```bash
python tools/data/verify_dataset.py \
  --json-output artifacts/data/zenodo_6126677_verification.json
python tools/data/download_dataset.py \
  --extract-dir data/raw/zenodo_6126677/extracted
```

The second command does not download an already-present archive whose checksum
is valid.  Before splitting, inspect the extracted naming/layout and create an
authoritative `image,group_id` CSV if adjacent frames are not encoded in the
path.  The splitter refuses numeric sequences without this mapping rather than
risk scene leakage.

```bash
python tools/data/split_dataset.py \
  --resplit-all \
  --groups-csv data/manifests/zenodo_6126677_groups.csv \
  --exclusions-csv data/manifests/zenodo_6126677_exclusions.csv \
  --image-mode copy
```

The processed images are isolated from the raw extraction. Missing JPEG EOI
markers are appended deterministically to processed copies so Ultralytics does
not repair files in place during training.

## Completed baseline and single Opt1 intervention

The completed `yolo11s_640` baseline is immutable historical evidence. Its
validation-optimal threshold is `0.31`, with macro-F1 `0.7871`, ripe F1
`0.8852`, and unripe F1 `0.6890`. Because it missed the `0.85` gate, the
held-out test was not opened. The exact pre-Opt1 contract is preserved at
`artifacts/perception/contracts/experiments_yolo11s_640_pre_opt1.json` with
SHA-256 `529dc43bddec4a1ba8061020aedb31640184552387ae78441f21c934a4a25f5e`.
Its receipt, weight, validation, and freeze hashes are recorded in
`artifacts/perception/optimization/yolo11s_640_opt1_decision.json`.

The first authorized training intervention was
`yolo11s_640_cls_pw05_opt1`. Relative to the historical YOLO11s run, it keeps
the same model, data, seed, 640 px input, schedule, and augmentation settings;
it changes only the output name and Ultralytics 8.4.92 `cls_pw` from the `0.0`
default to `0.5`. The run completed 109 epochs and selected epoch 59.

```bash
python tools/perception/run_training.py \
  --variant yolo11s_640_cls_pw05_opt1 --preflight
python tools/perception/run_training.py \
  --variant yolo11s_640_cls_pw05_opt1
```

The job writes only the fixed receipt and weight paths in `experiments.json`.

## Opt1 validation outcome and held-out-test gate

```bash
python tools/perception/run_inference.py \
  --variant yolo11s_640_cls_pw05_opt1 \
  --split val \
  --output artifacts/perception/yolo11s_640_cls_pw05_opt1_validation_predictions.json

python tools/perception/freeze_threshold.py \
  --variant yolo11s_640_cls_pw05_opt1 \
  --validation-bundle artifacts/perception/yolo11s_640_cls_pw05_opt1_validation_predictions.json \
  --output artifacts/perception/yolo11s_640_cls_pw05_opt1_frozen_threshold.json

python tools/perception/evaluate_optimization_promotion.py \
  --candidate-freeze artifacts/perception/yolo11s_640_cls_pw05_opt1_frozen_threshold.json \
  --output artifacts/perception/optimization/yolo11s_640_cls_pw05_opt1_promotion.json
```

The promotion tool reads validation evidence only. `promote_candidate` requires
unripe F1 `>= 0.7090`, macro-F1 `>= 0.7971`, and ripe F1 `>= 0.8752`.
`authorize_held_out_test` additionally requires validation macro-F1 `>= 0.85`.
Exit code `0` means both promotion and test authorization passed; `2` means the
candidate improved but validation macro-F1 remains below `0.85`; `3` means a
relative promotion condition failed. Codes `2` and `3` prohibit the test.

The completed Opt1 selected threshold `0.44` and produced validation macro-F1
`0.780912`, ripe F1 `0.878151`, and unripe F1 `0.683673`. Relative to the
historical baseline, the deltas are `-0.006208`, `-0.007095`, and `-0.005322`.
The promotion artifact therefore records `promote_candidate=false` and
`authorize_held_out_test=false`. Opt1 was not promoted, T30/P2 are not
accepted, and the formal test command below must not be run under this outcome.

Only after exit code `0` may this command run; that condition was not met:

```bash
python tools/perception/run_inference.py \
  --variant yolo11s_640_cls_pw05_opt1 \
  --split test \
  --frozen-threshold artifacts/perception/yolo11s_640_cls_pw05_opt1_frozen_threshold.json \
  --output artifacts/perception/yolo11s_640_cls_pw05_opt1_test_predictions.json
```

The test runner independently verifies `authorize_held_out_test=true`, then
reproduces threshold selection and checks all data, split, config, experiment,
weight, receipt, validation, freeze, and promotion bindings before claiming the
test. Formal validation is also required to contain exactly the 116 manifest
image keys in manifest order; every stored ground-truth box is reloaded from the
current validation label. Immediately before the one-time test claim, the
runner replays all 116 validation images with the current contracted weight,
the pinned validation config, and explicit NMS IoU `0.70`; the complete frozen
threshold selection must be reproduced exactly. The single receipt is fixed at
`artifacts/perception/zenodo_6126677_held_out_test_receipt.json` and remains
consumed after a crash or failed metric. The held-out macro-F1 gate remains
`0.85`; alternate experiment files, weights, receipts, and input sizes cannot
bypass it.

For validation-only diagnostics, `--weights /path/to/local.pt` is allowed.  Its
resolved path, SHA-256, dataset digest, configuration hashes, and environment
are recorded in the prediction bundle.  A historical or non-primary variant can freeze
a diagnostic threshold.  A primary validation bundle created with explicit
`--weights` or a non-contract `--imgsz` is rejected by `freeze_threshold.py` and
cannot produce a formal-test-eligible freeze.

No threshold or weight has been promoted to runtime. If a future, separately
authorized candidate is accepted, copy its selected confidence threshold into
`ros2_ws/src/strawberry_perception/config/perception.yaml` and publish the
accepted weight at the runtime path used by bringup.

## Audited-label Opt2 outcome

ADR 0009 later authorized one separate data-exposure intervention,
`yolo11s_640_unripe_x2_opt2`. Its derivative includes each of the 501 original
training images once and duplicates each of the 188 class-1-containing images
once, for 689 effective entries. It uses the 116 approved audited validation
labels and contains no test split.

The run completed 78 epochs, stopped normally after 50 epochs without fitness
improvement, and preserved 10 weights. A predeclared scan of all 10 selects
`best.pt` at confidence 0.54 with macro-F1 0.795696, ripe F1 0.886128, and
unripe F1 0.705263. This regresses the audited existing-checkpoint baseline
0.815818/0.894309/0.737327, so Opt2 is rejected. The formal test remains sealed
and no further training is authorized by this outcome. See
`artifacts/perception/t30_opt2_outcome_handoff_v1.json`.

## Pure tests

```bash
python -m unittest discover -s tools/perception/tests -p "test_*.py" -v
```

Latest result (2026-07-15): the focused perception workflow/security suite
passes 73/73. The complete dependency-light suite passes 278 tests with one
conditional skip; the ROS/colcon suite passes 171/171.
