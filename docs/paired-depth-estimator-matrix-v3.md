# Paired RGB-D depth-estimator matrix v3

## Purpose

This development matrix compares `center_median` and `geometry_layer` on the
same stored depth pixels. It removes simulator timing, detector variation, TF
timing, and motion as comparison confounders. It is not a formal P3/P4 rerun
and does not authorize robot motion.

## Inputs and conditions

Two prior synchronized wrist-camera captures contribute 20 frames each. Every
sample contains the detected box, depth image, camera intrinsics, camera-to-base
transform, target truth, and acquisition stamp. The evaluator removes RGB and
stores only a 15% margin around each detection in a compact 40-sample bundle.

The bundle SHA-256 is
`bcc45a26e47f182192f7de3900d9f7f2d533f81aea79471e518d76a3feb524bf`.
Both estimators receive identical arrays in all six conditions:

1. the observed detection box;
2. the same box shrunk by 10%;
3. the same box expanded by 10%;
4. a synthetic foreground band covering the central 25%;
5. a synthetic foreground band covering the central 45%; and
6. complete replacement of the fruit depth inside the box.

The box perturbations measure sensitivity to detector under- and over-coverage.
The foreground bands are deterministic depth interventions, not rendered leaf
occlusion and not new perception evidence.

## Preserved failure and repair

The first frozen run failed. With a 10%-smaller box, `geometry_layer` rejected
all 40 samples as ambiguous. In those frames, the selected fruit layer had
approximately 15,500--15,900 pixels while the competing layer had only
876--975 pixels. The v1 ambiguity rule compared geometric residuals but ignored
this strong support difference. The failed receipt SHA-256 is
`ea2c247666478ea3e3ce16e62cd2ed62ca8de7a7804030c3751ad6bc29201069`.

The repair adds `geometry_ambiguity_min_support_ratio: 0.50`. Two candidates
are now ambiguous only when they are close in geometric residual and the
runner-up contains at least half as many pixels as the selected layer. The
equal-support ambiguity test remains rejected.

## v3 result

| Condition | Centre median | Geometry layer |
|---|---:|---:|
| Observed box, median error | `4.889 mm` | `4.361 mm` |
| Observed box, P95 error | `4.927 mm` | `4.463 mm` |
| Box shrunk 10%, accepted | `40/40` | `40/40` |
| Box shrunk 10%, P95 error | `4.983 mm` | `4.206 mm` |
| Box expanded 10%, P95 error | `4.858 mm` | `4.850 mm` |
| Centre occlusion 25%, P95 error | `47.116 mm` | `4.850 mm` |
| Centre occlusion 45%, P95 error | `47.116 mm` | `5.886 mm` |
| Fruit depth absent | `40/40` false estimates | `40/40` rejected |

All eight diagnostic checks pass. A second invocation using only the compact
bundle produced a byte-identical receipt. The v3 receipt SHA-256 is
`bc8a4526b5a21bb3c62ad548212eb6d1ef8e24b8f2621c9eb3f98448915e08d4`.
The original four-case synthetic diagnostic also passes unchanged against the
repaired core; its v2 receipt SHA-256 is
`5ac3f21c3c3c324f4e6fc36ff61110900031b5e477060581d30f0393b4032726`.
The complete repository regression then built all seven ROS 2 packages and
passed 410/410 tests with zero errors, failures, or skips.

Reproduce it with:

```bash
python3 scripts/evaluate_paired_rgbd_depth_estimators.py \
  --bundle results/development/paired_rgbd_depth_estimator_matrix_v1/depth_bundle.npz \
  --output /tmp/paired_rgbd_depth_estimator_receipt.json
```

## Promotion boundary

The diagnostic status is `PASS`, but runtime promotion remains `BLOCKED`:

- the two captures are only slight variations of one requested wrist pose;
- the controlled occluders are injected depth bands rather than rendered
  leaves or the P4 benchmark occluder; and
- geometry-layer P95 sigma is `24.254 mm` on observed boxes,
  `44.316 mm` on 10%-smaller boxes, and above the 15 mm planning limit in both
  central-occlusion conditions.

The next gate must add at least five meaningfully different camera/target
viewpoints without grasp motion, followed by rendered partial/heavy occlusion.
Changing the sigma definition or enabling manipulation before those results
would overstate the evidence.
