# Rendered-occlusion localization matrix v1

## Outcome

The single frozen non-formal execution completed all 15 separately launched
Gazebo scenarios and all 300 paired frames. Infrastructure and the no-motion
safety boundary passed, but the diagnostic status is `FAIL` and runtime
promotion remains `BLOCKED`.

This result must not be rerun or rewritten. It is a post-submission development
study, not formal P3/P4 evidence.

## Frozen design

- Five distinct target positions, each tested with none, partial, and heavy
  rendered visual-only occlusion.
- Twenty frames per scenario, for 300 depth frames and 600 estimator
  observations.
- One truth-projected, unpadded integer bounding box published once per frame.
- `center_median` and `geometry_layer` consume the same stamped depth frame.
- Fixed base camera; perception, manipulation, orchestration, attachment,
  gripper commands, and robot control are disabled.
- Only simulated fruit positioning before measurement is allowed.

The exact design and implementation hashes are in
`config/rendered_occlusion_localization_matrix_v1.json` and ADR 0063.

## Aggregate result

| Occlusion | Geometry accepted | Geometry median error | Geometry P95 error | Geometry P95 sigma | Legacy P95 error |
|---|---:|---:|---:|---:|---:|
| None | `97/100` | `8.457 mm` | `15.432 mm` | `18.048 mm` | `3.973 mm` |
| Partial | `100/100` | `12.066 mm` | `18.755 mm` | `17.534 mm` | `3.615 mm` |
| Heavy | `90/100` | `16.512 mm` | `26.962 mm` | `23.340 mm` | `361.727 mm` |

All three frozen geometry-error checks pass. Under heavy rendered occlusion,
the geometry estimator reduces P95 error by about 92.5% relative to the legacy
centre median. This is useful diagnostic evidence, but it cannot override the
failed acceptance and condition-binding checks.

## Why the diagnostic failed

Two frozen checks failed:

1. `near_left + none` reports blue coverage `0.153846` even though its
   materialization receipt has `occluder: null`. The projected target ROI
   contains scene pixels that satisfy the blue-colour heuristic. Consequently
   the absolute `none <= 0.02` check fails, and heavy-minus-partial coverage is
   only `0.143519`, below `0.15`. This identifies a colour-mask binding defect,
   not an injected-occluder leak.
2. `far_left + none` produces `18/20` geometry poses, below the per-position
   requirement of `19/20`. The first two paired records are `NO_POSE`; the
   remaining 18 are accepted with P95 error `8.428 mm`. Runtime logs show the
   depth/CameraInfo cache subsequently recovering, so the next protocol needs
   a per-localizer warm-up acknowledgement rather than graph discovery alone.

The result remains `FAIL`; these explanations are not post-hoc waivers.

## Promotion blockers

Runtime use for pre-grasp planning remains prohibited because:

- geometry P95 sigma is above the 15 mm planning screen for all three
  aggregate conditions;
- only `79.38%`, `63.00%`, and `57.78%` of accepted none/partial/heavy samples
  respectively report sigma at or below 15 mm;
- the matrix uses oracle boxes rather than detector boxes;
- the blue boxes are controlled occluders, not natural leaves; and
- it uses the fixed base view, not the field-v3 wrist camera.

No result is connected to manipulation or control.

## Evidence

- Summary: `results/development/rendered_occlusion_localization_matrix_v1/summary.json`
- Summary SHA-256:
  `395e80cc76da74cc75deb78f7dd96dfcf702f75aadfec29c46601e15157c8c68`
- Frozen manifest SHA-256:
  `d5c2b6e663d731ee33a3d751e2ea9055e620e6c61844f01559a2bea3618b77b4`
- Evidence set: each of the 15 scenario directories preserves its condition
  receipt, rendered-condition probe, and paired trial JSON.
- The post-change 40-frame offline regression remains `PASS`; its v5 receipt
  SHA-256 is
  `a20df10b7aaf6ec0e30dbf9dba52c335b8bdd0c80e1b9684938c39a5c1f9fba6`.
- Regression: seven ROS 2 packages, `417/417` tests passed with no errors,
  failures, or skips.

Validate the frozen source contract without rerunning Gazebo:

```bash
python3 scripts/validate_rendered_occlusion_localization_matrix.py \
  --manifest config/rendered_occlusion_localization_matrix_v1.json
```

## Next bounded gate

A future v2 requires a new decision and a new output directory. It should use
an instance-aware occluder mask (or paired-render differential) instead of an
absolute blue-pixel heuristic, and it should require both localization nodes
to acknowledge populated sensor caches before counted frames begin. Only after
that diagnostic should non-oracle detector boxes, natural leaf occlusion, and
the wrist view be evaluated. Robot motion remains a separate authorization.
