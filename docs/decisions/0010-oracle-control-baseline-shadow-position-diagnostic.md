# ADR 0010: Continue downstream with Oracle control and baseline shadow diagnostics

- Status: Accepted
- Date: 2026-07-15
- Owners: T00 / T60

## Context

The audited `baseline__best` detector remains below the frozen T30 validation
gate: macro-F1 is 0.815818 at confidence 0.31. The authorized Opt2 unripe-x2
exposure intervention regressed to 0.795696 and is rejected. The held-out real
test remains sealed. At the same time, the corrected RGB/BGR simulator shadow
path and the camera-clear simulator perception pre-gate show that the baseline
can provide useful observational evidence in the simple Gazebo scene.

Blocking every downstream diagnostic on T30 would hide independent
manipulation and integration risks. Letting the below-gate detector command the
arm would violate the frozen stage gates and create an unsafe, misleading P3
claim.

## Decision

1. Oracle truth on `/strawberry/oracle/target_pose` remains the sole motion
   control source. Perception-controlled motion stays disabled.
2. `baseline__best` is the only primary shadow model. It is bound by SHA-256
   `ce6c998ed52ae97c8e8b51880adb60618733fff11be22378baa0044f9a126425`
   and confidence threshold 0.31. Opt2 remains an ablation and cannot replace it.
3. A bounded five-position diagnostic may run one fresh Gazebo world per
   position. Each trial moves ripe `strawberry_1` through the explicit
   `set_pose` service, waits for ground truth and Oracle control to agree, then
   runs the existing orchestrated pick-and-place while collecting YOLO only on
   shadow topics.
4. The first diagnostic version uses nominal lighting, no occluder, one seed,
   and diagnostic-only coordinate labels. These labels are not the five formal
   benchmark positions.
5. A motion failure is a valid diagnostic observation if the runner,
   source-isolation checks, position configuration, shadow stream, and shutdown
   all remain valid. Diagnostic completeness is therefore separate from motion
   success.
6. The result may guide later scene and manipulation work, but it cannot close
   T30, P2, P3, or P4; cannot replace the 135 positive plus 30 negative formal
   matrix; and cannot authorize access to the held-out real test.

The executable contract is
`config/t60_oracle_shadow_position_diagnostic.json`. It fails closed if the
control source changes, shadow/control topics overlap, the model hash changes,
the scope claims formal acceptance, or the v1 scene conditions claim validated
lighting/occlusion variation.

## Consequences

- Downstream manipulation work can continue without laundering the failed T30
  metric into an end-to-end acceptance claim.
- The same trials produce paired evidence: Oracle-controlled physical outcome
  and non-controlling YOLO observations at each position.
- Position sensitivity can be diagnosed now. Lighting and occlusion remain
  explicitly unmeasured until a separate, validated runtime scene-injection
  contract exists.
- Formal P3 remains blocked even if all five diagnostics succeed.
