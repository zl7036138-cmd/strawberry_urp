# ADR 0026: Accept the below-gate model for bounded simulation control

- Recorded: 2026-07-17
- Status: Accepted by project-owner waiver
- Scope: P3 simulation integration only

## Context

The clean-label retraining candidate selected `best.pt` at confidence `0.58`.
Its audited-validation macro/ripe/unripe F1 is
`0.8006746847/0.8870636550/0.7142857143`. It therefore does not meet the frozen
macro-F1 requirement of `0.85`, and the original numeric result must not be
rewritten as a pass.

The project owner explicitly decided on 2026-07-17 to use this model as the
engineering baseline and to continue perception-controlled simulator
integration despite that shortfall.

## Decision

1. Record T30's engineering disposition as `ACCEPTED_WITH_WAIVER`. The numeric
   gate remains `FAILED` with macro-F1 `0.8006746847`; no metric or threshold is
   changed.
2. Freeze the exact checkpoint, SHA-256, 19,162,586-byte size, 640-pixel input,
   and confidence threshold `0.58` in
   `config/p3_perception_control_waiver_v1.json`.
3. Authorize that checkpoint to publish `/strawberry/detections` and
   `/strawberry/target_pose`, with the orchestrator explicitly configured as
   `target_source=perception` and `control_target_topic=/strawberry/target_pose`.
4. Keep the Oracle provider stopped. Simulator truth may associate a stable
   fruit ID and verify the test scene, but may not choose maturity, select the
   target, or replace the RGB-D-derived target pose.
5. Keep all generic launch defaults fail-closed. Perception obtains motion
   authority only through the dedicated waiver launcher and its hash-checking
   preflight.
6. Begin with one clear, isolated, bounded smoke trial. Passing that smoke only
   proves that the waived model can drive the existing integration path; it
   does not pass the formal P3 robustness gate.

## Boundaries

- The held-out real-image test remains sealed and its receipt must remain
  absent.
- The formal 135 positive plus 30 negative trial matrix is not authorized by
  this decision.
- No new training, threshold tuning, model selection, real-world
  generalization, or sim-to-real claim is authorized.
- Reports and the final thesis must distinguish `ACCEPTED_WITH_WAIVER` from
  satisfying the original `macro-F1 >= 0.85` requirement.

## Consequences

P2 may be treated as engineering-complete with a documented perception waiver,
so P3 simulator integration can proceed. Scientific reporting retains the
shortfall as a limitation, and full P3 acceptance still depends on measured
perception-controlled end-to-end trials.
