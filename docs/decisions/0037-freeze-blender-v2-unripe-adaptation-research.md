# ADR 0037: Freeze one Blender-v2 unripe-adaptation research claim

- Recorded: 2026-07-26
- Status: Accepted
- Scope: New non-formal research phase after the completed P4/P5 work

## Context

The canonical Blender-v2 plant contains two ripe fruits and one unripe fruit.
The engineering checkpoint at confidence `0.58` detects the isolated ripe
fruits reliably, but the fresh no-motion diagnostic at
`results/development/blender_scene_v2_single_fruit_60f_v4/summary.json`
records `0/60` unripe detections. Because the unripe fruit remains undetected
when the other fruits are parked while the plant, camera, lighting, model, and
threshold remain fixed, this is an appearance/size/pose domain gap rather than
a depth-localization failure. The same diagnostic confirms that the second
ripe fruit is detectable when isolated, so its canonical failure is primarily
viewpoint or plant occlusion.

ADR 0032 consumed and rejected P4's sole simulator intervention. This decision
does not reopen P4, replace its evidence, or authorize another formal matrix.
The project owner asked to continue the project after the diagnostic and
geometry correction.

## Decision

1. Name the new work `blender_v2_unripe_adapt_research_v1`. It is explicitly
   non-acceptance research and may not be reported as P4, P5, T30, or real-world
   evidence.
2. Capture exactly 72 training and 24 synthetic held-out images from the
   tracked Blender-v2 world. Keep the plant and fixed base RGB-D camera, park
   the two non-target fruits, and cross two classes with three frozen lighting
   levels and 12 train or four held-out position/orientation poses.
3. Use disjoint train and held-out pose/orientation tuples. Labels come only
   from the 26 mm scene-radius truth projection. Capture one PNG per rendered
   configuration. Do not use old P3/P4 frames, formal seed labels, the real
   held-out test, or synthetic images from the rejected v1 adaptation.
4. Before training, inspect representative ripe and unripe samples from every
   lighting level and verify all hashes, labels, counts, split isolation, and
   the absence of a test split.
5. Authorize at most one training claim. Initialize from the exact engineering
   checkpoint
   `outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt` with SHA-256
   `e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70`.
   Train on the 501 consensus-corrected real training images plus the 72 new
   synthetic training images. Synthetic held-out images are validation-only;
   audited real validation and the sealed real test are not training inputs.
6. Run exactly 12 deterministic fine-tuning epochs at 640 px, batch 8, seed
   `20260726`, AMP disabled, SGD learning rate `0.0001`, and the frozen training
   YAML. Select `last.pt` only; do not search checkpoints, thresholds, seeds,
   learning rates, or augmentations. Keep runtime confidence fixed at `0.58`.
7. Apply the selected weight once to the 24-image synthetic held-out split and
   once to the existing 116-image audited real validation derivative.
   Promotion to a new simulator Shadow candidate requires:

   - synthetic held-out ripe and unripe F1 each at least `0.85`;
   - audited real macro-F1 at least `0.7806746847`;
   - audited real ripe F1 at least `0.8670636550`;
   - audited real unripe F1 at least `0.6942857143`;
   - a fresh no-motion live Blender-v2 diagnostic with each ripe fruit detected
     in at least 95% of frames and the isolated unripe fruit detected in at
     least 90% of frames; and
   - no arm trajectory, gripper command, pick action, attachment, formal test,
     or formal matrix execution.

8. Any failed gate rejects the candidate. There is no retry, threshold
   relaxation, alternate checkpoint, extra epoch, second capture, or scene
   change under this decision.

## Boundaries

The current checkpoint remains the active engineering baseline until every
gate passes. A passing candidate is still simulator Shadow-only and does not
authorize perception-controlled picking, physical-robot use, formal
acceptance, or a sim-to-real claim. The sealed real held-out test remains
unconsumed.
