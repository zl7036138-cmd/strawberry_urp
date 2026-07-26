# ADR 0006: BGR NumPy input for Ultralytics shadow inference

- Status: Accepted
- Date: 2026-07-15
- Scope: T30 runtime adapter and T60 shadow observation

## Context

The first T60 shadow smoke kept oracle control isolated and processed 837 RGB
frames, but its 99 detections were all `UNRIPE`; it produced no ripe
`TargetPose`. This was initially a possible simulator-domain mismatch, but the
run did not preserve the exact detector input or associate boxes with projected
Gazebo truth, so it could not establish that diagnosis.

The ROS camera contract is `rgb8`. The perception adapter converted each ROS
image to an RGB NumPy array and passed that array directly to Ultralytics
8.4.92. The installed `BasePredictor.preprocess` treats NumPy HWC input as BGR
and reverses its last axis to RGB. Supplying an already-RGB array therefore
swapped red and blue before inference. Both trained weights were separately
inspected and expose the correct class contract `{0: ripe, 1: unripe}`.

`results/t60/shadow_forensics_rgb_bgr_v1/summary.json` compares both input
paths on the same five simulator frames at the frozen historical threshold
0.31. The legacy RGB path produced zero detections. The BGR path produced one
`ripe` detection per frame, all associated with ripe target 1; the first
prediction confidence was 0.519437 and its approximate truth-box IoU was
0.774342. Every CvBridge RGB/BGR pair was verified as an exact channel swap.

## Decision

- Keep the public ROS image encoding contract as `rgb8`.
- Convert the ROS image to `bgr8` before passing its NumPy array to
  `YOLO.predict`. A named constant and unit tests make this library-specific
  boundary explicit.
- Keep the fail-fast weight class contract exactly `0=ripe, 1=unripe`.
- Preserve a dedicated `shadow_diagnostic` executable and
  `scripts/run_shadow_forensics.sh`. It records source hashes, both inference
  modes, projected Gazebo truth, box associations, overlays, and a JSON
  summary without allowing perception to control the arm.
- Do not start synthetic fine-tuning or change simulator visual assets on this
  evidence. Those interventions remain conditional on a later, separately
  split simulator perception pre-gate after the runtime defect is removed.
- Keep the real-image held-out test sealed. This runtime correction neither
  changes the below-threshold T30 validation results nor authorizes formal
  testing or perception control.

## Consequences

The corrected full-stack smoke at
`results/t60/oracle_with_yolo_shadow_bgr_v3/trial_01.json` retained oracle
control and completed the pick successfully. It observed 798 shadow detection
frames, 1,587 boxes, 1,548 ripe boxes, 780 shadow target poses, and latest
associated target ID 1. This proves that the prior zero-ripe outcome was caused
directly by the channel-order defect and that the repaired shadow path can
reach localization.

This single-scene smoke is diagnostic evidence, not the P3 acceptance result.
It does not measure simulator maturity macro-F1, negative-scene false picks,
or the frozen 135-trial end-to-end matrix. T30 remains unaccepted at validation
macro-F1 0.787121, and the orchestrator remains configured for oracle control.

The separately split no-motion simulator pre-gate subsequently collected 100
frames across five ripe and five only-unripe camera-clear scenarios. It passed
with ripe truth-associated frame recall 1.0, target-pose rate 1.0, and
false-ripe negative-frame rate 0.0. However, diagnostic unripe observation
recall was 0.0. This is sufficient to defer synthetic fine-tuning for the
current simple-scene NO_PICK requirement, but not to claim class-balanced
simulator maturity accuracy or sim-to-real transfer. Broader lighting and
occlusion evidence may still trigger a separately versioned sim-adapted model.
