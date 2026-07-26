# ADR 0034: Freeze the final P5 release

- Recorded: 2026-07-24
- Status: Accepted
- Scope: P5 reproducibility, reporting, and demonstration delivery

## Context

ADR 0033 accepted the separate Ubuntu 24.04 WSL2 reproduction but kept P5 open
until a headed demonstration was recorded, checked, and bound into the release.
The recording must not overwrite or be presented as formal P3/P4 evidence.

## Evidence

The positive headed demonstration at
`results/p5/headed_demo_capture_v4/run/trial.json`:

- uses Oracle truth as the only arm-control source;
- keeps the rejected simulator-adaptation model Shadow-only;
- completes the full state sequence through `DONE/SUCCESS`;
- records 0.036674 seconds planning and 89.207092 seconds execution; and
- observes 296 Shadow detection frames, 258 ripe detections, and 183 Shadow
  target poses.

The only-unripe headed demonstration at
`results/p5/headed_no_pick_capture_v1/run/trial.json`:

- uses the ADR-0026 model under its existing engineering waiver;
- reaches `DONE/NO_PICK`;
- emits no control target and starts no planning or execution;
- records no fruit acquisition and no unsafe control attempt; and
- is explicitly non-formal.

`artifacts/p5/video/strawberry_urp_demo_v1.receipt.json` validates a 288.7
second H.264 video at 1280 by 720 pixels. It includes the two headed segments,
accepted module results, the failed formal P3 result, the rejected P4
intervention, the accepted clean reproduction, and explicit limitations. The
video receipt binds the source clips, behavioral JSON receipts, storyboard,
and an immutable metric snapshot.

## Decision

Accept P5 as complete and freeze release `p5_release_v1`. Its status is
`FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4`: release engineering, clean
reproduction, reporting, and video delivery are complete, while the scientific
outcomes remain unchanged.

Formal P3 remains failed at 39/135 positive successes, P4 remains failed with
0/300 heavy-occlusion target poses, and T30 remains below the 0.85 macro-F1
gate under the ADR-0026 engineering waiver. These outcomes are limitations,
not release defects to be hidden or rewritten.

## Boundaries

The positive headed recording is an Oracle-controlled demonstration, not a
sample from the formal matrix. The only-unripe recording is one non-formal
engineering-waiver demonstration, not a new acceptance trial. The held-out
real-image test stays sealed. No physical-robot, fruit-damage, stem-cutting, or
sim-to-real claim is made.

P6 may address only reproduction defects, material corrections, packaging, and
delivery. It must not add features, retrain models, alter the localization
method, rerun formal P3, or attempt another P4 intervention.
