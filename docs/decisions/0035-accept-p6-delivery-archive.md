# ADR 0035: Accept the P6 delivery archive

- Recorded: 2026-07-24
- Status: Accepted
- Scope: Final packaging and delivery

## Context

ADR 0034 froze P5 after clean reproduction, reporting, and video delivery.
P6 permits only proofreading, packaging, delivery, and reproduction-defect
corrections. A compact archive is required without duplicating the 1.49 GB raw
dataset or the approximately 2.7 GB offline wheel payload.

## Decision

Accept the deterministic P6 archive at
`artifacts/p6/strawberry_urp_release_v1.zip` when its adjacent receipt reports
`PASS`, the ZIP CRC check succeeds, the embedded inventory matches every
member, and the nested P5 verifier returns `VERIFIED_FINAL_RELEASE`.

The archive includes project source, all ROS packages and tests, configuration,
dependency locks, documentation, the published engineering and simulator
adaptation checkpoints, frozen P5 summaries and handoffs, and the final video.
The raw Zenodo archive, wheel payload, build products, and complete per-trial
logs are deliberately omitted; their recreation or authoritative summaries
are documented and bound.

## Scientific status

Packaging completion does not change research results. T30 remains below its
numeric F1 gate under an engineering waiver, formal P3 remains failed at
39/135 positive successes, and P4 remains failed at 0/300 heavy-occlusion
target poses. The held-out real-image test remains sealed. No physical-robot
or sim-to-real claim is made.

## Boundary

The project delivery is complete after the archive receipt and final P6
handoff validate. Later work is limited to user-requested material corrections
or reproduction defects. New features, retraining, formal reruns, or benchmark
changes require a separately approved research phase.
