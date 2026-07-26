# ADR 0031: Freeze the single P4 simulator-perception intervention

- Recorded: 2026-07-18
- Status: Accepted
- Scope: Simulator-only P4 intervention after the failed formal P3 baseline

## Context

ADR 0030 records 84 perception failures and 12 grasp failures among the 96
failed positive trials in the consumed P3 matrix. Perception is therefore the
measured primary bottleneck. The baseline succeeds in 33/45 no-occlusion
scenarios, 6/45 partial-occlusion scenarios, and 0/45 heavy-occlusion scenarios.

An earlier single simulator-adaptation training claim already produced an exact
candidate at
`outputs/perception/yolo11s_640_sim_adapt_v1/weights/best.pt`. It scores 1.0
macro-F1 on the isolated 72-image synthetic held-out set at threshold 0.80, but
regresses audited real validation to 0.625537 macro-F1. ADR 0022 correctly
rejected it for general promotion and real-image use.

## Decision

1. P4's sole intervention is substitution of that exact existing checkpoint at
   threshold 0.80 for simulator perception. No new training, checkpoint search,
   threshold sweep, asset change, scene-condition relaxation, localization
   change, grasp change, or motion-planning change is permitted in the same
   intervention.
2. This decision narrowly supersedes ADR 0022's runtime prohibition only for a
   separately named simulator-only P4 qualification and, if qualified, a later
   post-intervention simulator matrix. ADR 0022's real-validation rejection and
   all prohibitions on real-image, physical-robot, and sim-to-real promotion
   remain in force.
3. Qualification is no-motion and uses 30 fresh worlds: the five frozen
   positions crossed with `nominal+none`, `dim+none`, and `nominal+heavy`, once
   with an isolated ripe fruit and once with an isolated unripe fruit. Each
   scenario records exactly 60 settled detection frames after condition and
   pose probes.
4. Qualification requires complete infrastructure and windows; at least 95%
   ripe-frame and target-pose rates in each no-occlusion condition; at least 60%
   ripe-frame and target-pose rates overall under heavy occlusion and at least
   50% at every heavy-occlusion position; no more than 5% false-ripe frames in
   the unripe scenarios; and at least 50% correct unripe-observation frames
   overall. No manipulation, orchestrator, or attachment process may start,
   and no arm or gripper motion goal may be sent.
5. Qualification has one execution only. If it fails, the intervention is
   rejected and no second model, threshold, training run, or scene change may
   replace it under P4.
6. A passing qualification does not accept P3 or P4. It authorizes only a new
   preflight that freezes a separately named, full 135-positive plus 30-negative
   post-intervention matrix. The consumed P3 baseline remains immutable and is
   never recomputed or merged with the later result.

## Boundaries

The held-out real-image test remains sealed. The candidate's real-validation
regression must be reported beside every simulator result. No output from this
intervention may be described as real-world maturity accuracy, sim-to-real
success, or physical-robot evidence.
