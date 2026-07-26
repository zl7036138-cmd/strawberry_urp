# ADR 0032: Reject the single P4 simulator-perception intervention

- Recorded: 2026-07-18
- Status: Accepted
- Scope: Outcome of the ADR-0031 simulator-only no-motion qualification

## Context

ADR 0031 froze one P4 intervention: substitute the already existing
simulator-adaptation checkpoint at threshold 0.80 without changing training,
assets, scene conditions, localization, grasping, or motion behavior. A passing
30-scenario no-motion qualification was required before any post-intervention
motion matrix.

## Evidence

`results/p4/sim_adapt_qualification_v1/summary.json` records:

- 30/30 complete, infrastructure-valid, cleanly stopped scenarios, with no
  manipulation, orchestrator, attachment, arm motion, or gripper motion;
- ripe detection in 300/300 nominal-heavy frames, improving the corresponding
  pre-intervention diagnostic from 0/300;
- ripe detection in 300/300 nominal-none and 300/300 dim-none frames;
- target-pose output in 292/300 nominal-none and 299/300 dim-none frames;
- correct unripe observations in 840/900 frames, with zero false-ripe frames;
  and
- target-pose output in 0/300 nominal-heavy frames, failing both the overall
  and every-position heavy-occlusion localization requirements.

Heavy-occlusion launch logs repeatedly report that the depth-derived point is
approximately 0.345 to 0.366 metres from the nearest ground-truth fruit, beyond
the frozen 0.080-metre association limit. The detection box is present, but its
central depth belongs to the foreground occluder. The localization module
therefore rejects the point and correctly avoids publishing an unsafe pose.

## Decision

Reject the ADR-0031 intervention qualification. The model substitution repairs
the measured heavy-occlusion detection failure but does not repair the full
perception-to-position chain, so it is not authorized for robot control or a
post-intervention 135+30 motion matrix.

The sole P4 intervention opportunity is consumed. Do not select another
checkpoint, lower the threshold, retrain, change the occluder, alter the
association tolerance, add a localization fallback, or combine a second fix
under this project benchmark. Such work would be a new research phase rather
than the frozen one-intervention P4 plan.

Proceed to P5 release and reporting with the failed formal P3 result and this
failed-but-informative P4 intervention. Report the demonstrated separation:
the simulator-specific model can recover class detection under heavy visual
occlusion, while the single-view bounding-box-centre depth method remains the
end-to-end blocker.

## Boundaries

The rejected candidate remains simulator-only and must not replace the
ADR-0026 controller. The held-out real-image test stays sealed. This outcome
does not accept T30, P3, P4, real-image generalization, physical-robot behavior,
or sim-to-real transfer.
