# ADR 0029: Freeze the formal P3 simulator matrix

- Recorded: 2026-07-17
- Status: Accepted
- Scope: Formal simulator end-to-end evaluation under the ADR-0026 waiver

## Context

ADR 0028 accepts the one-pose repeated development gate. The repository already
generated 135 positive and 30 balanced negative labels, but its three seed
values had not been connected to Gazebo's random-number generator. Running that
definition would have misrepresented labels as independent simulation seeds.

## Decision

1. The positive matrix is the complete product of three lighting levels, three
   occlusion levels, five frozen reachable positions, and Gazebo seeds
   `20260710`, `20260711`, and `20260712`: 135 trials.
2. The 30 only-unripe trials retain the deterministic design that balances each
   lighting level, occlusion level, and seed ten times and each position six
   times.
3. Every trial starts a fresh headless world in a distinct ROS domain. The
   scenario seed is passed explicitly through `simulation_seed` to
   `gz sim --seed`. The scene materializer's fixed identifier `20260710` is
   separately recorded as a receipt identifier and is not called a random seed.
4. Trial order is frozen by SHA-256 ranking with schedule seed `20260713`, so
   positives, negatives, and stress conditions are interleaved without manual
   selection.
5. A condition receipt and five-frame camera probe are mandatory before each
   behavioral trial. Only pre-behavior infrastructure failures may receive one
   retry; every attempt is preserved. Once the trial service accepts a request,
   its behavioral outcome is never rerun or discarded.
6. Formal simulator acceptance requires at least 80% positive end-to-end
   success, at least 95% negative `NO_PICK`, no more than 5% negative control
   attempts or acquired fruit, planning p95 no greater than five seconds,
   complete failure attribution, valid infrastructure, and clean shutdowns.
7. The exact ADR-0026 model, threshold, launch files, scene definitions,
   schedule, and evaluators are hash-bound before the single formal claim is
   consumed. No model, threshold, code, or configuration change is allowed
   after that point. An interrupted run may resume only within the same claim
   and may never replace a completed behavioral outcome.

## Boundaries

This is formal simulator P3 evidence under an engineering waiver. It does not
change the measured real-validation macro-F1 of 0.8006746847 into the original
0.85 detector gate, does not open the held-out real-image test, and does not
support physical-robot or sim-to-real claims.

Freezing this ADR and producing a passing preflight do not consume the formal
matrix. Trial consumption begins only when the single-use claim is created and
the first formal Gazebo process is started.
