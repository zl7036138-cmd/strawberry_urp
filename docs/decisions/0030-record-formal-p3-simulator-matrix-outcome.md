# ADR 0030: Record the formal P3 simulator-matrix outcome

- Recorded: 2026-07-18
- Status: Accepted
- Scope: Formal simulator P3 outcome under the ADR-0026 engineering waiver

## Context

ADR 0029 froze a single-use 165-scenario simulator matrix: 135 positive trials
covering the complete lighting, occlusion, position, and Gazebo-seed product,
plus 30 balanced only-unripe negative trials. The claim was consumed against
the authoritative v2 preflight and the exact ADR-0026 model at confidence
threshold 0.58.

The execution host stopped after trial 100 had completed and before trial 101
started. The run resumed under the same claim. Resume classification preserved
all 100 existing behavioral results and executed only the 65 missing scenarios.
Every scenario therefore has exactly one behavioral attempt; no outcome was
discarded or repeated.

## Evidence

`results/p3/formal_matrix_v1/summary.json` records:

- all 165 scenarios complete, unique, infrastructure-valid, and cleanly shut
  down;
- 39/135 positive end-to-end successes, or 0.2888888889, below the required
  0.80;
- 30/30 safe negative `NO_PICK` outcomes, with zero negative pick attempts and
  zero acquired unripe fruit;
- planning p95 of 0.0684945770 seconds from 51 planning observations, below the
  five-second limit;
- 84 perception failures and 12 grasp failures, with no unattributed failure;
- positive success of 33/45 with no occlusion, 6/45 with partial occlusion, and
  0/45 with heavy occlusion; and
- identical 13/45 positive success for each of the three Gazebo seeds.

The held-out real-image test receipt is absent and that test remains sealed.

## Decision

Reject the formal P3 simulator success-rate gate. P3 is not accepted and the
result must not be described as an 80% end-to-end pass.

Accept only the independently measured sub-results: negative-scene safety,
planning latency, infrastructure completeness, clean shutdown, and complete
failure attribution. These sub-results do not override the failed overall
gate.

The formal matrix is consumed. Its 165 behavioral outcomes are immutable and
must never be selectively rerun, replaced, or averaged with later development
runs. Any later evaluation is a separately named post-intervention experiment,
not a correction to this baseline.

P4 may now define one frozen bottleneck intervention. Perception is the primary
candidate because it accounts for 84 of the 96 positive failures. The 12 grasp
failures remain a separately recorded secondary issue. The intervention,
training/evaluation isolation, and post-intervention matrix must be frozen by a
new decision before any implementation or run.

## Boundaries

This is simulator evidence under the ADR-0026 metric waiver. It does not accept
the original T30 0.85 macro-F1 gate, does not authorize access to the held-out
real-image test, and does not support physical-robot or sim-to-real claims.
