# ADR 0020: Accept the synthetic-capture isolation preflight

- Status: Accepted
- Date: 2026-07-16

## Context

ADR 0019 authorized only a leakage-safe synthetic-capture preflight. It did
not authorize training. The preflight had to create disjoint synthetic train
and held-out partitions, exclude the earlier T70-D2 observations and formal
seed labels, preserve the frozen simulator assets, and produce truth-projected
RIPE/UNRIPE boxes before a simulator-specific training claim could be designed.

## Decision

Accept the completed preflight at
`data/processed/t70_sim_adaptation_preflight_v1`. Its hash-bound receipt proves:

- 36/36 capture groups and 288/288 images are complete;
- the train split contains 216 images, balanced as 108 RIPE and 108 UNRIPE;
- the synthetic held-out split contains 72 images, balanced as 36 RIPE and 36
  UNRIPE;
- each split covers all nine lighting-by-occlusion conditions evenly;
- encoded and source image hashes are unique within each split and have zero
  cross-split overlap;
- none of the 27 recorded T70-D2 source-image hashes appears in the new data;
- the formal labels `20260710`, `20260711`, and `20260712` are absent;
- all declared coordinates satisfy the 15 mm D2-separation rule; and
- the canonical synthetic-dataset digest is
  `a8f1f157e15f6a52f637b1cc11f275f3320bd4cae156206560a7ead6d17b76b5`.

The truth projection labels the complete rigid fruit sphere, so a target keeps
the same box at one camera pose when a visual-only occluder is introduced.
Four representative RIPE/UNRIPE and none/heavy samples were visually checked;
their projected boxes cover the rendered fruit as intended.

The receipt deliberately retains `training_unlocked=false`. The next decision
must freeze one exact mixed-training manifest, every training hyperparameter,
the baseline checkpoint identity, the synthetic held-out evaluation, the
audited real-validation non-regression screen, and the single-use claim before
any GPU training starts.

## Consequences

The capture and leakage-isolation preflight is complete. No model has been
trained, the real test split remains sealed, the robot did not move, and
perception control and the formal 135+30 matrix remain unauthorized. This ADR
does not accept T30, P2, full P3, T70, or P4. A future training run without the
separate frozen manifest and claim is invalid evidence.
