# ADR 0019: Freeze the simulator-adaptation preflight

- Status: Accepted
- Date: 2026-07-16

## Context

ADR 0017/0018 complete the bounded five-position diagnostic. The historical
baseline detects the ripe fruit in 600/600 clear/dim frames but in 0/300 heavy
frames across all five positions. The earlier negative simulator pre-gate also
shows zero unripe observations. These repeated failures justify preparing one
simulator-specific candidate, while the audited real-validation macro-F1
remains 0.815818 and T30 stays below its 0.85 gate.

Immediately mixing diagnostic frames into training would contaminate the only
frozen pre/post screen. Changing the simulator asset and model together would
also make attribution impossible. The benchmark's three seed labels are not
yet genuine independent scene randomizations and must not be reused as
synthetic-training labels.

## Decision

Freeze `config/t70_sim_adaptation_intervention.json`. Authorize only the
synthetic-capture preflight. Training remains disabled until a receipt proves:

- synthetic train and synthetic held-out render manifests are disjoint;
- image hashes are unique within each split and disjoint across splits;
- neither split uses the existing D2 artifacts;
- synthetic training coordinates stay at least 15 mm from every exact D2
  coordinate;
- the formal seed labels `20260710`, `20260711`, and `20260712` do not appear
  in synthetic training;
- both ripe and unripe truth-projected labels are present; and
- only the 501 real training images may join synthetic training. The 116
  audited real-validation images are non-regression evidence only, and the
  real test split remains inaccessible.

After the preflight, allow at most one hash-frozen fine-tuning claim from the
existing `baseline__best` checkpoint. Keep YOLO11s, input size, and simulator
assets fixed so the candidate changes only training exposure. Exact capture
counts and hyperparameters must be frozen before that claim is acquired.

The candidate is promoted only as a simulator-specific Shadow candidate if it
passes every predeclared screen: synthetic held-out macro-F1 at least 0.85;
D2 heavy ripe-frame rate at least 0.60 overall and 0.50 at every position;
heavy target-pose rate at least 0.50; clear/dim ripe rates at least 0.95 and
target-pose rates at least 0.90; simulator false-ripe negatives at most 0.05
and unripe observation rate at least 0.50; and audited real-validation
macro/ripe/unripe F1 no lower than the baseline minus 0.02.

## Consequences

No training, model promotion, perception-controlled motion, formal 165-trial
run, or held-out real test is authorized by this ADR alone. A successful
candidate remains separate from the real-data T30 model and cannot close T30,
P2, P3, or P4. A failed candidate consumes the single simulator-adaptation
claim and triggers analysis, not threshold relaxation or an automatic second
run.
