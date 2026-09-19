# P0 Metric Dictionary v1

Status: **FROZEN FOR G0 REVIEW**  
Scope: evidence and evaluation semantics; no production behavior change  
Effective baseline: source lineage beginning at `6f25868b0d946f7ce1a73206d3676889121f6712`

This dictionary fixes the unit, denominator, evidence source, and failure
semantics for every retained project gate. A later phase may add a metric, but
may not silently change one of these definitions or reuse its name for a new
quantity. Any intentional change requires a new protocol version and a phase
decision.

## 1. Result states and counting rules

Evidence admissibility and performance are separate:

- `VALID_COMPLETE`: all mandatory bindings and evidence for the claim verify.
- `VALID_INCOMPLETE`: the record is structurally coherent but one or more
  required observations are absent. Its affected gate is `INDETERMINATE`.
- `INVALID`: a binding is false, identifiers are ambiguous, an event is
  duplicated or out of order where uniqueness/order is required, or two fields
  contradict one another. The record is rejected and cannot contribute a
  passing value.

A gate has exactly three states: `PASS`, `FAIL`, or `INDETERMINATE`. Boolean
compatibility fields may be emitted only as derived views; `overall_pass` is
true only when every mandatory gate is `PASS`. Missing evidence is never
converted to zero events, and `INDETERMINATE` is never truthy.

Counts are integers over unique identities, not messages. Repeated messages do
not create new samples. A zero denominator is `NOT_APPLICABLE` only for a
declared stratum where the matrix required no such unit; it does not pass a
required aggregate gate. Assigned but invalid, not-started, timed-out, or
incomplete scenes remain visible in the scene accounting.

## 2. Frozen experimental units

| Unit | Definition | Forbidden substitution |
|---|---|---|
| Assigned scene | One scenario ID listed in the frozen pre-run matrix. | Only launched or completed scenes. |
| Physical target | One unique simulation fruit ID in that scene's pre-run manifest. | Detector box, message, frame, or local track ID. |
| Runtime target | A track ID scoped to one run and one motion epoch. | Reusing the same integer across runs/epochs as one identity. |
| Operation | One immutable operation/attempt ID from selection through terminal physical state. | Counting retries or feedback messages as targets. |
| Association | One eligible transition between two observations under the frozen tracking protocol. | Number of frames or all pairwise matches. |
| Localization sample | One unique accepted production pose paired offline with one visible truth target. | Quality values without a target/output identifier. |

Independence is reported at the assigned-scene level. Multiple targets, frames,
or operations in one scene are correlated observations and must not be described
as independent scene samples.

## 3. Mandatory task funnel

Every run set reports the following counts in order. Each target can enter a
layer only if it was present in the immediately preceding layer. Reports include
the adjacent conversion rate and the end-to-end rate from all task-domain ripe
targets; they must not present only a late conditional success rate.

1. `assigned_scene_count`: every matrix entry, including invalid, not-started,
   crashed, or missing-result entries. Also report `invalid_scene_count`,
   `started_scene_count`, and `terminal_scene_count` without shrinking the
   assigned denominator.
2. `task_domain_ripe_truth_count`: every manifest fruit labelled `RIPE` and
   within the declared task-domain construction constraints. Actual planner or
   controller results cannot remove a fruit from this count.
3. `visible_ripe_truth_count`: task-domain ripe truth whose projection and
   depth support satisfy the frozen offline visibility protocol. Visibility is
   computed from scene/camera truth, never from whether the detector returned a
   box.
4. `localized_ripe_truth_count`: a visible ripe truth identity uniquely matched
   to a valid, emitted production pose under the frozen matching tolerance. A
   rejection or missing pose stays in the visible denominator. Localization
   error is computed offline and cannot be supplied to control.
5. `stable_track_ripe_truth_count`: a localized truth identity associated with
   one runtime track that met the frozen stability rule before selection.
   Track resets, ID reuse, and ambiguous many-to-one mappings are not new
   successes.
6. `planner_accepted_ripe_truth_count`: a stable target for which the frozen
   production planning gate accepted the intended operation. Planning failure
   remains attrition from the stable-track denominator and cannot retroactively
   make the target unreachable.
7. `pick_started_ripe_truth_count`: a planner-accepted operation with both
   action acceptance and independently observed physical motion/stage start.
   `PICK_SENT` alone is the auxiliary `pick_commanded_count`, not pick start.
8. `identity_correct_physical_harvest_count`: a pick-started operation for which
   the selected runtime track maps uniquely to physical fruit `F`, and ordered
   contact, attachment, transport/detachment, and placement evidence all refer
   to that same `F`. `F` must be ripe. Equal counts are insufficient.
9. `terminally_confirmed_harvest_count`: an identity-correct harvest followed by
   a coherent action terminal result and independently observed attachment-free,
   stopped/home, cleanup, and scene-terminal state. A controller `DONE` or
   `SUCCESS` string alone is insufficient.

For the retained reachable-target gate, also report a parallel preregistered
denominator, `reachable_ripe_truth_count`, defined below. It is not a replacement
for the complete task funnel.

## 4. Reachability reference

`reachable_by_construction` means **reference construction eligibility**, not
observed MoveIt feasibility and not a claim of physical kinematic reachability.
It is assigned when the scene is materialized, before production execution:

```text
inside fixed Cartesian box
  x in [0.30, 0.72] m
  y in [-0.34, 0.34] m
  z in [0.48, 0.66] m
AND Euclidean clearance from the padded bin AABB >= 0.070 m
```

The padded bin AABB expands manifest interior bounds by
`[x- 0.05, x+ 0.05, y- 0.05, y+ 0.05, z- 0.03, z+ 0.00] m`. The reference
implementation is `_reachable_by_construction` in
`strawberry_sim/generalized_scene.py`; its P0 snapshot SHA-256 is
`c790b89f41bd05a90681cae5f8de808beb9ceadce3cf5ff9a4c97ebd4ba1bc3f`.
The scorer must recompute the label from the bound scene manifest and bound
algorithm version. A mismatch is `INVALID`.

The production planner may accept or reject a reference-eligible target, but
that outcome never changes this label. A later claim about true kinematic
reachability requires a separately named, independently preregistered oracle.

## 5. Detection, localization, tracking, and quality

The generalized simulation aggregate uses unique per-target records:

- `ripe_true_positive_count`: emitted ripe predictions uniquely matched to
  visible ripe truth under the frozen matching protocol.
- `ripe_prediction_count`: all emitted ripe predictions, including unmatched,
  duplicate, wrong-maturity, and rejected-for-localization predictions when the
  detector emitted them.
- `visible_ripe_recall = ripe_true_positive_count /
  visible_ripe_truth_count`; retained gate `>= 0.90`.
- `ripe_precision = ripe_true_positive_count / ripe_prediction_count`; retained
  gate `>= 0.95`.
- `localization_coverage = localized_ripe_truth_count /
  visible_ripe_truth_count`; always reported, including rejections.
- `localization_error_m`: Euclidean error of each uniquely matched accepted
  pose in the scoring frame. Report sample count, median, P95, and full failure
  counts. Retained project limits are median `<= 0.015 m` and P95 `<= 0.030 m`.
- `sigma_m`: the production quality value attached to the same identified pose;
  it is not an error guarantee. Report coverage, maximum, and fraction
  `<= 0.015 m`; the generalized formal v1 retained fraction gate is `>= 0.90`.
- `identity_switch_rate = identity_switch_count / association_count`; retained
  generalized formal v1 gate `<= 0.02`. Both numerator and eligible transition
  identifiers are mandatory.

The existing formal matrix serializes the P95 and sigma gates but omits the
project-level 15 mm median field. P0 therefore freezes and reports both without
editing the protected matrix. G4 must bind a protocol that includes the median
gate before any formal resource can be released.

The real-image result is a separate line: macro-F1 `>= 0.85` on the independently
held 115-image test split. Simulation metrics cannot satisfy it, and P0 does not
open images, labels, predictions, or metrics from that protected split.

## 6. Harvest, scene completion, and safety

- `accepted_target_success_rate = identity_correct_physical_harvest_count /
  planner_accepted_ripe_truth_count`; development gate `>= 0.80`. Missing
  operation evidence makes the affected numerator status `INDETERMINATE`, not
  zero or success.
- `reachable_target_success_rate = terminally_confirmed harvests of pre-run
  reachable ripe truth / reachable_ripe_truth_count`; generalized gate
  `>= 0.80`.
- A positive scene is complete only when every preregistered required reachable
  ripe target is terminally confirmed, all required operations have determinate
  terminal state, and no hard safety event occurred. The aggregate positive
  scene completion gate is `>= 0.70`. A `scene_complete` boolean is accepted
  only as a recomputed value and must agree with the event evidence and outcome.
- A negative scene is safe-no-pick only with complete observation time,
  independent no-motion/no-contact evidence, clean terminal state, and zero
  operation starts. Its retained rate gate is `1.00`. `NO_PICK` alone is not
  proof.

Hard safety counts are `unripe_pick_count`, `wrong_target_pick_count`,
`nonallowed_collision_count`, `joint_limit_violation_count`, and
`unsafe_motion_attempt_count`; every retained maximum is zero. Each zero requires
a bound independent monitor covering the full behavior interval. If monitor
coverage is absent, the count is unknown and its gate is `INDETERMINATE`.
Pre-motion collision rejection is reported separately and is not a physical
collision.

The multi-fruit development qualification remains five assigned runtime scenes,
all five terminal and clean, at least four scenes with at least two
identity-correct terminally confirmed harvests from the same uninterrupted
batch, accepted-target success rate `>= 0.80`, and all hard safety gates at zero
with complete evidence. Restarting between single-fruit successes does not meet
the gate.

## 7. Time and uncertainty reporting

Durations use monotonic timestamps from the bound producer clock. Event time and
receipt time are both retained; wall-clock time is only for cross-artifact
correlation. Timeout is an operation knowledge state, not evidence that motion
or attachment did not occur.

Every rate is accompanied by numerator, denominator, and status. Every
distribution is accompanied by its identified sample count and missing/rejected
count. Until a frozen sampling model supports interval estimation, thresholds
are preregistered observational acceptance lines and are not population
reliability guarantees.

## 8. Protocol bindings

Formal or qualification results must bind, by verified path and SHA-256, the
source commit/tree declaration, model, runtime configuration, environment,
matrix and materialized scenes, runner, recorder/evidence schema, scorer, metric
dictionary/protocol, resource authorization, and result. The matrix v1 SHA-256
is `2f9cc23d04947ab1ec1101010818a59e882d8cf04b6dd18cffb2ddc417493c2d`.
Syntactically plausible hash strings are not bindings.

Historical results are not rewritten to pretend these fields existed. They may
be recomputed from what is present, with every unsupported claim explicitly
`INDETERMINATE`.
