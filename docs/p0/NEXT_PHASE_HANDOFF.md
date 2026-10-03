# P0 to P1/P2 Handoff

Status: **PREPARED, NOT RELEASED UNTIL G0 DECISION**  
Candidate basis: P0 implementation through
`53909fa4a25881d7fd68d0eb65bf9221094a0069`

P0 ends with trustworthy failure/unknown classification, not improved robot
capability. P1 and P2 may investigate in parallel only after G0 acceptance;
work that depends on safe physical execution waits for G1.

## Frozen inputs

- Complete metric semantics: `METRIC_DICTIONARY_V1.md`.
- Required event/identity chain: `EVIDENCE_SCHEMA_V1.md`.
- Source/tree/resource rules: `run_identity_schema_v1.json`,
  `inherited_change_ledger_2026-09-09.json`, and
  `protected_resource_ledger_v1.json`.
- `reachable_by_construction` remains the pre-run conservative Cartesian and
  bin-clearance label. MoveIt rejection must not reduce its denominator.
- All retained thresholds, scene distributions, physical parameters, collision
  policy, and production model candidates remain unchanged unless escalated.

The nine inherited unstaged files remain user-owned history. A later WP may
analyze or deliberately incorporate an overlapping change, but must identify
the exact inherited hunk and cannot silently credit or discard it.

## P1 interface: C-M and C-P

P1 owns WP-10 through WP-13. Its minimum evidence outputs are:

- immutable `operation_id` and `attempt_id`, explicit owner, and outcomes for
  not-started, completed, failed-recovered, failed-unrecovered, and unknown;
- action accepted/rejected and independently observed physical motion start;
- timeout/cancel/late-result handling that retains ownership while physical
  state is unknown and prevents a new operation;
- identified contact, attach, carried-payload, detach, and stable placement
  transitions, including terminal attachment inventory;
- full-interval nonallowed-collision and joint-limit monitor coverage;
- independently sampled home/stop confirmation with tolerances, velocity bound,
  settle interval, and observed samples;
- one explicit scene-terminal observation after all operation and monitor
  obligations close.

Start with call-path/side-effect inventory and adversarial unit tests. Do not
run integrated motion until ownership isolation and stop conditions can be
observed. Do not broaden allowed collisions, treat timeout as no motion, or add
a fallback execution path.

## P2 interface: C-O and C-U

P2 owns WP-20 through WP-23. Its minimum evidence outputs are:

- acquisition time and recorder receipt time, source frame, `motion_epoch_id`,
  producer sequence, and stale/out-of-order rejection reason;
- runtime track identity scoped to run + epoch, with cross-view association and
  no integer-ID reuse across epochs;
- pose validity, explicit rejection reason, quality meaning, and identified
  localization/sigma records so coverage and error samples cannot diverge;
- evidence that a selected/wrist-confirmed target is the same runtime target;
- bounded detector-development candidates only, with development/validation
  resources separated from both formal resources.

P2 may use replay and non-motion diagnostics before G1. Any wrist collection
that moves the arm waits for G1. Rejection remains in coverage/recall and cannot
be relabelled unreachable.

## Integration contract

The recorder joins P1 and P2 outputs with the v1 envelope:
`run/scenario/event/producer/sequence/event-time/receive-time/epoch/operation/
attempt/runtime-track/physical-target`. Scoring truth remains isolated from
runtime control. The required success chain is:

```text
selected -> wrist confirmed -> plan accepted -> action accepted
-> physical motion -> contact -> attached -> detached -> placed
-> action terminal -> home/stopped -> scene terminal
```

Duplicate, missing, reversed, ambiguous, cross-operation, or wrong-identity
evidence fails or becomes indeterminate according to P0; it must never be
repaired with a summary boolean.

## Required regression at each WP

- Run the WP-specific adversarial tests first, then the complete pure suite.
- Capture a run identity for every new result. Declared-dirty source must match
  the inherited ledger exactly; additional development changes must be committed
  before evidence capture.
- Report full assigned-scene and target funnel counts, not only accepted or
  attempted targets.
- Keep engineering correctness, task performance, and scientific claims as
  separate conclusions.

## Resources and stop conditions

- Generalized Formal-30 remains `PROTECTED`; no behavioral access, seed-driven
  debugging, or scoring.
- The independent 115-image real test remains `PROTECTED`; no image, label,
  prediction, or metric access.
- No rename, new output directory, or new version suffix resets a consumed
  development/qualification block.
- Stop and escalate on unknown operation/attachment state, inability to observe
  the required release condition, a needed threshold/denominator/scope change,
  or a new fallback/control route.

G0 release should therefore be **PROCEED_WITH_LIMITS**: P1/P2 contract work is
allowed; integrated motion and formal resources are not.
