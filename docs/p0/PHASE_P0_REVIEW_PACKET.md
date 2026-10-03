# PHASE REVIEW PACKET

Phase: P0 — Baseline and Evaluation Foundation  
Requested decision: **PROCEED_WITH_LIMITS**  
Base approved gate: Master Plan G0  
Candidate identity:

- source commit: `53909fa4a25881d7fd68d0eb65bf9221094a0069`
- working tree: declared dirty
- inherited changes: exactly 9, byte-bound by
  `inherited_change_ledger_2026-09-09.json`; not staged or credited to P0
- model/config/environment manifest: baseline manifest + protected-resource
  ledger + environment identity; no behavior model executed
- scorer/protocol versions: development score v2; generalized summary v3 over
  legacy input v2; metric dictionary v1; evidence schema v1; run identity v1

Completed WPs:

| WP | Commit(s) | Acceptance |
|---|---|---|
| WP-00 | `b81eb39`…`fa24a7f` | SELF-ACCEPTED |
| WP-01 | `71fb177`, `e634738`, `9e1c730`, `53909fa` | SELF-ACCEPTED |

Contract and architecture delta:

- Changed: C-V artifact/resource binding; C-E units, denominators, tri-state
  gates, per-operation identity, evidence completeness, and formal claim guard.
- Preserved: all production behavior, models, thresholds, scene distribution,
  physics, motion parameters, and nine inherited changes.
- Removed: no runtime mechanism. New runtime branches/parameters: none.

Engineering correctness:

- Required tests: 795/795 pure tests; 229/229 benchmark+bringup pytest; 4/4
  protected-path tests.
- Adversarial cases: recovery failure, wrong ripe fruit, duplicate/missing/order
  errors, sample omission, contradictory completion, swapped/unverified binding,
  unknown physical state, and protected/duplicate resource access all closed.
- Invariant: missing or contradictory identity/physical evidence cannot PASS.
- Unresolved: v3 recorder lacks independent motion, collision/joint,
  attachment, home/stop, and scene-terminal producers; it is intentionally
  unable to qualify until P1/P2.

Scientific evidence:

- Hypothesis: none; P0 makes no new performance claim.
- Historical P3: 165/165 results and 165/165 receipt/probe hashes verify; old
  39/135 positive result and failed gate reproduce. Current same-path matrix
  hash differs, so provenance is `INDETERMINATE`.
- Generalized b01: 5 assigned/5 terminal; old and new gate both FAIL (0/5
  dual-fruit). Two identity-correct placements remain diagnostic; terminally
  confirmed count is unknown. Old safety PASS changes 5→0.
- Wrong-target/unripe/collision zero claims are not upgraded without monitor
  coverage. 45509/45510 are explicit recovery/home-stop FAIL.
- Supports: scorer integrity and limits of old evidence. Does not support:
  improved task performance, safe motion, Formal-30, real-image, or robot claims.

Failures and assumptions:

- Production launch model default remains absent; installed provenance remains
  partial. Neither was hidden or used.
- Project 15 mm median is frozen in metric v1 although the protected formal
  matrix serializes only P95; a G4 protocol must bind both.
- Budget: zero new behavior attempts, zero training, zero formal-resource use.

Regression and complexity:

- Previous P3 FAIL and b01 FAIL are preserved; only invalid legacy safety PASS
  claims changed.
- Added validation/status fields and one read-only recomputation tool; no
  control mechanism. Legacy v2/v3 adapters expire at G4.

Protected resources:

- Formal-30: `PROTECTED`, not accessed behaviorally.
- Real-test 115: `PROTECTED`, metadata only; images/labels/predictions/metrics
  not accessed.
- Legacy P3 and 462xx: `EXHAUSTED`; read-only recomputation only.
- Access-policy exceptions: none.

Evidence index: `baseline_manifest_2026-09-09.json`;
`METRIC_DICTIONARY_V1.md`; `EVIDENCE_SCHEMA_V1.md`;
`HISTORICAL_RECOMPUTATION_DIFF.md`; 795-test console receipt; bound machine
result/run identity under `results/p0/historical_recomputation_2026-09-09/`.

Proposed next phase: allow P1 and P2 investigation/interface implementation in
parallel under `NEXT_PHASE_HANDOFF.md`. Do not allow integrated motion based on
unknown state and do not release either formal resource. Explicit G0 decision
is required.
