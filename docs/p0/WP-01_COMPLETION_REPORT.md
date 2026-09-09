# WP-01 Completion Report

Status: **SELF-ACCEPTED; READY FOR G0 REVIEW**  
Scope: evaluation semantics, scorer validation, and offline historical
recomputation only  
Behavior change: **none**

## Delivered contract

`METRIC_DICTIONARY_V1.md` freezes experimental units, complete scene/target
denominators, the nine-layer task funnel, retained thresholds, scene completion,
and tri-state `PASS` / `FAIL` / `INDETERMINATE` semantics. In particular:

- `reachable_by_construction` is the pre-run conservative Cartesian/bin-
  clearance construction label, not a MoveIt outcome;
- `PICK_SENT` is a command diagnostic, not proof of planner acceptance or
  physical motion start;
- selected track, contacted fruit, and placed fruit must agree per operation;
- zero safety events pass only with full independent monitor coverage;
- localization median/P95 and coverage are reported together; sample arrays
  must correspond to identified expected samples;
- simulation and the independent real-image line cannot substitute for one
  another.

`EVIDENCE_SCHEMA_V1.md` defines the common event envelope and causal operation
chain. It also fixes C-M/C-P ownership for operation and physical state, C-O/C-U
ownership for observation/identity/quality, and C-E/C-V validation/binding
responsibilities. It records the current v3 recorder gaps instead of inventing
missing events.

## Scorer changes

The development scorer now emits schema v2 evidence states and:

- rejects `RECOVERY_HOME_FAILED` and contradictory terminal/history outcomes;
- reconstructs ordered contact-to-placement lifecycles;
- checks runtime-track-to-scene mapping against the actual physical fruit for
  each harvest, closing same-count/same-maturity wrong-fruit PASS;
- rejects duplicate, omitted, reversed, unknown-fruit, and maturity-conflicting
  physical events;
- reports identity-correct physical harvests separately from controller-reported
  harvested tracks;
- marks collision, joint-limit, home/stop, scene-terminal, planner-acceptance,
  and pick-start claims `INDETERMINATE` when independent evidence is absent;
- prevents raw legacy ratios from becoming qualification PASS values.

The generalized formal scorer now requires a verified BEHAVIOR run identity and
exact bound scene, scorer, protocol, and result files. It verifies source,
model, configuration, and file hashes, rejects swapped results, contradictory
outcomes/scene completion, inconsistent harvest counts, and selectively omitted
localization/sigma samples. It reports the retained 15 mm median as well as the
30 mm P95. Existing schema-v2 aggregate inputs are explicitly
`LEGACY_AGGREGATE_INDETERMINATE`: their arithmetic remains visible, but coherent
self-reported counts cannot qualify without the v1 event evidence chain.

The generalized claim schema is now v2 and binds ledger fingerprint,
`resource_id`, and `consumption_id`. Protected/unreleased use, exhausted new
consumption, wrong resource, and same-ID resume without `--resume` fail closed.

## Required adversarial cases

| Case | Result |
|---|---|
| Recovery-home failure previously safety-PASS | `FAIL` |
| Same-maturity wrong physical fruit with equal counts | `FAIL` |
| Missing localization or sigma samples | rejected |
| Contradictory `scene_complete` / outcome / counts | rejected |
| Duplicate, missing, or out-of-order physical lifecycle | `FAIL` |
| Wrong maturity/target binding | `FAIL` |
| Plausible but unverified hash/run identity | rejected |
| Swapped result after identity capture | rejected |
| Missing independent physical state | `INDETERMINATE`, never PASS |
| Protected or duplicate formal resource use | rejected |

## Historical recomputation

The read-only tool `scripts/recompute_p0_evidence.py` allowlists only exhausted
legacy inputs and explicitly rejects both protected resource prefixes. Its
machine output and run identity are indexed in
`HISTORICAL_RECOMPUTATION_DIFF.md`.

- Legacy P3 arithmetic reproduces exactly: 39/135 positive successes; old gate
  remains failed. All 165 result, condition-receipt, and condition-probe hashes
  verify. Its same-path matrix fingerprint has drifted, so complete P0
  provenance remains `INDETERMINATE`.
- Qualification 46204–46208 remains 0/5 dual-fruit scenes and failed. The five
  old run-safety PASS values become zero P0 passes; two correctly associated
  placements remain diagnostic but lack terminal safety proof.
- Development 45509/45510 change from two old safety passes to two explicit
  terminal/home-stop failures.

No old score was overwritten or used to tune production behavior.

## Verification

- Complete dependency-light repository suite: **795 passed**, 0 failed, 0
  skipped.
- Benchmark + bringup pytest regression: **229 passed**.
- Protected-input boundary tests: **4 passed**.
- Historical run identity capture and capture-time file/current-source
  verification: `PASS`.
- Inherited dirty validation: `PASS`, exactly 9 declared and 0 unexpected
  paths before and after the work.
- No simulation, training, robot motion, or formal evaluation was run.

## Acceptance and remaining gaps

WP-01 acceptance criteria are met: all named scorer counterexamples fail or
become explicitly indeterminate, historical changes are explained, and no old
score was preserved by weakening validation.

The current recorder cannot yet produce a qualifying chain. Independent
action/motion, collision/joint, attachment, home/stop, and scene-terminal
evidence must be added by P1/P2. The generalized schema-v2 aggregate adapter is
temporary and may diagnose but never qualify; it must be replaced before G4.
These are visible next-phase obligations, not P0 evidence PASS claims.
