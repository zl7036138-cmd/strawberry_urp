# P0 Historical Evidence Recomputation

Status: **COMPLETE — READ-ONLY OFFLINE**  
Run identity: `p0-historical-evidence-recomputation-2026-09-09`  
Bound source: `e63473821fc8e79d830fd1d8633376281bbd8192` with the nine inherited changes exactly declared

No historical output was overwritten. No simulation, training, robot motion, or
new behavioral attempt ran. The generalized Formal-30 materialization and the
independent 115-image real test were not accessed.

## Machine evidence

- Result:
  `results/p0/historical_recomputation_2026-09-09/final_result.json`,
  SHA-256
  `4aae543bb9aa216fb90691275643b1d95d7506964e5ecc0feec57ea8b29d5bc7`,
  19,803 bytes.
- Run identity:
  `results/p0/historical_recomputation_2026-09-09/run_identity.json`,
  SHA-256
  `a5a6c05903f7a4245f0b3230b00d81cde37f7de7a7d29cb3a00ebce40c95a029`,
  2,188 bytes.
- Identity validation with file and current-source verification: `PASS`.
- The recomputation tool has a hard allowlist for exhausted legacy inputs and
  explicit denials for both protected resource prefixes.

## Legacy P3 formal simulator matrix

The old arithmetic reproduces exactly from the 165 normalized trial records:

| Item | Preserved result | P0 interpretation |
|---|---:|---|
| Positive successes | 39 / 135 (0.288889) | `FAIL`, unchanged |
| Negative safe-no-pick | 30 / 30 under the legacy protocol | Not promoted; zero-event proof is incomplete under C-E v1 |
| Old formal P3 gate | `false` | `FAIL`, unchanged |
| Result hashes | 165 / 165 verified | Valid subset of provenance |
| Condition receipt hashes | 165 / 165 verified | Valid subset of provenance |
| Condition probe hashes | 165 / 165 verified | Valid subset of provenance |
| P0 evidence status | — | `INDETERMINATE` |

The preflight receipt and schedule still match their recorded hashes. The
historical summary expects
`config/p3_formal_matrix_v1.json` at SHA-256
`51fc9f9d08f93606a68dcc5e31adc7a2d332bec0db8e6ef7e908a2246c359be0`
and 6,090 bytes; the current file at that path is
`581bb0293564273b2ef8012e3fea57ba8f802ad080bcd8a6c4ac8bace3f09145`
and 6,104 bytes. Current code reproduces the stored arithmetic, but this byte
mismatch prevents complete reconstruction of the historical identity. The old
failure is retained; no missing provenance is backfilled.

## Generalized qualification b01, seeds 46204–46208

All 65 referenced runtime/sweep files used by the five selected scenes match
their recorded hashes. The old and P0 results differ as follows:

| Measure | Old schema v1 | P0 fail-closed recomputation |
|---|---:|---:|
| Assigned runtime scenes | 5 | 5 |
| Run-safety PASS | 5 | 0 |
| Identity-correct physical placements | not independently named | 2 |
| Scenes with two harvests | 0 / 5 | 0 / 5 |
| Aggregate gate | `FAIL` | `FAIL` |

Seed 46204 ends in `RECOVERY_HOME_FAILED` and is an explicit terminal/home-stop
`FAIL`. Seeds 46205 and 46207 are coherent failure receipts but lack independent
collision, joint, home/stop, attachment-terminal, and scene-terminal coverage,
so affected safety claims are `INDETERMINATE`. Seeds 46206 and 46208 each retain
one correctly associated contact-to-placement lifecycle, but the same missing
terminal safety evidence prevents either from becoming a safe, terminally
confirmed harvest.

The reported accepted-target ratio remains a legacy diagnostic based on
`PICK_SENT`. Its raw arithmetic is 2/2, but its qualification status is
`INDETERMINATE` because planner acceptance, physical motion start, and terminal
confirmation are not independently evidenced.

## Recovery-home counterexamples 45509 and 45510

Both old scores marked `run_safety_pass=true` even though the receipts terminate
as `RECOVERY_HOME_FAILED` while their internal state histories say `NO_PICK`.
P0 classifies both as terminal/home-stop `FAIL`; the old-to-new pass count is
2 to 0. Their source/run identities remain `INDETERMINATE`, so this closes the
scorer counterexample without turning the runs into new performance evidence.

## Claim boundary

This recomputation supports only these statements:

- the old P3 performance failure is arithmetically reproducible from its
  preserved normalized records;
- the generalized b01 multi-fruit failure remains a failure;
- legacy safety PASS values based on absence of error strings or controller
  terminal summaries are not admissible under P0;
- same-count wrong-fruit, duplicated/out-of-order lifecycle, contradictory
  terminal state, missing localization samples, and unverified bindings are now
  rejected by regression tests.

It does not support a new task-performance estimate, a safe-motion claim, a
generalized Formal-30 result, a real-image result, or any conclusion about a
physical robot.
