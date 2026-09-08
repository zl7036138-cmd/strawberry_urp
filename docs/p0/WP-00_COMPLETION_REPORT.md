# WP-00 Completion Report

Status: **SELF-ACCEPTED**  
Scope: baseline identity and protected-resource accounting only  
Behavior change: **none**

## Baseline recovered

- Audited source: `codex/wrist-readiness-diagnostics` at
  `6f25868b0d946f7ce1a73206d3676889121f6712`, 35 commits ahead of its
  upstream at the time of capture.
- P0 work continues on `codex/p0-evidence-foundation` from the same commit.
- The same nine inherited, unstaged tracked changes remain byte-bound in
  `inherited_change_ledger_2026-09-09.json`. They were neither staged nor
  credited to WP-00.
- Build/install directories exist, but the existing install identity is only
  `PARTIAL`: the symlink overlay resolves much source live, while generated
  products have no prior commit-bound clean-build receipt.
- The production launch declares `weights/yolo11s_640_best.pt`, which is absent.
  Recent evidence used the separately fingerprinted generalized v2 candidate;
  therefore no model is inferred from the launch default.

## Contract delivered

`strawberry_benchmark.run_identity` now provides:

- exclusive creation of a result identity sidecar;
- mandatory bindings for source, model, configuration, environment,
  scene/resource, runner, scorer, protocol, and result;
- file size and SHA-256 verification within the repository;
- exact clean/declared-dirty/unknown tree classification;
- byte-level verification of all nine inherited changes;
- `INDETERMINATE`, never `PASS`, for unknown source state;
- protected-resource validation and read-only access preflight;
- rejection of undeclared dirty files, changed bound files, protected access,
  duplicate consumption, exhausted budgets, and unreleased formal resources.

Engineering-only tests may explicitly mark only model and scene/resource as
`NOT_APPLICABLE`; behavior and perception runs cannot use that exemption.

## Resource status

- Historical P3 formal matrix: `EXHAUSTED`; result remains a failed historical
  evaluation and has legacy-incomplete provenance under the new schema.
- Generalized 30-scene formal matrix: `PROTECTED`; materialized, but no
  behavioral consumption found.
- Independent 115-image real test: `PROTECTED`; P0 inspected split counts and
  canonical hash metadata only, not images, labels, predictions, or metrics.
- Qualification blocks 460xx, 461xx, and 462xx: `EXHAUSTED`.
- Development blocks 451xx through 455xx: `EXHAUSTED` for fresh-qualification
  purposes and may not be renamed into new resources.

## Verification

- Pre-change scorer regression snapshot: `19 passed`.
- Final `strawberry_benchmark` package suite: `74 passed`.
- New identity/resource tests: `10 passed`.
- Actual inherited-dirty validation: `PASS`, 9 declared and 0 unexpected paths.
- Resource ledger structural validation: `PASS`, with 9 explicit
  `LEGACY_INCOMPLETE` warnings.
- Generalized formal access preflight: expected `FAIL` (not released).
- Real-image formal access preflight: expected `FAIL` (not released).
- No simulation, training, robot motion, or formal evaluation was started.

The validation receipt is
`results/p0/evidence_contract_validation_2026-09-09/result.json`
(`sha256:3d1e33679646670dc7afee7c0bcad1a0a0a220209ac279af2ae161c9c7827d44`).
Its independently verifiable sidecar is
`results/p0/evidence_contract_validation_2026-09-09/run_identity.json`
(`sha256:d8e92d57609b9d221ab28aabdd62dae2c1448f39f28d76fb1ac754d4397df505`).

## Acceptance decision

WP-00 acceptance criteria are met for P0 implementation work: the current
baseline is recoverable, inherited changes and protected resources are explicit,
new results have a fail-closed identity mechanism, and no robot behavior changed.

The following are preserved as facts rather than silently repaired:

- historical results without complete sidecars remain `LEGACY_PARTIAL` or
  `INDETERMINATE`;
- installed-product provenance remains `PARTIAL` until the later clean-build
  gate;
- the repository still lacks one unique production model default.

Proceed to WP-01. These limitations must not be converted into PASS by a scorer.
