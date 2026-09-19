# P0 evidence foundation

This directory records facts about the inherited workspace without rewriting
history or claiming that old evidence had provenance it did not contain.

The machine-readable contract is implemented by
`strawberry_benchmark.run_identity`. A new result is admissible only when all
eight artifact bindings verify and the current Git tree is either clean or
exactly matches the inherited-change ledger. `unknown` source state is reported
as `INDETERMINATE`, never `PASS`.

Behavior and perception runs must bind a real model and scene/resource.
Engineering-only tests may mark only those two fields `NOT_APPLICABLE`, with an
explicit reason; the configuration, environment, runner, scorer, protocol, and
result remain mandatory file bindings.

Examples from an installed or source overlay:

```text
strawberry-evidence-contract capture-run \
  --repository-root REPOSITORY --output RESULT_DIR/run_identity.json \
  --run-id RUN_ID --run-type BEHAVIOR --purpose PURPOSE \
  --tree-state declared_dirty \
  --inherited-change-ledger docs/p0/inherited_change_ledger_2026-09-09.json \
  --model MODEL --configuration CONFIG --environment ENVIRONMENT \
  --scene-or-resource SCENE --runner RUNNER --scorer SCORER \
  --protocol PROTOCOL --result RESULT

strawberry-evidence-contract validate-run RUN_IDENTITY.json \
  --repository-root REPOSITORY --verify-files --verify-current-source

strawberry-evidence-contract validate-ledger \
  docs/p0/protected_resource_ledger_v1.json

strawberry-evidence-contract validate-dirty \
  docs/p0/inherited_change_ledger_2026-09-09.json \
  --repository-root REPOSITORY

strawberry-evidence-contract check-resource \
  docs/p0/protected_resource_ledger_v1.json \
  --resource-id formal-simulation-generalized-v1-30-scenes \
  --consumption-id PROPOSED_RUN_ID
```

The final command is a read-only preflight. At P0 it must reject both protected
formal resources. It does not grant release and never updates the ledger.

Historical evidence remains classified as `LEGACY_PARTIAL` or
`INDETERMINATE` where source, dirty diff, environment, model, configuration,
runner, scorer, protocol, or result bindings are absent. File timestamps and
directory names are intentionally not accepted as substitutes.

P0 evaluation semantics and close-out artifacts:

- `METRIC_DICTIONARY_V1.md`: frozen units, full task funnel, denominators,
  reachability reference, retained thresholds, and tri-state gate rules.
- `EVIDENCE_SCHEMA_V1.md`: event envelope, per-operation identity/physical
  lifecycle, independent-monitor obligations, and P1/P2 ownership boundaries.
- `HISTORICAL_RECOMPUTATION_DIFF.md`: non-overwriting P3, 462xx, and 455xx
  recomputation findings with a machine-result/run-identity index.
- `WP-01_COMPLETION_REPORT.md`: implementation and adversarial-test result.
- `PHASE_P0_REVIEW_PACKET.md`: the G0 decision packet.
- `NEXT_PHASE_HANDOFF.md`: approved interface boundary for P1/P2 if G0 is
  accepted.
