# P0 evidence foundation

This directory records facts about the inherited workspace without rewriting
history or claiming that old evidence had provenance it did not contain.

The machine-readable contract is implemented by
`strawberry_benchmark.run_identity`. A new result is admissible only when all
eight artifact bindings verify and the current Git tree is either clean or
exactly matches the inherited-change ledger. `unknown` source state is reported
as `INDETERMINATE`, never `PASS`.

Examples from an installed or source overlay:

```text
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
