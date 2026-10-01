# ADR 0087-D plan-only runtime evidence — 2026-10-01

## Scope

Development-only runtime qualification of the ADR 0087 candidate boundary.
The run starts a generalized scene with `harvest_control_enabled=false`.
Neither the target selector nor the harvest orchestrator is running; the only
manipulation request is `/strawberry/qualify_grasp_candidates`.

This is not an execution claim. It proves that perception-derived candidate
geometry can reach the copied-PlanningScene evaluator and produce an
auditable, temporary certificate without moving the arm or gripper.

## Immutable receipt

- Scenario: `generalized_seed_045504` (development seed `45504`)
- Run: `adr0087d_45504_v2`
- Receipt: `.codex_tmp/adr0087d_runtime_20261001_seed45504_v2/candidate_qualification_probe.json`
- Receipt SHA-256: `108993D9C6984CD54A9E3B71AA99F4BA46BD3FB6A27A6640653AA1F150829F49`
- Truth-isolation audit: `.codex_tmp/adr0087d_runtime_20261001_seed45504_v2/truth_isolation.json`
- Process cleanup: `CLEAN`

The truth-isolation audit passed. In particular, base localization, wrist
localization, and the pick server have no subscription below
`/strawberry/ground_truth`.

## Qualification result

The perception-only probe selected stable mature track `1` from
`/strawberry/tracked_targets`:

```text
confidence        = 0.839717
position sigma    = 0.006850 m
observations      = 61
```

It generated the frozen fifteen-candidate sequence `G00`–`G14`, then the
first evaluator row certified `G00`:

```text
PREGRASP → APPROACH → GRASP_STATE → VIRTUAL_ATTACH
         → ESCAPE → TRANSPORT → BIN_APPROACH = FEASIBLE
```

The runtime receipt checker passed and records this ordering:

```text
CANDIDATE_QUALIFICATION_STARTED
  < CANDIDATE_EVALUATION_RESULT(G00, FEASIBLE)
  < CANDIDATE_QUALIFICATION_RESULT(PLAN_ONLY_CERTIFIED)
  < TARGET_COLLISION_RESTORED(target 1)
  < CANDIDATE_QUALIFICATION_CLEANUP
```

The copied-scene fingerprint before and after evaluation is identical.
The response and receipt both report `execution_dispatched=false`; controller,
gripper, and physical-attach counts during the evaluation are all zero. There
are no command, gripper, attach, contact, or holding evidence events in the
qualification interval.

## Boundary

This closes ADR 0087-D only. The `G00` result is intentionally unsurprising:
the scene's nominal grasp is feasible, so deterministic first-feasible search
stops immediately. It does not demonstrate a non-nominal recovery and it
does not authorize a later pick after collision cleanup.

ADR 0087-E remains separate: construct a runtime case where `G00` is rejected,
some later candidate is feasible, re-qualify it inside one live lifecycle, and
execute that exact authorized identity once.
