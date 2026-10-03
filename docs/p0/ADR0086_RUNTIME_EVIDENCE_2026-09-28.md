# ADR 0086 runtime evidence — 2026-09-28

## Scope

This is a development-only positive runtime qualification of the ADR 0086
read-only whole-chain authorization boundary.  It is not a formal acceptance
run and does not consume a hidden/formal seed.

## Immutable receipt

- Scenario: `generalized_seed_045504` (development seed `45504`)
- Run ID: `generalized_dev_seed_45504_adr0086_positive_v3_20260928T153133Z`
- Receipt: `.codex_tmp/generalized_harvest_seed_45504_adr0086_positive_v3/runtime_probe.json`
- Permanent closure copy: `artifacts/final_evidence/ADR0086_positive_runtime_probe.json`
- Receipt SHA-256: `0900F0F55E5DF97FDA6E0CFEEF7C2B476BE644FF52C54D7C6FDE53B89040830D`
- Recorder terminal outcome: `SUCCESS`
- Wall duration: `611.508 s`

The original development receipt remains in the local run directory. The
closure pass copied the exact bytes into the tracked final-evidence directory;
the manifest verifies the same content hash. This remains development evidence,
not a formal result.

## Authorization results

| Target | Certificate | Stages | Collision scene unchanged | First later controller command |
| --- | --- | --- | --- | --- |
| 2 | `FEASIBLE` | pregrasp, approach, grasp-state, virtual-attach, escape, transport, bin-approach | yes | monotonic ns `330013194048` |
| 3 | `FEASIBLE` | pregrasp, approach, grasp-state, virtual-attach, escape, transport, bin-approach | yes | monotonic ns `604717378522` |

For both evaluation intervals, the receipt reports `payload_state=EMPTY` and
zero controller commands, gripper commands, and physical attachment events.
The qualification checker passed for each target independently; no controller
command occurred inside either virtual evaluation interval.

Both targets then reached the orchestrator's `TARGET_HARVESTED` path after
normal grasp, placement, release, verification and return stages.

## Boundary of this claim

This proves the positive virtual-scene path in the live Gazebo process.  It
does **not** prove the required negative behavior for a deliberately blocked
bin; that remains the next ADR 0086 runtime test.
