# ADR 0087-E controlled challenge — positive runtime qualification (2026-10-02)

## Conclusion

ADR 0087-E is runtime-qualified for its bounded controlled-challenge scope.
With nominal and two earlier candidates blocked, the perception-driven system
selected `G03`, certified the complete seven-stage virtual chain, executed the
exact same geometry, established bilateral same-fruit contact, transported and
released the fruit in the bin, verified release, and returned home.

The strict receipt passed, the truth-isolation audit passed, process cleanup
was `CLEAN`, and the receipt is bound to the clean implementation commit
`0b6705bdbb9d089e1bab371db19383b67a717a11`.

This closes the ADR 0087-E claim that bounded candidate search can replace one
manual grasp-angle adjustment. It is not a claim of multi-fruit or formal
30-seed acceptance.

## Frozen challenge

- Development seed: `45504`
- Obstacle: `0.3970 0.1350 0.7800 0.012 0.034 0.002`
- World:
  `.codex_tmp/adr0087e_challenge_20261002/worlds/generalized_seed_045504_g00_g01_block_g02_open_z780.sdf`
- World SHA-256:
  `D9B2EFBDD0A95A67AACBD29D45EFAA407630A5BF037ACC34AEAAC07CB074597B`
- Candidate ring: 15 deterministic candidates; 5-degree first ring and
  10-degree outer ring
- Target selection: automatic deterministic stable-ripe ranking
  (`target_id=0` at the runner boundary); the selected runtime track was `2`
- Runtime simulator truth use: `false`

## Bound evidence

- Receipt:
  `.codex_tmp/adr0087e_challenge_20261002/execution_45504_small_tilt_z780_r2_bound/candidate_execution_probe.json`
- Permanent closure copy:
  `artifacts/final_evidence/ADR0087_controlled_positive_probe.json`
- Receipt SHA-256:
  `50745B32644FE9D5154D5D887F3192B88166CB185285A799187509C0FC1A7431`
- Truth-isolation receipt SHA-256:
  `2CD7D8E502766399524857B934AD18C5C0A7182D030A3538569791F5D874B0E2`
- Cleanup receipt SHA-256:
  `9B1C8DC673CF3F5147AC0C1DB28D1F3E06C3B00ABEDC1F276B185D09B9D9682F`
- Outcome: `ACTION_SUCCEEDED`
- Strict receipt validation: `passed=true`
- Cleanup: `CLEAN`
- Elapsed wall time: `212.033349888 s`

The closure manifest archives the exact probe, truth-isolation receipt, cleanup
receipt, challenge world and world materialization receipt. The expected hashes
are checked when `scripts/build_final_evidence_manifest.py` runs.

## Candidate trace

```text
G00 -> APPROACH_FAILED
G01 -> APPROACH_FAILED
G02 -> APPROACH_FAILED
G03 -> FEASIBLE
```

`G03` used `(tilt_x, tilt_y) = (0 deg, +5 deg)`. Its immutable identity was:

```text
geometry_fingerprint:
973092161252e316765cb09310b91bff2c149a3bb55f94230f634cd6c123d5eb

certificate_fingerprint:
a035adc23e86064660c6c1bf114c48951c72f0e29455701f97622d016ffa6ca2
```

The copied PlanningScene signature was unchanged before and after evaluation.
Controller commands, gripper commands and physical attachment during
qualification were all zero. The dispatch candidate ID, geometry fingerprint,
certificate fingerprint and scene signature matched the qualification result.

## Physical result

The authorized execution emitted the following successful lifecycle:

```text
PLAN
AUTHORIZED_G03
APPROACH
GRASP
CONTACT
HOLDING
RETREAT
ESCAPED
PLACE
AT_BIN
RELEASED
VERIFY
RETURN_ROUTE
DONE
```

The grasp contact classifier reported `BILATERAL_SAME_FRUIT`. The terminal
authorized result reported `payload_state=RELEASED`,
`recovery_disposition=AT_HOME`, `failure_code=NONE`, and `success=true`.
No contact-driven reauthorization was needed.

One finger's first open-result sample at the bin was below its strict measured
position target, but the payload had already been detached under the guarded
release lifecycle, release verification passed, the recorded place corridor
was reversed successfully, and the final action plus strict receipt both
passed. This observation is retained as release-actuator telemetry debt; it
does not alter the qualified candidate-search result.

## Regression

Before the bound runtime run, the complete pure suite reported:

```text
1040 tests, 0 errors, 0 failures, 3 skipped
```

No collision, contact, freshness, uncertainty, correction-distance or retry
threshold was relaxed.

