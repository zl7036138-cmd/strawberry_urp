# ADR 0087-E controlled challenge — negative physical result (2026-10-02)

## Conclusion

The first controlled challenge is a valid safety and diagnosis result, not a
positive ADR 0087-E qualification. The deterministic search rejected
`G00`–`G11`, certified and executed `G12`, and kept certificate geometry and
the perception-derived collision scene bound through execution. `G12` could
not produce bilateral same-fruit contact. The only remaining frozen
orientations were then re-qualified: `G13` failed at approach and `G14` failed
at transport. Neither was executed.

The scene is therefore physically unsatisfiable under the complete current
gate. It must not be made to pass by lowering collision, contact, freshness,
correction-distance, or retry limits. A different controlled challenge is
required for the positive `G00 fails, Gi succeeds` proof.

## Frozen scene

- Development seed: `45504`
- Challenge obstacle: `0.3070 0.1511 0.7000 0.002 0.002 0.002`
- World: `.codex_tmp/adr0087e_challenge_20261002/worlds/generalized_seed_045504_g00_mid_x_edge_box002.sdf`
- World SHA-256: `633B37BDE874A975736E11408439D4F3EEC4AEC15F4AFF81C5433263777859E0`
- Runtime truth-isolation audit: passed in every retained run

## Evidence progression

### r7 — collision-scene identity closed

Receipt:
`.codex_tmp/adr0087e_challenge_20261002/execution_45504_g00_mid_x_edge_box002_r7_fresh_scene_wait/candidate_execution_probe.json`

SHA-256:
`260A843B175DF0758EFF4AF481EDF759D5A7B5C457880E716C720F0E46C4C455`

The action-scoped lease captured four perception obstacles without simulator
truth. Initial and contact-correction qualification both used scene signature
`0a5d30997dbda94e60ac2f5b73a7224298d72086e5077b3e84da3d91193f9e2c`.
All translated candidates preserved scene isolation but failed at transport;
there was no second physical execution.

### r8 — corrected grasp rejoined the certified escape

Receipt:
`.codex_tmp/adr0087e_challenge_20261002/execution_45504_g00_mid_x_edge_box002_r8_escape_rejoin/candidate_execution_probe.json`

SHA-256:
`BACE802CD6EEE373D3E0643B079D654A0F938B0BB2B4CB35DC6A21441EDCFE67`

`G12-C01` passed all seven copied-scene stages, received a new certificate and
executed the exact certified geometry. It remained `RIGHT_SINGLE_FRUIT` and
returned home with `payload=EMPTY`; the single reauthorization was consumed.

### r9 — contact-frame sign measured physically

Receipt:
`.codex_tmp/adr0087e_challenge_20261002/execution_45504_g00_mid_x_edge_box002_r9_contact_sign/candidate_execution_probe.json`

SHA-256:
`84FF98A67FF5E6245F69142FC5B98179BF2276C32AB5C2EE415D4DABEEEE260A`

The original right-only contact had finger half-difference about 5.8 mm. The
physically correct +local-Y translation reduced it to about 2.9 mm; the old
sign had increased it to about 8.7 mm. This fixed the hand-frame convention,
but the left finger remained pinned at the fully open 0.040 m position for the
G12 tilt, so bilateral attachment was still correctly denied.

### r10 — remaining frozen orientations exhausted safely

Receipt:
`.codex_tmp/adr0087e_challenge_20261002/execution_45504_g00_mid_x_edge_box002_r10_next_orientation/candidate_execution_probe.json`

SHA-256:
`8C2E7619B403E9272735FE144316D9670C7B02BB908014FA3EF058F26EC48581`

After the first certified G12 physical failure and safe home recovery, the
single permitted contact-driven reauthorization evaluated the untried suffix:

```text
G13 -> APPROACH_FAILED
G14 -> TRANSPORT_FAILED
```

The copied scene remained isolated, target collision was restored, the
payload remained empty, and no fallback execution was dispatched.

## Implemented safety contracts

- The 0.75-second tracked-scene freshness threshold is unchanged. Lease
  acquisition only waits, while stationary, for at most three seconds.
- A lease binds one perception-derived obstacle manifest to one target and a
  bounded action duration; simulator truth is not read.
- Contact-derived candidates keep unique geometry fingerprints and require a
  fresh ADR 0086 certificate.
- Corrected grasps rejoin the source escape waypoint, but the diagonal join is
  explicitly planned and collision-checked as part of the new certificate.
- Physical failure may trigger only one reauthorization and at most one more
  physical execution. A unique single-fruit contact is treated as stronger
  directional evidence than an untried orientation, so the bounded
  full-to-partial same-orientation translation ladder is evaluated first.
- No safety threshold was lowered and no rejected candidate moved the robot.

## Follow-up controlled challenges

Two later challenge placements proved that first-feasible search and physical
grasp contact must be judged separately.

### G03 challenge — search succeeds, physical contact remains unilateral

- Obstacle: `0.3970 0.1350 0.7000 0.012 0.034 0.002`
- World SHA-256:
  `6D3F0F70A62EDC7FE4BCC54F339D044A06F75A521BE32DBB6AA13C56D83EBF0F`
- Runtime receipt:
  `.codex_tmp/adr0087e_challenge_20261002/execution_45504_g00_g01_block_g02_open_r4_centering/candidate_execution_probe.json`
- Receipt SHA-256:
  `7BB880099EE16E7E77418913FFF4A242EE1158F1D8FC904743B90D42C72D7525`

The copied-scene search produced the intended deterministic sequence:

```text
G00 -> APPROACH_FAILED
G01 -> APPROACH_FAILED
G02 -> APPROACH_FAILED
G03 -> FEASIBLE
```

Execution used the exact `G03` certificate. Physical closure produced
`RIGHT_SINGLE_FRUIT` with a 6.536 mm measured asymmetry. The evidence-derived
`G03-C01` translation received a new seven-stage certificate and was the only
second physical execution. It reduced the asymmetry to 3.274 mm, confirming
the correction direction, but the opposite finger remained pinned at its
fully open 40 mm position. Attachment was correctly denied and recovery ended
at home with an empty payload.

### Raised G02 challenge — mirrored contact and recovery fail-closed

- Obstacle: `0.3970 0.1350 0.7800 0.012 0.034 0.002`
- World SHA-256:
  `D9B2EFBDD0A95A67AACBD29D45EFAA407630A5BF037ACC34AEAAC07CB074597B`
- Runtime receipt:
  `.codex_tmp/adr0087e_challenge_20261002/execution_45504_g00_g01_block_g02_open_z780_r1/candidate_execution_probe.json`
- Receipt SHA-256:
  `B4DC1A07B72DD9DE6CDD3A2CB6563CD1BC4403DF7184B33331D0178A88663E68`

Raising the obstacle away from the closing height changed the first feasible
candidate to `G02` after `G00` and `G01` failed. The physical result mirrored
the earlier geometry: `LEFT_SINGLE_FRUIT`, with the other finger fully open.
The subsequent recovery retreat did not complete, so the coordinator withheld
home motion and did not attempt contact reauthorization. This is the intended
fail-closed result.

These runs show that the current 10-degree tilted terminal grasps are not yet
positive ADR 0087-E evidence in this plant geometry. The next design step is
to separate obstacle-avoiding approach geometry from the final bilateral
closure orientation, or introduce a smaller bounded tilt ring, and qualify
that change before another physical run.

They also exposed an obsolete harness assumption: the execution runner
defaulted to `track_id=1`, while the same perception scene assigned the stable
ripe target `track_id=2`. The runner now defaults to automatic deterministic
stable-ripe selection (`target_id=0`); an explicit positive ID is development
override only. No simulator truth is used for this selection.

## Verification

The complete ROS workspace regression after these changes reports:

```text
896 tests, 0 errors, 0 failures, 0 skipped
```

ADR 0087-E remains open until a new controlled scene produces an auditable
successful non-nominal grasp, transport, placement and safe return.
