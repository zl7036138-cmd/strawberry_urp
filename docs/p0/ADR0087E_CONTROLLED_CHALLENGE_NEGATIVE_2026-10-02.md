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
  physical execution. Untried frozen orientations are preferred over repeated
  same-orientation translation; at most four are evaluated.
- No safety threshold was lowered and no rejected candidate moved the robot.

## Verification

The complete ROS workspace regression after these changes reports:

```text
896 tests, 0 errors, 0 failures, 0 skipped
```

ADR 0087-E remains open until a new controlled scene produces an auditable
successful non-nominal grasp, transport, placement and safe return.
