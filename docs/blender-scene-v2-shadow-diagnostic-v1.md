# Blender scene v2: 60-frame Shadow diagnostic

- Date: 2026-07-25
- Scope: non-acceptance development diagnostic
- Scene: canonical `strawberry_orchard.sdf`
- Camera: fixed canonical RGB-D camera
- Motion: disabled; no Oracle provider, MoveIt, orchestrator, or attachment
- Model: `yolo11s_640_train_audit_v1/best.pt`
- Model SHA-256:
  `e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70`
- Confidence threshold: 0.58
- Window: 10 discarded warm-up frames followed by exactly 60 measured frames

## Result

The canonical scene contains two ripe fruits (`strawberry_1`,
`strawberry_3`) and one unripe fruit (`strawberry_2`).

| Observation | Frames | Rate |
|---|---:|---:|
| Any detection | 60/60 | 100% |
| At least one ripe detection | 60/60 | 100% |
| At least one unripe detection | 0/60 | 0% |
| Shadow target pose | 60/60 | 100% |

Every measured frame contains exactly one ripe box and no unripe box. The ripe
confidence is constant at `0.652302` in the static scene. Every associated
target pose has `target_id=1`; the second ripe fruit and the unripe fruit are
therefore not recovered in this window.

This is a material improvement over the earlier zero-ripe simulator failure,
but it is not a perception acceptance result. In this one static viewpoint, the
observed per-frame object coverage is one of two ripe fruits and zero of one
unripe fruit. The result does not measure lighting, viewpoint, occlusion, or
position robustness and does not alter the failed T30 numeric gate.

## Evidence and reproduction

- Runner: `scripts/run_blender_scene_v2_shadow_window.sh`
- Result:
  `results/development/blender_scene_v2_shadow_60f_v1/shadow_window.json`
- Result SHA-256:
  `31d72ea9ec1e0d859cd6923d4b947d9016a6281670b3f7d38658ddd511169ec5`
- Canonical world SHA-256:
  `aeb61d02ee7fe79cccd97bba34e43b9a9319cd66cfb45c91a6e6cd8929c8b487`
- Scene manifest SHA-256:
  `119bf72d545e8325b96af1522cf0d570b81200ce4d3a8808d519968ce0ddcb88`

Run from the repository root in Ubuntu 24.04 WSL:

```bash
bash scripts/run_blender_scene_v2_shadow_window.sh
```
