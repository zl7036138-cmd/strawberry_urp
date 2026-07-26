# Wrist RGB-D camera development baseline v1

- Date: 2026-07-25
- Scope: non-acceptance development baseline
- Scene: Blender plant v2
- Camera modes: historical `fixed` default plus opt-in `wrist`
- Wrist parent: `panda_hand`

## Design

`camera_mount:=fixed` preserves the historical world camera, static TF, and
existing evidence paths. `camera_mount:=wrist` removes that camera from a
temporary runtime world and loads an RGB-D sensor, housing visual, and
conservative box collision as fixed Panda child links. Robot state publisher
therefore owns the dynamic
`panda_link0 -> ... -> panda_hand -> strawberry_camera_link ->
strawberry_camera_optical_frame` TF chain. The camera remains on the existing
colour, depth, and CameraInfo topics.

The camera looks along the gripper approach direction. The historical `ready`
pose consequently observes the table rather than the plant. Wrist perception
uses an explicit settled observation pose before frame acquisition:

- hand position in `panda_link0`: `[0.28, 0.0, 0.72]` m;
- hand orientation XYZW: `[0.0, 0.9063077870, 0.0, 0.4226182617]`;
- detect and localize only after the observation motion has completed.

## Runtime evidence

The 8-second wrist-camera health sample passed with active controllers, RGB,
depth, CameraInfo, joint states, and all three truth streams:

- `results/development/wrist_camera_smoke_v2/runtime_health.json`
- `results/development/wrist_camera_smoke_v2/wrist_ready_rgb.png`

The bounded observation motion succeeded without fruit manipulation:

- planning time: `0.056549196` s;
- execution time: `8.235403072` s;
- `results/development/wrist_observation_smoke_v1/observation_motion.json`;
- `results/development/wrist_observation_smoke_v1/observation_rgb.png`.

At the observation pose, the frozen engineering-waived model and threshold
0.58 produced:

- ripe detection: 60/60 frames;
- confidence: 0.848081;
- detector ROI: 7392 px²;
- target pose: 53/60 frames;
- associated identity: `strawberry_3`;
- unripe detection: 0/60 frames.

Evidence:
`results/development/wrist_observation_shadow_60f_v2/shadow_window.json`.
Compared with the fixed canonical camera, wrist observation materially enlarges
the detected ripe fruit and recovers the second ripe identity that was hidden
in the fixed view. It does not solve the unripe appearance-domain failure.

Dynamic TF may lag the image timestamp under GPU inference load. Localization
therefore remains fail-closed by default. Wrist observation explicitly opts
into a latest-TF fallback only when a recent five-sample Panda joint window is
stationary, its maximum joint delta is at most 0.002 rad, and TF age is at most
5 seconds. No fallback is allowed while the arm is moving.

## Manipulation compatibility

One Oracle-controlled pick and place with the wrist housing included in the
MoveIt robot collision model passed:

- complete pick/place: 1/1;
- raw dual-finger contact: passed;
- unexpected fruit contact events: none;
- planning time: 0.1254 s;
- clean shutdown: passed.

Evidence:
`results/development/blender_scene_v2_wrist_oracle_smoke_v1/summary.json`.
This establishes mechanical compatibility only and is not visual acceptance.

## Usage

Start the wrist camera without perception:

```bash
ros2 launch strawberry_sim sim.launch.py camera_mount:=wrist
```

Run the bounded observation and Shadow window:

```bash
bash scripts/run_wrist_observation_shadow.sh \
  results/development/wrist_observation_shadow_60f_v3 \
  outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt \
  231
```

The fixed camera remains the default when `camera_mount` is omitted.

## Boundaries and next step

These runs do not alter the failed T30 gate, v1 P3/P4 outcomes, formal test
seal, or formal simulator claims. The next useful wrist-camera task is a
two-view settled observation diagnostic: retain the current centre view for
`strawberry_3`, add a second bounded view for `strawberry_1` and
`strawberry_2`, and fuse only target poses transformed into `panda_link0`.
Do not run inference during arm motion and do not train a new model before the
two-view visibility result is known.

An additional base-overview plus wrist-precision hardware mode was implemented
after this baseline. Its isolated topics, compute budget, results, and explicit
no-fusion boundary are documented in
`docs/dual-camera-development-v1.md`.
