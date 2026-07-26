# Base-overview plus wrist-precision RGB-D development baseline v1

- Date: 2026-07-25
- Scope: non-acceptance development baseline
- Scene: Blender plant v2
- Launch mode: `camera_mount:=dual`
- Cooperation model: sequential observation, no multi-camera fusion

## Design

Dual mode removes the historical world-fixed camera at runtime and adds two
collision-aware RGB-D housings to the Panda description:

| Role | Parent | Optical frame | ROS prefix | Resolution/rate |
|---|---|---|---|---|
| Base overview | `panda_link0` mast | `strawberry_base_camera_optical_frame` | `/camera/base` | 320x240 at 10 simulated Hz |
| Wrist precision | `panda_hand` | `strawberry_wrist_camera_optical_frame` | `/camera/wrist` | 640x480 at 30 simulated Hz |

The base optical origin is `[-0.10, -0.42, 1.05]` m in `panda_link0` and looks
toward the Blender plant. It provides a stable view of the whole plant, fruit,
planter, work surface, and collection bin. The wrist camera retains the
settled observation pose from the wrist-camera baseline.

The two topic and TF planes remain distinct. There is no implicit remapping:
perception must explicitly use `/camera/wrist/color/image_raw`, while
localization must explicitly use `/camera/wrist/depth/image_raw` and
`/camera/wrist/camera_info`. The base view does not publish a control target,
does not overwrite the wrist target, and is not geometrically fused with it.

The base sensor was initially 640x480 at 30 Hz. A complete pick then reached
the final verification stage but exceeded the 180-second wall-clock action
timeout because both RGB-D renderers reduced the simulation real-time factor.
The base overview was therefore reduced to 320x240 at 10 Hz; the wrist stream
keeps its original precision and rate.

## Runtime health and view

The final dual-camera health sample passed for both namespaces with active
controllers, RGB, depth, CameraInfo, joint states, and all three truth streams:

| Camera | RGB rate | Depth rate | Expected resolution | Result |
|---|---:|---:|---:|---|
| Base | 3.500 wall Hz | 3.500 wall Hz | 320x240 | pass |
| Wrist | 8.462 wall Hz | 6.969 wall Hz | 640x480 | pass |

Evidence:

- `results/development/dual_camera_smoke_v3/base_runtime_health.json`
- `results/development/dual_camera_smoke_v3/wrist_runtime_health.json`
- `results/development/dual_camera_smoke_v3/base_ready_rgb.png`
- `results/development/dual_camera_smoke_v3/base_camera_tf.json`
- `results/development/dual_camera_smoke_v3/wrist_camera_tf.json`

The measured wall rates are lower than sensor update rates because Gazebo runs
slower than real time while rendering RGB-D. Both streams exceed the existing
runtime-health minimum and remain continuous.

## Wrist Shadow evidence

With the frozen engineering-waived model, threshold 0.58, ten discarded warmup
frames, all three fruit loaded as MoveIt collision obstacles, and the arm
stopped at the observation pose, the wrist namespace produced:

- ripe detection: 60/60 frames;
- confidence: 0.8450506926 in all 60 frames;
- ROI area: 6,806 pixels in all 60 frames;
- target pose: 56/60 frames, identity `strawberry_3`;
- unripe detection: 0/60 frames;
- fruit manipulation during the window: none.

Evidence:
`results/development/dual_camera_wrist_shadow_60f_v3/shadow_window.json`,
SHA-256
`2f75cfd0442c72dd5d061bdebbb5fa7934deb35e526754ac7b544ed4f3c1cb7c`.

The four missing target-pose frames are bounded RGB/depth synchronization
misses. They are fail-closed and do not show base/wrist topic cross-talk. This
result still does not solve the unripe appearance-domain failure.

The first final-compute configuration attempt is retained at
`...dual_camera_wrist_shadow_60f_v2`. Its observation planner had omitted the
fruit collision objects, swept all three fruit from the plant, and consequently
recorded 0/60 detections. The observation runner now loads obstacle IDs 1, 2,
and 3 from `scene.yaml` before planning. The fresh v3 image shows all three
fruit still mounted; v2 is defect evidence and must not be pooled with v3.

## Manipulation compatibility and retained failures

The final fresh-world Oracle compatibility run with both camera housings and
the mast in the robot collision model passed:

- complete pick/place: 1/1;
- completion time: 125.6212 wall seconds;
- planning time: 0.0480 s;
- raw dual-finger contact: pass;
- unexpected fruit contact events: none;
- clean shutdown: pass.

Evidence:
`results/development/blender_scene_v2_dual_camera_oracle_smoke_v3/summary.json`,
SHA-256
`9602676daf1f35127e8fd04b018e42808d5378250876891dde69e0cef2df930c`.

Two earlier development results are intentionally retained:

- `...dual_camera_oracle_smoke_v1`: the original two-full-resolution design
  transported the fruit to the bin but exceeded the 180-second action timeout
  before completion;
- `...dual_camera_oracle_smoke_v2`: after compute reduction, a single run saw
  raw contact on both fingers but the right contact was no longer active when
  the attachment service checked it, so it failed closed with failure code 8.

The following fresh run succeeded, but one success does not establish a
reliability rate. A future formal reliability claim requires a separately
frozen multi-trial protocol; these development runs must not be pooled into a
retrospective acceptance sample.

## Usage

Start both cameras without perception:

```bash
ros2 launch strawberry_sim sim.launch.py camera_mount:=dual
```

Run the dual health sample:

```bash
bash scripts/run_dual_camera_smoke.sh \
  results/development/dual_camera_smoke_new 229
```

Run the settled wrist Shadow window in dual mode:

```bash
bash scripts/run_wrist_observation_shadow.sh \
  results/development/dual_camera_wrist_shadow_new \
  outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt \
  230 dual
```

For a manual dual-mode system launch with perception, route all three inputs
together:

```bash
ros2 launch strawberry_bringup system.launch.py \
  camera_mount:=dual \
  start_perception:=true \
  perception_image_topic:=/camera/wrist/color/image_raw \
  localization_depth_topic:=/camera/wrist/depth/image_raw \
  localization_camera_info_topic:=/camera/wrist/camera_info \
  allow_stationary_latest_tf_fallback:=true
```

`camera_mount:=fixed` remains the default. `camera_mount:=wrist` retains the
legacy single robot-mounted camera topics under `/camera/*`.

## Boundaries and recommended next work

This work does not alter the failed T30 numeric gate, the sealed real-image
test, the consumed v1 P3/P4 outcomes, or any formal simulator claim. In
particular, it does not justify rerunning or rewriting the historical 39/135
P3 result.

The best next engineering improvement is not immediate camera fusion. Use the
base stream to choose or verify a bounded observation region, move the arm,
wait for stationarity, and then use only the wrist stream for final detection
and localization. Before any autonomous handoff between views, add:

1. an explicit finite-state transition from base overview to wrist observe;
2. a base-view visibility/ROI diagnostic across all three fruit;
3. a repeated attachment reliability diagnostic, kept separate from visual
   acceptance;
4. a second settled wrist pose if `strawberry_1` and the unripe fruit remain
   outside the current wrist view.

That observation-only increment is now implemented and measured in
`docs/dual-camera-sequential-observation-v1.md`. It covers both ripe identities
across two wrist presets and preserves `pick_authorized=false`. The final
fresh-world sequence repeat passes 300/300 wrist TargetPose frames across five
clean runs. The follow-on no-motion observation-to-control handoff Shadow is
also complete: three final-code fresh worlds pass 45/45 control-side receipts,
180/180 wrist TargetPose frames, 7/7 retained planning-scene objects, zero
observed motion, zero control commands, and clean shutdown. The controller-free
planning-only pre-grasp gate is now also complete: three fresh worlds that
reached handoff each generated one valid plan, retained all seven collision
objects, and discarded the trajectory with zero commands. One of four attempted
worlds failed earlier at wrist readiness, so conditional planning is 3/3 but
end-to-end completion is only 3/4. The next work is to instrument and stabilize
that readiness gate and improve audited perception coverage; no trajectory may
be executed while the perception metric remains below its frozen acceptance
gate.
