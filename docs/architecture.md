# Architecture and interface contract

This document is the T00 authority. Downstream packages may extend internal
implementation, but they must not change these contracts without a recorded
architecture decision.

## Fixed scope

The system detects `RIPE` and `UNRIPE` strawberries, localizes a ripe target
from an eye-to-hand RGB-D camera, plans with MoveIt 2, grasps a rigid simulated
fruit, and places it into a fixed collection bin.

Excluded from v1: physical robot control, stem cutting, deformable fruit,
multi-camera fusion, sim-to-real claims, and a custom motion planner.

## Package boundaries

- `strawberry_interfaces`: shared messages, action, enums, and failure codes.
- `strawberry_sim`: Gazebo world, Panda integration, RGB-D camera, fruit assets,
  ground-truth publisher, and attach/detach behavior.
- `strawberry_perception`: YOLO inference and maturity detections.
- `strawberry_localization`: robust depth projection and TF transformation.
- `strawberry_manipulation`: MoveIt planning and pick-and-place action server.
- `strawberry_bringup`: launch composition and deterministic orchestrator.
- `strawberry_benchmark`: scenarios, trial runner, logs, metrics, and reports.

## Coordinate and time contract

- All nodes use simulation time (`use_sim_time=true`).
- Distances are metres and angles are radians.
- Camera messages carry the optical frame and the same acquisition timestamp.
- Localized target poses are published in `panda_link0`.
- Stale sensor or detection data older than 0.5 simulated seconds is rejected.

## ROS surface

Standard camera topics:

- `/camera/color/image_raw` (`sensor_msgs/Image`, RGB8)
- `/camera/depth/image_raw` (`sensor_msgs/Image`, 32FC1 metres)
- `/camera/camera_info` (`sensor_msgs/CameraInfo`)

The historical `camera_mount:=fixed` path remains the default and owns those
standard topics. Non-acceptance development mode `camera_mount:=dual` removes
the world camera and publishes two isolated data planes:

- base overview: `/camera/base/*`,
  `strawberry_base_camera_optical_frame`, 320x240 at 10 simulated Hz;
- wrist precision: `/camera/wrist/*`,
  `strawberry_wrist_camera_optical_frame`, 640x480 at 30 simulated Hz.

Perception, depth, and CameraInfo input topics are explicit launch parameters.
Dual mode routes all three to the wrist namespace for precise localization
only after arm motion has settled. The base stream is supervisory and is not
fused with wrist detections or target poses.

The optional sequential-observation development path uses the base stream to
choose one bounded wrist preset, stops the base inference processes, moves with
all fruit collision obstacles loaded, and only then starts wrist inference.
Localization remains confidence-first by default. A wrist preset may opt into
a pixel-space attention ROI through four disabled-by-default bounds; an empty
ROI publishes no target rather than falling back to another fruit. The
selection transition never authorizes a pick.

The wrist measurement boundary requires 15 consecutive detection timestamps
where at least one ripe detection identity has an identity-matched TargetPose
before collecting a new 60-frame window. A schema-v2 readiness receipt records
per-timestamp detection, ripe-detection, and TargetPose identities; attributes
non-ready frames to no detection, no ripe detection, missing TargetPose, or
identity mismatch; and reports streak-reset causes. A readiness timeout fails
closed but still writes this receipt before exiting. Its TargetPose delay is a
wall-clock callback receipt offset for pipeline diagnosis, not simulated sensor
age. RGB-D localization keeps a nominal 50 ms synchronization bound and a
0.5 s freshness limit. Sequential wrist observation may opt into a 105 ms
bound only while fresh joint-state samples prove the arm stationary; generic
and moving paths remain at 50 ms. Delayed detections are retried asynchronously
inside the freshness limit so TF waits do not block sensor callbacks.
The same stationary sequential path opts into a 30-sample ROS depth/CameraInfo
history and a 30-sample wrist-depth bridge publisher queue to absorb transport
bursts. Historical paths keep a depth of five. Queueing never widens the
50/105 ms synchronization bounds or the 0.5 s freshness bound.

After the final wrist window, an optional control-side handoff Shadow receives
15 additional TargetPose samples. It verifies the selected identity, bounded
acquisition age, `panda_link0` frame, Panda stationarity, and a read-only MoveIt
scene containing the four static objects plus all three fruit. The target fruit
remains a collision object. Its MoveIt configuration removes controller and
trajectory-execution parameters, runtime requires zero instantiated controller
endpoints, and the probe has no action client or publisher. It always records
`pick_authorized=false` and stops before `/strawberry/pick_and_place`.
Target samples more than 50 ms ahead of the probe's received `/clock` are
counted and ignored before the 15-sample window; they are never admitted and a
persistent clock mismatch therefore times out fail-closed.

An optional follow-on pre-grasp planning Shadow may run only after that handoff
passes. It derives the same hand/pre-grasp geometry as the production
pick-and-place executor, freezes 15 new TargetPose samples plus 10 stationary
joint-state samples, retains all seven collision objects including the selected
fruit, and asks MoveIt for a collision-checked `panda_hand` plan. The MoveIt
configuration remains controller-free. A valid trajectory is audited for its
endpoint and then discarded; trajectory execution, action clients, command
publishers, gripper commands, and target-collision removal are prohibited.
This path always records `pick_authorized=false` and is non-acceptance evidence.

Perception and localization own SIGTERM cleanup so callbacks and executor
workers drain before their ROS context and entities are destroyed. The
sequential runner rejects asynchronous task exceptions as dirty evidence.

The ROS color topic remains `rgb8`. Ultralytics 8.4.92 interprets a NumPy HWC
source as BGR and performs its own BGR-to-RGB reversal, so the perception
adapter must request `bgr8` from CvBridge at that internal library boundary.
Passing an `rgb8` NumPy array directly to `YOLO.predict` violates the runtime
contract. See ADR 0006.

Project interfaces:

- `/strawberry/detections` (`strawberry_interfaces/StrawberryDetectionArray`)
  contains a shared header and detections with `target_id`, maturity,
  confidence, and pixel ROI.
- `/strawberry/target_pose` (`strawberry_interfaces/TargetPose`) contains target
  identity, `PoseStamped`, source confidence, and estimated position sigma.
- `/strawberry/pick_and_place` (`strawberry_interfaces/PickAndPlace`) accepts a
  target pose and place pose; result includes outcome, failure code, planning
  time, and execution time.

T60 keeps control and observation namespaces physically separate:

- `/strawberry/oracle/target_pose` is the only control input during the oracle
  integration subgate.
- `/strawberry/shadow/detections` and
  `/strawberry/shadow/target_pose` are observation-only YOLO/depth outputs.
- `/strawberry/trial_status` schema v2 records the configured source and both
  topic names. Enabling shadow mode with the same target topic as control is a
  startup error.

`ground_truth_association_enabled` in localization assigns a simulation ID to
a depth estimate; it does not replace that estimate with a truth pose and is
not the oracle control path. ADR 0026 is the explicit exception to the original
T30 control gate: the hash-bound below-gate model may control bounded simulator
integration only through the dedicated waiver launcher. Generic launch
defaults remain fail-closed, Oracle is stopped in that mode, and the original
numeric result remains failed.

Before perception may control motion, the simulator pre-gate runs with no
manipulation node and exactly one visible fruit. Five fixed camera-clear
positions are evaluated for `RIPE` and for only-`UNRIPE`, ten frames each. It
requires at least 90% ripe truth-associated frame recall, at most 5%
only-unripe false-ripe frames, and at least 90% ripe target-pose production.
This operational pre-gate is separate from real-image T30 metrics and from the
135-trial P3/P4 matrices. It did not itself authorize control; ADR 0026 records
the later project-owner engineering waiver and its narrower scope.

Maturity constants are `UNKNOWN=0`, `RIPE=1`, and `UNRIPE=2`.

Failure codes are stable and stage attributable:

1. `NO_TARGET`
2. `LOW_CONFIDENCE`
3. `DEPTH_INVALID`
4. `TF_TIMEOUT`
5. `UNREACHABLE`
6. `PLANNING_FAILED`
7. `COLLISION`
8. `GRASP_FAILED`
9. `PLACE_FAILED`
10. `STALE_DATA`

## Data flow and orchestration

The deterministic state machine is:

`INIT -> ACQUIRE -> DETECT -> LOCALIZE -> SELECT -> PLAN -> APPROACH -> GRASP -> RETREAT -> PLACE -> VERIFY -> DONE|FAILED`

Selection filters for ripe, valid-depth targets, verifies IK reachability, then
sorts by confidence descending and distance ascending. No ripe target returns
`NO_PICK` without robot motion.

Depth or TF failure triggers at most three new-frame attempts. Planning may try
one alternate approach orientation. A grasp failure opens the gripper, returns
home only when no attachment remains and recovery motion is safe, records the
first failing stage, and ends the trial. An attachment that cannot be released
is a hard stop: recovery motion is withheld so the constrained fruit cannot
damage the scene or conceal the original failure.

## Dataset and perception reproducibility contract

`tools/data` owns archive verification, safe extraction, class filtering, and
deterministic split construction. The source archive is pinned by file size and
MD5 in `data/manifests/zenodo_6126677.json`. Source classes 0 and 1 become the
v1 `RIPE` and `UNRIPE` classes; class 2 (peduncle) remains in the untouched raw
data but is excluded from generated v1 labels. Official splits are preserved
when usable. Otherwise the fixed `20260710` seed and grouped 70/15/15 policy
apply, and ambiguous numeric image sequences fail closed until an authoritative
scene-group mapping is supplied.

The materialized split manifest records SHA-256 and byte size for every source
and output image/label. Path-independent canonical SHA-256 digests bind each
split and the complete dataset to file content, filtered labels, class and
scene assignments, seed, and split membership. Training, validation, and the
formal test re-hash the materialized files and verify those canonical digests
before reading data. Verification also requires `dataset.yaml` to point exactly
to the registered `images/train`, `images/val`, and `images/test` directories.
Every supported image and split label found there must appear in the manifest;
an extra file, wrong-suffix input, or nested input directory fails closed.

`tools/perception` owns pinned YOLO11n/YOLO11s experiment definitions, hash
preflight, training, validation inference, threshold freezing, and fixed
receipts. Each formal training run claims its configured receipt and binds the
resulting `best.pt` hash to the canonical dataset identity and experiment
configuration. The single held-out-test receipt is global to the dataset, not
per model or experiment file; it is claimed before any test image is read and
remains consumed after a crash or failed metric. The YOLO11n speed baseline and
historical YOLO11s run are validation-only. Only the single
`yolo11s_640_cls_pw05_opt1` candidate can become eligible, and only after its
immutable validation decision records the three relative-improvement checks
and validation macro-F1 at least 0.85. Formal test also requires a completed
matching training receipt, unchanged weight, reproducible frozen threshold,
and the passed promotion artifact; task and input size remain fixed to
`detect` and `imgsz=640`. The held-out test itself must report macro-F1 at least
0.85 before T30 can be accepted.

## Depth localization definition

The production localization node consumes bridged RGB-D and camera-info topics
with ROS sensor-data QoS, matching Gazebo's best-effort sensor publishers. It
loads `tf2_geometry_msgs` conversion registration and transforms the projected
point from `strawberry_camera_optical_frame` through the published optical TF
chain into `panda_link0`.

Depth and camera calibration are retained in a timestamp-indexed cache bounded
to 60 samples of each type and a two-second retention horizon. A delayed
detection selects the nearest coherent depth/`CameraInfo` pair within the
configured 0.05 s synchronization tolerance, requiring matching optical frame
IDs and image dimensions, then still applies the 0.5 s stale-data limit. This
absorbs bounded detector inference latency without silently localizing against
the newest unrelated depth frame or allowing unbounded memory growth.

For the rigid 0.035 m-radius v1 fruit, central-crop median depth measures the
visible sphere surface. The configured `surface_to_center_offset_m: 0.035`
moves that surface hit exactly 35 mm farther along its Euclidean camera ray to
estimate the fruit centre; it is not treated as a simple optical-Z increment.
The T40 acceptance gate moves a simulated fruit through 100 distinct known
positions and requires all measurements, median error no greater than 15 mm,
and p95 error no greater than 30 mm.

## Simulation startup and control invariants

The Panda is rooted to the Gazebo world through a fixed joint whose synthetic
`world` parent is marked static. This prevents the complete robot model from
drifting while preserving normal arm and finger motion.

Gazebo `DetachableJoint` instances start attached. When attachment support is
enabled, the world therefore starts paused. The attachment manager repeatedly
requests detach for every configured fruit, waits until all state topics
confirm the detached state at the declared initial poses, and only then resumes
physics through the world-control service. Attach, detach, and verification
services remain unavailable until this initialization gate completes.

The deployed `gz_ros2_control` position proportional gain is `1.0`. The arm
trajectory controller requires every joint to finish within `0.05 rad` and
allows an eight-second goal-time tolerance. Direct Cartesian execution adds a
25-second wall-time margin to its duration-derived deadline, then checks the
settled joint state and actual end pose instead of accepting the trajectory
timestamp alone.

If the controller reports success but the measured Cartesian endpoint exceeds
the unchanged 10 mm tolerance, the backend may replan once from that measured
pose. A second miss, controller failure, collision, IK failure, or settle
timeout fails immediately; the correction is never an unbounded retry.

## Pick-and-place definition

The grasp point is the fruit-centre position estimated from the median valid
depth in the central 30 percent of its bounding box. The Panda hand origin is
placed 0.1054 m behind that centre along its local tool axis. The nominal
pre-grasp is a further 0.15 m back on the same axis. Before any arm motion, the
selected fruit obstacle is prepared and the gripper is explicitly opened to
0.04 m per finger.

The guarded approach lifts vertically to 0.02 m above the higher endpoint,
reorients in place, moves to the `y=-0.10 m` safe corridor in `panda_link0`,
translates along that corridor, aligns over the target, and descends to the
pre-grasp. Cartesian pose segments are no longer than 0.01 m or 10 degrees.
MoveIt checks interpolated joint states at no more than 0.01 rad spacing so a
collision between Cartesian IK waypoints cannot be skipped. The table top is
padded upward by 0.05 m as a planning-only safety margin.

All manifest fruit are represented in MoveIt by 0.035 m-radius collision
spheres. Before preparing a simulated pick, the action server requires a fresh,
complete `/strawberry/ground_truth/poses` snapshot and synchronizes every fruit
sphere to that live scene state. The expected ID set must match the immutable
manifest exactly and the snapshot must be no more than two seconds old. The
selected sphere is then updated from the action goal, which remains the
authoritative commanded target pose. Simulation truth cannot select the target
or replace a perception result.

The selected sphere remains solid throughout transit and pre-grasp; only that
sphere is removed immediately before the final straight descent to open a
contact corridor. The other fruit remain collision obstacles. After closure
and confirmed dual contact, simulation attaches the rigid fruit. The arm
retreats 0.08 m on a collision-checked straight Cartesian path, lifts above the
box walls, aligns over the fixed bin pose, descends vertically through the open
top, opens, and detaches. Success requires the fruit centre to remain inside
the bin for one simulated second. After physical recovery and the final home
motion, every post-prepare exit restores the selected sphere at its latest live
simulation position. Unknown target IDs, stale/incomplete truth, synchronization
failure, and failed restore all fail closed instead of leaving an inaccurate
planning scene or untracked contact corridor.

ADR 0012 restricts this live fruit-truth synchronization to explicitly declared
Oracle-controlled simulation diagnostics. It is collision-geometry input, not
perception evidence. A future formal perception-controlled P3/P4 run may not
use it until a separate decision freezes the allowed planning-scene source; it
can never select the target or replace the action target pose.

The 0.035 m-radius rigid fruit is grasped with a 0.025 m per-finger close
command. Controller stall is an acceptable close result, but attachment still
requires fresh dual contact; commanding zero width is intentionally forbidden.

## Contact and collision diagnostics

Raw Gazebo contact sensors on the stock left- and right-finger collision meshes
are the primary attachment evidence. The named
`panda_left_contact_pad` and `panda_right_contact_pad` links are collision-free
TF frames, so they do not introduce artificial table contacts or cross-finger
self-collisions. If raw dual contact is unavailable, a strict deterministic
fallback requires a fresh fruit pose, both pad centres within 0.038 m of the
fruit centre, and the fruit centre projected between the two pads.

Each fruit also publishes to the shared raw
`/strawberry/sim/fruit_contacts` stream. It preserves complete Gazebo collision
pairs for guard-path validation and first-contact failure attribution; it is
diagnostic only and is not an attachment shortcut.

The oracle manipulation gate uses `scripts/run_oracle_pick_gate.sh`. Every
trial starts a fresh world and ROS domain, runs one ground-truth pick, verifies
raw dual-finger contact and the five-second planning bound, checks that no
unexpected fruit collision occurred, then requires a clean launch shutdown.
The aggregate gate passes at a success rate of at least 90 percent.
