# Strawberry URP

[English](README.md) | [简体中文](README.zh-CN.md)

Reproducible ROS 2 simulation for strawberry maturity detection, 3D localization,
and Panda pick-and-place evaluation.

For continuation in a new Codex task, repository, or VS Code workspace, start
with [`NEW_PROJECT_HANDOFF.md`](NEW_PROJECT_HANDOFF.md). It separates the frozen
v1 release from the newer Blender v2 simulator baseline and records the exact
next steps and unresolved boundaries.

For a structured Chinese learning path covering ROS 2, Gazebo, TF, RGB-D,
YOLO, MoveIt, testing, and the matching source modules, see
[`docs/learning-roadmap-zh.md`](docs/learning-roadmap-zh.md).

The final hand-in candidate is assembled as a submission-v2 supplement rather
than overwriting the frozen P5/P6 baseline. Its authoritative narrative and
gate checklist are
[`docs/submission-report.md`](docs/submission-report.md) and
[`docs/submission-checklist.md`](docs/submission-checklist.md). The supplement
adds the field-v3 fixed-scene perception pick while preserving the failed
YOLO/P3/P4 numeric results and their original evidence.

## Project baseline

- Platform: Ubuntu 24.04 on WSL2
- Middleware: ROS 2 Jazzy
- Simulation: Gazebo Harmonic
- Manipulation: MoveIt 2 with Franka Emika Panda
- Perception: YOLO11s, `RIPE` / `UNRIPE`
- Final deadline: 2027-03-01

The repository is a monorepo. ROS packages live under `ros2_ws/src`; datasets,
trained weights, generated logs, and build products are intentionally excluded
from Git and are reproduced from tracked manifests and scripts.

## Canonical scene revision

On 2026-07-25 the default simulator scene was revised from floating coloured
spheres to the project owner's Blender plant and fruit assets. The plant now
provides leaves, inflorescences, and pedicels; three graspable fruits are
separate rigid models mounted at calyx-centre connection points. The original
tabletop scene remains an explicit compatibility fixture. This is a new
simulator baseline: the frozen P3/P4 figures below remain historical v1 results
and must not be presented as measurements of the Blender v2 scene. See
[`ADR 0036`](docs/decisions/0036-adopt-blender-plant-scene-v2.md).

Opt-in robot-mounted RGB-D configurations are available through
`camera_mount:=wrist` and `camera_mount:=dual`; the historical world-fixed
camera remains the default. Dual mode adds a low-rate base overview camera and
keeps the full-resolution wrist camera as the sole precise perception input
after a settled observation motion. Both housings participate in MoveIt
collision checking. This is sequential camera cooperation, not multi-camera
fusion. See
[`docs/wrist-camera-development-v1.md`](docs/wrist-camera-development-v1.md)
and
[`docs/dual-camera-development-v1.md`](docs/dual-camera-development-v1.md).
The follow-on observation-only coordinator now performs a same-world
base-overview to settled-wrist handoff, covers both ripe identities across two
bounded wrist poses, and explicitly keeps picking disabled. Its final
fresh-world diagnostic records 300/300 matching wrist TargetPose frames across
five clean runs after a consecutive-readiness gate and stationary-only RGB-D
timing hardening. See
[`docs/dual-camera-sequential-observation-v1.md`](docs/dual-camera-sequential-observation-v1.md).
The subsequent no-motion control-side Shadow also passes three final-code
fresh worlds: 45/45 bounded TargetPose receipts, 180/180 wrist TargetPose
frames, all seven collision objects retained, zero arm drift, zero control
commands, and `pick_authorized=false`. The controller-free planning-only
pre-grasp Shadow is also complete for three worlds that reached handoff: 3/3
plans accepted, all trajectories discarded, all target collisions retained,
and zero control commands. One of four attempted worlds failed earlier at the
wrist readiness gate, so full end-to-end repeatability is not accepted. The
readiness gate now records identity-aware per-frame status, streak-reset causes,
and callback receipt delay, including a structured receipt on timeout. This
isolated a one-frame depth transport hole in a 299/300 baseline. A bounded
30-sample wrist-depth bridge/subscription queue then produced 300/300 final
TargetPose frames across five complete fresh worlds, with 5/5 no-motion
handoffs, 5/5 discarded pre-grasp plans, and zero control commands. This remains
development evidence; the next work is longer monitoring and audited perception
improvement, not trajectory execution.

The v1/v2 fruit geometry is now resolved consistently end to end. The
hash-frozen tabletop-v1 manifest retains its schema-v1 35 mm default, while
Blender plant v2 explicitly uses 26 mm. Bringup derives localization and
collision geometry from the selected scene contract and rejects mismatched
overrides. A fresh isolated, no-motion v2
diagnostic records ripe detection/TargetPose rates of 60/60 and 60/60 for
`strawberry_1`, 60/60 and 58/60 for `strawberry_3`, and 0/60 for the unripe
`strawberry_2`. This confirms an unripe appearance-domain gap after separating
the ripe occlusion/viewpoint issue.

ADR 0037 then consumed one bounded non-formal research claim using 72 new
Blender-v2 training renders, 24 disjoint synthetic held-out renders, and the
501 consensus-corrected real training images. The fixed `last.pt@0.58` reaches
synthetic ripe/unripe F1 `0.857143/0.000000`, so ADR 0038 rejects it before
audited real validation or live qualification. The previous engineering
checkpoint remains active; no retry, threshold change, control test, or
held-out real-test access occurred.

The independent Blender-v2 100-position localization gate then exposed inward
face winding on both canonical fruit-body meshes: all 100 measurements were
present, but the pre-repair median/P95 error was `44.062873/44.537073 mm`.
ADR 0040 authorizes only a mechanical face-order correction. With identical
positions, camera, 26 mm offset, crop, retries, and thresholds, the post-repair
gate passes `100/100` with median/P95 error `4.503614/5.045998 mm` and maximum
error `5.196535 mm`. Ripe detection remains `60/60` for both isolated ripe
fruits; the unripe limitation remains `0/60`.

The Blender-v2 manipulation geometry is now scene-bound instead of reusing the
archived 35 mm-fruit constants. A frozen 24-candidate exact-mesh sweep rejects
the legacy `0.1054/0.025 m` tool/close pair and selects a `0.0964 m`
tool-centre offset with a `0.022 m` per-finger close command. A gripper-only
runtime round trip confirms raw and processed bilateral target contact,
attach/detach, fruit restore, and reopen recovery without arm motion. The
follow-on controller-free pre-grasp gate succeeds on its first planning
attempt in `0.038638462 s`, produces 24 waypoints with `0.832448 mm` endpoint
error, retains all seven collision objects, and discards the trajectory with
zero control commands.

The subsequent Oracle execution qualification preserves one mechanically
successful but evidence-incomplete v1 run, then passes the measurement-repaired
single gate and all five independent fresh-world repetitions. Every repeat
completes `PLAN, APPROACH, GRASP, RETREAT, PLACE, VERIFY, DONE`, confirms raw
and processed bilateral contact plus attach/detach, reports no unexpected
fruit contact, returns home within `9.52e-11 rad`, and shuts down cleanly.
Planning spans `0.0357-0.0777 s`. These remain non-acceptance development
gates: `pick_authorized=false`; perception-derived execution, pose variation,
occlusion/contact robustness, formal acceptance, and physical hardware remain
unqualified.

The natural-plant dual-camera v3 gate now completes the base-camera selection,
MoveIt-planned wrist observation move, wrist-camera 60/60 target window,
stationary same-world handoff, and controller-free pre-grasp plan. Its
38-waypoint trajectory ends within `0.795461 mm / 0.005676 rad` and is
discarded with zero control commands. A new geometry-aware readiness check
then blocks execution: the perceived fruit centre is `29.177346 mm` from
truth, the `5.484708 mm` cross-jaw error exceeds the qualified
`3.857179 mm` bilateral-contact margin, and the fruit lies beyond the
qualified finger axial section. This is a useful no-motion system pass, not a
pick authorization; the next bounded task is depth-region/fruit-mask
diagnosis for the natural plant view.

The mainline follow-up found that the earlier 29 mm result was primarily a
runtime parameter-routing defect, not a failed depth estimator: renaming the
dual-camera localization nodes prevented the profile's 26 mm
surface-to-centre offset from matching the YAML node name. Passing the offset
explicitly reduces the same natural-plant target error to about 4.85 mm and
passes the exact grasp-envelope check.

The first perception-derived action then exposed a separate gripper-control
fault: the `0.25 s` controller stall window could expire at the fully open
`0.040 m` position before Gazebo produced the first measured finger motion.
The repair extends that bounded window to `1.0 s`, reads live joint state when
the controller returns an empty result state, aligns reached-goal validation
with the controller's `3 mm` tolerance, and still rejects any stalled close
with less than `2 mm` measured travel. An isolated same-pose diagnostic then
records `14.138 mm` measured close travel, raw and processed bilateral target
contact, `1.909 mm` fruit displacement, no non-target contact, reopen,
collision restoration, and final home recovery.

One subsequent, exactly-once perception-derived development action completes
`PLAN, APPROACH, GRASP, RETREAT, PLACE, VERIFY, DONE`. Its wrist estimate is
`4.934 mm` from truth; cross-jaw error is `2.573 mm` inside the qualified
`3.857 mm` margin, and axial position is `100.598 mm` inside the qualified
finger section. Both fingers make raw and processed target contact, the
attachment transitions to attached and back to detached, the fruit remains in
the collection bin, the gripper reopens, and the arm returns to `ready`.
This closes the current single-scene development mainline. It is not a formal
repeatability, varied-pose, hardware, or fruit-damage qualification. See
[`ADR 0057`](docs/decisions/0057-accept-natural-plant-perception-pick.md) and
[`docs/natural-plant-perception-pick-v1.md`](docs/natural-plant-perception-pick-v1.md).

The user-supplied `st1.blend` field is now an opt-in `field-v3` scene rather
than a replacement for Blender-v2. It retains 101 instanced background plants,
places the qualified v2 workcell in one outer-row bay, and uses a base overview
camera plus wrist RGB-D camera. After passing its no-motion localization and
collision-planning gates, the exact fixed-scene runner completed three
consecutive perception-derived pick/place cycles. Every cycle records
bilateral raw and processed fruit contact, `attached -> detached`, all seven
action stages, open-gripper recovery, and return to `ready`.

Field-v3 uses two explicit single-joint gripper controllers because DART does
not enforce the Panda right-finger mimic constraint. The backend sends both
goals together and independently validates both measured finger positions.
This is fixed-target, fixed-seed, non-formal simulator evidence; it does not
qualify varied plants/poses, arbitrary fruit coverage, fruit damage, hardware,
sim-to-real transfer, or the detector's failed numeric gate. Reproduction and
evidence hashes are in
[`docs/field-v3-integration.md`](docs/field-v3-integration.md) and
[`ADR 0060`](docs/decisions/0060-accept-field-v3-perception-pick-repeat.md).

## Stage status

| Stage | Status | Exit gate |
|---|---|---|
| P0 engineering baseline | Complete | WSL2, ROS/Gazebo/MoveIt smoke tests, 30-minute stability, frozen interfaces |
| P1 simulation and data | Complete | Simulation interfaces, pinned downloads, curated 732-image split, and content contracts are ready |
| P2 module implementation | Engineering-complete with waiver | Oracle pick and T40 localization pass; ADR 0026 accepts the 0.800675 detector for simulator engineering while preserving the failed 0.85 numeric gate and sealed test |
| P3 end-to-end integration | Formal gate failed | The single 135+30 matrix is complete: 39/135 positive successes (28.89%) versus the 80% gate; all 30 negatives safely return `NO_PICK` |
| P4 robustness intervention | Complete; qualification failed | The exact simulator-adaptation candidate restores heavy ripe detection to 300/300 frames but produces 0/300 heavy target poses, so ADR 0032 rejects promotion |
| P5 release | Complete; frozen with failed P3/P4 | Clean build/smoke, charts, reports, headed demonstrations, final video, and evidence manifest are verified |
| P6 delivery | Complete | Deterministic compact archive, embedded inventory, CRC verification, and terminal handoff are ready |

Architecture and interface contracts are authoritative in
[`docs/architecture.md`](docs/architecture.md).

Latest verified local baseline (2026-07-28): all seven packages build and all
396 colcon tests pass with no errors, failures, or skips in
`Ubuntu-24.04-URP`. The earlier release reproduction in
`Ubuntu-24.04-URP-Repro` remains unchanged. The dependency-light
WSL suite separately reports 337 passes and one conditional skip. The isolated
truth-target manipulation gate passes at 90%, and the T40 localization gate
passes all 100 positions with 1.345 mm median and 1.897 mm p95 error. See
[`results/p2/oracle_gate/summary.json`](results/p2/oracle_gate/summary.json) and
[`results/p2/localization_gate_v6/summary.json`](results/p2/localization_gate_v6/summary.json).

Post-submission development (2026-08-05) adds an opt-in, geometry-guided
depth-layer estimator for occluded detections. The current source builds and
passes 417/417 colcon tests. A 60-frame field-v3 no-motion Shadow produced
60/60 target poses at 3.148 mm median error with zero joint or control activity.
Its 28.1 mm median uncertainty remains too conservative for runtime promotion;
the frozen submission-v2 receipt remains 396/396 and is not regenerated. See
[`docs/occlusion-aware-localization-v1.md`](docs/occlusion-aware-localization-v1.md)
and [`ADR 0061`](docs/decisions/0061-introduce-geometry-layer-localization-shadow.md).

A separate 40-frame identical-input matrix then exposed and preserved a false
ambiguity under 10% detector-box shrinkage. The support-aware repair passes all
eight offline checks: it accepts 40/40 smaller boxes, keeps 45%-centre-occlusion
P95 error at 5.886 mm versus 47.116 mm for the old estimator, and rejects all
40 fruit-absent observations. Runtime promotion is still blocked by limited
viewpoint coverage, synthetic rather than rendered occlusion, and conservative
sigma. See
[`docs/paired-depth-estimator-matrix-v3.md`](docs/paired-depth-estimator-matrix-v3.md)
and [`ADR 0062`](docs/decisions/0062-use-layer-support-to-resolve-depth-ambiguity.md).

The next single-run rendered matrix completed 15/15 Gazebo scenes and 300/300
paired frames with zero robot or control motion. Geometry-layer P95 error
passed at 15.432/18.755/26.962 mm for none/partial/heavy occlusion, while the
legacy heavy result was 361.727 mm. The overall diagnostic is nevertheless
`FAIL`: one no-occluder ROI contaminated the blue-colour binding, and one clear
position produced 18/20 geometry poses instead of 19/20. Runtime promotion
remains `BLOCKED`; the result is preserved without rerun. See
[`docs/rendered-occlusion-localization-matrix-v1.md`](docs/rendered-occlusion-localization-matrix-v1.md)
and [`ADR 0064`](docs/decisions/0064-preserve-rendered-occlusion-localization-failure.md).

The T60 oracle integration subgate passes 10/10 with planning p95 0.058841 s;
one 11.6 mm controller endpoint miss was recovered by the single bounded
correction while retaining the 10 mm final tolerance. The original YOLO shadow
smoke produced no ripe target because the runtime supplied RGB NumPy input to
Ultralytics' BGR NumPy path. ADR 0006 fixes that boundary. The corrected smoke
recorded 798 frames, 1,587 boxes, 1,548 ripe boxes, and 780 shadow target poses
alongside a successful oracle-controlled pick. ADR 0026 subsequently authorizes
one hash-bound below-gate model for bounded simulator control without changing
its numeric result. The first perception-controlled isolated-fruit smoke passes
1/1 with `DONE/SUCCESS`. ADR 0027's repeated fresh-world development gate then
passes all 10 ripe positive trials and all 10 only-unripe negative trials;
every negative returns `NO_PICK`, with zero false picks and zero control
attempts. Planning p95 is 0.071302 s. This establishes repeatability only at
one clear pose under the metric waiver; it did not predict the subsequently
failed formal P3 matrix. Evidence is in
[`results/t60/oracle_gate_v2/summary.json`](results/t60/oracle_gate_v2/summary.json)
and
[`results/p3/perception_repeated_dev_gate_v1/summary.json`](results/p3/perception_repeated_dev_gate_v1/summary.json).

ADR 0029 froze and ADR 0030 records the consumed formal simulator matrix. The
135 positives are the complete 3-lighting x 3-occlusion x 5-position x 3-Gazebo
seed product; the 30 only-unripe negatives retain balanced marginals. All 165
scenarios have exactly one behavioral result, valid infrastructure, clean
shutdown, and complete failure attribution. Positive success is 39/135
(28.89%), so the 80% formal P3 gate fails. Negative safety passes at 30/30
`NO_PICK`, with zero pick attempts and zero acquired unripe fruit; planning p95
is 0.068495 s. Failures are 84 perception and 12 grasp. The held-out real test
remains sealed. Evidence is bound by
[`artifacts/p3/p3_formal_matrix_outcome_handoff_v1.json`](artifacts/p3/p3_formal_matrix_outcome_handoff_v1.json).

ADR 0031 then froze the single P4 intervention: substitute the already trained
simulator-adaptation candidate at threshold 0.80 without changing assets,
localization, grasping, or motion. Its 30-scenario no-motion qualification is
complete and infrastructure-valid. Ripe detection reaches 300/300 frames in
each of nominal-none, dim-none, and nominal-heavy; unripe classification reaches
840/900 frames with zero false-ripe frames. However, nominal-heavy produces
0/300 target poses because the box-centre depth samples the foreground
occluder, placing the reconstructed point 0.345-0.366 m from the nearest fruit.
ADR 0032 therefore rejects the intervention and prohibits a post-intervention
motion matrix. Evidence is bound by
[`artifacts/p4/p4_sim_adapt_qualification_outcome_handoff_v1.json`](artifacts/p4/p4_sim_adapt_qualification_outcome_handoff_v1.json).

ADR 0033 accepts the P5 clean-environment reproduction subgate. A separate
Ubuntu 24.04.4 WSL2 distribution built all seven packages and passed 246/246
tests with the RTX 4060 available through CUDA. Its frozen release smoke then
passed 5/5 clear ripe picks and 5/5 only-unripe `NO_PICK` trials, with zero
false picks, valid infrastructure, clean shutdown, and a behaviour-rate
difference of 0.0 from the current-environment reference (limit 0.05). This
does not alter the failed P3/P4 results or authorize held-out-test access.
Evidence is bound by
[`artifacts/p5/p5_clean_reproduction_handoff_v1.json`](artifacts/p5/p5_clean_reproduction_handoff_v1.json).
ADR 0034 freezes the final P5 release. The 288.7-second H.264 video contains
an Oracle-controlled successful headed pick/place, a perception-controlled
only-unripe `NO_PICK`, all accepted module metrics, the failed P3 result, the
rejected P4 intervention, and explicit limitations. P5 delivery is complete;
P6 is limited to packaging, corrections, and delivery defects.

The T70-D2 no-motion diagnostic now covers five reachable positions under
`nominal+none`, `dim+none`, and `nominal+heavy`, with exactly 60 settled Shadow
frames per scenario. All 15 worlds and 900 frames are complete. Ripe detection
is 300/300 frames for both no-occlusion light levels; target-pose availability
is 295/300 at nominal and 277/300 at dim. Heavy occlusion produces 0/300 ripe
detections at all five positions. ADR 0018 preserves and corrects an
injection-only blue-background confound without changing these model counts.
This is one-seed diagnostic evidence with no robot motion, not P4 acceptance;
it preceded and is separate from the consumed formal matrix. See
[`results/t70/shadow_multiposition_fixed_window_v1/summary_v2.json`](results/t70/shadow_multiposition_fixed_window_v1/summary_v2.json).

ADR 0020 accepts the subsequent capture-isolation preflight: 36/36 groups and
288/288 truth-labelled renders are complete, with a balanced 216-image train
split and 72-image synthetic held-out split. Source and encoded hashes are
unique and disjoint, no T70-D2 source image is reused, and no formal seed label
is used. The canonical dataset digest is
`a8f1f157e15f6a52f637b1cc11f275f3320bd4cae156206560a7ead6d17b76b5`.
This only accepts data capture and isolation. No training has started and the
receipt intentionally keeps `training_unlocked=false` until an exact
hyperparameter manifest and one-time claim are frozen. See
[`data/processed/t70_sim_adaptation_preflight_v1/preflight_receipt.json`](data/processed/t70_sim_adaptation_preflight_v1/preflight_receipt.json).

ADR 0021 now freezes the single simulator-adaptation claim without consuming
it. The mixed derivative contains exactly 501 registered real training images
plus 216 synthetic training images, while only the 72 synthetic held-out images
are exposed as in-training validation. The 116 audited real-validation images
remain post-selection non-regression evidence and the formal test remains
sealed. The derivative canonical digest is
`aa96dbfdd5bb7b74a52b9857ee114536b30e65584feb80a34428771554928582`.
The frozen environment preflight passed and the sole claim was then consumed.
Training completed all 30 epochs and produced eight checkpoints. Synthetic-only
selection chose `best.pt` at confidence 0.80 with macro-F1 1.000, but the same
candidate failed audited real-validation non-regression at macro/ripe/unripe F1
0.626/0.754/0.497. ADR 0022 therefore rejects the candidate and forbids retry,
threshold relaxation, runtime promotion, and remaining promotion screens. The
historical `baseline__best` remains the Shadow reference.

The separate no-motion simulator perception pre-gate also passes its
camera-clear operational thresholds: 50/50 ripe frames contain a
truth-associated ripe detection, 50/50 produce a target pose, and 0/50
only-unripe frames produce a false-ripe detection. The unripe fruit itself was
not detected in any negative frame, so this result establishes simple-scene
NO_PICK safety but not class-balanced simulator accuracy. It does not authorize
perception control or synthetic fine-tuning. See
[`results/t60/sim_perception_pre_gate_v1/summary.json`](results/t60/sim_perception_pre_gate_v1/summary.json).

ADR 0010 then froze a separate five-position downstream diagnostic: Oracle
remained the only arm-control source while the hash-bound `baseline__best`
model ran on shadow topics. All five trials were infrastructure-valid and all
five produced ripe detections plus shadow target poses. Oracle manipulation
succeeded at 1/5 positions; two failures were safely attributed to contact with
non-target `strawberry_2`, and two to Cartesian IK failure at the farther
coordinates. This exposes manipulation/scene-clearance work without promoting
the below-gate detector. It is not the formal five-position definition and
cannot close P2, P3, or P4. See
[`results/t60/oracle_shadow_position_diagnostic_v1/summary.json`](results/t60/oracle_shadow_position_diagnostic_v1/summary.json).

The first single-target isolation attempt then parked both non-target fruit in
Gazebo, but exposed a separate defect: MoveIt still held their immutable
initial collision coordinates. Its 2/5 result is retained only as ghost-obstacle
defect evidence. ADR 0011 now synchronizes all fruit collision spheres from a
fresh, complete simulation-truth snapshot before motion and restores the target
at its latest live position. The corrected v2 isolation run succeeds 5/5 with
no collision or IK failures; Shadow also emits ripe detections and target poses
in 5/5. These remain non-acceptance diagnostics under Oracle control, and the
held-out real-image test remains sealed. See
[`results/t60/oracle_shadow_single_target_position_diagnostic_v2/summary.json`](results/t60/oracle_shadow_single_target_position_diagnostic_v2/summary.json).

ADR 0012 then paired those five reachable poses with two explicitly positioned
non-target fruit over approximately 134-to-30 mm minimum surface clearance. The
complete v2 run is infrastructure-valid in 5/5 worlds and Oracle succeeds 4/5;
Shadow produces ripe detections and target poses in 5/5. The 65 mm layout is
reproducibly rejected because `panda_hand` contacts `strawberry_fruit_2` at the
last Cartesian approach waypoint, while the differently aligned 30 mm layout
succeeds. ADR 0013 therefore forbids interpreting this as a scalar clearance
threshold: relative direction and height matter. This remains non-acceptance
Oracle/Shadow evidence. See
[`results/t60/oracle_shadow_multifruit_clearance_diagnostic_v2/summary.json`](results/t60/oracle_shadow_multifruit_clearance_diagnostic_v2/summary.json).

## T30 data and model status

The three pinned local artifacts are present and verified at these paths:

| File | Destination | Integrity |
|---|---|---|
| [`strawberries.zip`](https://zenodo.org/records/6126677/files/strawberries.zip?download=1) (1,485,730,857 bytes) | `data/raw/zenodo_6126677/strawberries.zip` | MD5 `db8d5dcb4b8adebf1621788373fd3031` |
| [`yolo11s.pt`](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11s.pt) (19,313,732 bytes) | `weights/yolo11s.pt` | SHA-256 `85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5` |
| [`yolo11n.pt`](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n.pt) (5,613,764 bytes) | `weights/yolo11n.pt` | SHA-256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |

The curated real-image split contains 501 train, 116 validation, and 115 sealed
test images. The historical YOLO11s run reached validation macro-F1 `0.7871`
at threshold `0.31`, with ripe/unripe F1 `0.8852/0.6890`. The single controlled
candidate, `yolo11s_640_cls_pw05_opt1`, completed 109 epochs and selected epoch
59. At its validation-optimal threshold `0.44`, macro-F1 was `0.780912`, with
ripe/unripe F1 `0.878151/0.683673`. It therefore regressed the baseline, was
not promoted, and did not authorize the once-only held-out test.

T30's original numeric gate is not accepted. ADR 0026 records an explicit
engineering waiver so P2 simulator work may proceed, while the measured
macro-F1 remains below 0.85. No formal test score exists and the global test
receipt is absent. Machine-readable evidence, including every
receipt, weight, validation, freeze, and promotion SHA-256, is in
`artifacts/perception/t30_validation_gate_summary.json`.

A validation-only label-audit packet froze 21 screening candidates across 18
images. ADR 0007 conservatively excluded all 8 reviewer disagreements. The
user explicitly approved the remaining 13 final boxes without redraw (5 ripe,
8 unripe), and they were materialized into a separate, hash-bound validation
derivative; the original labels were not overwritten. Re-evaluating the frozen
historical predictions raises macro-F1 from `0.787121` to `0.816939`. A fresh,
uniform sweep of all 26 pre-existing checkpoints selects `baseline__best` at
threshold `0.31`, with macro-F1 `0.815818` and ripe/unripe F1
`0.894309/0.737327`. This still fails the `0.85` gate. Error attribution shows
that the 30 unripe false negatives are primarily low-confidence (15) or absent
detections (9), rather than localization errors (1). No new training was
started, no test image or label was accessed, and the formal test remains
sealed. The consolidated evidence is
`artifacts/perception/t30_audited_validation_handoff_v1.json`.

ADR 0009 authorized one subsequent data-exposure intervention: every one of
the 188 training images containing an unripe box was exposed exactly twice,
creating 689 effective train entries with ripe/unripe box exposure 1971/1020.
The run completed 78 epochs and preserved 10 checkpoints. A frozen scan selects
its `best.pt` at threshold `0.54`, but macro-F1 is only `0.795696`, with
ripe/unripe F1 `0.886128/0.705263`; it regresses the audited existing-checkpoint
baseline and is rejected. No further training or formal test is authorized.
Evidence is consolidated in
`artifacts/perception/t30_opt2_outcome_handoff_v1.json`.

ADR 0024 subsequently screened all 501 registered training images. The user
completed the 34-item second review; 18 items agree and all 16 disagreements
are conservatively excluded. ADR 0025 applies only 13 consensus corrections to
a versioned derivative (5 relabels and 8 additions), preserving the source
labels and omitting the test split. The single clean-label training claim
completed 115 epochs and preserved 14 checkpoints. Frozen audited-validation
selection chooses `best.pt` at threshold `0.58`, but macro/ripe/unripe F1 is
only `0.800675/0.887064/0.714286`. The original metric-driven outcome rejected
the candidate and did not authorize retry or test access. ADR 0026 later
accepts these exact bytes at threshold 0.58 as an engineering-waived simulator
control baseline; it does not rewrite the failed metric or open the test. The
model then completed one isolated perception-controlled pick with Oracle
stopped. The bound training outcome is
`artifacts/perception/t30_train_audit_outcome_handoff_v1.json`.
The waiver and smoke evidence are
`config/p3_perception_control_waiver_v1.json` and
`artifacts/p3/perception_control_waiver_smoke_handoff_v1.json`. The repeated
development-gate evidence is bound by
`artifacts/p3/perception_repeated_dev_gate_handoff_v1.json`.

## Generalized multi-plant harvesting

The repository now includes a separate fixed-base generalization path with
seeded 1–3 plant scene generation, multi-target RGB-D tracking, deterministic
fruit/static-obstacle safety ranking, dynamic wrist views, MoveIt preflight,
and a continuous harvest orchestrator. The legacy fixed scene and historical
evidence remain unchanged.

See [`docs/generalized-harvest-v1.md`](docs/generalized-harvest-v1.md) for the
runbook, ROS interfaces, truth boundary, and the frozen 30-seed acceptance
matrix. All 472 regression tests pass. A 120-scene development-only capture,
training, and qualification route is seed-disjoint from that formal matrix,
but the current perception model does not yet pass the randomized multi-target
development gate, so the formal matrix has not been executed. No acceptance is
claimed until development-only fine-tuning and independent development
requalification succeed, all 30 one-attempt runtime receipts exist, and the
aggregate evaluator returns `overall_pass: true`.
