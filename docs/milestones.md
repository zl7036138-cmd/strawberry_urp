# Milestones and gates

| Gate | Target date | Status | Evidence required |
|---|---:|---|---|
| P0 engineering baseline | 2026-07-31 | Accepted early | Ubuntu 24.04 WSL2, GPU, ROS/Gazebo/MoveIt smoke, `colcon test`, frozen interfaces |
| P1 simulation and data | 2026-08-31 | Complete | RGB-D/TF/ground truth topics, scene assets, pinned downloads, and the verified deterministic 501/116/115 split are ready |
| P2 module gates | 2026-10-15 | Engineering-accepted with ADR-0026 waiver | Oracle pick and T40 localization pass; detector macro-F1 is 0.800675 rather than 0.85, but the owner accepts it for simulator engineering only and the held-out test remains sealed |
| P3 integration | 2026-11-30 | Formal gate failed | The single matrix completed all 165 scenarios; positive success is 39/135 (28.89%) versus 80%, while all 30 negatives safely return `NO_PICK` |
| P4 robustness | 2027-01-15 | Single intervention complete; qualification failed | Heavy ripe detection improves to 300/300, but heavy target-pose output is 0/300; ADR 0032 rejects promotion and closes P4 intervention work |
| P5 release | 2027-02-15 | Complete | ADR 0034 freezes metrics, separate clean reproduction, report assets, headed demonstrations, 4:48 video, and final manifest while preserving failed P3/P4 outcomes |
| P6 delivery | 2027-03-01 | Complete | ADR 0035 accepts the deterministic compact archive, embedded inventory, CRC receipt, and terminal handoff |

A gate is accepted only from reproducible commands and machine-readable output.
The current oracle evidence is `results/p2/oracle_gate/summary.json`; all ten
trials used fresh worlds and distinct ROS domain IDs.

The accepted T40 evidence is
`results/p2/localization_gate_v6/summary.json`: 100 valid measurements from 100
positions, 1.3449193778 mm median error, 1.8965995386 mm p95 error, and
2.0236130149 mm maximum error. Both localization thresholds pass. This schema
v2 artifact records automatically selected empty ROS domain 113 and 19 SHA-256
fingerprints spanning localization, simulation, and the gate runner.

T30 does not satisfy its original numeric gate. The dataset, deterministic 501/116/115 split, YOLO11n
speed baseline, and both YOLO11s training runs are complete. The historical
YOLO11s validation macro-F1 is 0.7871 at threshold 0.31. The single controlled
`cls_pw=0.5` Opt1 run completed 109 epochs, selected epoch 59, and obtained
macro-F1 0.780912 at threshold 0.44, with ripe/unripe F1
0.878151/0.683673. It failed the predeclared promotion checks and the 0.85
authorization gate. It was not promoted; the held-out test was not run and its
global receipt does not exist. See
`artifacts/perception/t30_validation_gate_summary.json` for the bound evidence.

The validation-only label audit is resolved under ADR 0007. All 8 reviewer
disagreements remain `AMBIGUOUS_EXCLUDE`. The user approved the 13 agreed final
boxes without redraw (5 ripe and 8 unripe), and a separate audited validation
derivative was materialized without overwriting the original labels. Applying
the frozen prediction bundle to the audited labels gives macro-F1 0.816939.
ADR 0008 then froze and evaluated all 26 pre-existing checkpoints under one
uniform inference contract. `baseline__best` remains the winner at threshold
0.31 with macro-F1 0.815818 and ripe/unripe F1 0.894309/0.737327. The small
difference from 0.816939 is due to fresh uniform inference rather than the
historical prediction bundle. Both results fail the 0.85 gate. No new training
was authorized, no test image or label was accessed, and T30 remains
unaccepted.

ADR 0009 subsequently authorized exactly one unripe-exposure training run. It
duplicated each of the 188 unripe-containing training images once while keeping
the baseline model, seed, schedule, and augmentations fixed. The run stopped
normally after 78 epochs and preserved 10 checkpoints. Their frozen audited
validation scan selects `best.pt` at threshold 0.54 with macro-F1 0.795696 and
ripe/unripe F1 0.886128/0.705263. This regresses the audited baseline, so Opt2
is rejected. Formal-test eligibility is false and the held-out test remains
sealed.

ADR 0024 then screened all 501 training images, and ADR 0025 finalized the
34-item two-reviewer packet. Eighteen items agree, sixteen disagreements are
conservatively excluded, and only thirteen consensus changes are materialized
in a source-preserving derivative. The one clean-label retraining completed 115
epochs and preserved 14 checkpoints. Its frozen scan selects `best.pt` at
threshold 0.58 with macro/ripe/unripe F1
`0.800675/0.887064/0.714286`. This also regresses the audited baseline and is
rejected by the original promotion rule. The test remains sealed. ADR 0026
subsequently records an engineering waiver over the exact `best.pt` bytes at
threshold 0.58: T30/P2 are operationally accepted with waiver for simulator
integration, while the numerical failure remains explicit.

The waiver-controlled P3 smoke at
`results/p3/perception_control_waiver_smoke_v3/summary.json` passes 1/1. Oracle
was stopped, `/strawberry/target_pose` was the isolated control source, the
state machine reached `DONE/SUCCESS`, planning took 0.038362 s, execution took
69.874016 s, and shutdown was clean.

ADR 0027's follow-on result at
`results/p3/perception_repeated_dev_gate_v1/summary.json` passes its frozen
non-formal gate. Ten positive fresh worlds complete `DONE/SUCCESS`; ten
only-unripe fresh worlds complete `NO_PICK`, with zero false picks and zero
control attempts. All worlds, routing checks, liveness checks, and shutdown
checks pass, and positive planning p95 is 0.071302 s. This is one clear pose
under the ADR-0026 engineering waiver, not the formal 135+30 matrix, robustness
acceptance, real-image evidence, or sim-to-real evidence.

ADR 0029 froze the formal simulator contract, and ADR 0030 records its consumed
outcome. The authoritative v2 preflight bound 135 positive and 30 negative
scenarios, unique ROS domains 32-196, deterministic interleaving, balanced
negative marginals, and Gazebo RNG seeds `20260710/11/12` passed through
`gz sim --seed`. All 165 scenarios completed with one behavioral attempt each,
valid infrastructure, clean shutdown, and no unattributed failure. Positive
success is 39/135 (28.89%) rather than the required 80%; safe negative
`NO_PICK` is 30/30, with zero negative motion or fruit acquisition. Planning
p95 is 0.068495 s. The 96 positive failures divide into 84 perception failures
and 12 grasp failures. The formal P3 gate therefore fails and the matrix cannot
be rerun selectively. The held-out real-image test remains sealed.

ADR 0031 freezes the sole P4 intervention to the exact pre-existing
simulator-adaptation checkpoint at threshold 0.80. Its one-claim no-motion
qualification completes 30/30 scenarios and 1,800 settled frames with valid
infrastructure. The model produces ripe detections in all 900 ripe frames and
correct unripe detections in 840/900 frames with no false-ripe frame. It also
produces 591/600 no-occlusion target poses, but 0/300 heavy-occlusion target
poses. Logs show the center-depth point is 0.345-0.366 m from the nearest fruit,
past the 0.080 m association limit, because it lies on the foreground occluder.
ADR 0032 rejects the intervention, forbids a second P4 fix or post-intervention
motion matrix, and moves the project to P5 reporting.

ADR 0033 accepts the P5 clean reproduction subgate. The separate
`Ubuntu-24.04-URP-Repro` distribution builds seven packages and passes all 246
tests. Its ten-trial release smoke passes 5/5 clear ripe positives and 5/5
only-unripe negatives, with zero false picks and an absolute behaviour-rate
difference of 0.0 from the reference (maximum 0.05). Formal P3 and P4 remain
failed and the real-image test remains sealed. ADR 0034 subsequently accepts
P5 delivery. The positive headed demonstration completes `DONE/SUCCESS` under
Oracle control; the only-unripe demonstration completes `DONE/NO_PICK` under
the ADR-0026 engineering waiver with no control attempt. Their 288.7-second
H.264 final video and receipt are bound into `p5_release_v1`. P6 is now
defect-only packaging and delivery.

The T60 oracle integration subgate is accepted from
`results/t60/oracle_gate_v2/summary.json`: 10/10 successful fresh-world trials
in ROS domains 210–219, complete state histories, explicit oracle source
isolation, clean shutdowns, and 0.058841 s planning p95. The preceding v1
result remains preserved at 8/10. The pre-fix YOLO shadow smoke received 837
detection frames and 99 boxes while oracle control completed successfully;
all boxes were classified UNRIPE and no shadow target pose was emitted. ADR
0006 identified an RGB/BGR adapter defect. After correction, the v3 smoke
received 798 frames, 1,587 boxes, 1,548 ripe boxes, and 780 shadow target poses
while retaining successful oracle control. This repairs the shadow path but
does not accept T30 or the full P3 gate.

The follow-on no-motion simulator perception pre-gate at
`results/t60/sim_perception_pre_gate_v1/summary.json` passed its camera-clear
operational thresholds: 50/50 ripe frames had a truth-associated ripe box,
50/50 emitted a target pose, and 0/50 only-unripe frames emitted a false-ripe
box. The visible unripe fruit was not detected in any of its 50 frames, so this
does not establish class-balanced simulator maturity accuracy. Synthetic
fine-tuning remains deferred until a broader simulator gate demonstrates a
need; T30 and perception-controlled P3 remain unaccepted.

ADR 0010 authorizes continued downstream work only through an Oracle-control,
YOLO-shadow position diagnostic. Its v1 run is complete and source-isolated in
all five fresh worlds. The baseline shadow path produces ripe detections and
target poses in 5/5 scenarios, while Oracle manipulation succeeds in 1/5. Two
failures are non-target-fruit collisions and two are Cartesian IK failures.
These are diagnostic bottlenecks, not formal P3/P4 results: the positions are
diagnostic coordinates under nominal lighting with no occlusion, perception
never commands motion, and the real held-out test remains sealed.

The subsequent single-target v1 run parked both non-target fruit in Gazebo but
left their MoveIt collision spheres at the immutable initial poses. Its 2/5
result and three reported `strawberry_fruit_2` collisions are retained as
ghost-obstacle defect evidence, not reachability evidence. ADR 0011 requires a
fresh, complete simulation-truth snapshot to synchronize all MoveIt fruit
obstacles before motion. The corrected v2 run records that synchronization in
all five worlds and succeeds 5/5, with no failure attribution; Shadow produces
ripe detections and target poses in 5/5. This calibrates five camera-clear
single-target reachability candidates only. T30, P2, full P3, and P4 remain
unaccepted.

ADR 0012 pairs those candidates with two explicit non-target poses. The first
run is aggregate-incomplete because its fifth world had a pose-service timeout;
the bounded-retry v2 run is valid in all five worlds and configures all 15 model
poses on their first attempt. Oracle succeeds 4/5 and Shadow emits ripe
detections plus target poses in 5/5. The 64.6 mm layout repeatedly collides
`panda_hand` with `strawberry_fruit_2` at the last approach waypoint, whereas
the differently aligned 30 mm layout succeeds. ADR 0013 therefore rejects a
scalar clearance threshold and defers approach optimization until P4 bottleneck
selection. This is still diagnostic evidence, not a P3/P4 gate result.

ADR 0014 freezes the runtime scene-condition injection pre-gate before any
formal robustness run. Its preserved v1 execution produced valid artifacts for
all nine lighting-by-occlusion worlds, but correctly failed because the heavy
occluder covered 100% of the target ROI. After the one recorded geometry
revision, `results/t70/scene_condition_injection_pre_gate_v2/summary.json`
passes with 9/9 valid conditions and 45/45 measured frames. Mean scene
luminance is approximately 125.1, 173.8, and 185.3 for dim, nominal, and bright
with no occluder. Target-ROI blue coverage is 0.000, 0.549, and 0.743 for none,
partial, and heavy at every light level. This accepts only the injection
infrastructure; no YOLO metric, robot motion, formal P4 scenario, or held-out
real image was consumed.

The follow-on ADR 0015 pilot binds the benchmark labels to those nine condition
IDs and exercises three one-factor worlds at one previously verified
single-target pose. `results/t70/oracle_shadow_condition_pilot_v1/summary.json`
passes with 3/3 infrastructure-valid trials, 3/3 Oracle motion successes, and
all three runtime condition checks. YOLO remains Shadow-only. Its raw 2,410
frame counts prove liveness but include pre-parking startup frames and unequal
trial durations, so they are not a condition-normalized robustness metric.

ADR 0016 corrects that measurement boundary in the preserved v2 pilot. All 180
fixed post-setup, pre-motion Shadow frames are present and Oracle again succeeds
3/3. Nominal/no-occlusion gives 60/60 ripe frames and 60/60 target-pose frames;
dim/no-occlusion gives 60/60 and 42/60; nominal/heavy gives 0/60 and 0/60.
This is sufficient to identify heavy-occlusion perception as a diagnostic
failure mode, but not to accept P4 or start the formal matrix.

ADR 0017 then broadens only the no-motion measurement to five previously
verified positions and three one-factor conditions. The completed run contains
15/15 valid scenarios, 900/900 fixed Shadow frames, and the expected seven
materialized-world hashes. The single seed is explicitly a deterministic
materialization input and claims no independent random repetitions. Nominal
and dim conditions both produce ripe detections in 300/300 frames; target-pose
availability is 295/300 and 277/300, respectively. Heavy occlusion produces no
ripe detection or target pose in any of 300 frames across all five positions.

The immutable first summary failed only because the `near_left` square target
ROI contains 25% blue collection-bin background even when the injected
occluder is absent. ADR 0018 preserves that artifact and adds a structural,
background-relative injection check in `summary_v2.json`; all injection checks
pass and no YOLO count changes. This closes the bounded D2 diagnostic, not P4.
The formal 135+30 matrix remains unstarted.

ADR 0019 responds by authorizing only a leakage-safe synthetic-capture
preflight. It excludes D2 artifacts, exact D2 coordinates, and all three formal
seed labels from synthetic training; requires disjoint unique image hashes for
synthetic train and held-out splits; keeps audited real validation for
non-regression only; and leaves training disabled until the preflight receipt
passes. One simulator-specific fine-tuning claim may then be frozen. This does
not authorize perception control or change any stage-gate status.

ADR 0020 accepts that isolation preflight. The completed data contains 36/36
groups and 288/288 renders, split into balanced 216-image train and 72-image
synthetic held-out partitions across all nine conditions. Encoded and source
hashes are unique and disjoint; D2 overlap is zero; the formal seed labels are
absent; and the canonical digest is
`a8f1f157e15f6a52f637b1cc11f275f3320bd4cae156206560a7ead6d17b76b5`.
No training or robot motion occurred. The next bounded milestone is to freeze
the exact mixed-training and hyperparameter manifest plus its one-time claim;
training remains locked until then.

ADR 0021 completes that freeze. The verified derivative contains 501 original
real training images and 216 synthetic training images, with the 72 synthetic
held-out images as the only in-training validation. The 116 audited real
validation images remain post-selection non-regression evidence. Exact
30-epoch fine-tuning parameters, `baseline__best`, evaluation thresholds, and
the no-retry claim boundary are hash-bound. The environment receipt passes on
the RTX 4060 and reports `training_unlocked=true`, while
`training_started=false` and `claim_consumed=false`. The next milestone is the
single claim-consuming training run; this still does not start formal testing
or authorize perception control.

ADR 0022 closes the claim with rejection. Training completed 30/30 epochs in
507.14 seconds and preserved eight checkpoints. Synthetic held-out selection
chose `best.pt` at confidence 0.80 with macro-F1 1.000. The same frozen
checkpoint/threshold failed audited real non-regression at macro/ripe/unripe F1
0.625537/0.754430/0.496644. A diagnostic-only real threshold sweep reaches
macro-F1 0.800035 at 0.47 but still misses the UNRIPE minimum. The claim is
consumed, no retry or relaxation is authorized, and D2/negative promotion
screens stop under fail-fast. No stage-gate status changes.

ADR 0023 isolates fruit-asset realism without training or motion. The valid v4
run completes 18/18 scenarios and 1,080/1,080 fixed frames using the original
real-image baseline. Texture-only B is indistinguishable from legacy A for ripe
confidence, while native shaped/calyx C increases mean ripe confidence from
0.660601 to 0.758152. All three variants detect zero unripe targets in 180
frames. Scene simplification is therefore a partial ripe-confidence factor, not
the root cause of the unripe or audited-real failure. No stage gate changes.

ADR 0024 rebaselines the active project path to P2/T30 and pauses further T70
expansion. A validation-only counterfactual shows independent class thresholds
would select 0.36 for ripe and 0.31 for unripe, producing macro-F1 0.817223,
still 0.032777 below the gate. Only 8/30 unripe false negatives have a
model-input minimum side below 32 px, so calibration and input resolution are
not selected as the next primary remedies. The authorized read-only screen then
processed exactly 501 registered training images with `baseline__best` and
generated 34 review candidates: 22 cross-class conflicts and 12 high-confidence
unmatched boxes. No validation/test inference, training, label modification,
motion, or stage-gate acceptance occurred. The packet is pending user review.

ADR 0056 accepts the Blender-v2 natural-plant v3 diagnostic as a working
no-motion dual-camera chain. Base selection, one collision-planned observation
move, 60/60 wrist target poses, stationary handoff, and controller-free
pre-grasp planning all pass; the trajectory is discarded with zero control
commands. The execution-readiness gate then fails closed because the perceived
centre is 29.177346 mm from truth and lies outside both qualified grasp-geometry
limits. Perception execution remains unauthorized. The next bounded milestone
is a frozen natural-plant depth-region or fruit-mask localization diagnostic,
followed by the same geometry check and a separate repeatability decision.
