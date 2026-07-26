# Implementation task handoff contract

Every implementation task must declare:

1. The frozen interface version and upstream artifacts it consumes.
2. The exact package or artifact it owns.
3. Commands used to build and test it.
4. Quantitative exit-gate results.
5. Remaining risks and whether they block downstream work.

Task completion is not accepted from screenshots or narrative alone. It must
include reproducible commands and machine-readable test or metric output.

## Current P5 handoff

- Status: P5 complete and frozen by ADR 0034; formal P3/P4 failures remain.
- Environment: the separate `Ubuntu-24.04-URP-Repro` WSL2 distribution runs
  Ubuntu 24.04.4, ROS 2 Jazzy, Ultralytics 8.4.92, PyTorch 2.13.0+cu130, and
  exposes the RTX 4060 through CUDA.
- Clean build: seven packages and 246 tests, with zero errors, failures, or
  skips. Receipt:
  `results/p5/clean_build_v1/summary.json`.
- Clean smoke: 5/5 clear ripe positives succeed; 5/5 only-unripe negatives
  return `NO_PICK`; false picks and negative control attempts are zero. The
  10/10 behaviour rate differs from the frozen reference by 0.0, within the
  0.05 limit.
- Frozen evidence:
  `artifacts/p5/p5_clean_reproduction_handoff_v1.json` and ADR 0033.
- Boundaries: formal P3 remains failed at 39/135, P4 remains rejected, the
  held-out real test remains sealed, and no physical-robot or sim-to-real claim
  is authorized.
- Video: `artifacts/p5/video/strawberry_urp_demo_v1.mp4` is 288.7 seconds,
  H.264, 1280×720. Its receipt binds the successful Oracle-controlled headed
  demo, the safe perception-controlled only-unripe `NO_PICK`, metric snapshot,
  and storyboard.
- Final verifier: `python3 scripts/verify_p5_release.py` returns
  `VERIFIED_FINAL_RELEASE`.
- P6 delivery: `artifacts/p6/strawberry_urp_release_v1.zip` contains the compact
  source/model/evidence/video release with an embedded SHA-256 inventory. ADR
  0035 and `artifacts/p6/p6_delivery_handoff_v1.json` are the terminal handoff.
- Next action: none required. Only user-requested proofreading, material
  corrections, or verified reproduction-defect fixes remain in scope; no new
  feature, training, formal rerun, or additional P4 intervention.

## Current T30 handoff

- Input contract: Zenodo record 6126677, two v1 maturity classes, fixed split
  seed `20260710`, YOLO11n speed baseline, historical YOLO11s model, and the
  single completed `cls_pw=0.5` Opt1 candidate at 640 px.
- Owned code: offline/reproducibility scaffolding under `tools/data` and
  `tools/perception`, plus the pinned dataset and experiment manifests.
- Implemented checks: archive size/hash verification, safe extraction,
  scene-group split guards, per-file content hashes, path-independent canonical
  dataset/split digests, model-weight hash preflight, fixed training receipts,
  validation-only threshold freezing, an immutable Opt1 promotion decision,
  and one global dataset-level receipt for the once-only test evaluation. The
  YOLO11n and historical YOLO11s variants are prohibited from formal test; Opt1
  test is fixed to `detect` at 640 px and requires validation authorization.
  Unregistered
  images/labels, wrong-suffix or nested model inputs, and incorrect split
  directories are rejected.
- Quantitative evidence: 732 retained images split 501/116/115. Historical
  YOLO11s validation at threshold 0.31 gives macro-F1 0.787121, ripe F1
  0.885246, and unripe F1 0.688995. Opt1 completed 109 epochs and selected
  epoch 59; at threshold 0.44 it gives macro-F1 0.780912, ripe F1 0.878151,
  and unripe F1 0.683673. The deltas are -0.006208, -0.007095, and -0.005322,
  respectively. Its mean/p95 inference times are 13.460/42.202 ms over 116
  validation images.
- Audited validation evidence: the user approved all 13 consensus boxes without
  redraw (5 ripe, 8 unripe), while all 8 disagreements remain conservatively
  excluded. A separate derivative contains 116 label files and adds only those
  13 boxes; original labels are unchanged. The frozen historical prediction
  bundle scores macro-F1 0.816939 on this derivative. A predeclared sweep of all
  26 existing checkpoints selects `baseline__best` at threshold 0.31 with
  macro-F1 0.815818 and ripe/unripe F1 0.894309/0.737327. The 30 unripe false
  negatives consist of 15 low-confidence detections, 9 absent detections, 5
  wrong-class detections, and 1 localization failure.
- Gate status: not accepted. Opt1 failed the macro-F1 >= 0.7971 and unripe F1
  >= 0.7090 promotion checks, as well as validation macro-F1 >= 0.85 test
  authorization. It was not promoted. The held-out test was not run, no formal
  test score exists, and the global test receipt is absent. The historical
  machine-readable handoff is
  `artifacts/perception/t30_validation_gate_summary.json`; the audited follow-on
  handoff is `artifacts/perception/t30_audited_validation_handoff_v1.json`.
- Remaining risks: unripe detection/recall remains the primary bottleneck.
  The immutable validation-only audit packet freezes 21 possible missing-label
  candidates across 18 images with source hashes, overlays, crops, and a closed
  decision vocabulary. The audit workflow is complete, but the resulting
  checkpoint sweep remains 0.034182 below the 0.85 gate. Error attribution
  points to unripe confidence/recall rather than box localization. Any next
  training intervention required a separately frozen decision. ADR 0009 then
  authorized one x2 exposure run over the 188 unripe-containing train images.
  It completed 78 epochs, preserved 10 weights, and selected `best.pt` at
  threshold 0.54, but macro/ripe/unripe F1 was only
  0.795696/0.886128/0.705263. The intervention regressed the audited baseline
  and is rejected. No additional training, formal-test access, or perception
  motion control is authorized. The bound outcome handoff is
  `artifacts/perception/t30_opt2_outcome_handoff_v1.json`.
- T30 rebaseline: ADR 0024 returns the active critical path to perception and
  pauses new T70 expansion. Reusing immutable audited-validation predictions,
  independent class thresholds improve macro-F1 only from 0.815818 to 0.817223;
  the unripe optimum remains 0.31. Target-scale analysis finds only 8/30 unripe
  false negatives below 32 px at the 640-px model input. Together with the
  failed checkpoint sweep, unripe-x2 run, and simulator-mixed candidate, this
  selects training-label quality as the one remaining pre-training hypothesis.
- Training-label audit outcome: the hash-bound `baseline__best` screen covered
  all 501 registered training images without traversing validation or test. The
  user completed the 34-item second review. After correcting nine semantic
  no-op entries, the reviewers agree on 18 items and disagree on 16; every
  disagreement is conservatively excluded. ADR 0025 authorizes and materializes
  only the 13 consensus changes: five relabels and eight additions. The source
  labels remain unchanged. The 617-entry train/audited-val derivative has
  canonical digest
  `c3ba01af84b143e98d307232c70c2eae009dcc4cfed1fabfcd5521becca93981`
  and contains no test split.
- Clean-label training outcome: the single ADR-0025 claim completed 115 epochs
  in 1456.47 seconds and preserved 14 checkpoints. The frozen scan selects
  `best.pt` at threshold 0.58 with macro/ripe/unripe F1
  `0.800675/0.887064/0.714286`. Macro and unripe F1 fail their promotion gates,
  so the candidate is rejected and remains Shadow-only. Error attribution
  finds 21 low-confidence, 13 absent, 5 wrong-class, and 1 localization unripe
  false negative. Evidence is bound by
  `artifacts/perception/t30_train_audit_outcome_handoff_v1.json`.
- Current authorization: ADR 0026 accepts the exact clean-label `best.pt` at
  threshold 0.58 for bounded perception-controlled simulator integration.
  T30/P2 are engineering-accepted with waiver, while macro-F1 0.800675 remains
  a failed 0.85 numeric gate. No new training, held-out-test access, formal
  135+30 matrix, or sim-to-real claim is authorized.

## Current T40 handoff

- Input contract: synchronized simulated depth, `CameraInfo`, detections in the
  camera optical frame, published TF, and truth positions used only by the gate.
- Owned code: `strawberry_localization`, its 100-position runtime gate, and
  `scripts/run_localization_gate.sh`.
- Runtime integration: sensor-data QoS, `tf2_geometry_msgs` conversion
  registration, the optical TF chain, and a 35 mm surface-to-centre correction
  along the camera ray. A count- and time-bounded timestamp cache matches
  delayed detections to coherent depth/calibration frames.
- Quantitative gate: `results/p2/localization_gate_v6/summary.json` reports
  100/100 valid measurements, median 1.3449193778 mm, p95 1.8965995386 mm,
  maximum 2.0236130149 mm, and `passed: true`.
- Evidence provenance: the runner automatically selected and verified empty ROS
  domain 113. Schema v2 records the daemon-free domain probe and 19 SHA-256
  fingerprints spanning localization, simulation, and the runner.
- Gate status: accepted. The result is comfortably below the 15 mm median and
  30 mm p95 limits.

## Current T50 handoff

- Input contract: architecture document and scene manifest, base frame
  `panda_link0`, truth pose for the oracle-only gate.
- Owned code: `strawberry_manipulation`, attachment/contact support in
  `strawberry_sim`, and `scripts/run_oracle_pick_gate.sh`.
- Build evidence: the latest `scripts/build_and_test.sh` reports seven packages
  and 233 colcon tests, with zero errors, failures, or skips. The separate WSL
  pure-suite count is recorded in the current T70 handoff below.
- Quantitative gate: `results/p2/oracle_gate/summary.json` reports 9/10 success,
  nine raw dual-contact grasps, planning maximum 0.1095 s, and ten clean exits.
- Cross-module regression: after the T40 camera-TF and bringup changes,
  `results/p2/oracle_regression_after_t40_v5/summary.json` passed 1/1 with raw
  dual contact, 0.1071 s planning time, no unexpected contacts, and clean
  shutdown.
- Remaining risk: R9 is non-blocking because the 90% gate is met, but its lack
  of margin must be revisited during T70 robustness work.

## Current T60 handoff

- Input contract: accepted T40 localization, T50 PickAndPlace action, Gazebo
  truth catalog/poses, ADR 0005, and the explicit ADR-0026 model waiver.
- Owned code: `strawberry_bringup` oracle provider/orchestrator/launch,
  configurable localization topics, the bounded Cartesian correction in
  `strawberry_manipulation`, `scripts/run_t60_oracle_gate.sh`, the hash-checking
  `scripts/run_p3_perception_control_smoke.sh`, and the frozen repeated-gate
  validator, runner, and summarizer.
- Control boundary: generic defaults retain Oracle control and Shadow-only
  YOLO. The dedicated ADR-0026 launcher instead stops Oracle, publishes the
  model on `/strawberry/detections` and `/strawberry/target_pose`, and explicitly
  routes `/strawberry/target_pose` to the orchestrator. Status schema v2 records
  the selected source/topic and complete state history.
- Build evidence: the post-repeated-gate full workspace build reports seven
  packages and 233 colcon tests with zero errors, failures, or skips.
- Quantitative gate: `results/t60/oracle_gate_v2/summary.json` reports 10/10
  success, planning p95 0.058841 s, ten clean shutdowns, and one successful
  bounded endpoint correction. The immutable failed precursor
  `results/t60/oracle_gate_v1/summary.json` reports 8/10.
- Shadow evidence: the preserved pre-fix v2 result had 837 frames, 99 UNRIPE
  boxes, and no shadow target. ADR 0006 and
  `results/t60/shadow_forensics_rgb_bgr_v1/summary.json` identify the direct
  cause as an RGB NumPy array supplied to Ultralytics' BGR NumPy interface.
  The corrected
  `results/t60/oracle_with_yolo_shadow_bgr_v3/trial_01.json` retains successful
  oracle control while recording 798 frames, 1,587 boxes, 1,548 ripe boxes,
  780 shadow target poses, and latest associated target ID 1.
  The bound machine-readable handoff is
  `artifacts/t60/shadow_channel_diagnostic.json`.
- Simulator perception pre-gate: the no-motion v1 result collected all 100
  required camera-clear frames. It reports 50/50 ripe truth associations,
  50/50 ripe target poses, and 0/50 false-ripe negative frames. The gate passes,
  but its diagnostic unripe observation recall is 0/50, so it demonstrates
  simple-scene NO_PICK safety rather than class-balanced simulator accuracy.
- Position diagnostic: ADR 0010 and
  `config/t60_oracle_shadow_position_diagnostic.json` bind Oracle control and
  the `baseline__best` shadow model. The v1 run is infrastructure-valid in 5/5
  fresh worlds. Motion succeeds in 1/5; two failures are attributed to
  non-target `strawberry_2` collisions and two to Cartesian IK. Shadow produces
  a ripe detection and target pose in every scenario. The result and hashes are
  bound by `artifacts/t60/oracle_shadow_position_diagnostic_handoff_v1.json`.
- Single-target isolation: the pre-fix v1 run parked both non-target models in
  Gazebo but left their MoveIt collision spheres at the initial manifest poses.
  Its 2/5 result and three `strawberry_fruit_2` contacts are preserved only as
  ghost-obstacle defect evidence. ADR 0011 now requires a fresh, complete
  `/strawberry/ground_truth/poses` snapshot, synchronizes all fruit obstacles,
  and restores the selected obstacle at its latest live position. The post-fix
  v2 result is valid in 5/5 worlds and succeeds 5/5 with no collision or IK
  failure; Shadow produces ripe detections and target poses in 5/5. The combined
  lineage and hashes are bound by
  `artifacts/t60/oracle_shadow_position_diagnostics_handoff_v2.json`.
- Paired multi-fruit clearance: ADR 0012 fixes two neighbor poses against the
  same five reachable targets. The v1 aggregate is incomplete due to one
  pose-service timeout. Its bounded-retry v2 replacement is valid in 5/5,
  succeeds 4/5 under Oracle control, and records Shadow ripe/target evidence in
  5/5. The only failure is a reproducible final-approach collision between
  `panda_hand` and `strawberry_fruit_2` at position 4. Position 5 succeeds at a
  smaller scalar clearance, so ADR 0013 freezes geometry-dependent rather than
  threshold-based interpretation. Evidence is bound by
  `artifacts/t60/oracle_shadow_clearance_handoff_v3.json`.
- Waived perception-control smoke: the isolated base-pose v3 run stops Oracle,
  obtains the target from YOLO plus RGB-D localization, and completes 1/1 at
  `DONE/SUCCESS`. Planning is 0.038362 s, execution is 69.874016 s, source
  isolation and the full state history pass, and shutdown is clean. Evidence is
  bound by `artifacts/p3/perception_control_waiver_smoke_handoff_v1.json`.
- Compatibility regression: the adjusted trial client also retains Oracle
  behavior at 1/1 `DONE/SUCCESS`, 0.030847 s planning, complete state history,
  source isolation, and clean shutdown.
- Repeated perception-control development gate: ADR 0027 freezes ten positive
  and ten negative fresh worlds at the clear base pose. The v1 result passes:
  positive manipulation is 10/10, negative `NO_PICK` is 10/10, false picks are
  0, negative control attempts are 0, planning p95 is 0.071302 s, and all
  infrastructure and shutdown checks pass. The hash-bound handoff is
  `artifacts/p3/perception_repeated_dev_gate_handoff_v1.json`.
- Gate status: Oracle integration, the bounded ADR-0026 smoke, and ADR-0027's
  non-formal repeated simple-scene gate are accepted. The formal P3 gate is
  rejected by ADR 0030: positive success is 39/135 (28.89%) versus 80%. The
  real held-out test remains sealed and the detector's numeric T30 gate remains
  failed.
- Formal-matrix outcome: all 165 scenarios completed with one behavioral
  attempt each, valid infrastructure, clean shutdown, and no unattributed
  failure. The interrupted host session ended after completed trial 100; the
  same claim preserved those 100 results and executed only missing trials
  101-165. All 30 negatives safely return `NO_PICK`, negative control attempts
  and acquired fruit are zero, and planning p95 is 0.068495 s. Positive
  failures are 84 perception and 12 grasp. Evidence is
  `artifacts/p3/p3_formal_matrix_outcome_handoff_v1.json`.
- P4 intervention outcome: ADR 0031 freezes the exact rejected simulator
  adaptation checkpoint at threshold 0.80 as the only simulator-only change.
  Its 30/30 no-motion qualification is infrastructure-valid and observes ripe
  detections in 900/900 ripe frames, correct unripe detections in 840/900
  frames, and zero false-ripe frames. No-occlusion target poses are 591/600,
  but heavy-occlusion target poses are 0/300. The foreground occluder corrupts
  box-centre depth, producing points 0.345-0.366 m from fruit and triggering the
  frozen 0.080 m association rejection. ADR 0032 rejects promotion, closes the
  single P4 intervention, and forbids the post-intervention motion matrix.
  Evidence is `artifacts/p4/p4_sim_adapt_qualification_outcome_handoff_v1.json`.
- Remaining risks: the five isolated camera-clear coordinates are only
  reachability candidates, not a replacement for the consumed formal matrix.
  One paired multi-fruit layout
  remains collision-infeasible under the current approach. Scene-condition
  injection now has a passing non-acceptance pre-gate, but no formal robustness
  intervention has been frozen and the unripe asset is not detected. Simulator truth remains
  diagnostic-only for non-target planning-scene geometry; it may never select
  the target or replace YOLO/localization evidence in a formal run without a
  separate decision.

## Current T70 handoff

- Input contract: ADR 0014, the immutable base orchard SDF, fixed seed
  `20260710`, three lighting levels, three occlusion levels, one visible ripe
  target, and two parked non-target fruit.
- Owned code: `config/t70_scene_conditions.json`, deterministic world
  materialization and runtime probing in `strawberry_sim`, explicit
  `world_file` launch forwarding, and
  `scripts/run_scene_condition_gate.sh` plus its hash-checking summary.
- Pre-gate safety boundary: the blue occluders are static visual-only models without
  collision elements. Perception, localization, orchestration, attachment, and
  manipulation are not started. Every output states `formal_acceptance=false`
  and `held_out_test_consumed=false`.
- Quantitative pre-gate: the v1 run retains a correct failure at 1.000 heavy ROI
  coverage. The revised v2 run has 9/9 infrastructure-valid conditions, 45/45
  measured frames, nine unique world hashes, no blockers, all three light-order
  checks, and all three occlusion-order checks. Target-ROI blue coverage is
  0.000/0.549/0.743 for none/partial/heavy.
- Build evidence: seven ROS packages and 233 colcon tests pass with zero
  failures, errors, or skips. The final dependency-light WSL result is recorded
  in the T70 handoff artifacts.
- Gate status: injection infrastructure pre-gate accepted. T70 and P4 are not
  accepted. The 135 positive and 30 negative formal trials are complete, but
  P3 fails at 39/135 positive successes. The one P4 intervention qualification
  also fails because heavy target-pose availability is 0/300 despite 300/300
  ripe detections. T30 remains numerically failed, and the held-out real-image
  test remains sealed.
- Condition mapping and pilot: ADR 0015 freezes the exact nine-ID mapping and a
  three-world one-factor pilot. Its v1 result is complete and passes with 3/3
  Oracle motion successes at the known single-target pose. The baseline,
  dim-light, and heavy-occlusion probes measure 173.779/125.059/173.269 mean
  luminance and 0.000/0.000/0.743 blue coverage, respectively.
- Shadow measurement boundary: 2,410 detection frames, 1,876 boxes, 1,789 ripe
  boxes, and 1,667 target poses prove the observation path remains live. These
  are lifetime counts that include the pre-parking startup interval and unequal
  trial durations; they do not measure condition-normalized accuracy.
- Fixed-window replacement: ADR 0016 and the schema-v2 manifest preserve v1 and
  add exactly 60 post-setup, pre-motion Shadow frames per condition. The v2 run
  is complete with 180/180 frames and 3/3 Oracle picks. Ripe/target-pose frame
  counts are 60/60 and 60/60 for nominal-none, 60/60 and 42/60 for dim-none,
  and 0/60 and 0/60 for nominal-heavy.
- Five-position no-motion diagnostic: ADR 0017 freezes five previously verified
  camera-clear coordinates, three one-factor conditions, one explicitly
  non-independent materialization seed, and a 60-frame post-setup window. A
  tracked visual-only occluder follows 70% of each target displacement. The run
  is complete in 15/15 fresh worlds with 900/900 frames, seven expected world
  hashes, and no robot motion.
- Injection-check correction: the immutable v1 summary exposes a valid metric
  confound at `near_left`, where the base world's blue collection bin occupies
  25% of the square target ROI even with no injected occluder. ADR 0018
  preserves that failure and adds a structural/background-relative v2 check;
  all five position checks then pass without changing runtime or YOLO counts.
- Multiposition Shadow result: nominal/no-occlusion has 300/300 ripe-detection
  frames and 295/300 target-pose frames; dim/no-occlusion has 300/300 and
  277/300; nominal/heavy has 0/300 and 0/300. Heavy failure therefore repeats
  at all five positions. The largest dim localization gap is `near_right` at
  41/60 target-pose frames.
- Current interpretation: the below-gate model's heavy-occlusion failure is no
  longer a one-position anomaly. The later formal matrix confirms 0/45 positive
  success under heavy occlusion and 84 total perception failures. Preserve the
  diagnostic lineage separately from the formal result. ADR 0019 froze a
  leakage-safe simulator-adaptation preflight:
  D2 artifacts and formal seed labels are excluded from training, synthetic
  train/held-out hashes must be disjoint, and real validation is non-regression
  evidence only. ADR 0020 accepts the completed capture preflight: 36 groups,
  288 images, balanced 216/72 train/held-out splits, disjoint unique hashes,
  zero D2 overlap, and no formal seed labels. No training has started.
- Authoritative evidence: the prior one-position lineage remains
  `artifacts/t70/oracle_shadow_condition_pilot_handoff_v3.json`; the current
  five-position lineage is
  `artifacts/t70/shadow_multiposition_fixed_window_handoff_v1.json`; the
  capture result is bound by
  `artifacts/t70/synthetic_capture_preflight_handoff_v1.json`.
- Synthetic capture evidence: the authoritative receipt is
  `data/processed/t70_sim_adaptation_preflight_v1/preflight_receipt.json` and
  its canonical dataset digest is
  `a8f1f157e15f6a52f637b1cc11f275f3320bd4cae156206560a7ead6d17b76b5`.
  It states `training_preflight_satisfied=true` but deliberately retains
  `training_unlocked=false`.
- Completed prerequisite: the exact 501-real-train plus 216-synthetic-train
  input contract, baseline checkpoint, hyperparameters, evaluation screens,
  and single-use claim boundary now validate.
- Simulator-adaptation training freeze: ADR 0021 and
  `tools/perception/sim_adaptation_v1_contract.json` now bind exactly 501 real
  plus 216 synthetic training images, 72 synthetic held-out validation images,
  `baseline__best`, 30 epochs, all hyperparameters, promotion screens, and a
  no-retry claim. The mixed-data canonical digest is
  `aa96dbfdd5bb7b74a52b9857ee114536b30e65584feb80a34428771554928582`.
- Frozen training preflight:
  `artifacts/perception/training/yolo11s_640_sim_adapt_v1_preflight.json`
  passed on `/opt/strawberry_venv` and the RTX 4060 before claim acquisition.
- Training outcome: the sole claim at
  `artifacts/perception/training/yolo11s_640_sim_adapt_v1.claim.json` completed
  30/30 epochs in 507.14 seconds with eight checkpoints. Synthetic-only
  selection chose `best.pt` at confidence 0.80 and macro-F1 1.000.
- Rejection: the frozen candidate produced audited real macro/ripe/unripe F1
  0.625537/0.754430/0.496644 and failed all non-regression checks. ADR 0022
  rejects it; no retry, real-selected threshold, runtime promotion, motion, or
  formal testing is authorized. D2/negative promotion screens were not run
  under fail-fast.
- Authoritative outcome artifacts:
  `artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_synthetic_heldout_v1/summary.json`,
  `artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_real_nonregression_v1/summary.json`,
  and
  `artifacts/perception/optimization/yolo11s_640_sim_adapt_v1_failure_analysis_v1.json`.
- Subsequent architecture decision: ADR 0026 accepts the exact clean-label
  `best.pt` for bounded simulator control while preserving the numerical
  failure and test seal. ADR 0027's repeated development gate passed, but ADR
  0030 records failure of the consumed formal matrix at 39/135 positives. The
  ADR 0031's exact simulator-only substitution then restores detection but
  fails the no-motion target-pose qualification. ADR 0032 consumes the single
  P4 intervention opportunity and prohibits a post-intervention motion matrix.
  The next action is P5 reproducibility and reporting, not another model or
  localization change.
- Headed development demonstration: `scripts/run_headed_oracle_shadow_demo.sh`
  and `config/headed_oracle_shadow_demo.rviz` add a fail-closed WSLg entry point
  with an immutable rejected-candidate hash and explicit Oracle-only routing.
  The run at
  `results/development/headed_oracle_shadow_rejected_candidate_20260716_v1`
  completed one clear single-fruit pick with `DONE/SUCCESS`, full state history,
  520 Shadow frames, 459 ripe boxes, and 371 Shadow target poses. It is bound by
  `artifacts/t70/headed_oracle_shadow_demo_handoff_v1.json`; it is not model,
  P3, T70, P4, timing, or robustness acceptance evidence.
- Strawberry asset ablation: ADR 0023 and
  `config/t70_strawberry_asset_ablation_v4.json` bind 18 no-motion A/B/C
  scenarios. The valid v4 run records 1,080/1,080 frames. Native fruit shape
  plus radial calyx raises ripe confidence from 0.660601 to 0.758152, while
  texture alone is neutral and every variant still misses the unripe target.
  Scene simplicity is a partial factor only. The canonical world remained
  unchanged for the frozen v1 matrix. ADR 0036 subsequently creates a new
  Blender v2 simulator baseline; it does not retroactively change this
  ablation. Evidence is bound by
  `artifacts/t70/strawberry_asset_ablation_handoff_v4.json`.
