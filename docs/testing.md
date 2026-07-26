# Testing and benchmark workflow

The benchmark core is deliberately independent of ROS. It can generate the
fixed scenario manifest, validate trial logs, calculate metrics, and apply the
acceptance thresholds from `config/project.yaml` using only Python 3's standard
library.

## Unit tests

Run every dependency-light suite from the repository root:

```bash
python3 scripts/run_pure_tests.py
```

The current 2026-07-16 WSL result is 338 tests with 337 passes and one
conditional skip, with no failures or errors. This result is separate from the
ROS/colcon suite.

Build and test the complete ROS workspace with:

```bash
bash scripts/build_and_test.sh
```

The current clean-distribution result from 2026-07-24 is seven packages and 246
colcon tests, with zero errors, failures, or skips.

To run only the dependency-light benchmark suite, without sourcing ROS:

```bash
cd ros2_ws/src/strawberry_benchmark
python3 -m unittest discover -s test -v
```

The same tests run through a ROS workspace after dependencies are installed:

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon test --packages-select strawberry_benchmark
colcon test-result --verbose
```

## Dataset and perception preflight

The T30 archive, pretrained weights, curated split, speed baseline, historical
YOLO11s run, and single Opt1 run are complete. Exact identities are listed in
`tools/perception/README.md` and the final validation-only handoff is
`artifacts/perception/t30_validation_gate_summary.json`. Opt1 completed 109
epochs, selected epoch 59, and reached validation macro-F1 0.780912 at
threshold 0.44. It failed promotion and test authorization.

Do not run the independent test split under the current decision. Every phase verifies all
materialized content hashes and path-independent canonical dataset/split
digests. Formal training writes a fixed receipt binding its output weight hash
to that dataset and configuration. The held-out test has one global,
dataset-level receipt at
`artifacts/perception/zenodo_6126677_held_out_test_receipt.json`; it is consumed
before reading test images, cannot be redirected through another experiment
file, and cannot be used by the validation-only YOLO11n speed baseline. Formal
Opt1 evaluation is fixed to `task=detect` and `imgsz=640`. The promotion tool
returned the non-promotion outcome: `promote_candidate=false` and
`authorize_held_out_test=false`. The formal test was not run, its receipt is
absent, and no independent-test score exists. Verification also
rejects unregistered images/labels, wrong-suffix or nested model inputs, and a
`dataset.yaml` whose split paths do not point to the registered directories.

The validation-only missing-label screen has been materialized as an immutable
human-review packet without changing labels:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/build_label_audit_packet.py \
  --analysis artifacts/perception/diagnostics/yolo11s_val_threshold_031/summary.json \
  --output-dir artifacts/perception/label_audit/yolo11s_val_threshold_031_candidates_v1
```

Its `packet_summary.json` records 21 candidates across 18 validation images,
four contact sheets, `labels_modified=false`, and `test_split_accessed=false`.
Reviewers must use only the closed decision vocabulary in `review_template.csv`.
No label may change until two independent reviews are resolved under a frozen
policy. Model-predicted boxes are screening evidence, not ground truth, and
must never be used alone to compute a corrected F1.

After the second reviewer completes a copy of `reviewer_2_template.csv`, validate
both reviews and create a no-mutation resolution receipt with:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/finalize_label_audit.py \
  --packet-dir artifacts/perception/label_audit/yolo11s_val_threshold_031_candidates_v1 \
  --reviewer-1 artifacts/perception/label_audit/yolo11s_val_threshold_031_candidates_v1/reviewer_1_codex_draft.csv \
  --reviewer-2 artifacts/perception/label_audit/yolo11s_val_threshold_031_candidates_v1/reviewer_2_user.csv \
  --disagreement-policy conservative_exclude \
  --policy-record docs/decisions/0007-conservative-label-audit-disagreement-policy.md \
  --output artifacts/perception/label_audit/yolo11s_val_threshold_031_resolution_v1.json
```

ADR 0007 freezes the conservative policy for this packet. The accepted receipt
retains all 13 agreements (5 ripe, 8 unripe) and resolves all 8 disagreements
to `AMBIGUOUS_EXCLUDE` without case-by-case override. The user explicitly
approved all 13 proposed final boxes without redraw. Materialize only this
validation derivative with:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/materialize_audited_validation.py \
  --resolution artifacts/perception/label_audit/yolo11s_val_threshold_031_resolution_v1.json \
  --approval artifacts/perception/label_audit/yolo11s_val_threshold_031_box_approval_v1.json \
  --output-dir data/processed/zenodo_6126677_val_audit_v1
```

The derivative preserves the original validation labels, refuses test paths,
and records every original/derived label hash. Re-evaluate the already frozen
historical prediction bundle without accessing the test split:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/evaluate_audited_validation.py \
  --derivative-manifest data/processed/zenodo_6126677_val_audit_v1/audit_manifest.json \
  --output artifacts/perception/label_audit/yolo11s_val_threshold_031_audited_metrics_v1.json
```

This label-only comparison produces macro-F1 0.816939 at threshold 0.31.
ADR 0008 separately freezes a fresh, uniform inference sweep over exactly the
26 checkpoints already produced by the baseline and Opt1 runs:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/sweep_audited_checkpoints.py \
  --derivative-manifest data/processed/zenodo_6126677_val_audit_v1/audit_manifest.json \
  --output-dir artifacts/perception/label_audit/yolo11s_audited_checkpoint_sweep_v1

/opt/strawberry_venv/bin/python \
  tools/perception/analyze_audited_checkpoint_errors.py \
  --sweep-summary artifacts/perception/label_audit/yolo11s_audited_checkpoint_sweep_v1/summary.json \
  --output artifacts/perception/label_audit/yolo11s_audited_checkpoint_error_analysis_v1.json
```

The sweep winner is `baseline__best`: macro-F1 0.815818 at threshold 0.31,
with ripe/unripe F1 0.894309/0.737327. It fails the 0.85 gate. These commands
authorize neither a new training run nor the once-only held-out test.

## ADR 0009 targeted training outcome

The user-authorized Opt2 run is fully consumed and must not be rerun under a
different output name. Its fail-closed sequence is:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/materialize_unripe_exposure_training.py

/opt/strawberry_venv/bin/python \
  tools/perception/run_targeted_training.py --preflight

/opt/strawberry_venv/bin/python \
  tools/perception/run_targeted_training.py

/opt/strawberry_venv/bin/python \
  tools/perception/evaluate_targeted_training.py
```

The derivative contains 689 effective training entries and 116 audited
validation entries, with no test split. Training completed 78 epochs and
preserved 10 weights. The validation command intentionally returns a nonzero
gate result: selected `best.pt` has macro-F1 0.795696 at threshold 0.54 and
ripe/unripe F1 0.886128/0.705263. The error analysis command is:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/analyze_targeted_training_errors.py
```

Opt2 is rejected. Its output does not authorize formal-test access, another
training run, or runtime promotion.

## T40 localization gate

After a green workspace build, choose a new empty result directory and run:

```bash
bash scripts/run_localization_gate.sh results/p2/localization_gate_manual
```

The gate starts the production localization node with Gazebo, evaluates 100
distinct truth positions, writes `summary.json` and `samples.csv`, and checks
for unhandled runtime failures during shutdown. The accepted 2026-07-13 run is
`results/p2/localization_gate_v6/summary.json`: 100/100 measurements, median
1.3449193778 mm, p95 1.8965995386 mm, maximum 2.0236130149 mm, and
`passed: true`. Before launch, the runner automatically selected domain 113 and
probed it with a daemon-free ROS graph query, observing no external nodes. Its
schema v2 provenance records that probe and 19 SHA-256 fingerprints spanning
the localization configuration/source/launch, simulation assets and source,
and the runner itself.

## Generate the frozen scenario manifest

From the repository root:

```bash
PYTHONPATH=ros2_ws/src/strawberry_benchmark \
python3 -m strawberry_benchmark.cli generate-scenarios \
  --config config/benchmark.yaml \
  --output benchmark_scenarios.jsonl
```

The command must report 135 positive trials, 30 negative trials, and 165 total
unique trial IDs. Positive trials are the full Cartesian product of occlusion,
lighting, position, and seed. Negative trials contain only `UNRIPE` fruit and
have balanced marginals: each occlusion, lighting, and seed appears 10 times,
and each position appears 6 times.

## Trial result contract

Trial runners create a `TrialResult` for every manifest row and write it with
`write_trial_results_jsonl` or `write_trial_results_csv`. Both formats carry the
same schema version and fields. A result records:

- scenario identity, mode (`ORACLE` or `END_TO_END`), conditions, and seed;
- expected/predicted maturity, confidence, pick attempt, and physical pick;
- end-to-end success, first failure stage/code, localization error, and timing;
- optional UTC timestamp and JSON metadata for immutable run configuration.

For a positive trial, `success=true` means a ripe fruit remained in the bin for
the required one simulated second. For a negative trial, it means the system
correctly completed `NO_PICK`. `fruit_picked=true` in an end-to-end negative
trial is a false pick. Failed results should always set both
`first_failure_stage` and `failure_code`; missing attribution remains valid for
ingestion but is reported as `unattributed_failure_count`.

Writers replace their destination atomically. A `trial_id` may appear only once
in an input log; aggregation rejects duplicates instead of silently averaging
reruns.

## Metrics and acceptance

Summarize JSONL or CSV logs with:

```bash
PYTHONPATH=ros2_ws/src/strawberry_benchmark \
python3 -m strawberry_benchmark.cli metrics \
  --results trial_results.jsonl \
  --output metrics.json
```

Apply every threshold in `config/project.yaml` with:

```bash
PYTHONPATH=ros2_ws/src/strawberry_benchmark \
python3 -m strawberry_benchmark.cli evaluate \
  --results trial_results.jsonl \
  --project-config config/project.yaml \
  --perception-macro-f1 0.87 \
  --output acceptance.json
```

Use `--perception-macro-f1` for the once-only independent real-image test-set
score. Without the override, the tool derives a diagnostic macro F1 from
end-to-end trial predictions when both maturity classes are present.

Metric definitions are fixed as follows:

- positive success is calculated over positive trials; oracle and end-to-end
  rates use their corresponding positive subsets;
- false-pick rate is physically picked fruit divided by all end-to-end negative
  trials; an arm motion without acquisition is not a false pick;
- localization median/p95 use all recorded non-null millimetre errors;
- planning p95 uses all recorded non-null planning times and linear percentile
  interpolation;
- failure attribution counts the first stage and stable architecture failure
  code for every unsuccessful trial.

The evaluator exits `0` only when all seven gates pass, `1` when any gate fails,
and `2` when data are incomplete or input/configuration is invalid. A missing
metric is `NOT_EVALUATED`, never an implicit pass.

## Oracle manipulation gate

After a green workspace build, run ten isolated truth-target trials from WSL:

```bash
bash scripts/run_oracle_pick_gate.sh 10 110 results/p2/oracle_gate
```

Each trial starts a fresh Gazebo world and ROS domain, performs one complete
pick-and-place, and shuts the launch down before the next trial. The summary
requires at least 90% action success, raw contact from both stock finger
sensors on every success, planning time no greater than five seconds, no
unexpected fruit collision, and clean shutdown logs. Contact with the bin is
allowed only when its first occurrence is in `VERIFY` or `DONE`, after release.

The 2026-07-13 baseline is `9/10` and passes. Successful planning times ranged
from 0.0325 s to 0.1095 s. The sole failure was a fail-closed 21.2 mm measured
grasp-pose miss before contact; it produced no fruit pick or unexpected
collision. Machine-readable evidence is in
`results/p2/oracle_gate/summary.json`.

After the T40 camera-TF and bringup changes, a focused oracle regression also
passed 1/1 with raw dual-finger contact, 0.1071 s planning time, no unexpected
fruit contacts, and clean shutdown. Its evidence is
`results/p2/oracle_regression_after_t40_v5/summary.json`.

## T60 dual-path integration gate

Run ten orchestrated oracle trials, each in a fresh world and ROS domain:

```bash
bash scripts/run_t60_oracle_gate.sh 10 210 results/t60/oracle_gate_v2
```

Unlike the T50 runner, this client invokes `/strawberry/run_trial` and requires
the complete state history, configured oracle source/topic, selected target ID,
planning time, terminal action result, and clean shutdown. The accepted v2
result is 10/10 with 0.058841 s planning p95. The failed 8/10 v1 result is kept
under `results/t60/oracle_gate_v1`.

The shadow smoke adds these launch arguments to the same runner:

```bash
start_perception:=true shadow_enabled:=true \
model_path:=/absolute/path/to/yolo11s_640/weights/best.pt \
confidence_threshold:=0.31
```

Its client requires at least one `/strawberry/shadow/detections` frame while
the final status still identifies oracle as the control source. The preserved
pre-fix v2 smoke recorded 837 frames and 99 boxes, but zero ripe boxes and zero
shadow `TargetPose` messages.

ADR 0006 traced that result to an RGB/BGR adapter defect. Reproduce the
same-frame A/B diagnostic in a fresh output directory with:

```bash
bash scripts/run_shadow_forensics.sh \
  outputs/perception/yolo11s_640/weights/best.pt \
  results/t60/shadow_forensics_manual 224 5
```

The accepted diagnostic evidence is
`results/t60/shadow_forensics_rgb_bgr_v1/summary.json`: the legacy RGB NumPy
path produced zero detections, while the correct BGR path produced five ripe
detections associated with ripe target 1 across five frames. Its first
prediction has confidence 0.519437 and truth-box IoU 0.774342.

The corrected full-stack v3 smoke recorded 798 frames, 1,587 boxes, 1,548 ripe
boxes, and 780 shadow target poses while the oracle-controlled pick succeeded.
It proves the repaired observation path is live through localization, but it
is not a T30 or P3 perception acceptance test.

Run the no-motion, camera-clear simulator perception pre-gate in a fresh output
directory with:

```bash
bash scripts/run_sim_perception_gate.sh \
  results/t60/sim_perception_pre_gate_manual \
  outputs/perception/yolo11s_640/weights/best.pt \
  225 10
```

The accepted v1 result collected 10 frames at each of five ripe positions and
five only-unripe positions. It reports ripe truth-associated frame recall 1.0,
ripe target-pose rate 1.0, and false-ripe negative-frame rate 0.0, so the
operational pre-gate passes. Its diagnostic unripe observation recall is 0.0:
the visible unripe fruit was never detected. The result therefore supports
simple-scene NO_PICK safety only; it is not class-balanced maturity evidence,
does not authorize motion, and does not replace T30 or the full P3/P4 matrices.

## T60 Oracle-control position diagnostic

ADR 0010 freezes a bounded five-position diagnostic with the audited
`baseline__best` model on shadow topics. The manifest verifies the exact model
SHA-256 before any Gazebo process starts. Run it only in a new output directory:

```bash
bash scripts/run_t60_position_diagnostic.sh \
  results/t60/oracle_shadow_position_diagnostic_manual \
  outputs/perception/yolo11s_640/weights/best.pt \
  220 \
  config/t60_oracle_shadow_position_diagnostic.json
```

Each scenario starts a fresh world and ROS domain, applies the target position
through Gazebo `set_pose`, waits for ground truth and the Oracle control stream
to agree, and then invokes the normal orchestrator. YOLO output remains on
`/strawberry/shadow/*`. The summary treats a safely reported motion failure as
a valid diagnostic observation; routing leakage, missing position
acknowledgement, missing shadow frames, client infrastructure errors, or dirty
shutdown invalidate the run.

The v1 result is complete in
`results/t60/oracle_shadow_position_diagnostic_v1/summary.json`: 5/5 valid
trials, 1/5 Oracle motion success, and shadow ripe detections plus target poses
in 5/5. Across 2,203 shadow frames it records 4,231 boxes, 3,879 ripe boxes, and
2,036 target poses. Log attribution identifies two non-target-fruit collisions
and two Cartesian IK failures. The coordinates are diagnostic-only, with
nominal lighting and no occluder. This result cannot close T30/P2/P3/P4 or
replace the formal 135 positive plus 30 negative matrix.

To separate reachability from multi-fruit clearance, the single-target manifest
uses the same runner while parking `strawberry_2` and `strawberry_3` outside the
work area:

```bash
bash scripts/run_t60_position_diagnostic.sh \
  results/t60/oracle_shadow_single_target_position_diagnostic_manual \
  outputs/perception/yolo11s_640/weights/best.pt \
  220 \
  config/t60_oracle_shadow_single_target_position_diagnostic.json
```

The preserved pre-fix v1 result is not valid isolation evidence. Gazebo and the
Oracle stream acknowledged the parked fruit, but MoveIt retained immutable
initial collision coordinates; the resulting 2/5 run reported three ghost
collisions with `strawberry_fruit_2`. ADR 0011 fixes that mismatch by requiring
a fresh, exact-ID simulation-truth snapshot and synchronizing every MoveIt fruit
sphere before motion. Missing, stale, incomplete, or non-finite truth fails
closed.

The first valid post-fix result is
`results/t60/oracle_shadow_single_target_position_diagnostic_v2/summary.json`:
5/5 infrastructure-valid trials, 5/5 Oracle motion success, no collision or IK
failure, and Shadow ripe detections plus target poses in 5/5. Its five launch
logs each record `MoveIt live fruit scene synchronized`. Across 5,079 detection
frames it records 5,574 boxes, 5,275 ripe boxes, and 4,924 target-pose
observations. These camera-clear positions are reachability candidates, not the
formal benchmark positions. The run remains Oracle-controlled and cannot close
T30/P2/P3/P4 or consume the held-out test.

Run the paired multi-fruit clearance diagnostic in a new directory with:

```bash
bash scripts/run_t60_position_diagnostic.sh \
  results/t60/oracle_shadow_multifruit_clearance_diagnostic_manual \
  outputs/perception/yolo11s_640/weights/best.pt \
  225 \
  config/t60_oracle_shadow_multifruit_clearance_diagnostic.json
```

ADR 0012 fixes both neighbor poses and reuses the five isolated target poses.
The manifest rejects overlapping fruit and records a decreasing nominal
surface-clearance sequence of approximately 134, 109, 92, 65, and 30 mm. Pose
service calls retry at most once and record their attempt count. The preserved
v1 run is aggregate-incomplete because position 5 had a setup timeout; its
summary correctly labels that event `INFRASTRUCTURE_FAILURE`.

The authoritative v2 result is
`results/t60/oracle_shadow_multifruit_clearance_diagnostic_v2/summary.json`:
5/5 infrastructure-valid trials, 4/5 Oracle motion success, all 15 model poses
configured on the first attempt, and Shadow ripe detections plus target poses
in 5/5. The 65 mm layout reproducibly fails at the last approach waypoint with
`panda_hand` against `strawberry_fruit_2`; the 30 mm layout succeeds. Under ADR
0013 this is geometry-dependent evidence, not a scalar minimum-clearance
threshold. It remains outside T30/P2/P3/P4 acceptance.

## T70 scene-condition injection pre-gate

Before running any formal robustness scenario, generate and observe all nine
lighting-by-occlusion worlds in a fresh directory:

```bash
bash scripts/run_scene_condition_gate.sh \
  results/t70/scene_condition_injection_pre_gate_manual \
  180 \
  config/t70_scene_conditions.json \
  ros2_ws/src/strawberry_sim/worlds/strawberry_orchard.sdf
```

The runner materializes a distinct hash-bound SDF for each condition, starts it
in a fresh ROS domain, places one target and parks both non-target fruit, waits
for settled RGB frames, and measures five frames. The blue occluder has a visual
element but no collision element. The summary rejects missing/edited receipts,
wrong world or config hashes, incomplete frames, dirty logs, insufficient light
separation, or unordered target-ROI occlusion.

The immutable v1 result is valid failure evidence: 9/9 runtime records were
complete, but heavy coverage was 1.0 and exceeded the 0.98 ceiling. ADR 0014
records the single heavy-geometry revision. The authoritative v2 summary at
`results/t70/scene_condition_injection_pre_gate_v2/summary.json` passes with
9/9 conditions, 45/45 frames, no blockers, and unique world hashes. Across all
three light levels, blue coverage is exactly 0.000 for none, approximately
0.549 for partial, and approximately 0.743 for heavy. The no-occluder mean
luminance values are approximately 125.059, 173.779, and 185.282.

This command does not start perception or manipulation and does not consume the
held-out real-image test. A pass authorizes later use of the injector only; it
cannot close T30, P2, P3, or P4 and is not the formal 135-positive plus
30-negative benchmark.

## T70 Oracle/Shadow condition pilot

After the nine-condition injection pre-gate passes, run only the frozen
three-scenario integration pilot in a new directory:

```bash
bash scripts/run_t70_condition_pilot.sh \
  results/t70/oracle_shadow_condition_pilot_v2_manual \
  outputs/perception/yolo11s_640/weights/best.pt \
  190 \
  config/t70_oracle_shadow_condition_pilot_v2.json
```

The manifest hash-binds `benchmark.yaml`, the scene-condition config, base
world, and below-gate Shadow model. It also requires exact one-to-one mapping
from the benchmark's level labels to all nine runtime condition IDs. The pilot
uses one known single-target pose and compares `nominal+none`, `dim+none`, and
`nominal+heavy`. Each fresh world runs the five-frame condition probe before
the normal Oracle-controlled state machine.

The v1 result at
`results/t70/oracle_shadow_condition_pilot_v1/summary.json` passes with 3/3
valid trials, 3/3 Oracle picks, no blockers, and all one-factor condition checks
true. Measured luminance is 173.779 at nominal and 125.059 at dim; heavy blue
coverage is 0.743. Shadow receives 2,410 frames, 1,876 boxes, 1,789 ripe boxes,
and 1,667 target poses, establishing channel liveness.

Those Shadow totals begin during launch before scene parking and cover unequal
wall-clock durations, so they are not suitable for comparing detector quality
between conditions. This pilot is non-acceptance evidence and cannot close
T30/P2/P3/P4 or replace the formal benchmark.

ADR 0016 adds the schema-v2 manifest and a fixed 60-frame Shadow window after
scene setup but before robot motion. The default runner now writes
`results/t70/oracle_shadow_condition_pilot_v2`. Its accepted diagnostic result
contains all 180 frames, passes all infrastructure and condition checks, and
again succeeds 3/3 under Oracle control. The normalized frame results are:

| Condition | Ripe detection frames | Target-pose frames | Oracle pick |
|---|---:|---:|---:|
| nominal + none | 60/60 | 60/60 | success |
| dim + none | 60/60 | 42/60 | success |
| nominal + heavy | 0/60 | 0/60 | success |

The heavy result is a diagnostic perception failure at one position and seed.
It does not authorize a model change by itself and is not a formal P4 score.

## T70 five-position fixed-window diagnostic

Run the frozen no-motion diagnostic only in a new output directory:

```bash
bash scripts/run_t70_multiposition_diagnostic.sh \
  results/t70/shadow_multiposition_fixed_window_manual \
  outputs/perception/yolo11s_640/weights/best.pt \
  195 \
  config/t70_shadow_multiposition_fixed_window.json
```

The runner expands five previously verified camera-clear positions by the
three one-factor conditions `nominal+none`, `dim+none`, and
`nominal+heavy`. A schema-v2 scene config moves the visual-only occluder by
70% of the target displacement from the accepted anchor. Every scenario first
collects five scene-condition probe frames and then exactly 60 settled Shadow
frames. Oracle, manipulation, orchestration, attachment, and robot motion are
explicitly disabled.

The authoritative run at
`results/t70/shadow_multiposition_fixed_window_v1` contains all 15 scenarios
and all 900 required frames. Its immutable `summary.json` correctly preserves
the initial absolute-blue check failure: the `near_left` target ROI includes
25% of the base world's blue collection-bin background even though no
benchmark occluder exists. ADR 0018 adds `summary_v2.json`, which verifies
no-occlusion from the receipt and SDF and measures heavy foreground relative
to the local background. The corrected injection check passes without
changing any YOLO observation.

Across five positions, nominal/no-occlusion produces ripe detections in
300/300 frames and target poses in 295/300. Dim/no-occlusion produces 300/300
and 277/300. Nominal/heavy produces 0/300 and 0/300. These are descriptive
diagnostic counts under one materialization seed, not model acceptance or a
formal P4 robustness score.

## T70 synthetic-capture isolation preflight

Validate the frozen manifests without starting ROS or training:

```bash
python3 scripts/validate_t70_synthetic_capture_preflight.py \
  --manifest config/t70_synthetic_capture_preflight.json
```

Run the capture into a new output directory:

```bash
bash scripts/run_t70_synthetic_capture_preflight.sh \
  data/processed/t70_sim_adaptation_preflight_v1
```

The authoritative receipt reports 36 groups and 288 images: 216 train and 72
synthetic held-out, with both splits class-balanced and evenly distributed over
the nine lighting-by-occlusion conditions. It also verifies unique hashes
within each split, zero train/held-out hash overlap, zero overlap with the 27
recorded D2 source-image hashes, and exclusion of all formal seed labels. The
canonical dataset digest is
`a8f1f157e15f6a52f637b1cc11f275f3320bd4cae156206560a7ead6d17b76b5`.

This command starts only headless simulation, pose control, and truth-based
capture. Perception, localization, orchestration, attachment, and manipulation
remain disabled. A passing receipt satisfies the capture preflight but retains
`training_unlocked=false`; it is not model, T70, or P4 acceptance.

## T70 simulator-adaptation training freeze

Materialize the exact mixed derivative once:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/materialize_sim_adaptation_training.py \
  --contract tools/perception/sim_adaptation_v1_contract.json
```

Verify an existing derivative without training:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/materialize_sim_adaptation_training.py \
  --contract tools/perception/sim_adaptation_v1_contract.json \
  --verify data/processed/yolo11s_640_sim_adapt_v1/training_manifest.json
```

The authoritative derivative contains 717 training images (501 real and 216
synthetic) plus 72 synthetic held-out validation images. It contains no test
split and no audited real-validation copy. Its canonical digest is
`aa96dbfdd5bb7b74a52b9857ee114536b30e65584feb80a34428771554928582`.

The training environment was frozen, without consuming the claim, with:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/run_sim_adaptation_training.py \
  --contract tools/perception/sim_adaptation_v1_contract.json \
  --write-preflight
```

The receipt binds the RTX 4060, PyTorch/Ultralytics environment, baseline
checkpoint, exact YAML, derivative, source manifests, materializer, and runner.
It records `training_unlocked=true`, `training_started=false`, and
`claim_consumed=false`. Running the same command without a mode flag is the
separate claim-consuming operation; do not invoke it as a smoke test.

The sole claim was consumed successfully: 30/30 epochs completed in 507.14
seconds and eight checkpoints were preserved. The frozen synthetic evaluator
tested all eight on the 72-image synthetic held-out split. `best.pt` won at
confidence 0.80 with macro-F1 1.000. Applying that checkpoint and threshold
once to the 116 audited real-validation images produced macro/ripe/unripe F1
0.625537/0.754430/0.496644, failing all three non-regression checks.

ADR 0022 applies fail-fast rejection. D2 and simulator-negative promotion
screens were not run because they cannot reverse the failed aggregate decision.
The offline threshold sweep in the failure-analysis artifact is diagnostic
only: its real optimum is 0.47 with macro-F1 0.800035, but UNRIPE F1 remains
0.703518 below the 0.717327 minimum. It does not authorize retuning or promotion.

## Headed Oracle-control and rejected-candidate Shadow demo

Run one visible, explicitly non-acceptance development trial from WSLg into a
new output directory:

```bash
bash scripts/run_headed_oracle_shadow_demo.sh \
  results/development/my_new_headed_demo 223
```

The runner refuses a missing WSLg display, an unexpected model hash, an
existing output directory, or an unbuilt workspace. It starts Gazebo and RViz,
parks fruit 2 and fruit 3 outside the work scene, moves the single ripe target
to the frozen clear position, and runs exactly one orchestrated pick. Motion is
always controlled by `/strawberry/oracle/target_pose`; the rejected adaptation
candidate is fixed at confidence 0.80 on the Shadow topics. The output contract
states that neither the formal real test nor the formal simulator matrix is
consumed.

The authoritative development run at
`results/development/headed_oracle_shadow_rejected_candidate_20260716_v1`
finished `DONE/SUCCESS`, traversed the complete 13-state workflow, and retained
source isolation. Planning and headed execution took 0.0474 and 82.14 seconds,
respectively; these timings are descriptive and are not benchmark evidence.
The candidate produced 459 ripe boxes in 520 Shadow frames and 371 localized
target observations. Intermittent depth/CameraInfo synchronization warnings
occurred under the dual-GUI plus GPU-inference load, so this single clear ripe
scene neither promotes the candidate nor clears its unripe/occlusion risks.

## T70 strawberry visual-asset A/B/C diagnostic

Validate the frozen v4 manifest and model bindings:

```bash
python3 scripts/validate_t70_strawberry_asset_ablation.py \
  --manifest config/t70_strawberry_asset_ablation_v4.json \
  --model outputs/perception/yolo11s_640/weights/best.pt
```

Run the 18 no-motion scenarios into a new directory:

```bash
bash scripts/run_t70_strawberry_asset_ablation.sh \
  results/t70/my_new_asset_ablation \
  outputs/perception/yolo11s_640/weights/best.pt \
  160 config/t70_strawberry_asset_ablation_v4.json
```

The runner compares A legacy solid spheres, B camera-scale albedo on the same
spheres, and C native sphere/cone strawberry bodies with radial calyxes. Ripe
and unripe targets each use three positions. Every scenario discards 30 warmup
detection frames and measures exactly 60. It starts no Oracle, orchestrator,
attachment, manipulation, training, or robot motion.

The authoritative v4 result has all 18 scenarios and 1,080 measured frames.
A/B/C ripe correct-class rates are all 180/180, with mean confidence
0.660601/0.660598/0.758152. A/B/C unripe correct-class rates are all 0/180.
Only C minus A meets the frozen material-effect threshold, through a +0.097551
ripe-confidence change. Renderer-only v2/v3 and full v1 evidence are preserved
as infrastructure failures because their C mesh rendered white; they are not
model evidence. See ADR 0023 and the v4 handoff.

## T30 rebaseline and training-label audit

Recompute the validation-only rebaseline into a new output path. This reads
only the existing audited-validation artifacts and registered validation
images; it does not run a model:

```bash
python3 tools/perception/rebaseline_t30_validation.py \
  --output artifacts/perception/rebaseline/my_t30_rebaseline.json
```

Export the complete registered training split with ADR-0024's fixed audit
settings. Use a new output path; this command runs inference but does not train
or calculate a formal metric:

```bash
/opt/strawberry_venv/bin/python \
  tools/perception/export_train_audit_predictions.py \
  --output artifacts/perception/label_audit/my_train_screen/predictions.json
```

Build a bounded human-review packet from that new bundle:

```bash
python3 tools/perception/build_train_label_audit_packet.py \
  --bundle artifacts/perception/label_audit/my_train_screen/predictions.json \
  --output-dir artifacts/perception/label_audit/my_train_screen/review_packet
```

The authoritative v1 export covers 501/501 training images and the review
packet contains 34/34 candidates: 22 cross-class conflicts and 12 unmatched
predictions. The packet and its reviewer-1 draft do not modify labels or
authorize training. The validation and held-out-test splits are explicitly
outside this inference run.

Finalize the two completed reviews under the conservative disagreement rule:

```bash
python3 tools/perception/finalize_train_label_audit.py \
  --reviewer-1 artifacts/perception/label_audit/t30_train_label_screen_v1/review_packet/reviewer_1_codex_draft.csv \
  --reviewer-2 artifacts/perception/label_audit/t30_train_label_screen_v1/review_packet/reviewer_2_user.csv \
  --output artifacts/perception/label_audit/t30_train_label_screen_v1/review_resolution_v1.json
```

Materialize and verify the ADR-0025 derivative. This preserves source labels
and refuses any test directory:

```bash
python3 tools/perception/materialize_train_label_audit.py \
  --contract tools/perception/train_label_audit_v1_contract.json
```

The single-use training claim and its validation-only checkpoint sweep are:

```bash
/opt/strawberry_venv/bin/python tools/perception/run_targeted_training.py \
  --contract tools/perception/train_label_audit_v1_contract.json

/opt/strawberry_venv/bin/python tools/perception/evaluate_targeted_training.py \
  --contract tools/perception/train_label_audit_v1_contract.json
```

The completed run is rejected at macro/ripe/unripe F1
`0.800675/0.887064/0.714286`; these commands are receipts, not permission to
repeat the consumed claim.

## ADR-0026 perception-control smoke

Validate the user waiver without starting ROS or opening the held-out test:

```bash
python3 scripts/validate_p3_perception_control_waiver.py \
  --manifest config/p3_perception_control_waiver_v1.json
```

Run one fresh, bounded, headless perception-controlled trial into a new output
directory and an unused ROS domain:

```bash
bash scripts/run_p3_perception_control_smoke.sh \
  results/p3/my_perception_control_smoke 225
```

The validator checks the model size/SHA-256, both historical evidence hashes,
the unchanged below-gate metrics, perception-only routing, and the absence of
the held-out-test receipt. The runner parks the two non-target fruit, keeps the
known camera-clear ripe fruit at its base pose, stops Oracle, and routes
`/strawberry/target_pose` to the orchestrator. Generic system-launch defaults
remain unchanged.

The authoritative v3 run reaches `DONE/SUCCESS` in 1/1 trial with source
isolation, a complete state history, 0.038362 s planning, 69.874016 s
execution, and clean shutdown. It is an engineering smoke under a documented
metric waiver, not the formal P3 matrix.

## ADR-0027 repeated perception-control development gate

Validate the frozen manifest, waiver, model, evidence, and held-out-test seal
without starting ROS:

```bash
python3 scripts/validate_p3_perception_repeated_dev_gate.py \
  --manifest config/p3_perception_repeated_dev_gate_v1.json
```

Run ten positive and ten negative fresh-world trials in ROS domains 180-199:

```bash
bash scripts/run_p3_perception_repeated_dev_gate.sh
```

The runner refuses an existing output directory. It starts perception control
with Oracle stopped. Positive trials expose only the clear ripe base-pose fruit;
negative trials expose only the unripe fruit at the same pose and require ten
settled detection-array frames before accepting `NO_PICK`.

The authoritative v1 result passes 10/10 positive picks and 10/10 negative
`NO_PICK` trials, with zero false picks, zero negative control attempts,
0.071302 s planning p95, valid infrastructure, and clean shutdown throughout.
It is a one-pose development gate under ADR 0026, not the formal 135+30 matrix.

## ADR-0029 formal P3 simulator matrix

Validate the exact model, previous gate, matrix, positions, condition assets,
Gazebo seed forwarding, schedule, and held-out-test seal without consuming the
formal claim:

```bash
python3 scripts/validate_p3_formal_matrix.py \
  --manifest config/p3_formal_matrix_v1.json \
  --require-unclaimed
```

The authoritative no-simulation preflight is already frozen at
`artifacts/p3/p3_formal_matrix_preflight_v2`. It contains the 165-line scenario
manifest with SHA-256
`072836297996959ce2165071ecc53f29f00b64a3ea6344d019a247b18ea71caf`.
The older v1 preflight must not be used; the current contract rejects its path.

The following command is claim-consuming and must not be used as a smoke test:

```bash
bash scripts/run_p3_formal_matrix.sh \
  config/p3_formal_matrix_v1.json \
  artifacts/p3/p3_formal_matrix_preflight_v2/preflight_receipt.json \
  results/p3/formal_matrix_v1 \
  false
```

For an interrupted run, the only permitted continuation uses the same claim,
preflight, and result directory with the final argument `true`. A completed or
started behavioral attempt is never rerun; only a missing pre-behavior attempt
may use its single infrastructure retry. The runner is headless and is expected
to take several hours.

The authoritative run is now consumed and must not be started again. It stopped
after completed trial 100 and resumed under the same claim from missing trial
101; the classifier preserved the first 100 results. The final summary contains
165 unique scenarios, one behavioral attempt per scenario, no infrastructure
retry, valid infrastructure, clean shutdown, and no unattributed failure.

The formal P3 gate fails: positive success is 39/135 (28.89%) rather than 80%.
Negative safety passes at 30/30 `NO_PICK`, with no negative pick attempt or
acquired unripe fruit. Planning p95 is 0.068495 seconds. Failure attribution is
84 perception and 12 grasp. Verify the immutable outcome without rerunning it:

```bash
python3 scripts/validate_p3_formal_matrix.py \
  --manifest config/p3_formal_matrix_v1.json
python3 scripts/summarize_p3_formal_matrix.py \
  --output-dir results/p3/formal_matrix_v1 \
  --manifest config/p3_formal_matrix_v1.json \
  --preflight artifacts/p3/p3_formal_matrix_preflight_v2/preflight_receipt.json \
  --model outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt
```

The summarizer returns status 1 because the acceptance gate fails; it still
writes and verifies `results/p3/formal_matrix_v1/summary.json`. ADR 0030 and
`artifacts/p3/p3_formal_matrix_outcome_handoff_v1.json` are the authoritative
interpretation and hash-bound handoff.

## ADR-0031 P4 simulator-adaptation qualification

The single no-motion qualification is frozen in
`config/p4_sim_adapt_qualification_v1.json`. Its preflight binds 30 scenarios,
the exact simulator-adaptation checkpoint, threshold 0.80, critical ROS source
files, and schedule SHA-256
`8af748f4fda35e2b7750f9501cebf56bd26645798c9b7d29bcfcdd653d326e92`.
The claim is already consumed; do not start this command again:

```bash
bash scripts/run_p4_sim_adapt_qualification.sh \
  config/p4_sim_adapt_qualification_v1.json \
  artifacts/p4/p4_sim_adapt_qualification_preflight_v1/preflight_receipt.json \
  results/p4/sim_adapt_qualification_v1 \
  false
```

All 30 worlds complete with no manipulation or motion. Ripe detection is
300/300 for nominal-none, dim-none, and nominal-heavy. Target-pose output is
292/300, 299/300, and 0/300 respectively. Unripe observations are 840/900 with
zero false-ripe frames. The summarizer correctly returns status 1 because the
heavy target-pose checks fail. ADR 0032 rejects the candidate and forbids both
a second P4 intervention and a post-intervention motion matrix. The immutable
handoff is `artifacts/p4/p4_sim_adapt_qualification_outcome_handoff_v1.json`.

## P5 clean release reproduction

The clean reproduction is frozen by ADR 0033. In the separate
`Ubuntu-24.04-URP-Repro` distribution, run:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/strawberry_venv/bin/activate
bash scripts/build_and_test.sh
bash scripts/run_p5_release_smoke.sh \
  results/p5/release_smoke_clean_v1 clean 220 \
  results/p5/release_smoke_reference_v1/summary.json
```

The accepted build receipt contains seven packages and 246 tests with zero
errors, failures, or skips. The smoke result contains 5/5 successful clear
ripe trials and 5/5 only-unripe `NO_PICK` trials, valid infrastructure and
shutdown, zero false picks, and an absolute reference difference of 0.0
against the 0.05 limit. This is release reproducibility evidence only; it does
not replace formal P3, reopen P4, or consume the sealed real-image test.

The final P5 media check is
`artifacts/p5/video/strawberry_urp_demo_v1.receipt.json`: H.264, 1280×720,
288.7 seconds, with source clips and metrics bound by SHA-256. The positive
clip is explicitly Oracle-controlled and non-formal; the only-unripe clip ends
in `DONE/NO_PICK` without a control attempt. The complete release is verified
with:

```bash
python3 scripts/generate_p5_release.py
python3 scripts/verify_p5_release.py
```

The expected terminal status is `VERIFIED_FINAL_RELEASE`.
