# Natural-plant perception pick development gate v1

## Outcome

The canonical Blender-v2 plant scene now has one successful, exactly-once
perception-derived pick-and-place development result. The base camera selects
the ripe target, the wrist camera supplies the final RGB-D localization, the
same-world handoff and controller-free pre-grasp checks pass, and the
perception pose drives the full manipulation action to `DONE`.

This is a bounded non-acceptance development gate. It does not establish
fresh-world repeatability, varied-pose or occlusion robustness, physical
hardware transfer, or fruit-damage safety.

## Root-cause chain

The work resolved two independent faults rather than changing the detector or
relaxing grasp geometry:

1. Renamed localization nodes did not match the top-level YAML node name, so
   the 26 mm surface-to-centre correction silently fell back to zero. The
   runners now pass `surface_to_center_offset_m:=0.026` explicitly.
2. The gripper controller's `0.25 s` stall timeout could finish a close goal
   before the first measured finger displacement after arm motion. The
   controller now uses a bounded `1.0 s` stall window.

The manipulation backend does not trust `stalled` by itself. It validates the
measured commanded-joint position, falls back to live `/joint_states` only
when the controller result state is empty, requires at least 2 mm travel for a
stalled close, and uses the controller's 3 mm goal tolerance for a
`reached_goal` result.

## Isolated gripper evidence

The preserved diagnostic is:

`results/development/blender_v2_grasp_pose_gripper_diagnostic_v4`

It uses the frozen target centre from the failed v3 perception action, moves
through the same wrist observation, pre-grasp, and grasp poses, and performs
only close, reopen, retreat, collision restoration, and home recovery.
Attachment and place motion remain disabled.

Key observations:

- free-space control close reaches 0.022 m per finger;
- grasp-pose close travels 14.138 mm from the 0.040 m open state;
- settled fingers are approximately 0.02597/0.02604 m;
- raw and processed left/right target contacts are present;
- no non-target contact is observed;
- fruit displacement during close is 1.909 mm; and
- reopen, retreat, collision restoration, and home recovery succeed.

The source summary was generated before the reached-goal tolerance was aligned
with the controller and therefore contains `diagnostic_passed=false`. The
current-policy evaluator consumes the immutable source summary and produces a
passing `validation.json` with no violations.

Source summary SHA-256:

`2d3725f4873df25d780d79f69fa9ac9914bae9ea20cb12cfbe0949d8fa0e3da6`

Validation SHA-256:

`023e570a27f24573056830b0757ffb39cd365e60ae3725faf5028b0082760e56`

## Full perception-derived action

The preserved successful result is:

`results/development/blender_v2_perception_pick_once_v4`

Execution readiness:

- target centre:
  `[0.419383150, -0.056053162, 0.550309779] m`;
- simulation-truth safety reference:
  `[0.419064753, -0.053479813, 0.546111838] m`;
- Euclidean error: 4.934 mm;
- cross-jaw error: 2.573 mm, below the 3.857 mm limit; and
- axial hand position: 100.598 mm, inside the
  `[58.532, 112.249] mm` range.

Action outcome:

- status: succeeded;
- feedback:
  `PLAN → APPROACH → GRASP → RETREAT → PLACE → VERIFY → DONE`;
- planning time: 0.1353 s;
- execution time: 107.0932 s;
- finger minima during grasp:
  0.025578/0.026015 m;
- raw contact counts: left 1093, right 1101;
- attachment becomes true at 64.8277 s and false at 114.3124 s;
- final attachment state is detached;
- gripper returns to approximately 0.040 m per finger; and
- arm returns to the `ready` joint configuration.

Execution-readiness SHA-256:

`35d0088f072d48eeb0533a5768c9c1efd9c28ab021960f413e22da0cdffb123b`

Action-result SHA-256:

`6f1f522b1ce1c0c1492b1abe8d81d66e7fceae8778c91b331ff3866f05492603`

## Reproduction

Build and test:

```bash
bash scripts/build_and_test.sh
```

Run an isolated, no-attachment gripper diagnostic using a fresh output
directory:

```bash
bash scripts/run_blender_v2_grasp_pose_gripper_diagnostic.sh \
  results/development/blender_v2_grasp_pose_gripper_diagnostic_next
```

Evaluate a completed isolated result:

```bash
python scripts/evaluate_blender_v2_grasp_pose_gripper_diagnostic.py \
  --input results/development/blender_v2_grasp_pose_gripper_diagnostic_next/summary.json \
  --output results/development/blender_v2_grasp_pose_gripper_diagnostic_next/validation.json
```

Run one fully gated perception-derived action using a fresh output directory:

```bash
bash scripts/run_blender_v2_perception_pick_once.sh \
  results/development/blender_v2_perception_pick_once_next
```

The full runner refuses an existing output directory, verifies the detector
hash, performs the observation motion, same-world handoff, planning-only
shadow, and geometry readiness checks, and only then sends exactly one action.
