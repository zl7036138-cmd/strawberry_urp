# ADR 0056: Accept the v3 no-motion gate and block perception execution

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Blender-v2 natural-plant dual-camera handoff, controller-free
  pre-grasp planning, and geometry-aware execution readiness

## Context

ADR 0055 authorized one final non-motion gate after the v2 DDS topic snapshot
defect. The resulting
`results/development/blender_v2_perception_handoff_pregrasp_v3/summary.json`
passes and has SHA-256
`5973d2347e816669a0aeb2c5343b35ace42f8f262d94128ab077835832ffcbf0`.

The base camera selected target 1 and the `lower` observation preset from
60/60 support frames. MoveIt found a collision-free wrist observation path in
one attempt (`0.024113108 s`) and the startup-verified arm action client
executed it in `23.304810362 s`. The wrist pipeline then produced target 1 in
60/60 measured TargetPose frames and satisfied the 15-consecutive-frame
readiness gate.

The stationary handoff received 15 target samples and 10 joint samples. Its
maximum target age was `0.078 s`; observed joint-state and MoveIt-state deltas
were both zero. All seven collision objects, including the selected fruit,
remained in the planning scene. Controller-free pre-grasp planning succeeded
in one attempt (`0.022236882 s`), produced 38 waypoints, and ended within
`0.795461 mm / 0.005676 rad` of the requested pose. The trajectory was
discarded and the total control-command count remained zero.

Before permitting execution, a geometry-aware readiness check compared the
frozen perceived centre with simulation truth strictly as execution-safety
evidence. The perceived centre was
`[0.410457133, -0.047995105, 0.573445780] m`; target 1 truth was
`[0.419064753, -0.053479813, 0.546111838] m`. The Euclidean error was
`0.029177346 m`. In the commanded hand frame, the actual fruit centre would be
`[0.008607620, 0.005484708, 0.123733942] m`.

The qualified v2 grasp geometry allows at most `0.003857179 m` cross-jaw
centre error and an axial hand-z range of
`[0.058531696, 0.112249034] m`. The observed cross-jaw error is
`0.005484708 m`, and the observed axial position is `0.123733942 m`; both
checks fail. The result
`results/development/blender_v2_perception_handoff_pregrasp_v3/execution_readiness.json`
has SHA-256
`d6e5842ce221f5a55ddf43fe293650fe66dfed7ec03bf7ed62d9458a211c95bd`.

The wrist RGB image shows natural plant material and calyx/leaf structure
inside the detection region. The existing central depth crop plus fixed
26 mm camera-ray surface correction is therefore not yet a reliable fruit
centre estimator in this view.

## Decision

1. Accept v3 only as a non-acceptance dual-camera observation, same-world
   handoff, and controller-free pre-grasp planning gate.
2. Add
   `scripts/evaluate_blender_v2_perception_execution_readiness.py` as a
   fail-closed, geometry-aware execution-readiness evaluator.
3. Require the frozen handoff target, passing no-motion gate, exact scene
   manifest, and passing exact-mesh grasp geometry as its inputs.
4. Use simulation truth only to verify whether the already selected,
   perception-commanded pose lies inside the qualified grasp envelope. Truth
   may not select a target, replace the perceived command, or repair the pose.
5. Preserve the failed readiness result. Set
   `execution_readiness_passed=false`,
   `perception_execution_authorized=false`, and `pick_authorized=false`.
6. Do not execute the generated trajectory or start a pick action.
7. Treat foreground calyx/leaf depth in the box-centre crop as the leading
   hypothesis, not a proven root cause. The next diagnostic must freeze and
   compare depth-region strategies or an object mask without consuming the
   held-out test, substituting truth, or relaxing grasp tolerances.

## Consequences

The project now has a working natural-plant dual-camera perception handoff and
collision-checked pre-grasp planner, plus an explicit guard that prevents a
plausible-looking but mechanically unsafe perceived pose from being executed.
Perception-derived grasp execution remains unauthorized until a separately
frozen localization improvement passes this geometry check and its own
repeatability gate.
