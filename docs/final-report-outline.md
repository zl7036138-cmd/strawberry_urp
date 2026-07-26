# Final report evidence outline

Proposed title: **基于 ROS 2、YOLO 与 Gazebo 的草莓成熟度识别及机械臂抓取搬运仿真研究**

## 1. Research objective and scope

Explain the reproducible closed loop: RGB-D acquisition, maturity detection,
depth/TF localization, MoveIt planning, temporary attachment, placement, and
failure attribution. State that the work is simulation-only and does not model
stem cutting, soft fruit, damage, or real-robot transfer.

## 2. System design

Use `docs/architecture.md` as the interface authority. Describe the fixed
camera, `panda_link0` base frame, simulation time, metre units, detection and
target-pose topics, PickAndPlace action, and bounded state-machine retries.

## 3. Data and perception

Report the deterministic 501/116/115 split and the audit history. The final
engineering-waived model has audited validation macro/ripe/unripe F1 of
0.800675/0.887064/0.714286 at confidence 0.58. The 0.85 numeric gate failed;
the held-out test was not consumed. Do not convert the owner waiver into a
metric pass.

## 4. Localization and manipulation modules

Report T40's 100/100 measurements, 1.345 mm median and 1.897 mm p95 error.
Report the truth-target manipulation result of 9/10 and the T60 Oracle
integration result of 10/10, including the one bounded endpoint correction.

## 5. End-to-end experiments

Distinguish the non-formal clear-scene development gate from the formal
matrix. The development gate passed 10/10 ripe and 10/10 only-unripe trials at
one clear pose. The consumed formal matrix passed infrastructure checks but
achieved only 39/135 positives (28.89%) against the 80% gate. All 30 negative
trials safely returned `NO_PICK`, with zero pick attempts and zero acquired
unripe fruit.

Use these generated figures:

- `artifacts/p5/release_v1/p3_success_by_occlusion.svg`
- `artifacts/p5/release_v1/p3_success_by_position.svg`
- `artifacts/p5/release_v1/p4_detection_vs_target_pose.svg`

## 6. Robustness intervention and bottleneck

The sole simulator-only checkpoint restored heavy-occlusion ripe detections to
300/300 frames. It still produced 0/300 target poses because bounding-box
centre depth sampled the foreground occluder; the reconstructed point was
0.345-0.366 m from the nearest fruit and failed the 0.080 m truth-association
limit. ADR 0032 rejected the checkpoint and prohibited a motion matrix.

## 7. Discussion

Separate the established results from limitations:

- localization and Oracle control are quantitatively sound in the tested
  simulator scope;
- negative-scene fail-closed behavior is strong;
- end-to-end robustness is limited primarily by perception under occlusion and
  by grasp geometry at two near positions;
- improving detection alone is insufficient when centre-depth localization is
  occluded;
- no real-image test score, real-robot result, or sim-to-real result exists.

## 8. Reproduction and conclusion

Reference `docs/reproduction.md` and the hash-bound P5 evidence manifest. ADR
0033 accepts the clean-environment reproduction and ADR 0034 freezes the final
P5 release and video. Conclude with the system's reproducibility contribution
and its measured limitations, not with an unsupported P3/P4 acceptance claim.
