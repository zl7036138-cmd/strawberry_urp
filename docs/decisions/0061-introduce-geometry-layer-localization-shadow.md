# ADR 0061: Introduce opt-in geometry-layer localization

- Recorded: 2026-08-05
- Status: Accepted
- Scope: Synthetic-depth plus fixed-scene ROS/Gazebo Shadow; no motion

## Context

ADR 0032 preserved a useful separation between detection and localization.
The rejected P4 intervention detected ripe fruit in 300/300 heavy-occlusion
frames, but the central depth crop reconstructed the foreground occluder. All
300 points failed the 0.080 m truth-association guard, so no target pose was
published. ADR 0060 later accepted field-v3 only for one fixed clear target and
did not claim varied-occlusion robustness.

The frozen implementation uses the median of the central 30% of a detection.
Increasing that crop or selecting the farthest depth would not be safe: the
larger region can contain background, another fruit, or several leaves. A new
estimator therefore needs an independent geometric consistency check and must
fail closed when the observation is ambiguous.

## Decision

1. Add an opt-in `geometry_layer` estimator beside the existing
   `center_median` implementation.
2. Split valid box depths at bounded discontinuities and select a layer only
   when its median agrees with the apparent size of a sphere whose radius is
   the scene-bound `surface_to_center_offset_m`.
3. Project the selected layer's pixel centroid, not the detection-box centre.
4. Reject a missing fruit layer, an invalid radius, an out-of-tolerance layer,
   or two similarly plausible layers.
5. Keep both frozen localization configurations byte-for-byte unchanged; they
   use the node's `center_median` default. Put all opt-in parameters in the new
   `localization_geometry_layer_v1.yaml` development configuration. This ADR
   does not promote the new estimator to runtime.
6. Accept the deterministic four-case offline diagnostic only as
   synthetic-depth development evidence.
7. Do not rerun or modify the consumed P3/P4 matrices, use the rejected P4
   checkpoint for motion, access the sealed real test, or authorize control.

## Evidence

`scripts/evaluate_occlusion_depth_estimator.py` produces
`results/development/occlusion_aware_depth_layer_v1/receipt.json`.

The diagnostic verifies:

- both estimators return 0.50 m for a clear synthetic fruit;
- the legacy estimator returns the 0.18 m foreground layer when the box centre
  is occluded;
- the geometry estimator recovers the 0.50 m fruit layer still visible at the
  box edges;
- the geometry estimator rejects an absent fruit layer; and
- the geometry estimator rejects two equally plausible layers.

A fresh field-v3 no-motion Shadow run at
`results/development/field_v3_geometry_layer_shadow_v1` then produced ripe
detections and target poses in all 60 measured frames. Its 60 target-1 samples
had 3.148 mm median and maximum position error, while the median reported
position sigma was 0.028098 m. The collision/handoff audit recorded zero joint
delta and no control, pick, trajectory, or gripper command. The summary passed
and is bound by SHA-256
`c2e3e48d9aa31fde40d1e0bf3d906da0668cfe215db9b2d06d8f3b14160d8e72`.

The 3.148 mm value is better than the historical 4.856 mm field-v3 record, but
the runs are not an identical-frame formal comparison. The new run covers one
fixed target and natural field-scene occlusion, not the P4 benchmark occluder.

## Consequences

The codebase now has a bounded candidate for the specific P4 localization
failure mechanism without rewriting any frozen result. Unit and offline tests
exercise its selection and rejection paths, and one fixed-scene ROS/Gazebo
Shadow confirms end-to-end publication with zero motion.

The candidate is intentionally conservative: its 28.1 mm median uncertainty is
much larger than the observed error. The next step is a newly labelled
no-motion matrix across varied fruit poses, scales, and controlled occlusions,
with both estimators evaluated on identical recorded frames. Until that gate
exists and passes, `geometry_layer` is not an accepted field-v3 runtime setting
and must not be used to claim improved grasp success.

ADR 0062 subsequently resolves one support-blind false ambiguity found by an
identical-frame recorded-depth matrix. That repair remains opt-in and does not
change this ADR's no-motion promotion boundary.
