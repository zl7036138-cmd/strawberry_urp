# ADR 0042: Accept the post-winding v2 localization outcome

- Recorded: 2026-07-27
- Status: Accepted
- Scope: Outcome of the ADR-0041 non-formal requalification

## Evidence

The single post-winding execution completed all 100 frozen positions. It used
the same camera-clear world materialization, Cartesian grid, 26 mm
surface-to-centre offset, production localization node, three settled depth
frames, and thresholds as the failed pre-repair run.

`results/development/blender_v2_localization_accuracy_100_post_winding_v1/summary.json`
records:

| Metric | Required | Actual | Passed |
|---|---:|---:|---|
| Valid positions | 100/100 | 100/100 | Yes |
| Median 3-D error | <=15 mm | 4.503614 mm | Yes |
| P95 3-D error | <=30 mm | 5.045998 mm | Yes |
| Maximum 3-D error | informational | 5.196535 mm | Yes |

Every sample succeeded on its first publication attempt. The logs contain no
unhandled runtime failure. No robot motion, gripper command, attachment,
planning, perception model, pick action, formal matrix, or real held-out test
access occurred.

The post-repair natural-plant regression at
`results/development/blender_scene_v2_single_fruit_60f_v5_post_winding/summary.json`
preserves `60/60` correct-class detections for each ripe fruit. TargetPose is
available in `59/60` frames for each ripe fruit. The unripe fruit remains
undetected at `0/60`, preserving the known appearance-domain limitation rather
than hiding it.

Gazebo RGB visual QA passed for both repaired fruit classes. The ripe and
unripe bodies, calyx, colors, and retained plant render normally, with no gross
mesh hole or inside-out artifact. The receipt is
`artifacts/simulation/blender_v2_body_winding_visual_qa_v1/visual_qa_receipt.json`.

## Decision

Accept `blender_v2_localization_accuracy_100_post_winding_v1` as passing
camera-clear Blender-v2 localization development evidence. The 26 mm
surface-to-centre model is now qualified over the frozen 100-position volume
when the fruit surface is directly visible.

Keep the failed pre-repair result as immutable causal evidence. Adopt the
outward-winding ripe and unripe OBJ files as the canonical Blender-v2 assets.

## Boundaries

This result does not qualify natural leaf occlusion, multi-view localization,
unripe detection, gripper geometry, attachment, trajectory execution,
perception-controlled picking, formal acceptance, physical hardware, or
sim-to-real behavior. The next prerequisite remains v2 gripper/TCP/contact
qualification under Oracle control. The real held-out test remains sealed.
