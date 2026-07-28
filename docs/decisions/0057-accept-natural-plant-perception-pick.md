# ADR 0057: Accept one natural-plant perception-derived pick

- Recorded: 2026-07-28
- Status: Accepted
- Scope: Resolve ADR 0056 readiness and gripper blockers, then preserve one
  exactly-once Blender-v2 perception-derived pick-and-place development result

## Context

ADR 0056 blocked execution because its v3 perceived centre was 29.177 mm from
truth and outside the qualified gripper envelope. The follow-up found that
renaming the dual-camera localization nodes prevented the profile's 26 mm
surface-to-centre parameter from matching the YAML node name. Passing the
scene-bound offset explicitly reduced the natural-plant error to approximately
5 mm without changing the detector, depth-region algorithm, or grasp envelope.

The first newly authorized perception action reached the final grasp pose but
the fingers remained at approximately 0.040 m, no target contact occurred, and
attachment failed. An isolated same-pose diagnostic reproduced a controller
`stalled` result with zero measured close travel in free space and at the
grasp pose. The plant leaves and stems are visual-only geometry, both complete
finger collision sensors reported no contact, and the target fruit did not
move, excluding the perception pose and plant collision as the cause of that
zero-travel result.

The gripper controller used a `0.25 s` stall timeout. Increasing this bounded
window to `1.0 s` allowed the first measured displacement after arm motion. The
successful isolated run at
`results/development/blender_v2_grasp_pose_gripper_diagnostic_v4/summary.json`
has SHA-256
`2d3725f4873df25d780d79f69fa9ac9914bae9ea20cb12cfbe0949d8fa0e3da6`.
It records 14.138 mm measured close travel, raw and processed bilateral target
contact, no non-target contact, 1.909 mm fruit displacement, reopen, collision
restoration, and final home recovery. Current-policy validation passes with no
violations.

One subsequent exactly-once perception action at
`results/development/blender_v2_perception_pick_once_v4/trial_01.json` has
SHA-256
`6f1f522b1ce1c0c1492b1abe8d81d66e7fceae8778c91b331ff3866f05492603`.
Its readiness record passes with a 4.934 mm centre error, 2.573 mm cross-jaw
error, and 100.598 mm axial position. The action completes
`PLAN, APPROACH, GRASP, RETREAT, PLACE, VERIFY, DONE`, confirms raw and
processed bilateral target contact, attaches and detaches the selected fruit,
verifies it in the bin, reopens the gripper, and returns the arm home.

## Decision

1. Supersede ADR 0056's perception-execution block for this exact bounded
   development runner and canonical Blender-v2 scene.
2. Keep the explicit scene-bound 26 mm localization offset at every renamed
   localization-node launch boundary.
3. Use a 1.0 s gripper stall timeout. Do not disable stall detection.
4. Treat empty controller result state as missing evidence and fall back only
   to the live commanded-joint value from `/joint_states`.
5. Align reached-goal validation with the controller's 3 mm tolerance while
   independently requiring at least 2 mm measured travel before accepting a
   stalled close.
6. Accept the preserved isolated result as the gripper-control repair
   qualification and the preserved v4 action as one successful
   perception-derived end-to-end development result.
7. Keep both results non-formal. Do not infer repeatability, varied-pose
   robustness, hardware readiness, or fruit-damage safety.
8. Do not consume the held-out real test or promote the detector's failed
   numeric acceptance result.

## Consequences

The current project mainline now demonstrates a real plant scene, base-camera
overview, wrist RGB-D localization, safe three-dimensional pre-grasp planning,
measured bilateral grasp contact, simulated attachment, transport, bin
placement, release, verification, and recovery in one perception-derived
closed loop.

The next manipulation evidence, if pursued, should be a separately frozen
fresh-world repeatability gate rather than ad hoc repeated picks. Perception
coverage for the second ripe fruit and the unripe fruit, natural occlusion
variation, deformable-contact or fruit-damage modelling, and physical hardware
remain open.
