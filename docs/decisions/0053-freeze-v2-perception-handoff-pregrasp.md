# ADR 0053: Freeze v2 perception handoff and pre-grasp planning

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One non-acceptance dual-camera integration gate after ADR 0052

## Context

ADR 0052 qualifies five Oracle-controlled manipulation round trips in the
canonical Blender-v2 plant. Earlier dual-camera work also demonstrated base
overview selection, wrist-camera confirmation, a no-motion control-side
handoff, and controller-free pre-grasp planning. Those earlier planning
receipts predate the qualified `0.0964 m` Blender-v2 tool-centre offset, while
the post-contact planning requalification in ADR 0047 used an Oracle target.

ADR 0026 permits the exact below-threshold detector for bounded simulation
control under an explicit engineering waiver. ADR 0028 accepted its simple
single-fruit repeated development result, but neither decision qualifies a
perception-derived target in the current natural-plant scene.

## Decision

1. Freeze one gate named
   `blender_v2_perception_handoff_pregrasp_v1`.
2. Start one fresh canonical Blender-v2 world with dual cameras and attachment
   disabled. Run the exact waived model at confidence `0.58`.
3. Use the base camera only to select target 1 and the bounded `lower` wrist
   observation preset. Stop base inference before the collision-checked
   observation-pose motion.
4. After the arm settles, use only the wrist RGB-D pipeline on
   `/strawberry/shadow/target_pose`. The Oracle provider and topic,
   orchestrator, production pick action, attachment backend, and pose-control
   bridge must remain absent.
5. Require 60 base measurement frames with at least 48 target-1 supports,
   15 consecutive wrist-ready frames, and target 1 in all 60 measured wrist
   TargetPose frames.
6. Require the no-motion handoff to receive at least 15 coherent target
   samples and 10 stationary joint samples, keep all seven collision objects,
   and create or send no control interface or command.
7. Require controller-free planning to load profile `blender_v2_26mm`, use the
   `0.0964 m` tool-centre and `0.15 m` stand-off, retain all collision objects,
   generate one valid pre-grasp trajectory, and explicitly discard it.
8. Execute at most once in ROS domain 230 and the frozen non-overwriting
   result directory. A failure is preserved; no retry, alternate target,
   threshold change, synchronization relaxation, collision relaxation, or
   execution is authorized.

## Boundaries

The observation-pose motion is authorized solely to place the wrist camera.
No perception-derived grasp trajectory, gripper command, attachment, pick
action, final approach, retreat, or placement is authorized by this decision.
A pass may support a separately frozen single execution gate, but it is not
formal acceptance, does not consume the held-out real-image test, does not
remove the model waiver, and makes no real-hardware claim.
