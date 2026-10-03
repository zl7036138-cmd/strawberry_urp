# ADR 0055: Preserve v2 topic-discovery failure and freeze v3

- Recorded: 2026-07-27
- Status: Accepted
- Scope: One final measurement-only rerun after the ADR-0054 graph defect

## Context

The single ADR-0054 invocation is preserved at
`results/development/blender_v2_perception_handoff_pregrasp_v2`. The bounded
repair succeeded:

- base selection again chose target 1 and `lower`;
- the collision-free wrist observation plan completed in one attempt;
- planning/execution time was `0.041388307 / 23.756146127 s`;
- the executed endpoint error was approximately `0.5 mm / 0.70 deg`; and
- the wrist receipt recorded target 1 in all 60 measured TargetPose frames
  after a satisfied 15-frame readiness gate.

The runner then launched a new `ros2 topic list --no-daemon` process. Its
instantaneous inventory did not include `/strawberry/shadow/target_pose`, even
though the immediately preceding receipt had subscribed to that exact topic
and recorded 60 messages. It also omitted active depth, camera-info, joint,
and ground-truth topics. The fresh CLI process therefore observed an
incomplete DDS graph and triggered the new fail-closed check. The runner
stopped before handoff or pre-grasp planning.

The runtime node inventory contained both wrist perception and localization
nodes and no Oracle or orchestrator node. The runner source starts no Oracle
provider. The v2 stop is thus a diagnostic-snapshot defect, not missing target
data and not an Oracle isolation failure.

## Decision

1. Preserve the complete v2 directory and failed summary.
2. Keep the topic inventory as non-authoritative diagnostic evidence.
3. Remove only the requirement that a fresh no-daemon CLI process immediately
   rediscover the target publisher.
4. Treat the 60-frame wrist receipt as authoritative positive topic evidence:
   it must be complete, satisfy the 15-frame readiness gate, and contain only
   target 1 TargetPose identities.
5. Continue to fail if the runtime topic inventory contains the Oracle target,
   or if the runtime node inventory contains any Oracle or orchestrator node.
6. Freeze one final gate named
   `blender_v2_perception_handoff_pregrasp_v3` with all ADR-0053 parameters in
   ROS domain 232 and a new non-overwriting result directory.
7. Execute v3 at most once. A failure is preserved; no further rerun is
   authorized without another decision.

## Boundaries

This is still observation-pose motion, perception handoff auditing, and
controller-free pre-grasp planning only. No grasp trajectory execution,
gripper command, attachment, pick action, formal acceptance, held-out-test
access, or physical hardware is authorized.
