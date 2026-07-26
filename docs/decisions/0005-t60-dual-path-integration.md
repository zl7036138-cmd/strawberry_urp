# ADR 0005: T60 oracle control with YOLO shadow observation

## Status

Accepted on 2026-07-15 for the T60 oracle integration subgate. This decision
does not accept T30, P2, or the perception-driven P3 gate.

## Context

T40 localization and T50 truth-target manipulation passed their module gates,
but both YOLO11s validation runs remained below the frozen macro-F1 threshold.
Blocking all integration work on T30 would hide control-chain defects, while
allowing an unaccepted detector to command the robot would invalidate the
stage boundaries.

## Decision

- Oracle control publishes only on `/strawberry/oracle/target_pose`.
- YOLO shadow inference publishes detections on
  `/strawberry/shadow/detections`; its localization output is
  `/strawberry/shadow/target_pose`.
- The orchestrator subscribes to one configured control topic. Shadow
  callbacks only increment diagnostic counters and cannot send an action goal.
- Startup is fail-closed: a trial is not accepted until the PickAndPlace action
  server is ready.
- Trial status schema v2 records the configured source/topic, selected control
  ID, full transition history, action timing, failure attribution, and shadow
  frame/detection/target counts.
- A controller-successful Cartesian endpoint that misses the existing 10 mm
  verification tolerance receives at most one correction replanned from the
  measured pose. The tolerance is not relaxed and a second miss fails.
- The held-out real-image test remains sealed. The historical YOLO11s weight
  may be used only as a named shadow diagnostic, never as an accepted runtime
  model.

## Evidence and consequences

The first T60 run, `results/t60/oracle_gate_v1/summary.json`, failed at 8/10
with retreat and bin-entry endpoint misses. After the bounded correction was
added, `results/t60/oracle_gate_v2/summary.json` passed 10/10 in independent
ROS domains 210–219; planning p95 was 0.058841 s. One trial measured an 11.6 mm
miss, applied the single correction, reached 0 mm reported error, and
completed successfully.

The shadow smoke in
`results/t60/oracle_with_yolo_shadow_v2/trial_01.json` received 837 inference
frames and 99 boxes while oracle target 1 controlled the successful action.
All 99 boxes were classified UNRIPE, so no shadow TargetPose was produced.
This is evidence of simulator-domain mismatch, not a perception acceptance
result.

T60 oracle integration may be marked accepted as a subgate. Full P3 remains
pending until a detector passes T30 and the perception control path meets the
simple-scene end-to-end and negative-trial requirements.
