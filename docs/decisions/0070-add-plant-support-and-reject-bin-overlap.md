# ADR 0070: Add plant support and reject generalized bin overlap

## Status

Accepted as a development scene-and-grasp safety milestone. Continuous
multi-fruit harvest and the formal 30-seed acceptance matrix remain unaccepted
and sealed.

## Context

Seed 44008 repeatedly reached a wrist-confirmed target but failed the strict
dual-contact attachment gate. An oracle-coordinate development diagnostic
separated three possible causes: controller failure, fruit motion, and static
scene interference.

Both finger controllers closed correctly under empty load. Before this change,
however, generalized fruit was dynamic and unsupported, so one finger could
push it away before the second finger arrived. Holding the fruit at its authored
plant location then exposed a second defect: the diagnostic target occupied the
collection bin's back-wall work volume. One finger contacted
`collection_bin::bin_link::back_wall_collision`; this was not a legitimate
grasp candidate even though its centre was inside the old rectangular reach
bounds.

Gazebo truth was used only to diagnose and score development probes. It did not
enter detection, localization, tracking, target selection, observation, motion,
or grasp decisions.

## Decision

- Add a generalized-scene-only detachable plant support for every fruit. The
  support attaches to the world-fixed Panda base and holds the fruit at its
  generated pose before picking.
- Keep gripper attachment gated by fresh physical or strict geometric
  dual-finger contact. After that attachment is confirmed, release the plant
  support. If support release is not confirmed, roll back the gripper
  attachment and fail closed.
- Allow one bounded contact recovery: reopen, move through a collision-checked
  pregrasp, rotate the finger orientation by 90 degrees, and close once more.
  This does not relax the dual-contact gate.
- Require every generated fruit collision sphere to remain at least 2 mm away
  from the conservative collection-bin outer volume.
- Define `reachable_by_construction` using both the existing arm reach bounds
  and 70 mm collection-bin centre clearance, matching the runtime selector's
  unchanged 50 mm static margin plus 20 mm minimum target clearance.
- Require a retry after a reached wrist observation to use a view whose
  translation differs by at least 40 mm. If no such view exists, return
  `REOBSERVATION_UNAVAILABLE` rather than repeating the same evidence.

## Evidence

An empty-load probe closed the left finger to 22.20 mm and the right finger to
22.07 mm, excluding the two finger controllers as the cause. With plant support
enabled, the diagnostic fruit did not move at all, and the contact trace
identified the collection-bin wall as the non-target obstruction. The strict
attachment gate rejected the attempt.

On a bin-clear seed-44008 development fruit, both physical fingers contacted
the fruit at 26.20 mm and 26.07 mm. The contact gate passed, the plant support
released after gripper attachment, and the fruit followed the gripper 80 mm
upward. It was then placed into the collection bin and passed the stable in-bin
check; the action returned `pick-and-place completed` and cleanup was `CLEAN`.
This is an oracle-coordinate diagnostic of the grasp mechanics, not evidence
that the perception-controlled continuous batch is accepted.

The revised generator validated 4,500 development scenes spanning 500 disjoint
seeds, all three profiles, and all three position bands with zero validation
failures. No formal seed was materialized or executed.

A fresh seed-44008 development runtime contained no bin contact. Target 1
passed wrist confirmation with 5.4 mm raw correction and 6.8 mm fused
uncertainty. The subsequent observation-to-pregrasp Cartesian transition failed
IK at waypoint 5 of 26; remaining candidates also failed bounded IK/collision
checks. The system performed no unsafe motion and cleanup was `CLEAN`.

The final build passed 518 tests with zero errors, failures, or skips.
Machine-readable evidence is in
`config/generalized_runtime_progress_v7.json`.

## Consequences

Generalized scenes no longer begin with free-floating fruit or fruit intersecting
the collection bin, and failed wrist evidence cannot be retried from an
effectively identical viewpoint. The next bottleneck is connected motion
feasibility between a successful wrist observation and a safe pregrasp. Until
that transition and a repeated multi-fruit batch are proven on development
seeds, the formal 30 one-attempt scenarios remain sealed.
