# ADR 0060: Accept the field-v3 perception-pick repeat

- Recorded: 2026-07-28
- Status: Accepted
- Scope: Fixed-seed, fixed-target field-v3 perception-derived
  pick-and-place development repeat

## Context

ADRs 0058 and 0059 accepted the opt-in field scene through no-motion RGB-D
localization, collision parity, and discarded pre-grasp planning, while
leaving execution blocked.

The bounded execution runner then exposed three independent runtime issues:

1. the original bin-centre release pose was at the edge of Panda reach;
2. MoveItPy execution of the final named home posture could wait without a
   bounded controller result; and
3. DART did not enforce the Panda right-finger mimic constraint, while the
   installed parallel-gripper controller commands only one joint.

The field release point moved to `[-0.45, 0.25, 0.45]` with a 0.12 m place
transit clearance. Named home recovery now plans with MoveIt and executes
through the existing bounded `FollowJointTrajectory` action path. Grasp
descent may try three additional base-Z orientations, each with collision
checks and no unbounded retry.

The gripper repair is simulation-specific and explicit. The original
`panda_gripper_controller` continues to command `panda_finger_joint1`; a
second `panda_gripper_right_controller` commands `panda_finger_joint2`.
The action backend sends both goals before waiting and accepts the command
only when each joint independently passes measured-position validation.
This avoids treating an unsupported DART mimic relationship as physical
evidence.

Three consecutive fresh-world results passed:

| Result | SHA-256 | Action motion time | DONE elapsed |
|---|---|---:|---:|
| `results/development/field_v3_perception_pick_v15c/trial_01.json` | `44d98f669260d488352e2f2192a5bde021d8dc1baa14303844859763cf3b8a4c` | 208.2554 s | 265.8144 s |
| `results/development/field_v3_perception_pick_v16/trial_01.json` | `06099efb7b49b2cd7f1d35978d7f3f65ce5db6216fbbd9728ffdeef540543074` | 267.9171 s | 312.8196 s |
| `results/development/field_v3_perception_pick_v17/trial_01.json` | `11b492152b358ef1302e64609432e9f79b664a3c8ab4b3490759329ffb5b5b8d` | 227.7318 s | 275.9872 s |

Every result records `success=true`, `result_success=true`,
`failure_code=0`, and
`PLAN, APPROACH, GRASP, RETREAT, PLACE, VERIFY, DONE`. Both fingers have raw
and processed target contact, attachment changes from attached to detached,
the final attachment state is false, and both fingers reopen to approximately
0.040 m. The final arm state matches the named `ready` posture within
`6.24e-13 rad`.

The accepted checkpoint remains
`outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt`, SHA-256
`e3aca77e627469a1e9cf43c0776438623d881332163e03afe1fbcc55aba1af70`.
The runner binds target 1 to the reviewed field ROI and freezes its
field-specific confidence threshold at 0.31. Simulator truth is used only for
identity/readiness auditing and collision synchronization, not as the
commanded target position.

## Decision

1. Accept the three results as a consecutive, perception-derived field-v3
   development repeat for the exact fixed scene, target, seed, model, ROI,
   thresholds, geometry, and runner.
2. Supersede ADRs 0058 and 0059 only where they block this exact bounded
   runner. Set the field manifest's `motion_authorized` flag to true with
   scope `exact_bounded_perception_pick_runner`.
3. Keep ordinary `field_v3.launch.py` no-motion by default: attachment and
   pose-control services remain disabled unless the execution runner enables
   the required attachment backend explicitly.
4. Keep the default project scene at Blender-v2. Field-v3 remains opt-in.
5. Require both explicit finger controllers and independent measured-position
   validation. Do not restore reliance on DART mimic behavior.
6. Keep the pre-action DDS subscription retry limited to one stable-target
   discovery failure before any action feedback, motion, or contact.
7. Keep the result non-formal. It does not qualify target/pose variation,
   arbitrary plant coverage, fruit-damage safety, deformable contact,
   physical hardware, sim-to-real transfer, or the sealed real test.
8. Do not reinterpret the detector's failed 0.85 numeric gate as passed.

## Consequences

The opt-in field-v3 path now demonstrates a reproducible fixed-scene closed
loop using the large strawberry field, the qualified workcell plant and fruit,
base overview camera, wrist RGB-D localization, three-dimensional collision
planning, measured bilateral contact, simulated attachment, transport, bin
release, verification, and recovery.

The next useful work is documentation and learning, not further ad hoc
optimization. Any future manipulation study should freeze a new varied-pose
or varied-plant contract before motion. Hardware work requires a separate
controller, calibration, safety, and risk decision.
