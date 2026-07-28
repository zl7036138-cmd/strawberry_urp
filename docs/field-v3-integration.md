# Strawberry field-v3 integration

## Current scope

Field-v3 turns the supplied `st1.blend` scene into an explicit alternative to
the accepted Blender-v2 plant scene. It does not replace the v2 default or its
preserved perception-derived pick.

The source contains a 5 x 12 metre ground surface, three planting ridges, 103
visible plants, 24 shared plant mesh variants, a sign, one Blender camera, and
two Blender lights. The Gazebo export:

- keeps 101 background plants as static visual instances;
- exports each of the 24 plant variants once at a 0.45 visual reduction ratio;
- omits two source plants around the selected outer-row workcell;
- fills that bay with the qualified v2 plant and three independent fruit
  models;
- uses collision boxes only for the ground and three ridges;
- relocates the collection bin away from the ridge and base-camera mast; and
- continues to use the ROS/Gazebo base and wrist RGB-D sensors rather than the
  Blender presentation camera; and
- spawns the Panda directly in the previously qualified dual-camera
  observation posture so the wrist camera looks down into the lower field
  workcell without commanding a trajectory; and
- binds target 1 to the reviewed lower-right wrist ROI. Without this identity
  constraint, the slightly higher-confidence upper fruit is correctly
  associated to simulator target 3 and the readiness gate rejects the
  detection-ID/target-ID mismatch.

The resulting field has about 173,000 instanced background visual triangles
at runtime while storing about 42,000 plant-variant triangles on disk.

## Launch and bounded execution

After building and sourcing the ROS workspace:

```bash
ros2 launch strawberry_sim field_v3.launch.py headless:=false
```

The wrapper selects `strawberry_field_v3.sdf`,
`scene_field_v3.yaml`, dual cameras, a fixed seed, and disabled attachment and
pose-control services. The ordinary `sim.launch.py` still selects
`strawberry_orchard.sdf` and the accepted v2 manifest by default.

The accepted end-to-end development runner is:

```bash
bash scripts/run_field_v3_perception_pick_headed.sh \
  "$PWD/results/development/field_v3_perception_pick_next" \
  "$PWD/outputs/perception/yolo11s_640_train_audit_v1/weights/best.pt" \
  230 0 false
```

The arguments are output directory, checkpoint, ROS domain ID, post-run hold
seconds, and whether to show windows. Use `true` for a headed demonstration.
The output directory must not already exist. The runner verifies the
checkpoint hash, uses target 1 in the reviewed lower-right wrist ROI, runs the
handoff/pre-grasp/readiness gates, and permits at most one pre-action DDS
subscription retry before any feedback, motion, or contact.

## Preserved no-motion evidence

The field scene passed its render and sensor gate in
`results/development/field_v3_no_motion_v2`:

- both base and wrist health records passed with no violations;
- all three controllers remained active but no motion was requested;
- base RGB arrived at 3.625 Hz and wrist RGB at 8.181 Hz during the
  eight-second wall-time measurements; and
- both camera TF chains and captured RGB frames were present.

The first perception shadow run intentionally failed closed because the
higher-confidence upper fruit was detector ID 1 but simulator target 3.
Localization was stable; the candidate identity was wrong. After binding
target 1 to the reviewed lower-right wrist ROI, the final consolidated
`results/development/field_v3_perception_shadow_v6` run completed:

- 60/60 measured frames contained a ripe detection and target pose;
- the 15-consecutive-frame readiness gate passed with no identity mismatch;
- all 60 accuracy samples were associated with target 1;
- median, P95, and maximum centre error were all 4.856 mm, against
  15 mm / 30 mm limits;
- all nine expected MoveIt objects were present before and after planning;
- the fresh collision-scene handoff passed with no missing objects or joint
  motion; and
- MoveIt produced one 47-waypoint collision-aware pre-grasp plan with
  0.754 mm / 0.00885 rad endpoint error, then discarded it.

This preserved no-motion result qualifies field-v3 rendering, dual-camera
sensing, candidate selection, wrist RGB-D localization, collision parity, and
controller-free pre-grasp planning.

## Full perception-derived execution evidence

After the no-motion gates, three consecutive fresh-world runs completed:

| Run | Result | Action motion time | DONE elapsed |
|---|---|---:|---:|
| v15c | `field_v3_perception_pick_v15c/trial_01.json` | 208.2554 s | 265.8144 s |
| v16 | `field_v3_perception_pick_v16/trial_01.json` | 267.9171 s | 312.8196 s |
| v17 | `field_v3_perception_pick_v17/trial_01.json` | 227.7318 s | 275.9872 s |

For all three:

- `success=true`, `result_success=true`, and `failure_code=0`;
- feedback is exactly
  `PLAN -> APPROACH -> GRASP -> RETREAT -> PLACE -> VERIFY -> DONE`;
- both fingers report raw and processed contact with target 1;
- attachment transitions `attached -> detached`, ending detached;
- the gripper returns to approximately 0.040 m per finger; and
- the arm returns to the named `ready` posture within `6.24e-13 rad`.

The result-file SHA-256 values and exact policy decision are recorded in
[`ADR 0060`](decisions/0060-accept-field-v3-perception-pick-repeat.md).
Generated results remain intentionally excluded from Git.

The gripper uses two explicit single-joint controllers. DART does not apply
the Panda URDF mimic constraint, so the left and right action goals are sent
together and each measured finger position must pass independently. A close
may be accepted as contact-stalled only after at least 2 mm measured travel.

## Qualification gates

Motion must remain disabled until these gates pass in order:

1. **Passed:** SDFormat/mesh load with no missing resources or material
   collapse.
2. **Passed:** stable simulation progress and measured sensor output.
3. **Passed:** base and wrist colour, depth, camera-info, and TF availability.
4. **Passed:** target-1 visibility, identity, and in-bounds depth from the
   selected observation pose.
5. **Passed:** agreement between Gazebo ridge collisions and scene-specific
   MoveIt boxes.
6. **Passed:** no-motion three-dimensional localization accuracy.
7. **Passed:** collision-aware pre-grasp shadow planning with trajectory
   discard.
8. **Not run as a separate field artifact:** an Oracle-only execution. The
   stricter perception-derived runs below exercise the same manipulation path,
   but do not replace Oracle as an isolation diagnostic if one is needed.
9. **Passed:** three consecutive perception-derived fixed-scene
   pick/place/release/recovery runs.

The first field-v3 pre-grasp attempt correctly rejected all three plans because
the source ridge-1 collision overlapped `panda_link0` and `panda_link1`.
The exporter now clips only the approach-side ridge strip to the ground front
edge at x=0.20 m. Gazebo and MoveIt use the same clipped box; the retained
ridge still covers the workcell plant and fruit. The fresh v6 run then passed
on its first planning attempt.

The manifest now authorizes motion only for the exact bounded runner accepted
by ADR 0060; ordinary field launch remains no-motion by default. Blender-v2
remains the regression reference and default project scene.

This evidence is non-formal and fixed-scene. It does not cover other fruits,
random fruit/plant poses, strong occlusion variation, deformable plants,
fruit-damage modelling, physical calibration, hardware safety, or sim-to-real
transfer. The detector still has a failed audited macro-F1 gate and the real
held-out test remains sealed.
