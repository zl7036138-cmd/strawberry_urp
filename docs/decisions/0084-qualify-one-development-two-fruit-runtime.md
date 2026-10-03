# ADR 0084: Qualify one development two-fruit runtime without claiming the gate

## Status

Accepted as a development behavior milestone. One truth-isolated seed completed
two distinct ripe-fruit pick, place, reverse-route and home cycles in one batch.
The five-scene development gate and the formal 30-seed matrix remain closed.

## Context

Seed 45504 had already proved the visual selection, wrist confirmation, physical
contact, attachment, carried-fruit transport and first-fruit return path. The
second fruit repeatedly stopped before release. The latest v8 trace localized
that stop to the second deterministic drop slot `(0.27, -0.37, 0.45)`: after
adding the carried-fruit radius and unchanged 15 mm localization uncertainty,
the nominal payload proxy penetrated the work-table AABB by 31 mm. Every valid
wrist-roll branch therefore failed the final vertical-descent preview before
motion.

The drop bank previously assumed that a point inside the collection bin was
automatically a valid payload center. That assumption ignored the active scene
profile, the payload envelope and the nearby table boundary.

## Decision

- Materialize the bounded drop-slot bank once at orchestrator startup from the
  active scene YAML, including its collision profile, collection-bin bounds,
  plant positions and fruit collision radius.
- Require every candidate center to be inside the active bin interior and to
  have strictly positive nominal sphere-to-AABB clearance after adding the
  unchanged target-refinement uncertainty.
- Filter first, then sort safe candidates deterministically by distance to the
  robot base. Preserve retry-to-slot stability and fail closed when the
  configured `max_targets` exceeds the qualified capacity.
- Keep the attached-body MoveIt preview authoritative. The static filter proves
  only that a slot is not geometrically invalid by construction; it does not
  bypass IK, joint-limit, self-collision or live-scene collision checks.
- Do not alter maturity, uncertainty, collision, controller or retry thresholds.
  Gazebo truth remains scoring-only and cannot influence runtime selection.

## Evidence

The behavior commit is `a97529f26cafa93c903dab06e528acd4a5dd5cec`.
The dependency-light suite passed 939 tests with zero failures or errors and
three environment skips. The complete ROS/colcon suite passed 796 tests with
zero failures, errors or skips.

Development run
`.codex_tmp/generalized_harvest_seed_45504_qualified_drop_slot_20260928_v9_retry1`
used `ROS_DOMAIN_ID=230` and ended naturally after 623.46 wall seconds:

- runtime track 4 used slot 0 `(0.35, -0.45, 0.45)` and physically harvested
  ripe simulation target 1;
- runtime track 3 used the newly qualified slot 1 `(0.27, -0.45, 0.45)`;
- the second grasp's initial right-only contact triggered the existing bounded
  contact-centering retry, after which ripe simulation target 4 attached;
- the second payload completed retreat, `clear_corridor` transport, vertical
  descent, release and physical `PLACED` confirmation;
- both recorded place segments reversed successfully, followed by all six
  bounded home segments;
- the orchestrator terminated `SUCCESS` with harvested tracks `[4, 3]`, both
  runtime-to-physical identity associations correct, truth isolation passing
  with no violations, and process cleanup `CLEAN`.

The immutable local evidence hashes are recorded in
[`config/generalized_runtime_progress_v18.json`](../../config/generalized_runtime_progress_v18.json).
The first v9 invocation used invalid Fast DDS domain ID 233 and exited before
any ROS node or motion started; it is infrastructure noise, not a behavior
attempt and is excluded from the result above.

## Safety boundary and consequences

The runtime recorder reported zero unripe picks, collisions, joint-limit
violations and unsafe-motion attempts. Those counters are not yet independent
proof. The strict score correctly remains `INDETERMINATE` because independent
collision, joint-limit, home/stop and scene-terminal telemetry were absent.
Therefore `run_safety_pass` and `evidence_integrity_pass` remain false, and this
run cannot be promoted to a formal safety or generalization result.

This is the first current-code proof of two different ripe fruits completing
the full behavior chain in one generalized development batch. It closes the
specific second-slot geometry defect, but it is only one repeatedly used
development seed. Next, post-grasp failure handling must preserve a held fruit
unless release is explicitly authorized, the missing independent telemetry
must be bound into the recorder, and a fresh five-scene qualification batch
must meet the unchanged development gate before the formal matrix can open.
