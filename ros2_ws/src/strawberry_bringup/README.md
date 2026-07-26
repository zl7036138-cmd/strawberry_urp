# strawberry_bringup

This package composes the simulation and optional perception, localization,
manipulation, and orchestration stages.

When `start_perception:=true`, the `confidence_threshold` launch argument is
passed as a floating-point ROS parameter to both `strawberry_perception` and
`strawberry_localization`. Its default is `0.60`, matching both packages'
standalone configuration fallback. Supply a validated replacement once at the
system boundary instead of editing the two package configurations separately:

```bash
ros2 launch strawberry_bringup system.launch.py \
  start_perception:=true \
  model_path:=/absolute/path/to/best.pt \
  confidence_threshold:=0.60
```

Leaving `start_perception` at its fail-closed default (`false`) starts neither
perception nor localization, so the shared threshold is not consumed by a
partial perception chain.

T60 oracle trials use explicit source isolation:

```bash
ros2 launch strawberry_bringup system.launch.py \
  start_oracle_provider:=true \
  start_manipulation:=true \
  start_orchestrator:=true \
  enable_attachment:=true \
  target_source:=oracle \
  control_target_topic:=/strawberry/oracle/target_pose
```

For shadow diagnostics, also enable perception, set
`shadow_enabled:=true`, and keep its default output topics under
`/strawberry/shadow`. The historical threshold/weight are diagnostic only;
they are not promoted and cannot authorize perception control.

ADR 0026 defines the only current exception: one exact below-gate checkpoint
is engineering-accepted for bounded simulator integration. Do not reproduce
its long launch command manually. The repository-level launcher verifies the
model and evidence hashes, proves the held-out-test receipt is absent, stops
Oracle, and supplies all explicit control-topic overrides:

```bash
bash scripts/run_p3_perception_control_smoke.sh \
  results/p3/my_perception_control_smoke 225
```

This does not change the fail-closed defaults above or claim the original T30
metric has passed.
