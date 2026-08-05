# strawberry_localization

This package converts strawberry detections and synchronized RGB-D data into
target poses in `panda_link0`.

The package-local `localization.yaml` keeps a numeric `0.60`
`confidence_threshold` fallback for direct node startup. Full-system launches
override it from `strawberry_bringup`'s shared `confidence_threshold` argument,
using the same floating-point value as `strawberry_perception`. This preserves
one runtime cutoff across detection publication and depth localization.

The accepted runtime remains on `depth_estimator_mode: center_median`. An
opt-in `geometry_layer` development estimator is available for bounded
no-motion occlusion studies. It searches the detection box for depth layers,
uses the known fruit radius and apparent box size to predict a plausible
surface depth, and rejects missing or ambiguous layers. It never substitutes
simulation truth. Run its dependency-light diagnostic with:

```bash
python3 scripts/evaluate_occlusion_depth_estimator.py
```

See [`docs/occlusion-aware-localization-v1.md`](../../../docs/occlusion-aware-localization-v1.md)
and ADR 0061 before enabling it in a ROS launch.
