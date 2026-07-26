# strawberry_localization

This package converts strawberry detections and synchronized RGB-D data into
target poses in `panda_link0`.

The package-local `localization.yaml` keeps a numeric `0.60`
`confidence_threshold` fallback for direct node startup. Full-system launches
override it from `strawberry_bringup`'s shared `confidence_threshold` argument,
using the same floating-point value as `strawberry_perception`. This preserves
one runtime cutoff across detection publication and depth localization.
