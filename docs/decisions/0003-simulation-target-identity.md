# ADR 0003: Simulation target identity association

Status: accepted

YOLO detections receive positive, frame-local IDs. Those IDs cannot be used as
Gazebo model IDs because detector ordering changes with confidence and scene
content. In simulation, `strawberry_localization` therefore associates its
already-computed base-frame position with the nearest live ground-truth fruit
position within 0.08 m and copies only that fruit's identity into
`TargetPose.target_id`.

The ground-truth position never replaces the RGB-D estimate and ground-truth
maturity is not consulted. Detection, localization error, and false-ripe picks
therefore remain observable. Association is fail-closed when the catalog,
PoseArray, timestamp, or distance gate is invalid. It can be disabled for a
future non-simulation adapter.
