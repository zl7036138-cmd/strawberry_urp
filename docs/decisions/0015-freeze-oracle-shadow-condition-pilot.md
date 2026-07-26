# ADR 0015: Freeze a three-scenario Oracle-control condition pilot

- Status: Accepted
- Date: 2026-07-15

## Context

ADR 0014 proves that all nine lighting-by-occlusion worlds change the RGB
stream as intended. It does not prove that a materialized world can coexist
with the full MoveIt/Gazebo stack. Running the formal 165-trial matrix now would
be premature because T30 remains below 0.85 and perception-controlled motion is
not authorized.

## Decision

Bind the benchmark level names exactly to the nine runtime condition IDs. The
mapping is ordered and fail-closed: lighting is `dim, nominal, bright`,
occlusion is `none, partial, heavy`, and each ID is `light__occlusion`.
Benchmark scenario seeds remain reserved for later scene randomization; they do
not alter the already deterministic lighting/occlusion SDF.

Run only three single-target pilot scenarios at the previously verified
`[0.47, -0.075, 0.50] m` position, with both non-target fruit parked:

1. `nominal+none` as the camera-clear baseline;
2. `dim+none` as a one-factor lighting stress;
3. `nominal+heavy` as a one-factor occlusion stress.

Each trial starts a fresh hash-bound world. The runtime condition probe first
rechecks five RGB frames, then the normal state machine runs with Oracle as the
sole control source. The below-gate `baseline__best` model is observational on
the Shadow topics only. A pilot pass requires complete receipts, source
isolation, expected RGB condition differences, and 3/3 Oracle motion success.
Shadow ripe detections and target poses are recorded but are not pass criteria.

## Consequences

This pilot can reveal integration breakage or condition-specific YOLO behavior
without consuming the formal matrix. It cannot close T30, P2, full P3, or P4,
cannot authorize perception control, and cannot consume the held-out real-image
test. Any failure is diagnostic evidence and must not be hidden by retrying the
same scenario indefinitely.

The completed v1 run passes: all three condition probes are valid and all three
Oracle-controlled picks succeed. Shadow totals are retained only as liveness
evidence because the existing client starts counting during launch, before both
non-target models are parked, and the three wall-clock durations differ. They
must not be compared as condition-normalized detector rates. A later perception
study must define a post-setup, fixed-frame measurement window first.
