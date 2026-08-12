# ADR 0067: Bind wrist confirmation to the selected target and preserve frozen localization evidence

## Status

Accepted on 2026-08-12.

## Context

Fresh development replays of seed 44012 corrected two preliminary
interpretations in ADR 0066. The reported `0.854 m` value was an
apparent-fruit-size consistency residual from a rejected depth layer, not a
base-to-wrist position disagreement. The fresh pre-hint and later runs also did
not reproduce a MoveIt start-state-out-of-bounds diagnostic.

The real wrist-confirmation defect was identity ambiguity. The eye-in-hand view
could contain multiple accurately localized ripe fruits, while the localizer
selected the track with the smallest uncertainty without knowing which target
the base camera and orchestrator were confirming. A second issue was that the
frozen geometry estimator intentionally charged the full detector-box size
residual to position uncertainty. A correctly localized wrist fruit could
therefore exceed the 15 mm runtime gate despite having only 6--8 mm scoring
error.

The first full regression also exposed that ADR 0066 had changed a source file
bound by the frozen rendered-occlusion matrix. The binding test correctly
failed rather than silently accepting changed historical evidence.

## Decision

Publish the current base-camera target pose to the wrist localizer as a bounded
3-D attention hint. Require a stable ripe wrist track within 50 mm of that hint
and retain the base-owned track ID in the confirmation message. The hint is
perception-derived; it contains no simulator truth, fixed target ID, fixed ROI,
or fixed scene coordinate.

Reset wrist-local tracks whenever a hint is published. The orchestrator sends
the hint once before observation motion and again after motion succeeds, so a
confirmation must be rebuilt from the normal number of fresh stationary
frames. Reject ID mismatches and all invalid quality measurements explicitly
and fail closed.

Keep the historical localization core byte-for-byte equal to its frozen
27,119-byte SHA-256 binding. Move the support-dominant near-tie preprocessing
into `generalized_depth.py`, which is outside that old evidence binding. Apply
the wrist runtime's 0.5 uncertainty calibration there only after the unchanged
core has passed every geometry and ambiguity rejection gate. This does not
increase the accepted depth interval or relax target-selection safety limits.

Keep the 30 formal seeds sealed. A development stage log may demonstrate
pipeline reachability, but only a terminal recorder receipt can prove a
completed pick or batch.

## Evidence

- The pre-hint replay demonstrated the identity ambiguity without a truth-fed
  runtime decision.
- With the spatial hint, a mismatched fruit was no longer accepted; the next
  bottleneck was reported explicitly as
  `WRIST_CONFIRMATION_REJECTED_HIGH_UNCERTAINTY`.
- With bounded wrist uncertainty calibration, seed 44012 crossed confirmation
  and logged `APPROACH`, `GRASP_POSE`, `RETREAT`, and `PLACE`.
- That last recorder ended in `WALL_TIMEOUT` while its last event was
  `PICK_SENT`, so it is not terminal success evidence.
- The frozen localization core again matches SHA-256
  `14ae123a98b7083ef87bca8eb782ff024e49bc99056efc594c0076465985b9ec`.
- The targeted set passed 78/78 tests. The complete ROS 2 workspace passed
  493 tests with zero errors, failures, or skips.

## Consequences

Wrist confirmation now follows the target selected by the base camera instead
of selecting an unrelated fruit by uncertainty alone. The pipeline has
progressed from wrist observation to guarded placement without weakening the
safety gates, and the earlier frozen evidence is again auditable.

The generalized runtime gate is still not passed. The next milestone requires
a recorder that remains active through the longest bounded action, one valid
terminal single-pick receipt, and then a completed multi-fruit development
batch. The machine-readable record is
`config/generalized_runtime_progress_v4.json`.
