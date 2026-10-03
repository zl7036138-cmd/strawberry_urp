# ADR 0072: Remove runtime truth identity and recover global scanning

## Status

Accepted as a development safety and recovery milestone. Continuous
multi-fruit harvest and the formal 30-seed matrix remain unaccepted and
sealed.

## Context

The first perception-controlled complete pick in development seed 44008 used
a nearest-ground-truth lookup to translate a visual track ID into a Gazebo
fruit entity. That proved the motion chain but violated the generalized
runtime boundary. The old generalized fruit assets also disabled gravity, and
the old bin check accepted a coordinate volume instead of physical support.

Later development probes exposed two additional batch faults. Frame-level
position uncertainty was exponentially averaged even though the position
itself was fused, so consistent eye-in-hand observations did not become more
certain. After a failed target, the arm could remain in a wrist observation
pose, occlude the base camera and prevent the next global scan.

## Decision

- Generalized execution never subscribes to fruit ground-truth poses and no
  longer resolves a visual target by nearest truth position.
- A grasp attachment is requested through a shared contact-resolved service.
  The simulator accepts it only when exactly one fruit has fresh contact with
  both fingers; zero or multiple candidates fail closed. The resolved entity
  remains private to the simulator through detach and bin verification.
- Generalized-only ripe and unripe fruit assets enable gravity. The fixed
  regression scene and its historical evidence remain unchanged.
- Generalized bin success requires fresh, continuous contact between the
  released fruit and the collection-bin floor for the configured stability
  interval. Coordinate inclusion remains only in the legacy path.
- Propagate tracker uncertainty through the same weighted position smoother.
  Consistent measurements reduce variance; spatial innovation raises it. A
  5 mm systematic floor remains, and the 15 mm execution gate is unchanged.
- When final connected feasibility fails after wrist confirmation, consume the
  one permitted retry by moving directly to a cached observation view at least
  40 mm from the prior view. A third attempt is impossible.
- Add `/strawberry/move_home`. Every failed target attempt must complete a
  collision-checked recovery-home motion before global scanning resumes. A
  failed recovery stops the batch.
- Preserve every Cartesian collision sample while executing dense trajectories
  at the configured 0.30 rad/s limit with a 0.05 s minimum waypoint duration.

## Evidence

All evidence below is development-only and uses seed 44008. No hidden formal
scenario was opened.

- v18 restored the previously useful random layout with gravity-enabled
  assets. The batch rejected a 17.42 mm wrist estimate at the unchanged 15 mm
  gate and skipped other IK/collision-infeasible candidates. Cleanup was
  `CLEAN`.
- v19 accepted the same target after uncertainty propagation (wrist sigma
  below 15 mm) and reached the post-refinement connected-path gate. It failed
  safely because the current observation state could not connect to the
  grasp/retreat route. Cleanup was `CLEAN`.
- v20 used a cached second observation view with a 200 mm baseline. Both wrist
  confirmations passed with approximately 9.7--9.9 mm reported sigma. The
  second final evaluation failed closed when the tracked collision scene could
  not be synchronized. Cleanup was `CLEAN`.
- v21 returned home after skipping the first target, re-established base-camera
  scanning, selected a different target and reached the physical grasp stage.
  Both bounded closes remained asymmetric, so no contact-resolved attachment
  was requested successfully. The action recovered and the batch returned home
  again. Cleanup was `CLEAN`; no collision, false attachment or unsafe motion
  was accepted.

An isolated development diagnostic with the prior non-gravity asset confirmed
that fresh bilateral contact can resolve exactly one physical entity. It also
demonstrated why coordinate-based bin verification was insufficient. The new
gravity-plus-floor-contact chain has not yet completed end to end and is not
claimed as accepted.

Machine-readable evidence is in
`config/generalized_runtime_progress_v9.json`. The final build passed 535 tests
with zero errors, failures or skips.

## Consequences

The generalized controller no longer needs fixed target IDs, truth identity
association or truth coordinates to attach and verify a fruit. A failed target
can no longer strand the arm in front of the base camera, and consistent wrist
observations can pass the existing uncertainty gate without weakening it.

The remaining runtime bottleneck is physical centering of the fruit between
both fingers. Development acceptance still requires gravity-enabled,
contact-resolved, floor-verified harvests of at least two fruits across more
than one seed. Only after that gate passes may the one-attempt formal 30-seed
matrix be opened.
