# ADR 0069: Repair generalized depth failures without relaxing safety

## Status

Accepted as a development safety milestone. Continuous multi-fruit harvest and
the formal 30-seed acceptance matrix remain unaccepted and sealed.

## Context

The terminal seed-44012 run recorded in progress v5 proved one complete
perception-controlled pick and failure recovery, but its first target failed to
form gripper contact. A read-only RGB-D replay showed that a weak 33-pixel near
surface was selected over a 172-pixel fruit surface. Expanding only that support
conflict window fixed the original frame, but cross-seed diagnostics then found
two more dangerous cases with 52.7 mm and 107.9 mm error whose uncertainty was
below the 15 mm control gate.

The failures had three observable causes:

1. a weak near layer could anchor the 60 mm foreground cutoff and delete a
   substantially supported fruit surface behind it;
2. applying the cutoff per pixel could slice the near tail from a complete
   depth cluster into a tiny, spuriously geometry-consistent layer; and
3. using a partially visible depth layer's centroid for bearing introduced a
   consistent 5--10 mm one-sided occlusion bias even when range was correct.

Gazebo truth was used only to score the development diagnostics. It did not
enter localization, tracking, selection, motion, or grasp decisions.

## Decision

- Require at least 20 percent aggregate crop support inside a prospective
  foreground band before it can delete farther layers.
- Preserve depth-cluster membership when filtering; retain or remove an entire
  cluster instead of applying a scalar pixel cutoff.
- Allow the generalized-only support ranker to compare candidates inside a
  40 mm geometry-residual window. It may mask a runner-up only when the runner-up
  has less than half the dominant candidate's pixel support. Similarly supported
  conflicts remain ambiguous in the frozen core and fail closed.
- Use the detector box centre for the base-camera ray bearing while retaining
  the selected depth layer for range. The wrist localizer already used this
  bearing rule.
- Keep every execution safety gate unchanged: 15 mm localization uncertainty,
  50 mm wrist correction, 20 mm target clearance, MoveIt collision and joint
  checks, and dual-finger contact before simulated attachment.

## Evidence

Six development seeds (44004, 44005, 44007, 44008, 44011, and 44012) were
replayed as read-only, truth-scored RGB-D diagnostics. The final code localized
19 targets; all 19 were within 30 mm, the median error was 5.83 mm, and the
maximum was 9.22 mm. Twelve targets passed the unchanged 15 mm uncertainty gate
and their maximum error was 7.40 mm. The remaining seven were correctly marked
too uncertain for control. Every diagnostic reported zero published commands
and no formal seed consumption.

The formerly dangerous seed-44007 sample changed from 63.0 mm error with 2.8 mm
reported uncertainty to 4.5 mm error with 46.6 mm uncertainty. This both repairs
the position and ensures fail-closed control behavior.

Runtime follow-up did not prove two successful picks. Seed 44012 correctly
returned `NO_PICK` because its two candidates had only 16.3 mm clearance. On
seed 44008, one target was rejected for wrist uncertainty and the other either
failed strict dual-contact attachment or lacked a qualifying wrist confirmation
on the repeat. No collision, limit violation, or unsafe attachment was accepted,
and all process-group cleanup receipts were `CLEAN`.

The final build covered all seven ROS packages and passed 510 tests with zero
errors, failures, or skips. The frozen localization core remains unchanged at
SHA-256 `14ae123a98b7083ef87bca8eb782ff024e49bc99056efc594c0076465985b9ec`.
Machine-readable evidence is in `config/generalized_runtime_progress_v6.json`.

## Consequences

Depth localization is materially safer across the sampled development
distribution, but the overall runtime generalization gate is still not passed.
The next milestone is wrist-visibility and grasp-contact repeatability, followed
by at least two successful fruits in multiple development batches. The formal
30 one-attempt scenarios must remain sealed until those development gates pass.
