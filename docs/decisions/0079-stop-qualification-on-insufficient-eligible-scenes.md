# ADR 0079: Stop qualification when eligible scenes are insufficient

## Status

Accepted. Qualification batch 0 is complete and failed its scene-selection
gate. No behavior run was started, no safety threshold was relaxed, and the
formal 30-seed matrix remains sealed.

## Context

ADR 0078 froze a zero-motion qualification protocol on commit
`9300ce903780e589c02771e3e7a7879a7d475ec4`. Untouched seeds 46001–46018 had
to be materialized and evaluated in matrix order. A scene was eligible only if
perception produced at least two stable ripe tracks and the existing MoveIt
service found connected current-state, pre-grasp, grasp, and retreat paths for
both. Five eligible scenes were required before any continuous-harvest motion.

The discovery block had yielded only one eligible scene in 18, so qualification
was deliberately designed to fail closed rather than search indefinitely for
favorable seeds.

## Decision

- Accept the complete 18-scene zero-motion qualification receipt as the result
  of batch 0.
- Stop before behavior execution because only one scene, seed 46016, was
  eligible and five were required.
- Do not run the single eligible scene in isolation: it cannot satisfy the
  predeclared five-scene gate and doing so would create selectively sampled
  behavior evidence.
- Do not lower maturity, 15 mm uncertainty, 50 mm wrist correction, collision,
  reach, or retry thresholds.
- Do not consume another qualification block merely to accumulate favorable
  scenes. The next block is allowed only after an evidence-backed change to
  scene generation or safe planning behavior, with a reproducing test and a new
  frozen commit.
- Keep formal claims and the 30 hidden seeds sealed.

## Evidence

All 18 probes reached `COMPLETED`; all 18 ROS graph truth-isolation audits
passed; all 18 process groups shut down `CLEAN`. Runtime truth use and trajectory
execution were false for every probe. Perception produced 68 candidate tracks,
26 of which were mature and passed the data gates. MoveIt evaluated those 26,
accepted seven, and reported five collision rejections. Only
`qualification_b00_46016` contained two feasible mature tracks.

The immutable sweep summary is retained locally at
`.codex_tmp/generalized_feasibility_qualification_b00/sweep_summary.json`. It
is 31,722 bytes and has SHA-256
`4a9aae4a0e203c0551ca207f2449e31268f6606a0b56ea912c5c92188c0a6e72`.
That summary binds the matrix, formal matrix, model, Git commit, all 18
materialized YAML/SDF pairs, and every probe, graph-audit, and cleanup receipt.
The machine-readable project conclusion is
`config/generalized_runtime_progress_v15.json`.

The current regression baseline remains 735 dependency-light tests with zero
failures or errors and two environment skips, plus 617 ROS 2/colcon tests with
zero failures, errors, or skips.

## Consequences

The audit and qualification tooling are proved usable, but the multi-fruit
development gate is not passed. Evidence now localizes the main blocker to a
mismatch between random scene placement and the arm's conservative connected
MoveIt workspace: only 7/26 accepted ripe tracks were path-feasible and only
1/18 scenes contained two.

The next legitimate implementation step is to improve that placement/planning
contract using development evidence, not to weaken safety gates. Any such
behavior change invalidates batch 0 for qualification and must use the next
unused hundred-seed block after code freeze.
