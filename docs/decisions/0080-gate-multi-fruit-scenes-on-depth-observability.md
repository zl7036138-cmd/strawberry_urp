# ADR 0080: Gate multi-fruit scenes on depth observability

## Status

Accepted as a development-gate correction. The default camera layout is
unchanged, no behavior motion was executed, and the formal 30-seed matrix
remains sealed.

## Context

ADR 0079 concluded that MoveIt reachability was the primary qualification
blocker. That conclusion was incomplete because the old zero-motion gate began
with stable detector tracks. It did not separately prove that two generated
ripe fruits were actually visible in the base-camera depth image or that their
accepted 3D positions were correct. A generated ripe fruit can be outside the
image, hidden by the arm or plant, or associated with the wrong depth surface.
MoveIt feasibility downstream cannot repair any of those failures.

Development diagnostics on seeds 45101 and 45104 made the distinction
observable. The two scenes generated six ripe fruits, but only four passed a
depth-support visibility test and only two of those four were localized within
30 mm. On seed 45104, MoveIt reported the scene eligible while three ripe
fruits were depth-visible, only one was correctly localized, and one additional
accepted ripe position had a 71.5 mm error.

## Decision

- Preserve the existing base-camera pose as the baseline. Expose mast, camera
  translation, and camera rotation as global launch parameters only so fixed
  hardware layouts can be compared across seed batches. They are not target,
  track, or seed-specific control inputs.
- Score camera observability in a read-only development component. Reuse the
  capture tool's depth-support rule to distinguish projection into the image
  from genuine depth visibility.
- Require a schema-v3 observability receipt with zero commands and no runtime
  truth use before a schema-v2 scene can pass the multi-fruit gate.
- Require at least two depth-visible ripe truth targets, at least two ripe
  targets localized within 30 mm, and zero false accepted ripe localizations.
  Apply the existing requirement for two stable, ripe, connected MoveIt paths
  only after those perception conditions pass.
- Keep truth structurally outside perception, target selection, orchestration,
  and manipulation. The diagnostic subscribes to truth only to score a frozen
  no-motion frame; its output cannot feed target poses or motion commands.
- Default the sweep tool to development matrix v2 so an ordinary invocation
  cannot silently bypass the observability gate. Keep matrix v1 available only
  for reproducing historical receipts.

## Evidence

The baseline two-scene aggregate reports four depth-visible ripe fruits, two
correctly localized ripe fruits, 50% visible-ripe localization recall, and
66.67% precision among accepted ripe positions. Its SHA-256 is
`51d08ea4b6439dbe022b1378679cf33af1bc8533fdaeed696c847fa728792a09`.

An integrated no-motion seed-45104 run passed the live truth-isolation audit
and exited `CLEAN`. Its MoveIt receipt had
`eligible_for_multi_fruit_runtime: true`, while the new observability gate
returned false. The observability, MoveIt, graph-audit, and cleanup receipt
hashes are respectively:

- `83fe259a32d3e0681aa1e004803e220776e101b38c9a5ff870e381d8a3c96e05`
- `6aca346e7631f19abbc7df1402b2272e9f88f38b37d5254290dd3d76b5899201`
- `30b6c04783845610df06dae1774dedf93be07f742cab0fdb5dbedcaa1628c3ea`
- `bed1091c30abb008defb1a9add228640a47d8cb588ee4005d53f7ae95a8cb6c5`

The dependency-light suite passes 750 tests with zero failures or errors and
two skips. The full ROS 2/colcon result contains 632 tests with zero failures,
errors, or skips.

## Consequences

The earlier statement that the blocker was only a randomized-workspace versus
MoveIt mismatch is superseded. Reachability remains a downstream constraint,
but current evidence places base-camera depth observability and 3D association
in front of it. A scene that merely generates two ripe fruits or two apparently
feasible tracks is no longer sufficient evidence for a multi-fruit behavior
trial.

The next development experiment must use a fresh discovery block and diagnose
cross-scene localization failures without tuning a pose to individual seeds.
Because 45101 and 45104 were used during this correction, discovery batch 0 is
tainted; the next discovery block is 45201–45218. Qualification 46101–46118 and
the formal 30-seed matrix remain untouched.
