# ADR 0071: Preview the connected observation-to-pregrasp route

## Status

Accepted as a development motion-safety milestone. Perception-controlled
continuous multi-fruit harvest and the formal 30-seed matrix remain unaccepted
and sealed.

## Context

The prior runtime checked an observation pose and a target's grasp poses in
separate requests. Seed 44008 demonstrated that both checks could pass while
the Cartesian transition from the reached wrist observation to pregrasp failed
IK. Moving to a view that cannot safely continue wastes a target attempt and
can leave the arm in a posture from which recovery is difficult.

The first connected-preview probe also exposed a MoveIt observation trajectory
whose sampled IK branch produced an excessive controller duration. A valid
endpoint and collision-free plan are insufficient if the resulting trajectory
has unreasonable cumulative travel, duration or point count.

## Decision

- Add the target pose to `MoveToObservation.srv`; the observation service must
  know both endpoints of the intended observation-and-pick chain.
- Plan the observation without executing it, preserve its actual endpoint
  joint state, and seed every segment of the guarded approach preview from the
  prior hypothetical endpoint.
- Use the same lift, in-place reorientation, safe-corridor translation and
  pregrasp descent geometry for preview and execution.
- Reject a view with zero controller execution if the connected preview fails,
  allowing the orchestrator to try the next bounded dynamic view.
- Re-evaluate the wrist-fused pose from the reached state before sending the
  pick action. Both bounded pregrasp orientations are considered.
- Reject direct arm trajectories before controller submission if they exceed
  512 points, 40 rad cumulative joint travel or 60 s nominal duration.
- Treat a strict attachment rejection after successful closing as single-side
  contact evidence eligible for the existing one-time orthogonal retry. The
  physical dual-contact gate remains mandatory.

## Evidence

In development seed 44008 v13, two observation candidates were rejected before
motion because their planned endpoints could not continue to a safe pregrasp.
A later candidate reached wrist confirmation. The post-refinement gate rejected
one target before sending a pick, and another target later reached the physical
grasp stage. Its left finger contacted the fruit but the right finger did not;
the strict attachment service refused the fruit. There was no collision or
limit violation and process-group cleanup was `CLEAN`.

In an independent rerun of the same development seed (v14), five disconnected
views were rejected with zero motion before a connected view was reached. The
later wrist confirmation timed out and the batch failed closed. It again had no
collision, limit violation or erroneous attachment, and cleanup was `CLEAN`.
These are development diagnostics, not selected formal trials.

The final build passed 525 tests with zero errors, failures or skips.
Machine-readable evidence is in
`config/generalized_runtime_progress_v8.json`.

## Consequences

The system no longer assumes that independently feasible observation and grasp
poses form a feasible continuous route, and pathological direct trajectories
cannot reach the controller. The remaining runtime bottleneck is repeated
dual-finger contact and cycle time, not the previously observed disconnected
observation-to-pregrasp transition. Continuous multi-fruit acceptance remains
deferred until at least two fruits are harvested in development batches across
more than one seed.
