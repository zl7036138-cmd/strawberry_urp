# Final architecture and evidence map

Status: closure-pass architecture summary, 2026-10-03

Code baseline: `codex/wp01-evidence-validation@e21b704`

## 1. Final system in one chain

```text
Gazebo field + Panda + base/wrist RGB-D
  → YOLO maturity detections
  → depth + CameraInfo + TF
  → TrackedTargetArray
  → deterministic safe target selection
  → wrist observation and near-field confirmation
  → G00…G14 bounded grasp candidates
  → copied-PlanningScene whole-chain feasibility
  → AuthorizedGraspPlan identity binding
  → exact MoveIt execution
  → payload lifecycle
  → bin release, verification and recorded-route return
  → completed-track tombstone and rescan
```

The final design separates three questions that earlier versions mixed:

1. **Can the robot hold and release a payload safely?** ADR 0085.
2. **Can the complete nominal post-grasp chain exist before closure?** ADR 0086.
3. **If nominal is blocked, can a bounded alternative be certified and executed exactly?** ADR 0087.

## 2. Perception and localization

The base RGB-D camera discovers mature and immature candidates. YOLO produces
class/confidence/bounding boxes; robust depth, camera intrinsics, and TF produce
3-D estimates in `panda_link0`. The tracker supplies stable IDs, observation
counts, uncertainty, freshness, and maturity to the selector. The wrist camera
does not form a fused global point cloud; it performs sequential close-range
confirmation of the selected target.

Runtime generalized control does not subscribe to Gazebo fruit truth for
selection or localization. Truth is limited to simulation adaptation and
offline scoring/audit components.

## 3. Target selection and observation

Targets must be mature, fresh, sufficiently observed, below the uncertainty
limit, not previously completed, and collision/IK feasible. Selection is
deterministic. Dynamic wrist viewpoints are checked before motion; a target
that cannot be observed or safely preflighted is rejected rather than admitted
by lowering thresholds.

## 4. Payload lifecycle — ADR 0085

```text
EMPTY → CONTACT → HOLDING → ESCAPED → AT_BIN → RELEASED
```

Payload ownership, collision semantics, and recovery authorization are
separate. After `HOLDING`, an ordinary plan/execution failure cannot detach,
open, or send the arm home. The carried object stays represented in the
PlanningScene and the executor reports motion withheld. Release is authorized
only after `AT_BIN`. This is a safety backstop, not a claim about flexible
plant contact.

## 5. Whole-chain authorization — ADR 0086

Before the first gripper command, a copied PlanningScene checks:

```text
PREGRASP → APPROACH → GRASP_STATE → VIRTUAL_ATTACH
→ ESCAPE → TRANSPORT → BIN_APPROACH
```

Evaluation issues no arm, gripper, or physical-attach command and does not
mutate the live scene. Failure returns a stable stage code, restores the target
collision object, and leaves payload state `EMPTY`. Positive and deliberately
blocked-transport runtime paths are both qualified.

## 6. Bounded adaptive grasp extension — ADR 0087

Fifteen deterministic local-X/Y tilt candidates (`G00`–`G14`) are generated
with immutable pregrasp/grasp/escape geometry and fingerprints. First-feasible
search preserves all failures and stops on the first complete ADR 0086
certificate. `AuthorizedGraspPlan` binds target ID, candidate ID, geometry
fingerprint, scene signature, certificate time, and certificate fingerprint.
Execution fails closed on any mismatch and uses the exact certified geometry.

Two final profiles are frozen in `config/final_run_profiles.yaml`:

- **standard/conservative**: adaptive execution disabled; nominal whole-chain
  authorization remains active.
- **bounded adaptive demo**: explicit opt-in reproduction of ADR 0087-E.

The controlled challenge proves mechanism, not statistical superiority:

```text
G00 → APPROACH_FAILED
G01 → APPROACH_FAILED
G02 → APPROACH_FAILED
G03 → FEASIBLE → ACTION_SUCCEEDED
```

## 7. Evidence hierarchy

| Layer | Accepted statement | Primary evidence |
| --- | --- | --- |
| Historical formal baseline | P3 `39/135` failed; P4 heavy localization failed; negatives `30/30` safe | P3/P4 artifacts and submission report |
| Fixed field-v3 | Current code completed one visual single-fruit round trip; historical three-run repeat remains development evidence | ADR 0060, ADR 0083 |
| Generalized multi-fruit development | One repeatedly used seed completed two distinct ripe fruits | ADR 0084; strict safety evidence still incomplete |
| Payload/authorization | Retained-payload policy and whole-chain positive/negative runtime paths work | ADR 0085–0086 |
| Adaptive extension | One controlled non-nominal case automatically selected and executed G03 | ADR 0087-E |
| Generalization | Five-scene gate and formal 30-seed matrix | **Not passed / sealed** |

## 8. Model evidence must remain separated

### Real-image maturity model

- Audited validation macro-F1: `0.800675`.
- Original threshold: `0.85`.
- Result: failed numeric gate; approved only through a bounded simulator
  engineering waiver.

### Generalized simulator detector

- Simulator-development qualification reports ripe precision `95.65%` and
  recall `97.78%` on its simulator-only qualification split.
- It is simulator-only evidence and cannot be presented as real-image or
  sim-to-real performance.

## 9. Known limitations

- Vegetation and fruit are predominantly rigid-body approximations. Flexible
  leaves, peduncle compliance, force-controlled passive centering, damage, and
  detachment mechanics are not validated.
- No mobile base, field navigation, real Panda execution, cutting, or
  sim-to-real safety claim.
- ADR 0087 has a controlled mechanism proof, not a general-scene A/B result.
- One two-fruit development run lacks four independent safety telemetry streams;
  the five-scene gate remains closed.
- `action_server.py` has broad responsibilities; decomposition is maintenance
  debt, not a closure-pass refactor target.

## 10. Module-to-decision index

| Final concern | Main modules | Key ADRs |
| --- | --- | --- |
| Perception/localization/tracking | `strawberry_perception`, `strawberry_localization` | 0002–0009, 0061–0064, 0069 |
| Target selection and observation | `target_selector.py`, `harvest_orchestrator.py` | 0065–0080 |
| Payload lifecycle and collision policy | `payload_lifecycle.py`, `collision_policy.py`, `carried_scene.py` | 0085 |
| Whole-chain feasibility | `whole_chain.py`, `moveit_backend.py` | 0086 |
| Candidate generation/search | `grasp_candidates.py`, `grasp_candidate_search.py` | 0087-A/B |
| Certificate and exact execution | `grasp_authorization.py`, `candidate_execution.py` | 0087-C/E |
| Evidence and acceptance | benchmark/bringup recorders and checkers | 0068, 0078–0080, 0084, 0086–0087 |

## 11. Final reading order

1. `README.md` or `README.zh-CN.md`
2. this document
3. `NEW_PROJECT_HANDOFF.md`
4. `docs/submission-report.md`
5. `docs/decisions/0085-...` through `0087-...`
6. `artifacts/final_evidence/manifest.json`

The detailed ADR history remains the audit trail; it is not the expected entry
point for reviewers.
