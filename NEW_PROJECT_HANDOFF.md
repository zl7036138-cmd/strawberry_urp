# Strawberry URP final handoff

Status date: 2026-10-03

Repository: `https://github.com/zl7036138-cmd/strawberry_urp`

Development branch: `codex/wp01-evidence-validation`
Code baseline before closure documents: `e21b7043508e950277ce3e84c92d0464741802b2`

## 1. Start here

Read in this order:

1. `docs/FINAL_ARCHITECTURE.md`
2. this handoff
3. `docs/submission-report.md`
4. `docs/FINAL_CLOSURE_STATUS.md`
5. `artifacts/final_evidence/manifest.json`

The 87 ADRs remain the detailed audit trail. Reviewers are not expected to read
them sequentially.

## 2. One-sentence status

The project is a simulation-only ROS 2/Gazebo/MoveIt strawberry harvesting
system with base/wrist RGB-D perception, maturity detection, 3-D localization,
multi-target tracking, collision-aware target selection, whole-chain pre-grasp
authorization, bounded adaptive grasp candidates, explicit payload lifecycle,
bin verification, and bounded recovery. It is ready for closure packaging, not
for further feature expansion.

## 3. What is actually proven

### Historical formal baseline

- Real-image audited YOLO macro-F1: `0.800675 < 0.85`; numeric gate failed.
- Formal P3 positives: `39/135`; overall gate failed.
- Only-unripe negatives: `30/30` safe `NO_PICK`.
- P4 heavy occlusion: `300/300` ripe detection frames but `0/300` valid target
  poses; gate failed.
- Held-out real-image test remains sealed.

These results are immutable and are not replaced by later demonstrations.

### Post-formal engineering evidence

- Fixed field-v3: historical three-run repeat plus one 2026-09-23 current-code
  visual single-fruit round trip with bin verification and return home.
- Generalized development: seed 45504 completed two distinct ripe fruits in one
  batch (ADR 0084). Four independent safety telemetry streams were not bound,
  so strict safety evidence remained `INDETERMINATE`; the five-scene gate did
  not pass.
- ADR 0085: `EMPTY → CONTACT → HOLDING → ESCAPED → AT_BIN → RELEASED` payload
  lifecycle; a held payload cannot be automatically detached/opened/homed after
  ordinary failure.
- ADR 0086: positive and deliberately blocked-transport runtime qualification.
  Rejected picks remain `EMPTY`, restore target collision, and execute no grasp
  motion, gripper close, or physical attach.
- ADR 0087: deterministic `G00–G14`, first-feasible search, certificate identity
  binding, plan-only qualification, and a controlled positive execution where
  `G00–G02` fail and `G03` completes grasp/place/release/return.

## 4. Final profile decision

`config/final_run_profiles.yaml` is authoritative.

### Standard / conservative — final default

```text
adaptive_candidate_execution_enabled=false
whole_chain_virtual_bin_blocker_enabled=false
```

The nominal grasp still passes the ADR 0086 whole-chain authorization gate and
uses the ADR 0085 payload lifecycle. This is the default system claim.

### Bounded adaptive demo — explicit opt-in

```text
adaptive_candidate_execution_enabled=true
runner=scripts/run_generalized_candidate_execution.sh
```

This reproduces the ADR 0087-E controlled challenge. It is not a general-scene
success-rate claim and must not be presented as the default runtime path.

## 5. Final architecture

```text
base RGB-D → YOLO → depth/CameraInfo/TF → tracked targets
→ deterministic target selection → wrist confirmation
→ G00…G14 → copied-scene seven-stage evaluation
→ AuthorizedGraspPlan → exact MoveIt execution
→ CONTACT/HOLDING/ESCAPED/AT_BIN/RELEASED
→ release verification → recorded-route return → rescan
```

Core files:

- `ros2_ws/src/strawberry_manipulation/strawberry_manipulation/payload_lifecycle.py`
- `.../whole_chain.py`
- `.../grasp_candidates.py`
- `.../grasp_candidate_search.py`
- `.../grasp_authorization.py`
- `.../candidate_qualification.py`
- `.../candidate_execution.py`
- `ros2_ws/src/strawberry_bringup/launch/generalized_harvest.launch.py`

## 6. Model evidence: never mix the two lines

### Real-image maturity model

- Audited real validation macro-F1: `0.800675`.
- Original threshold: `0.85`.
- Status: failed numeric gate; exact checkpoint allowed only through the bounded
  simulator engineering waiver.

### Generalized simulator detector

- Simulator-only qualification: ripe precision `95.65%`, recall `97.78%`.
- Status: development evidence for generated Gazebo scenes only.
- No real-image, field-hardware, or sim-to-real claim.

## 7. Permanent evidence

Key runtime receipts have been copied from `.codex_tmp` to
`artifacts/final_evidence/` without modification. `manifest.json` records byte
counts, SHA-256 hashes, and expected-hash verification for:

- ADR 0086 positive runtime;
- ADR 0086 blocked transport;
- ADR 0087 plan-only qualification;
- ADR 0087 controlled positive execution;
- truth-isolation and cleanup receipts;
- controlled challenge world;
- canonical and noncanonical final pure-test receipts.

The Windows receipt is intentionally retained as a noncanonical failure: it
shows that Linux process-group, POSIX path, UTF-8, and frozen-venv contracts are
not portable to plain Windows Python. The authoritative receipt is generated
inside `Ubuntu-24.04-URP`.

## 8. Reproduction and tests

Windows path:

```text
C:\Users\12753\Documents\New project\strawberry_urp
```

WSL path:

```text
/mnt/c/Users/12753/Documents/New project/strawberry_urp
```

Dependency-light regression:

```bash
/opt/strawberry_venv/bin/python scripts/run_pure_tests.py
```

Canonical closure result: 1040 tests, 0 failures, 0 errors, 3 environment
skips. GitHub Actions runs the explicitly recorded clone-safe subset of the
same dependency-light runner; tests needing intentionally ignored datasets,
weights, or local historical artifacts remain in the full local receipt. The
full local ROS/colcon closure run also passes: 897 tests,
0 failures, 0 errors, 0 skips. Its machine-readable receipt is
`artifacts/final_evidence/FINAL_ROS_TEST_RECEIPT.json`. Full ROS/Gazebo testing
remains local because it requires ROS 2 Jazzy, Gazebo Harmonic, MoveIt, and the
frozen perception environment.

Complete ROS build/test:

```bash
bash scripts/build_and_test.sh
```

## 9. Known limitations and technical debt

- Vegetation, fruit, and peduncle dynamics are rigid approximations; no flexible
  contact, damage, cutting, or passive-centering validation.
- No mobile base, field navigation, physical Panda, or sim-to-real safety.
- ADR 0087 proves a controlled mechanism, not general statistical improvement.
- Five-scene multi-fruit qualification and formal 30-seed evaluation remain
  closed.
- One ADR 0087-E bin-open sample is retained as release-actuator telemetry debt.
- `action_server.py` is broad and should be decomposed only if development
  resumes; do not refactor it during closure.

## 10. Repository publication state

At the start of the closure pass, GitHub defaulted to `main` while the complete
system lived on `codex/wp01-evidence-validation`, 117 commits ahead. Do not
present `main` as final until the closure branch is reviewed, CI passes, and a
PR is merged.

Required publication order:

1. verify closure files, manifest, pure tests, and package;
2. commit and push the closure branch;
3. open/review PR to `main`;
4. merge only after CI is green;
5. create annotated tag and GitHub Release;
6. publish the final archive hash.

## 11. Licensing and assets

The repository is multi-license. Package metadata is authoritative: three
packages are `AGPL-3.0-only`, five are `Apache-2.0`. See `LICENSE`,
`THIRD_PARTY_NOTICES.md`, and `ASSET_PROVENANCE.md`. Dataset, model weights,
Panda resources, and user-supplied Blender assets keep their own provenance and
must not be silently relicensed.

## 12. Do not do during closure

- Do not run the sealed formal 30-seed matrix.
- Do not tune on new challenge outcomes or add ADR 0088.
- Do not enable adaptive execution by default.
- Do not turn the simulator detector's precision/recall into a real-image claim.
- Do not rewrite failed P3/P4 results as passed.
- Do not claim flexible vegetation, nondestructive picking, or hardware safety.
- Do not merge/tag/release from a dirty or unverified tree.
