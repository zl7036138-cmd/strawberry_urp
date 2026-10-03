# URP final closure checklist

This checklist supersedes the submission-v2 packaging checklist for current
code. The old archive remains immutable historical evidence.

## Scientific integrity

- [x] Real-image audited macro-F1 remains `0.800675 < 0.85`.
- [x] Formal P3 remains `39/135` positives and `30/30` safe only-unripe
  `NO_PICK`.
- [x] P4 remains `300/300` heavy ripe detections and `0/300` valid heavy target
  poses.
- [x] Held-out real-image test remains sealed.
- [x] Simulator-detector metrics are separated from real-image metrics.
- [x] ADR 0087 is labelled a bounded controlled mechanism proof, not a general
  success-rate improvement.
- [x] One development two-fruit run is not presented as the five-scene gate.
- [x] No hardware, damage, cutting, flexible-plant, or sim-to-real claim.

## Final architecture and profiles

- [x] ADR 0085 payload lifecycle is documented.
- [x] ADR 0086 positive and blocked-transport runtime paths are documented.
- [x] ADR 0087 A–E candidate/certificate/execution chain is documented.
- [x] `standard_conservative` is the final default profile.
- [x] `bounded_adaptive_demo` is explicit opt-in.
- [x] `adaptive_candidate_execution_enabled` remains false by default.
- [x] No ADR 0088, Servo, soft-collision, or MTC expansion is included.

## Evidence and tests

- [x] Key ADR 0086/0087 raw receipts are archived under
  `artifacts/final_evidence/`.
- [x] Challenge world, truth-isolation, and cleanup receipts are archived.
- [x] Manifest verifies critical expected hashes.
- [x] Canonical Ubuntu/WSL pure suite: 1040 tests, 0 failures, 0 errors, 3 skips.
- [x] Noncanonical Windows failures are retained and clearly labelled.
- [x] GitHub Actions pure-test workflow is present.
- [ ] GitHub Actions is green on the pushed closure commit.
- [x] Current closure baseline full ROS/colcon receipt is archived: 897 tests,
  0 failures, 0 errors, 0 skips.

## Documentation and publication

- [x] README and Chinese README point to final documents.
- [x] `NEW_PROJECT_HANDOFF.md` reflects ADR 0085–0087.
- [x] `docs/FINAL_ARCHITECTURE.md` provides a short reviewer entry point.
- [x] milestones and risk register include post-P6 extensions.
- [x] license index, third-party notices, and asset provenance are present.
- [x] Clean-tree final archive builder is present and refuses dirty sources.
- [ ] Closure branch committed and pushed.
- [ ] PR to `main` reviewed and merged after CI passes.
- [ ] Annotated final tag created from merged commit.
- [ ] GitHub Release created from the tag.
- [ ] Final delivery archive generated, verified, and attached with SHA-256.

## Human review before hand-in

- [ ] Confirm names, student IDs, supervisor, and institution cover page.
- [ ] Confirm required archive filename and upload-size limit.
- [ ] Play the final video from beginning to end.
- [ ] Open final DOCX/PDF on the submission computer.
- [ ] Clone the default branch into a clean directory and follow the short
  reproduction path.
- [ ] Keep archive SHA-256 and the final evidence manifest beside the upload.
