# Final closure status

Date: 2026-10-03

Closure policy: no new robot capability; align release, documentation, evidence,
licensing, and reproducibility.

## Frozen conclusions

- The final safe default is the **standard/conservative** profile.
- ADR 0087 adaptive execution is an **explicit bounded extension**, not the
  default runtime and not a general success-rate claim.
- Historical P3/P4 failures and the real-image YOLO `0.800675 < 0.85` result are
  preserved.
- The generalized simulator detector is reported separately from the real-image
  model.
- One development seed proved two-fruit behavior; the five-scene gate and formal
  30-seed matrix remain closed.

## Closure deliverables

- `docs/FINAL_ARCHITECTURE.md`: short final architecture and ADR index.
- `config/final_run_profiles.yaml`: explicit standard and adaptive-demo profiles.
- `artifacts/final_evidence/`: permanent copies of key ADR 0086/0087 receipts,
  challenge world, hashes, and final test receipt.
- `.github/workflows/pure-tests.yml`: externally visible dependency-light test.
- `artifacts/final_evidence/FINAL_TEST_RECEIPT.json`: canonical Ubuntu/WSL
  dependency-light result, 1040 tests with 0 failures and 0 errors.
- `artifacts/final_evidence/FINAL_ROS_TEST_RECEIPT.json`: full ROS/colcon result,
  897 tests with 0 failures and 0 errors.
- `LICENSE`, `THIRD_PARTY_NOTICES.md`, `ASSET_PROVENANCE.md`: publication and
  provenance boundary.
- Updated README, handoff, risks, milestones, submission report/checklist.
- `scripts/package_final_delivery.py`: clean-tree, commit-bound final archive
  builder for use after the release commit is frozen.

## Publication gates

The following remain administrative release actions, not architecture work:

1. verify the final evidence manifest (local closure tests are complete);
2. commit and push the closure pass on `codex/wp01-evidence-validation`;
3. review a PR from that branch to `main`;
4. merge only after GitHub pure-test CI passes;
5. create an annotated final tag and GitHub Release from the merged commit;
6. attach the final delivery archive and publish its SHA-256.

Do not change the default branch, merge, tag, or publish a Release from an
unverified dirty working tree.
