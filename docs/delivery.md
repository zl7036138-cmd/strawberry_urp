# P6 delivery guide

The final P5 release is frozen by ADR 0034 with status
`FINAL_RELEASE_FROZEN_WITH_FAILED_P3_P4`. The delivery archive contains the
project source, ROS packages, tests, configuration, exact dependency files,
published simulator checkpoints, final P5 evidence, report material, and the
demonstration video.

The adjacent `strawberry_urp_release_v1.receipt.json` records the archive
SHA-256, member count, embedded inventory digest, and CRC result. ADR 0035 and
`p6_delivery_handoff_v1.json` record the terminal delivery decision.

## Verify after extraction

From the extracted repository root:

```bash
python3 scripts/verify_p5_release.py
```

Expected status:

```text
VERIFIED_FINAL_RELEASE
```

For a full clean deployment, follow `docs/reproduction.md`.

## Large external artifacts not embedded

To keep the release practical, the archive does not embed:

- `data/raw/zenodo_6126677/strawberries.zip` (1,485,730,857 bytes);
- the 2.7 GB `.cache/wheels` wheel payload; or
- build, install, log, and complete per-trial runtime directories.

The dataset source, size, and digest are documented in `README.md`. The exact
wheel versions are in
`requirements/perception-linux-cp312-wheelhouse.txt`; run
`scripts/cache_perception_wheels.ps1` on Windows to recreate the wheelhouse and
verify every PyPI SHA-256. The small wheelhouse manifest is included.

## Frozen scientific status

- T30 numeric macro-F1 gate: failed; ADR-0026 engineering waiver only.
- Formal P3: failed at 39/135 positive successes.
- Only-unripe formal safety: 30/30 `NO_PICK`.
- P4 qualification: failed at 0/300 heavy-occlusion target poses.
- P5 reproducibility and release delivery: passed.
- Held-out real-image test: sealed and not consumed.

P6 permits proofreading, packaging, delivery, and reproduction-defect fixes
only. It does not authorize feature work, retraining, another formal matrix,
or another P4 intervention.
