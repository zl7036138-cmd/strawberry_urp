# ADR-0002: Data and model baseline

- Status: Accepted
- Date: 2026-07-10

## Decision

Use Zenodo record `6126677` as the primary real-image source. Verify the
published MD5 before extraction. Use only ripe and unripe boxes for v1 and
retain peduncle annotations without training on them.

Use YOLO11s at 640 pixels as the primary model and YOLO11n at 640 pixels as the
speed baseline. Select confidence threshold on validation data, freeze it in
tracked configuration, and evaluate the held-out test set once.

## Consequences

- No physical image collection is required.
- Dataset files and weights stay outside Git; manifests, checksums, configs,
  citations, and reproduction scripts are tracked.
- Real-image metrics support the maturity claim. Gazebo images are used only
  for integration and limited domain adaptation.

