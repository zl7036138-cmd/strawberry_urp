# URP submission checklist

This checklist defines the final submission-v2 packaging gate. It supplements
the frozen P5/P6 release and never replaces its failed formal P3/P4 results.

## Scientific integrity

- [x] Audited YOLO validation macro-F1 remains `0.800675`; the `0.85` gate is
  still failed.
- [x] Formal P3 remains `39/135` positive successes and `30/30` safe
  only-unripe `NO_PICK`.
- [x] P4 remains `300/300` heavy ripe detection frames and `0/300` heavy target
  poses.
- [x] The held-out real-image test remains sealed.
- [x] field-v3 is labelled fixed-scene, fixed-target, fixed-seed and non-formal.
- [x] No hardware, fruit-damage, stem-cutting or sim-to-real claim is made.

## Engineering evidence

- [x] Seven ROS 2 packages build.
- [x] The latest baseline records 396 tests with zero errors, failures or skips.
- [x] field-v3 no-motion sensing, localization, collision parity and pre-grasp
  planning pass.
- [x] Three consecutive field-v3 perception-derived runs complete all seven
  action stages.
- [x] Every accepted field-v3 run records bilateral raw/processed contact,
  attach/detach, gripper reopening and arm recovery.
- [x] The simulator and controller manager use a dedicated robot-description
  channel, removing the nondeterministic startup-pose race.
- [x] A fresh submission demonstration recording passes its media and outcome
  checks.

## Submission artifacts

- [x] Source repository and reproduction guide.
- [x] Submission report in Markdown.
- [x] Submission report in DOCX and PDF.
- [x] Final field-v3 submission video and receipt.
- [x] Current colcon test receipt bound to the submission commit.
- [x] `strawberry_urp_submission_v2.zip` with embedded inventory.
- [x] Archive CRC and member SHA-256 verification.
- [x] Git working tree clean and branch pushed.
- [ ] Draft PR converted to ready only after the local package verifies.

## Human review before hand-in

- [ ] Confirm student names, student IDs, supervisor and college-specific cover
  page fields.
- [ ] Confirm the institution's required filename and upload-size limit.
- [ ] Play the final MP4 from beginning to end.
- [ ] Open the final DOCX and PDF on the submission computer.
- [ ] Keep the external archive SHA-256 receipt beside the uploaded ZIP.
