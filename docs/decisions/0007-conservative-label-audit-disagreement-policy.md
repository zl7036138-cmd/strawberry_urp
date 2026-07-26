# ADR 0007: Conservative exclusion for label-audit disagreements

- Status: Accepted
- Date: 2026-07-15
- Scope: T30 validation-only label audit
- User confirmation: explicit confirmation received before resolution receipt or label changes

## Context

The frozen validation-only audit packet contains 21 model-screened candidates
across 18 images. Reviewer 1 and reviewer 2 agree on 13 candidates and disagree
on 8: `C002`, `C003`, `C005`, `C007`, `C012`, `C014`, `C015`, and `C019`.
Two disagreements (`C003` and `C015`) are direct ripe/unripe conflicts; the
others contrast a class or false-positive decision with `AMBIGUOUS_EXCLUDE`.

A third independent adjudicator would be methodologically stronger, but it is
an avoidable scheduling dependency for this project. Selecting only favorable
disagreements after inspecting their effect on F1 would create metric-driven
label leakage. The rule must therefore be frozen before any label materialization,
retraining, or corrected-metric computation.

## Decision

- Keep all 13 exact reviewer agreements as resolved decisions.
- Resolve every reviewer disagreement, without case-by-case override, to
  `AMBIGUOUS_EXCLUDE`.
- In this policy, exclusion means that the candidate is not added to the
  correction worklist. It does not remove the source image or alter any
  existing label.
- Apply this policy only to the immutable
  `yolo11s_val_threshold_031_candidates_v1` packet and its two hash-bound review
  files. A future audit requires a new packet and decision record.
- Do not reopen individual exclusions based on the resulting validation F1.
- Treat all model-proposed boxes as review aids. Even an agreed missing object
  requires a human-drawn or explicitly human-approved final box before a label
  can be materialized.
- Preserve the original validation split and labels. Any audited labels must be
  written as a separately versioned derivative with complete hashes.
- Keep the held-out test sealed. This decision neither accepts T30 nor
  authorizes training, formal test access, or perception-controlled motion.

## Consequences

The resolution produces a manual annotation worklist of 13 candidates: 5 ripe
and 8 unripe. The 8 disagreements are conservatively excluded. Because a true
missing object may remain uncorrected, a later validation result can remain a
lower-bound estimate and this limitation must be reported.

If the audited validation result remains below the frozen 0.85 gate, the
project must not selectively recover excluded candidates. The next permitted
choices are a separately frozen full validation-label audit or independent
third-reviewer adjudication of a new versioned packet.
