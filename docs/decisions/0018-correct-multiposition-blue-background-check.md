# ADR 0018: Correct the multiposition blue-background injection check

- Status: Accepted
- Date: 2026-07-16

## Context

The first T70-D2 run completed all 15 scenarios, 900 fixed Shadow frames, and
the expected seven distinct materialized worlds. Its immutable `summary.json`
nevertheless reports `condition_injection_valid=false`. At `near_left`, both
`nominal+none` and `dim+none` measure exactly 0.25 blue coverage in the square
target ROI. The base orchard contains a blue collection bin; the circular fruit
does not fill the projected square, so background pixels in the corners satisfy
the blue mask. The receipt and SDF contain no benchmark occluder in either
no-occlusion world. The v1 absolute `none <= 0.02` image check therefore
confounds existing scene background with injected occlusion.

## Decision

Preserve `summary.json` and every runtime artifact. Add a schema-v2 corrective
summary rather than overwrite or rerun the diagnostic.

For each position, validate no-occlusion structurally: both receipts must mark
the occluder disabled and both SDF files must omit the named benchmark
occluder. Require the measured blue background to remain stable across nominal
and dim lighting within the already frozen 0.02 tolerance. Validate heavy
occlusion structurally as one visual-only model with no collision element.
Measure its visible contribution relative to the larger of the two local
background coverages, reusing the already frozen 0.35 heavy-coverage minimum
as the required foreground delta, and retain the 0.98 total-coverage ceiling.

Do not change any Shadow frame, classification, model threshold, or performance
interpretation. The correction affects only whether the intended visual
condition was injected and visible.

## Consequences

The corrected check is valid at positions whose background is already blue and
continues to fail closed on a missing/physical occluder, unstable background,
insufficient foreground addition, or total target coverage. Because this is an
infrastructure correction made after the v1 result, both summaries remain in
the evidence lineage. Neither summary is a model gate or formal robustness
result.
