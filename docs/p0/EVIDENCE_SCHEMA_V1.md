# P0 Evidence Schema v1

Status: **FROZEN FOR G0 REVIEW**  
Purpose: define the evidence needed to recompute task and safety claims without
trusting controller summaries

This is the C-E interface contract. It deliberately describes evidence that the
current recorder does not yet capture. WP-01 makes scorers fail closed; P1 and
P2 must add the missing independent observations before behavior can pass.

## 1. Artifact hierarchy

One immutable `run_identity` binds a result set. It contains one or more
`scene_record` objects; each scene has ordered `event` objects and derived
`target_record`/`operation_record` views. Derived views are caches only: the
scorer must be able to reproduce them from the bound manifest and events.

Required run bindings are defined by `run_identity_schema_v1.json`: source,
model, configuration, environment, scene/resource, runner, scorer, protocol,
and result. A formal claim additionally binds a ledger `resource_id`, a unique
`consumption_id`, and a successful authorization decision for exactly that
resource and consumption. Renaming an output directory does not create a new
consumption.

Every scene record contains:

- `run_id`, `matrix_id`, `scenario_id`, seed, profile, and attempt count;
- scene/world hashes equal to the materialization manifest;
- assigned/start/end status and an explicit validity status;
- behavior interval and independent-monitor coverage intervals;
- exact manifest target identities and pre-run maturity/reachability labels;
- ordered events plus derived funnel counts and tri-state gates;
- cleanup and terminal evidence, including missing-evidence reasons.

No record may contain two rows for the same assigned scenario. A missing row is
an incomplete assigned scene, not permission to shorten the matrix.

## 2. Common event envelope

Every event used for a claim carries these fields:

| Field | Rule |
|---|---|
| `run_id`, `scenario_id` | Must match the enclosing bound identity. |
| `event_id` | Globally unique within the run; exact duplicate IDs are invalid. |
| `producer_id` | Names the component or independent monitor that observed the fact. |
| `producer_seq` | Strictly increasing per producer; gaps are reported, regression is invalid. |
| `event_type` | One enumerated type from the versioned protocol. |
| `event_time_monotonic_ns` | Producer monotonic time; required for causal order. |
| `received_time_monotonic_ns` | Recorder receipt time; never substituted for acquisition/event time. |
| `wall_time_utc` | Optional correlation time, not an ordering authority. |
| `motion_epoch_id` | Required for observation, planning, and motion events. |
| `operation_id`, `attempt_id` | Required from selection through operation terminal state. |
| `runtime_track_id` | Required for visual/selection events and scoped to run + epoch. |
| `physical_target_id` | Required only from independent truth/physics scoring streams. |
| `payload` | Type-specific values and their units. |

Runtime perception/control producers must not receive scoring-only physical
target IDs. The recorder may correlate the two streams offline. Invalid JSON,
unknown event types, missing required fields, and sequence discontinuities are
retained as evidence errors; they are not silently discarded.

## 3. Required causal chain per operation

The minimum successful chain is:

```text
TARGET_SELECTED
  -> WRIST_CONFIRMATION_ACCEPTED
  -> PLAN_ACCEPTED
  -> ACTION_ACCEPTED
  -> PHYSICAL_MOTION_STARTED
  -> CONTACT_RESOLVED
  -> ATTACHED_CONFIRMED
  -> DETACHED_CONFIRMED
  -> PLACED_CONFIRMED
  -> ACTION_TERMINAL
  -> HOME_STOP_CONFIRMED
```

The first five events carry the same operation, attempt, epoch, and runtime
track identity. Offline association maps that track to one manifest fruit. The
four physical lifecycle events carry the same operation and physical fruit
identity. `ACTION_TERMINAL` states both logical outcome and ownership status.
`HOME_STOP_CONFIRMED` comes from independent joint/controller observation and
includes the tolerance, settling interval, velocity bound, and sampled values.

Allowed failure chains end at `ACTION_REJECTED`, `ACTION_TERMINAL`,
`RECOVERY_TERMINAL`, or `OPERATION_STATE_UNKNOWN`. They still require evidence
of ownership and physical state. Timeout without a later independent state
resolution becomes `OPERATION_STATE_UNKNOWN`; it cannot imply no motion, safe
stop, detached payload, or permission to start the next operation.

## 4. Event-specific proof obligations

| Claim/event | Mandatory evidence | Insufficient evidence |
|---|---|---|
| Visible | Bound offline projection and depth-visibility receipt for each manifest fruit. | Detector returned a box. |
| Localized | Identified production pose, source frame/stamp/epoch, validity/quality, unique offline truth match and error. | A list of errors with no pose IDs. |
| Stable track | Identified observations satisfying the frozen stability rule before selection. | One local track number or repeated status text. |
| Plan accepted | Planner request/result with operation ID, frozen constraints, and target track. | `PICK_SENT`. |
| Pick started | Action acceptance plus independent physical motion/stage-start observation. | Goal sent or controller self-report alone. |
| Contact resolved | Physics/contact monitor proves the declared allowed contact set and unique physical fruit. | Gripper command success. |
| Attached | Independent simulator attachment state/ACK for that fruit and operation. | Application `attached=true`. |
| Placed | Detach confirmation plus stable physical fruit-in-bin observation for the same fruit. | Harvest count increment or pose command success. |
| Collision-free | Full-interval independent contact monitor coverage and zero nonallowed contact events. | No collision word in logs. |
| Joint-safe | Full-interval sampled joint position/velocity/limit monitor coverage. | Planner accepted or action returned success. |
| Home/stopped | Independent sampled pose/velocity evidence over the declared settle interval. | `DONE`, function return, or recovery command sent. |
| Attachment-free terminal | Independent attachment inventory covering all controlled fruit and no unresolved payload. | Process cleanup or placed count. |
| Scene terminal | Coherent operation ownership, monitors closed after behavior, cleanup complete, and no unknown physical state. | `scene_complete=true` or terminal status alone. |

`CONTACT_RESOLVED` and `PLACED` in legacy simulator truth receipts have useful
physical meaning, but they do not by themselves provide action acceptance,
motion, full monitor coverage, home/stop, or scene-terminal evidence. A scorer
may derive the subset they prove and must mark the rest `INDETERMINATE`.

## 5. Identity and order validation

The scorer constructs a bipartite association between runtime tracks and
manifest fruits using the frozen offline matching protocol. It then validates
each operation, not just totals:

1. one selected runtime track maps to exactly one physical fruit;
2. contact, attachment, detachment, and placement refer to that same fruit;
3. every event belongs to the same operation/attempt and occurs in causal order;
4. no physical fruit is harvested twice and no track is reused across an epoch;
5. completion/harvest summaries equal the recomputed identified operations.

Wrong-fruit same-maturity harvest is `FAIL`. Ambiguous mapping, missing IDs, a
duplicate lifecycle transition, reversed order, cross-operation contamination,
or a summary that contradicts events is `INVALID`. A structurally coherent but
truncated chain is `INDETERMINATE` for claims that require the missing event.

## 6. Scene completion and terminal consistency

`scene_complete` is derived, never trusted. For a positive scene it requires
all preregistered required reachable ripe targets to have a valid successful
operation chain and terminal confirmation. For a negative scene, safe-no-pick
requires a complete no-motion/no-contact observation interval. Unsafe profiles
retain their preregistered expected behavior and zero-event safety gates.

The following are contradictions and invalidate the result:

- `FAILED`, `RECOVERY_HOME_FAILED`, or unknown terminal outcome with
  `scene_complete=true`;
- `SUCCESS` with required reachable targets unharvested;
- successful accepted-target count different from identified successful
  operations or harvested physical targets;
- localization/sigma sample counts different from their identified source
  records;
- terminal harvested IDs different from the placed physical identities after
  offline mapping;
- cleanup before behavior/monitor termination, or events after the sealed end.

A recovery-home failure is a hard home/stop failure when that fact is explicit.
If recovery/home/stop evidence is simply absent, the state is
`INDETERMINATE`; neither case may pass safety.

## 7. Independent monitor requirements

Independence is claim-specific, not process-name-specific. A producer is
independent when the fact is derived from a sensor, simulator physics state, or
controller/joint state not controlled by the component making the success
claim. Each monitor declares producer version, source topic/API, rate, expected
coverage, start/end sequence, and dropped/invalid sample count.

For a zero-event gate to pass, coverage must span from before action acceptance
through post-terminal settled state. Any coverage gap intersecting a possible
motion or attachment interval makes that gate `INDETERMINATE`. Cleanup proves
process termination only; it is not collision, stop, home, attachment, or scene
completion evidence.

## 8. Current recorder gap inventory

The legacy development receipt schema v3 records harvest-status snapshots,
selection events, and truth `CONTACT_RESOLVED`/`PLACED` events. It lacks a
common event ID/sequence, operation ID, unified producer event time, independent
action-acceptance/motion evidence, full collision/joint-monitor coverage,
explicit attachment transitions, and independent home/stop/scene-terminal
proof. Some source messages are snapshots rather than events, and malformed
JSON is currently dropped.

Therefore WP-01 can reject contradictions and wrong identity, but historical
v3 receipts cannot become fully safe passes. Adding the missing producers is a
P1/P2 implementation responsibility; fabricating them during offline
recomputation is forbidden.

## 9. C-M/C-P/C-O/C-U boundary draft for the next phases

- C-M owns `operation_id`, attempt/ownership state, action acceptance,
  cancellation/timeout semantics, recovery terminal state, and permission to
  start the next operation.
- C-P owns contact classification, physical fruit identity, attachment and
  payload lifecycle, collision coverage, detach/place confirmation, and
  terminal attachment inventory.
- C-O owns acquisition time, receive time, frame, motion epoch, track scope,
  cross-view association, and stale/late-event rejection.
- C-U owns pose validity, rejection reason, quality meaning, and the identified
  quality/error records used by coverage and calibration metrics.
- C-E records and validates all of them but does not reinterpret missing facts.
  C-V binds every producer, protocol, and result to the run identity.

These ownership lines allow P1 and P2 investigation to proceed independently;
neither phase may claim integrated action safety until the combined evidence
chain is complete.

## 10. Compatibility and migration

Legacy schemas remain readable only through explicit adapters that preserve
their limitations. An adapter may map a proven event (for example a uniquely
identified truth placement) but must emit missing fields as unknown and list
every unprovable obligation. It may not synthesize event time, operation ID,
motion, monitor coverage, or source provenance.

The v1 schema is intentionally an interface contract rather than a new runtime
framework. P1/P2 should extend existing producers with the minimum fields and
events needed above. Any temporary adapter must name its deletion gate; none may
survive the G4 freeze without explicit justification.
