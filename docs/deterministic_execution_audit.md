# Deterministic-Execution Audit — Safety-Critical Loops

Phase 11 item: "Deterministic-execution audit across safety-critical
loops (safety, rollover, hydraulic)."

This audit reviews the three crates whose control loops sit directly in
a safety-relevant real-time path — `tpt-t-construction-hydraulic`,
`tpt-t-construction-rollover`, and `tpt-t-construction-safety` — for
properties that matter to worst-case execution time (WCET) and
determinism: heap allocation (unbounded worst-case latency, GC-free Rust
notwithstanding — the allocator itself can block, fault, or contend), 
unbounded loop iteration counts, recursion, and any other source of
input-dependent timing variance beyond simple arithmetic.

It is a source-level review against the code as it exists today, not a
measured WCET analysis on target hardware — that requires an actual
Cortex-A72 (or equivalent) target and belongs to Phase 12 hardware
bring-up, as already noted for the mine/wear crates' SIMD timing claims.
Where this audit found a real gap, it says so rather than describing the
code as better than it is.

## `tpt-t-construction-hydraulic` — clean

Reviewed: `pid.rs`, `valve.rs`, `realtime.rs`, `compensation.rs`,
`sensors.rs`, `filter.rs`.

- `PidController::step`, `Biquad::process`, `pressure_compensated_flow_fraction`,
  `load_sense_pressure` are all straight-line arithmetic: no branches
  whose cost depends on input magnitude, no loops, no allocation.
- `share_flow` and `SensorFrame<N>` iterate over a caller-provided slice
  or a fixed `N`-length array; the only "loop" in the hot path is a
  single bounded `for` over a size fixed at compile time (`N`) or at the
  call site by the caller passing equal-length slices (asserted, not
  silently truncated).
- `pin_and_elevate` (SCHED_FIFO + core pinning) is a one-shot setup call,
  not part of the steady-state loop; its `Result`-based, privilege-gated
  fallback (documented in the module) means the *loop itself* never
  blocks on it.
- `LoopBudget` measures wall-clock overruns after the fact (for
  diagnostics/telemetry) — it does not itself introduce timing variance
  into the loop it's measuring.
- **No heap allocation anywhere in this crate's non-test code** (verified
  by grep for `Vec<`, `HashMap`, `Box<`, `String::`, `.clone()` — none
  found outside `#[cfg(test)]`).

**Verdict: meets the deterministic-execution bar as designed.** This is
the crate the 1kHz/<100µs loop budget in `todo.md` Phase 3 was written
against, and the source supports that claim.

## `tpt-t-construction-rollover` — clean

Reviewed: `tilt_estimation.rs`, `cog_estimation.rs`, `brake.rs`.

- `ComplementaryTiltFilter::update`, `static_stability_factor`,
  `critical_roll_angle_rad`, `rollover_margin_rad`, `should_apply_brakes`
  are all pure, allocation-free, branch-shallow functions over `f32`
  scalars — no loops of any kind, bounded or otherwise.
- **No heap allocation anywhere in this crate's non-test code** (same
  grep as above, clean).

**Verdict: meets the deterministic-execution bar.** This is the simplest
of the three crates to reason about — every function here is O(1) with
no input-dependent control flow at all.

## `tpt-t-construction-safety` — partial gap, documented below

Reviewed: `perception.rs`, `tracking.rs`, `classification.rs`,
`warning_zones.rs`, `response.rs`, `tmr.rs`.

**Clean:** `classify_object`, `zone_radius_at_bearing_m`,
`is_within_dynamic_zone`, `ZoneRadii::classify`,
`speed_command_fraction`, `PresenceEscalation::update`, `effective_zone`,
and the Phase 11 `tmr` module's `vote_exact`/`vote_f32` are all O(1),
allocation-free, pure functions — the same standard as the hydraulic and
rollover crates. `response.rs` even has an explicit test
(`the_decision_path_is_fast_enough_for_a_100ms_response_budget`)
asserting this.

**Gap: `perception::fuse_detections` and `tracking::TrackManager::update`
both allocate on the heap every call, and neither has a compile-time
bound on the number of detections/tracks it processes:**

- `fuse_detections` allocates a `Vec<usize>` union-find array, a
  `HashMap<usize, Vec<usize>>` for clustering, and a `Vec<FusedObject>`
  result — sized by `detections.len()`, which is a runtime slice length
  with no upper bound enforced anywhere in this crate. Its nested
  `for i in 0..n { for j in (i+1)..n { ... } }` pairwise-distance pass is
  also `O(n^2)`, again over an unbounded `n`.
- `TrackManager::update` allocates a `Vec<bool>` per call and grows its
  internal `Vec<Track>` without a capacity limit.
- This is a real inconsistency with the rest of the workspace's
  zero-alloc/const-generic convention for exactly this kind of
  fixed-cadence sensor data — e.g. `tpt-t-construction-hydraulic::SensorFrame<N>`,
  `tpt-t-construction-payload::PointCloud<N>`, and
  `tpt-t-construction-core::DispatchQueue<N>` all bound their capacity at
  compile time via a const generic for precisely this reason. Perception
  and tracking predate that convention (Phase 7, before the const-generic
  pattern was established in Phase 4/9) and were never revisited.
- **Practical impact:** on a general-purpose OS with an unbounded input
  size, this is "just" an allocator call and a data-dependent loop bound
  — not incorrect, and the existing `fusion_of_a_busy_scene_completes_in_well_under_fifty_milliseconds`
  test demonstrates it comfortably meets the <50ms budget for a
  representative (300-detection) scene on this dev machine. But it means
  this crate's WCET cannot be bounded from the source alone the way
  the hydraulic/rollover crates' can, and a heap allocation's worst case
  (page fault, allocator lock contention under a hard real-time
  scheduling class) is exactly the kind of unbounded-latency source a
  SIL 2/3 certification review will ask about — see
  `docs/compliance/iec_61508.md`.

**Recommended remediation (not yet implemented):** bound
`fuse_detections`/`TrackManager` by a `MAX_DETECTIONS`/`MAX_TRACKS` const
generic backed by fixed-size arrays, replacing the `HashMap`-based
clustering with a fixed-capacity union-find array (the same pattern
`tpt-t-construction-mine`'s EKF and ICP modules already use for their
fixed-size matrix math). This is scoped as follow-up work rather than
done inline in this audit, since it changes this crate's public API
(`fuse_detections`'s signature and `FusedObject`'s ownership of a `Vec`
of sensors) and deserves its own review rather than being folded
silently into an audit pass.

## Summary

| Crate | Allocation-free | Bounded loops | Verdict |
|---|---|---|---|
| `tpt-t-construction-hydraulic` | Yes | Yes | Clean |
| `tpt-t-construction-rollover` | Yes | Yes (O(1) everywhere) | Clean |
| `tpt-t-construction-safety` | Partial — `response`/`classification`/`warning_zones`/`tmr` clean; `perception`/`tracking` allocate | Partial — same split | Gap in `perception`/`tracking`, tracked above |
