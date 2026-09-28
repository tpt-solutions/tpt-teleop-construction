# Architecture Overview

Phase 13 item: "Crate-level API docs + architecture documentation pass."
This is the workspace-wide map; each crate's own `lib.rs` doc comment is
the authoritative description of what that crate does — this document
is about how the crates relate to each other and to the project's
overarching design choices, not a restatement of per-crate detail.

## Design ethos

Three decisions run through every crate in this workspace and explain
most of what looks unusual compared to a typical Rust project:

1. **Zero-bloat, hand-rolled math/data structures** (spec.txt §7's
   dependency matrix, enforced by `deny.toml`'s ban list). Where a
   typical project would pull in `nalgebra`, `rustfft`, `crossbeam`, or
   `serde`, this workspace has its own 3×3 matrix ops (`tpt-t-construction-mine::matrix3`),
   radix-2 FFT (`tpt-t-construction-wear::fft`), lock-free ring buffers
   (`tpt-teleop-ring`), and POD byte serialization
   (`tpt-t-construction-telemetry::record`, `::audit`) instead. This is
   a deliberate trade of dependency-tree size and audit surface against
   implementation and maintenance effort.
2. **Zero-allocation, fixed-capacity control loops** wherever a function
   sits in a real-time or safety-critical path — const-generic containers
   (`SensorFrame<N>`, `PointCloud<N>`, `DispatchQueue<N>`,
   `BamReassembler<const MAX_BYTES: usize>`) instead of `Vec`/`HashMap`.
   `docs/deterministic_execution_audit.md` is the honest accounting of
   where this held (hydraulic, rollover) and where it doesn't yet
   (safety's `perception`/`tracking` modules) — a design principle stated
   here is not the same as a guarantee verified everywhere, and that
   audit is what actually checked.
3. **Honest scope boundaries over false completeness.** Every "we didn't
   actually verify/build/certify this" is written down rather than
   implied away: `core::simd`-style claims are flagged as
   auto-vectorization-friendly approximations (nightly `portable_simd`
   isn't available on stable Rust), the compliance docs in
   `docs/compliance/` are gap analyses rather than certification claims,
   and Phase 12's hardware-blocked items are marked as exactly that in
   `todo.md` rather than left silently unchecked.

## Crate layering

Dependencies point downward only — nothing below depends on anything
above it, so there's no cycle to reason about:

```
Layer 0 (foundation)
  tpt-teleop-ring          vendored lock-free SPSC/MPSC ring buffer

Layer 1 (built on layer 0)
  tpt-t-construction-core     event loop, lifecycle FSM, message bus (uses tpt-teleop-ring, rkyv)
  tpt-t-construction-payload  cut/fill volume, point cloud ring (uses tpt-teleop-ring)

Layer 2 (domain/subsystem crates — no cross-crate dependencies except where noted)
  hydraulic, powertrain, terrain, safety, rollover, fatigue, wear,
  telemetry, mine, tunnel, excavator, haul, loader, drill, can
  dozer                       (the one exception: depends on payload for cut/fill tracking)

Layer 3 (integration adapters, built on layers 1-2)
  tpt-t-construction-teleop   teleoperation adapter (uses core, hydraulic)
  tpt-t-construction-hil      hardware-in-loop harness (uses hydraulic, sim)

Layer 4 (aggregation / non-production)
  tpt-t-construction-sim      headless simulator; its own [dependencies] are
                              empty — it only pulls in ~10 domain crates as
                              [dev-dependencies] for its end-to-end
                              integration test (tests/open_pit_mine_haulage.rs)
  tpt-t-construction-viz      egui visualizer binary (uses sim)
```

Most Layer 2 crates are deliberately standalone: an excavator's
kinematics don't need to know about a haul truck's speed management, and
keeping them independent means every subsystem can be developed, tested,
and (eventually) safety-reviewed in isolation. The two exceptions —
`dozer` depending on `payload`, and the Layer 3/4 crates — exist because
those integrations are the actual point of that crate (a dozer's cut/fill
tracking *is* payload volume tracking applied to blade passes; the HIL
harness's entire job is running a real hydraulic controller against a
simulated plant).

## Data flow through a work cycle

The core event-driven pattern (`tpt-t-construction-core::event_loop::MachineController`)
is: subsystems publish `MachineEvent`s (`RequestTransition`, `FaultRaised`,
`SelfTestCompleted`) onto a lock-free bus; the controller consumes them
one at a time and drives `MachineFsm`'s `Idle -> Moving -> Working ->
Dumping -> Returning -> Idle` lifecycle. This is intentionally decoupled
from the *teleoperation* handover state machine
(`tpt-t-construction-teleop::safety_fsm::HandoverFsm`,
`AUTONOMOUS <-> REQUESTING_TELEOP <-> TELEOP_ACTIVE <->
RETURNING_TO_AUTONOMY <-> EMERGENCY_STOP`) — a machine can be `Working`
in the lifecycle FSM while `TeleopActive` in the handover FSM at the same
time; they track orthogonal concerns and neither implies the other.

A representative end-to-end path, as exercised by
`tpt-t-construction-sim/tests/open_pit_mine_haulage.rs`:

```
sensors (sim: lidar.rs, hydraulics.rs, powertrain.rs)
  -> perception/estimation (safety::perception, mine::ekf, rollover::tilt_estimation)
  -> subsystem decision (excavator::swing_cycle, haul::speed, safety::response)
  -> MachineEvent published to the core bus
  -> MachineController::tick advances MachineFsm, may raise a Fault
  -> Fault, if Critical/EmergencyStop, triggers teleop::fault_handover::handover_response
  -> hydraulic::PidController + valve_mapping translate commands to valve duties
  -> telemetry::TelemetryLogger records the tick; telemetry::AuditLog records
     the safety-relevant event (Phase 11) in its hash-chained trail
```

## Further reading

- `docs/deterministic_execution_audit.md` — the Phase 11 source-level
  review of which control loops are actually allocation-free and
  bounded, and which aren't yet.
- `docs/compliance/` — gap-analysis mappings against ISO 19014, MSHA,
  and IEC 61508, and what they are and are not (see that directory's own
  `README.md`).
- `todo.md` — the phase-by-phase build log; every phase's checklist
  items link back to the modules that implement them, including honest
  notes on the ones that don't or can't (yet).
