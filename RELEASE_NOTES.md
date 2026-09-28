# v1.0.0

The first tagged release of `tpt-teleop-construction`: a hyper-optimized,
zero-bloat, safety-critical middleware workspace for autonomous heavy
machinery and mining equipment (excavators, haul trucks, dozers,
loaders, drills, TBMs), built out across 13 phases from an initial
design spec and empty crate skeletons. 23 crates, 513 passing tests,
`cargo build/test/clippy(-D warnings)/fmt(--check)` clean across the
whole workspace, and a `cargo deny check` that now actually passes
(advisories/bans/licenses/sources) rather than never having been run.

## What's in this release

- **Core infrastructure**: a lock-free message bus and machine lifecycle
  state machine (`tpt-t-construction-core`), a vendored lock-free
  SPSC/MPMC ring buffer (`tpt-teleop-ring`), rkyv zero-copy telemetry
  messages, and a shift-start self-test framework.
- **Simulation & visualization**: a headless 6DOF vehicle/hydraulic/
  powertrain/sensor simulator (`tpt-t-construction-sim`) validated at
  10x real-time, and an egui-based visualizer
  (`tpt-t-construction-viz`).
- **Hydraulic & powertrain control**: a 1kHz `SCHED_FIFO` PID loop with
  pressure compensation, flow sharing, and vibration filtering
  (`tpt-t-construction-hydraulic`); torque arbitration and thermal
  derating (`tpt-t-construction-powertrain`).
- **Terrain & payload sensing**: real-time terrain modeling, slope
  stability, and obstacle detection (`tpt-t-construction-terrain`);
  cut/fill volume calculation and payload weighing
  (`tpt-t-construction-payload`).
- **Autonomous machine control**: dedicated crates for the excavator,
  haul truck, dozer, loader, and drill implements, each covering its
  own work-cycle logic (trenching, haulage, grading, loading, blast
  patterns).
- **Underground/GPS-denied navigation**: a custom EKF fusing LiDAR,
  odometry, and IMU; 2D ICP scan matching; UWB multilateration;
  ventilation-aware routing (`tpt-t-construction-mine`); TBM/roadheader
  advance-rate and segment-installation control
  (`tpt-t-construction-tunnel`).
- **Safety systems**: 360° perception fusion, object tracking/
  classification, and a deterministic warn/slowdown/stop response with
  a dedicated zero-false-stop validation suite
  (`tpt-t-construction-safety`); rollover tilt/CoG monitoring and
  automatic braking (`tpt-t-construction-rollover`); operator fatigue
  monitoring (`tpt-t-construction-fatigue`).
- **Maintenance & telemetry**: a custom radix-2 FFT for vibration-based
  fault detection (`tpt-t-construction-wear`); 100Hz telemetry logging
  and post-shift export (`tpt-t-construction-telemetry`).
- **Multi-machine coordination**: dispatch queueing, truck-arrival
  prediction, and a 10Hz coordination loop
  (`tpt-t-construction-core`), validated end-to-end by an
  "Open-Pit Mine Haulage" integration test spanning shift start through
  shift end.
- **Teleoperation integration**: a universal handover state machine
  (`AUTONOMOUS <-> REQUESTING_TELEOP <-> TELEOP_ACTIVE <->
  RETURNING_TO_AUTONOMY <-> EMERGENCY_STOP`), fault-driven handover
  triggers, diesel-vibration joystick filtering, and a locally-defined
  Domain Teleoperation Interface reconciled against the wire format the
  separate `tpt-teleop` sister repo actually implements
  (`tpt-t-construction-teleop`).
- **Safety certification groundwork**: a triple-modular-redundancy
  voter, a hash-chained tamper-evident audit trail, a source-level
  deterministic-execution audit of every safety-critical control loop,
  and honest gap-analysis mappings against ISO 19014, MSHA, and IEC
  61508 SIL 2/3 (`docs/compliance/`) — mappings, not certification
  claims.
- **The software half of hardware bring-up**: a zero-alloc CAN/J1939
  parser (`tpt-t-construction-can`) and a hardware-in-loop test harness
  whose `HardwareBackend` trait already runs a real PID control loop
  against a simulated plant (`tpt-t-construction-hil`).

## What's explicitly not in this release

Several `todo.md` Phase 12 items are unchecked on purpose, not by
oversight: physical hydraulic valve/PWM driver bring-up, real LiDAR/
radar/camera sensor integration, target embedded-platform (Cortex-A72)
performance validation, extreme-environment validation, and a full
12-hour field trial all require physical machine hardware this
sandboxed development workspace was never going to be able to provide.
`todo.md` records each one as explicitly hardware-blocked, and
`docs/compliance/` records the certification/regulatory-process
requirements (independent V&V, tool qualification, MSHA approval,
formal SIL assessment) that no software change can satisfy on its own.

## Known engineering compromises, documented in-repo

- `core::simd`/`portable_simd` is still nightly-only on stable Rust, so
  the mine/wear crates' "SIMD" cut/fill and FFT work use an
  auto-vectorization-friendly style instead of literal SIMD intrinsics
  — noted at each call site and in `todo.md`.
- `docs/deterministic_execution_audit.md` documents that
  `tpt-t-construction-safety`'s `perception`/`tracking` modules aren't
  yet allocation-free/bounded-capacity like the rest of the
  safety-critical code, with a specific proposed remediation.
- `docs/dependency_audit.md` documents a real security advisory
  (RUSTSEC-2026-0235, rkyv) against a direct dependency, verified
  non-applicable to this workspace's actual usage and tracked as
  follow-up work rather than papered over.

## Verification

```
cargo build --workspace --all-targets
cargo test --workspace          # 513 tests, 0 failures
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo deny check                # advisories ok, bans ok, licenses ok, sources ok
```
