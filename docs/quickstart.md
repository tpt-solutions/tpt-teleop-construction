# Quickstart - tpt-teleop-construction

> Status: scaffold. This document grows as the crates land. The commands below
> are enough to build and test the workspace today.

## Prerequisites

- Rust stable (toolchain pinned via `rust-toolchain.toml`). No nightly is
  required: SIMD hot paths use the `core::arch`-backed `f32x4` shim in
  `tpt-t-construction-core::simd` (`portable_simd` is still unstable).
- `cargo-deny` for the dependency license audit: `cargo install cargo-deny`.

## Build & test the workspace

```sh
# Whole workspace (stable)
cargo build --workspace --all-targets
cargo test  --workspace

# Lint / format
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

# Dependency license audit (MIT chain)
cargo deny check

# Hot-path audits
bash tools/lock-audit.sh     # hard gate: no locks in crates/*/src
bash tools/alloc-audit.sh    # informational allocation report
```

## Crate layout

```
crates/
  tpt-t-construction-core/       # machine state machine + lock-free bus + SIMD shim (Phase 1)
  tpt-t-domain-bridge/           # DTI trait, wire types, safety FSM, assist API (vendored)
  tpt-t-construction-sim/        # headless 6DOF/hydraulic/diesel simulator (Phase 2)
  tpt-t-construction-visualizer/ # egui visualizer - excluded from workspace (Phase 2)
  tpt-t-construction-hydraulic/  # 1 kHz PID valve control (Phase 3)
  tpt-t-construction-powertrain/ # torque management, shifting, derating (Phase 3)
  tpt-t-construction-terrain/    # 3D terrain, slope stability, obstacles (Phase 4)
  tpt-t-construction-payload/    # cut/fill volumes, weighing, classification (Phase 4)
  tpt-t-construction-excavator/  # trenching, loading, grading (Phase 5)
  tpt-t-construction-haul/       # haul roads, speed, dump queueing (Phase 5)
  tpt-t-construction-dozer/      # grading, cut/fill, pushing (Phase 5)
  tpt-t-construction-loader/     # bucket digging, truck loading (Phase 5)
  tpt-t-construction-drill/      # blast patterns, hardness, collars (Phase 5)
  tpt-t-construction-mine/       # LiDAR SLAM, ICP, UWB, ventilation (Phase 6)
  tpt-t-construction-tunnel/     # TBM advance, cutter torque (Phase 6)
  tpt-t-construction-safety/     # proximity detection fusion (Phase 7)
  tpt-t-construction-rollover/   # tilt/CG, auto brake (Phase 7)
  tpt-t-construction-fatigue/    # eye tracking, steering analysis (Phase 7)
  tpt-t-construction-wear/       # SIMD FFT predictive maintenance (Phase 8)
  tpt-t-construction-telemetry/  # 100 Hz NVMe logging (Phase 8)
  tpt-t-construction-teleop/     # DomainTeleopInterface adapter (Phase 10)
```

## Simulation

The simulator (`tpt-t-construction-sim`) runs headless at 10x real time, so a
12-hour shift scenario completes in roughly an hour of wall-clock time; the
egui visualizer (`cargo run --manifest-path
crates/tpt-t-construction-visualizer/Cargo.toml`) renders terrain, zones,
trajectories, pressures, and spectra for debugging.
