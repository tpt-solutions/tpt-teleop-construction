# tpt-teleop-construction

Heavy Machinery & Mining Operations middleware for the `tpt` robotics ecosystem —
a hyper-optimized, zero-bloat, safety-critical Rust workspace for autonomous
excavators, haul trucks, dozers, loaders, drill rigs, and tunnel boring
machines in open-pit mines, underground mines, quarries, and large-scale
civil construction.

See [`spec.txt`](./spec.txt) for the full design document and
[`bridge spec.txt`](./bridge%20spec.txt) for the `tpt-teleop-domain-bridge`
integration contract this workspace adapts to. Outstanding work is tracked in
[`todo.md`](./todo.md).

## Workspace layout

All crates live under `crates/` and use the `tpt-t-construction-` prefix
(the repository itself keeps the full `tpt-teleop-construction` name).

| Crate | Purpose |
|---|---|
| `tpt-teleop-ring` | Vendored lock-free SPSC/MPSC ring buffer used for IPC/telemetry |
| `tpt-t-construction-can` | Zero-alloc CAN/J1939 parsing: identifier/PGN decoding, engine SPN decoders, BAM transport reassembly |
| `tpt-t-construction-core` | Central event loop, machine state machine, lock-free message bus |
| `tpt-t-construction-excavator` | Autonomous excavator: trenching, truck loading, grading |
| `tpt-t-construction-haul` | Autonomous haul truck: road navigation, dump queueing, payload tracking |
| `tpt-t-construction-dozer` | Autonomous dozer: slope grading, cut/fill tracking, material pushing |
| `tpt-t-construction-loader` | Wheel/front-end loader: bucket digging, lifting, truck loading |
| `tpt-t-construction-drill` | Automated drilling rigs: blast hole patterns, rock hardness detection |
| `tpt-t-construction-hil` | Hardware-in-loop test harness: a `HardwareBackend` trait run today against a simulated plant |
| `tpt-t-construction-mine` | Underground navigation: LiDAR SLAM, UWB positioning, ventilation-aware routing |
| `tpt-t-construction-tunnel` | TBM/roadheader control: advance rate, cutter head torque, segment installation |
| `tpt-t-construction-hydraulic` | Proportional hydraulic valve control: 1kHz PID, pressure comp, flow sharing |
| `tpt-t-construction-powertrain` | Diesel engine/transmission control: torque management, derating |
| `tpt-t-construction-terrain` | Real-time 3D terrain modeling, slope stability, obstacle detection |
| `tpt-t-construction-payload` | Cut/fill volume calc, on-board payload weighing, material classification |
| `tpt-t-construction-safety` | Proximity detection system (PDS): perception fusion, dynamic warning zones |
| `tpt-t-construction-rollover` | Rollover protection: tilt/CoG estimation, automatic braking |
| `tpt-t-construction-fatigue` | Operator fatigue monitoring: eye tracking, steering analysis, rest enforcement |
| `tpt-t-construction-wear` | Predictive maintenance: SIMD FFT vibration analysis |
| `tpt-t-construction-telemetry` | 100Hz high-frequency telemetry logging to local NVMe |
| `tpt-t-construction-sim` | Headless simulator: vehicle/hydraulic/engine dynamics, synthetic sensors |
| `tpt-t-construction-viz` | egui-based visualizer: terrain heatmaps, trajectories, pressures, spectra |
| `tpt-t-construction-teleop` | Adapter to `tpt-teleop-domain-bridge`: teleoperation handover, valve translation |

## Sister repositories

Part of the `tpt` ecosystem alongside `tpt-teleop` (core teleoperation
platform), `tpt-teleop-fleet`, `tpt-t-agri`, `tpt-teleop-marine`,
`tpt-teleop-medical`, and `tpt-teleop-defense`.

## Building

```sh
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo deny check
```

## License

Licensed under either of

* MIT license ([`LICENSE-MIT`](./LICENSE-MIT))
* Apache License, Version 2.0 ([`LICENSE-APACHE`](./LICENSE-APACHE))

at your option. The dependency chain is strictly MIT/BSD/ISC: `cargo-deny`
rejects Apache-only dependencies, and dual-licensed dependencies resolve
under MIT (see [`deny.toml`](./deny.toml)).

### Source file header convention

Every source file carries:

```rust
// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions
```
