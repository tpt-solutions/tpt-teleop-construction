# tpt-teleop-construction — Project Todo

Heavy Machinery & Mining Operations middleware workspace. License: MIT OR Apache-2.0. TPT Solutions.

## Phase 0 — Repository & Workspace Setup
- [x] Init git repo
- [x] Cargo workspace `Cargo.toml` listing all `tpt-t-construction-*` member crates
- [x] `LICENSE-MIT` and `LICENSE-APACHE` (Copyright TPT Solutions), dual-license header convention for source files
- [x] `deny.toml` (cargo-deny): enforce MIT/BSD/ISC chain, reject Apache-only deps, force dual-licensed crates to resolve as MIT
- [x] CI pipeline: build, test, clippy, fmt, cargo-deny check
- [x] `README.md`: overview, workspace layout, sister-repo links, license
- [x] Stub out empty crate skeletons (Cargo.toml + lib.rs) for all crates listed below

## Phase 1 — Core Infrastructure
- [x] `tpt-t-construction-core`: event loop, machine state machine (Idle→Moving→Working→Dumping→Returning), lock-free message bus
- [x] Integrate/vendor lock-free SPSC/MPSC ring buffer (tpt-teleop-ring) for IPC/telemetry
- [x] rkyv zero-copy message type definitions
- [x] Full self-test framework (shift-start diagnostics)

## Phase 2 — Simulation & Developer Experience
- [x] `tpt-t-construction-sim`: 6DOF vehicle dynamics, tire/track-soil + rollover physics
- [x] `tpt-t-construction-sim`: hydraulic system dynamics (valve response, pressure drops, cylinder forces)
- [x] `tpt-t-construction-sim`: diesel engine torque curves and transmission shifting
- [x] `tpt-t-construction-sim`: synthetic LiDAR/radar returns from terrain and obstacles
- [x] `tpt-t-construction-sim`: proximity-detection scenario scripting (virtual humans/vehicles)
- [x] egui-based visualizer: 3D terrain with cut/fill heatmaps
- [x] Visualizer: machine trajectories and payload weights
- [x] Visualizer: proximity detection zones and object classifications
- [x] Visualizer: hydraulic pressures and valve positions
- [x] Visualizer: vibration frequency spectra
- [x] Validate 10x real-time sim performance (12hr shift in ~1hr wall-clock)

## Phase 3 — Hydraulic & Powertrain Foundation
- [x] `tpt-t-construction-hydraulic`: 1kHz PID loop on pinned core (SCHED_FIFO), direct PWM valve output, <100µs loop budget
- [x] `tpt-t-construction-hydraulic`: pressure compensation, flow sharing, load-sensing logic
- [x] `tpt-t-construction-hydraulic`: zero-allocation static arrays for sensor/control data
- [x] `tpt-t-construction-hydraulic`: vibration filtering (reject 50-200Hz diesel/pump noise)
- [x] `tpt-t-construction-powertrain`: torque management, gear shifting
- [x] `tpt-t-construction-powertrain`: thermal/emissions-based engine derating

## Phase 4 — Terrain & Payload Sensing
- [ ] `tpt-t-construction-terrain`: real-time 3D terrain modeling, LiDAR surface reconstruction
- [ ] `tpt-t-construction-terrain`: slope stability analysis, bearing capacity estimation
- [ ] `tpt-t-construction-terrain`: dynamic obstacle detection
- [ ] `tpt-t-construction-payload`: SIMD cut/fill volume calc (<10ms/scan), slab-allocated zero-copy ring buffers
- [ ] `tpt-t-construction-payload`: on-board scale payload weighing
- [ ] `tpt-t-construction-payload`: material type classification (dirt, rock, ore)

## Phase 5 — Autonomous Machine Control
- [ ] `tpt-t-construction-excavator`: trench digging to exact depth/width
- [ ] `tpt-t-construction-excavator`: truck loading with minimal swing cycle time
- [ ] `tpt-t-construction-excavator`: grading to designed slope
- [ ] `tpt-t-construction-excavator`: boom-arm-bucket kinematics, hydraulic cylinder coordination
- [ ] `tpt-t-construction-excavator`: obstacle avoidance (utilities, boulders, other machines)
- [ ] `tpt-t-construction-haul`: haul road navigation, edge drop-off avoidance
- [ ] `tpt-t-construction-haul`: speed management (downgrades, curves, visibility)
- [ ] `tpt-t-construction-haul`: dump-point/crusher queueing
- [ ] `tpt-t-construction-haul`: payload weighing integration, per-shift tonnage tracking
- [ ] `tpt-t-construction-dozer`: slope grading (real-time blade position feedback)
- [ ] `tpt-t-construction-dozer`: real-time cut/fill volume tracking
- [ ] `tpt-t-construction-dozer`: material pushing to stockpiles/hoppers/crushers
- [ ] `tpt-t-construction-loader`: bucket digging and lifting
- [ ] `tpt-t-construction-loader`: truck loading with payload weighing
- [ ] `tpt-t-construction-drill`: blast hole pattern execution (depth/angle)
- [ ] `tpt-t-construction-drill`: rock hardness detection via drill rate/torque
- [ ] `tpt-t-construction-drill`: collar positioning via LiDAR/GNSS

## Phase 6 — Underground & GPS-Denied Navigation
- [ ] `tpt-t-construction-mine`: LiDAR SLAM (custom EKF fusing LiDAR + wheel odometry + IMU)
- [ ] `tpt-t-construction-mine`: SIMD (portable-simd) ICP achieving 10Hz updates on Cortex-A72
- [ ] `tpt-t-construction-mine`: UWB beacon integration (<0.5m accuracy)
- [ ] `tpt-t-construction-mine`: ventilation-aware routing
- [ ] `tpt-t-construction-mine`: bolt carrier / shotcrete rig coordination (safe working distances)
- [ ] `tpt-t-construction-tunnel`: TBM/roadheader advance rate optimization
- [ ] `tpt-t-construction-tunnel`: cutter head torque management
- [ ] `tpt-t-construction-tunnel`: segment installation coordination

## Phase 7 — Safety Systems
- [ ] `tpt-t-construction-safety`: 360° LiDAR/radar/camera fusion perception pipeline (<50ms)
- [ ] `tpt-t-construction-safety`: object tracking/classification (human/light vehicle/other equipment)
- [ ] `tpt-t-construction-safety`: dynamic warning zones based on speed/direction
- [ ] `tpt-t-construction-safety`: deterministic 50% slowdown (<100ms) + full stop on continued human presence
- [ ] `tpt-t-construction-rollover`: real-time tilt angle + center-of-gravity estimation
- [ ] `tpt-t-construction-rollover`: automatic brake application on rollover threshold
- [ ] `tpt-t-construction-fatigue`: camera-based eye tracking
- [ ] `tpt-t-construction-fatigue`: steering pattern analysis
- [ ] `tpt-t-construction-fatigue`: mandatory rest enforcement

## Phase 8 — Maintenance & Telemetry
- [ ] `tpt-t-construction-wear`: custom SIMD FFT vibration analysis (pre-allocated, 10kHz, dedicated core)
- [ ] `tpt-t-construction-wear`: bearing wear / gear tooth damage / imbalance signature detection
- [ ] `tpt-t-construction-wear`: hydraulic pressure, engine hours, undercarriage wear monitoring
- [ ] `tpt-t-construction-telemetry`: 100Hz logging of all machine parameters to local NVMe
- [ ] `tpt-t-construction-telemetry`: post-shift analysis export, OEM warranty claim support

## Phase 9 — Multi-Machine Coordination & End-to-End Integration
- [ ] `tpt-t-construction-core`: excavator/haul-truck queue management algorithm
- [ ] `tpt-t-construction-core`: truck arrival prediction from GPS position/speed
- [ ] `tpt-t-construction-core`: swing-cycle adjustment, 10Hz zero-alloc coordination loop
- [ ] End-to-end "Open-Pit Mine Haulage" scenario test in sim (shift start → self-test → dispatch → haul → load → return → dump → maintenance alert → shift end)

## Phase 10 — tpt-teleop Integration
- [ ] `tpt-t-construction-teleop`: adapter crate scaffold, depends on tpt-teleop-domain-bridge
- [ ] Implement Domain Teleoperation Interface: `on_teleop_engage` / `on_teleop_disengage`
- [ ] Implement `on_control_command`, `get_domain_state`, `get_sensor_feed`
- [ ] Translate ControlCommand → hydraulic proportional valve PWM via `tpt-t-construction-hydraulic` (pressure comp, flow sharing)
- [ ] Implement operator implement-switching (e.g. bucket ↔ hammer)
- [ ] Diesel-vibration filtering on joystick control inputs
- [ ] Wire into universal safety state machine (AUTONOMOUS ↔ REQUESTING_TELEOP ↔ TELEOP_ACTIVE ↔ RETURNING_TO_AUTONOMY ↔ EMERGENCY_STOP)
- [ ] Implement fault detection → `request_teleop_assistance()` handover trigger
- [ ] Reconcile aspirational `ControlCommand` (operator_id/InputState/ButtonState) against the actual flat 56-byte POD type in tpt-teleop-core (open decision, not yet resolved upstream)

## Phase 11 — Safety Certification & Compliance
- [ ] Deterministic-execution audit across safety-critical loops (safety, rollover, hydraulic)
- [ ] Redundant safety monitoring / TMR voter implementation
- [ ] Complete audit trail logging for certification
- [ ] ISO 19014 (earth-moving machinery safety) compliance review
- [ ] MSHA regulation compliance review
- [ ] IEC 61508 SIL 2/3 functional safety certification documentation
- [ ] Zero-false-stop validation suite for proximity detection

## Phase 12 — Hardware Bring-Up & Field Validation
- [ ] Zero-alloc CAN/J1939 parser implementation
- [ ] Hardware-in-loop testing harness
- [ ] Physical hydraulic valve/PWM driver bring-up on target machine
- [ ] Real LiDAR/radar/camera integration replacing sim synthetic data
- [ ] Target embedded platform (e.g. Cortex-A72) performance validation against spec'd budgets
- [ ] Extreme-environment validation (-40°C to +50°C, 10g vibration)
- [ ] Full 12-hour shift field trial

## Phase 13 — Release Readiness
- [ ] Crate-level API docs + architecture documentation pass
- [ ] Final full-tree cargo-deny/license audit
- [ ] v1.0.0 tag and release notes
