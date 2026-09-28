// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The `HardwareBackend` trait: the one seam a hardware-in-loop test
//! harness needs, so the *same* control code (here, a
//! [`tpt_t_construction_hydraulic::PidController`] driving a proportional
//! valve toward a pressure setpoint) can run against a simulated plant
//! today ([`crate::simulated::SimulatedBackend`]) and against a real
//! valve/pressure-transducer pair once Phase 12 hardware exists
//! ([`crate::physical::PhysicalBackend`]), without the harness itself
//! changing.

/// An error a [`HardwareBackend`] can report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HilError {
    /// No physical hardware is attached/reachable. Every
    /// [`crate::physical::PhysicalBackend`] construction in this
    /// workspace returns this today — see that module's docs.
    NoHardwareAttached,
}

/// The minimal I/O surface a hardware-in-loop test needs from whatever
/// it's driving: command a single proportional valve, and read back the
/// resulting actuator pressure. A real bring-up would extend this trait
/// (more channels, CAN frames via `tpt-t-construction-can`, etc.) as
/// more of the machine comes under test — this is deliberately the
/// smallest useful slice, matching the one closed loop this workspace
/// can actually validate today (see `docs/deterministic_execution_audit.md`'s
/// review of the hydraulic PID loop).
pub trait HardwareBackend {
    /// Commands the valve to `duty` (`-1.0..=1.0`, sign is direction).
    fn write_valve_command(&mut self, duty: f32);

    /// The actuator's current sensed pressure (Pa).
    fn read_pressure_pa(&self) -> f32;

    /// Advances the backend by `dt_s` of wall-clock (simulated or real)
    /// time, applying whatever valve command was last written.
    fn step(&mut self, dt_s: f32);
}
