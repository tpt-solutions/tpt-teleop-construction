// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The simulated [`HardwareBackend`]: wires `tpt-t-construction-sim`'s
//! plant models (`SpoolValve`, `Cylinder`, `orifice_flow_m3_s`) up
//! behind the same trait a physical backend would implement, so the
//! harness's control code is exercised end-to-end today, not just typed
//! against a trait nothing implements yet.
//!
//! The rig this models is a **blocked-actuator bench test** — the piston
//! held stationary (`piston_velocity_m_s = 0.0`) while chamber A pressure
//! builds or bleeds purely from valve-metered flow — the standard way to
//! characterize a valve/controller pair's pressure response before ever
//! driving a real, moving load. A positive valve command meters chamber A
//! to supply pressure (filling); a negative command meters it to tank/
//! return pressure (draining) — the two directions a single proportional
//! spool valve on a real machine actually provides, and necessary for a
//! PID loop to have authority to correct an overshoot rather than being
//! stuck only ever able to raise pressure further.

use tpt_t_construction_sim::hydraulics::{orifice_flow_m3_s, Cylinder, SpoolValve};

use crate::backend::HardwareBackend;

pub struct SimulatedBackend {
    valve: SpoolValve,
    cylinder: Cylinder,
    supply_pressure_pa: f32,
    discharge_coeff: f32,
    max_orifice_area_m2: f32,
    fluid_density_kg_m3: f32,
    pending_command: f32,
}

impl SimulatedBackend {
    pub fn new(valve_tau_s: f32, supply_pressure_pa: f32) -> Self {
        SimulatedBackend {
            valve: SpoolValve::new(valve_tau_s),
            cylinder: Cylinder {
                bore_area_m2: 0.02,
                rod_area_m2: 0.01,
                pressure_a_pa: 0.0,
                pressure_b_pa: 0.0,
                volume_a_m3: 5.0e-3,
                volume_b_m3: 5.0e-3,
                bulk_modulus_pa: 1.6e9,
            },
            supply_pressure_pa,
            discharge_coeff: 0.6,
            max_orifice_area_m2: 2.0e-6,
            fluid_density_kg_m3: 850.0,
            pending_command: 0.0,
        }
    }

    pub fn valve_position(&self) -> f32 {
        self.valve.position
    }
}

impl HardwareBackend for SimulatedBackend {
    fn write_valve_command(&mut self, duty: f32) {
        self.pending_command = duty;
    }

    fn read_pressure_pa(&self) -> f32 {
        self.cylinder.pressure_a_pa
    }

    fn step(&mut self, dt_s: f32) {
        let position = self.valve.step(self.pending_command, dt_s);
        // Positive position meters toward supply (filling); negative
        // meters toward tank/return pressure, taken as 0 Pa (draining).
        let target_pressure_pa = if position >= 0.0 {
            self.supply_pressure_pa
        } else {
            0.0
        };
        let delta_pressure_pa = target_pressure_pa - self.cylinder.pressure_a_pa;
        let flow_in_a_m3_s = orifice_flow_m3_s(
            position.abs(),
            self.discharge_coeff,
            self.max_orifice_area_m2,
            delta_pressure_pa,
            self.fluid_density_kg_m3,
        );
        self.cylinder.step(flow_in_a_m3_s, 0.0, 0.0, dt_s);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pressure_builds_toward_supply_with_the_valve_held_open() {
        let mut backend = SimulatedBackend::new(0.05, 2.0e7);
        backend.write_valve_command(1.0);
        for _ in 0..100 {
            backend.step(0.001); // 100ms at 1kHz: partway through filling
        }
        // Meaningful partial build-up, but nowhere near supply yet — the
        // valve itself is still ramping open (tau = 50ms) and the chamber
        // hasn't had time to fill.
        assert!(backend.read_pressure_pa() > 1.0e6);
        assert!(backend.read_pressure_pa() < 1.8e7);
    }

    #[test]
    fn held_open_long_enough_pressure_reaches_near_supply() {
        let mut backend = SimulatedBackend::new(0.05, 2.0e7);
        backend.write_valve_command(1.0);
        for _ in 0..5_000 {
            backend.step(0.001); // 5s at 1kHz: comfortably past the fill transient
        }
        assert!(backend.read_pressure_pa() > 1.9e7);
    }

    #[test]
    fn zero_command_never_builds_pressure() {
        let mut backend = SimulatedBackend::new(0.05, 2.0e7);
        backend.write_valve_command(0.0);
        for _ in 0..1000 {
            backend.step(0.001);
        }
        assert_eq!(backend.read_pressure_pa(), 0.0);
    }
}
