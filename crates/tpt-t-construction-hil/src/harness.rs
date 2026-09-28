// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The harness itself: a real [`tpt_t_construction_hydraulic::PidController`]
//! (the actual Phase 3 control loop, not a stand-in) driving whatever
//! [`HardwareBackend`] it's given toward a pressure setpoint. This is the
//! part of "hardware-in-loop testing harness" (todo.md Phase 12) that's
//! genuinely software: the same [`HilHarness`] runs unchanged against
//! [`crate::simulated::SimulatedBackend`] today, and would run unchanged
//! against a real backend once Phase 12 hardware exists for
//! [`crate::physical::PhysicalBackend`] to actually connect to.

use tpt_t_construction_hydraulic::{PidController, PidGains};

use crate::backend::HardwareBackend;

pub struct HilHarness<B: HardwareBackend> {
    backend: B,
    controller: PidController,
}

impl<B: HardwareBackend> HilHarness<B> {
    pub fn new(backend: B, gains: PidGains, output_min: f32, output_max: f32) -> Self {
        HilHarness {
            backend,
            controller: PidController::new(gains, output_min, output_max),
        }
    }

    pub fn backend(&self) -> &B {
        &self.backend
    }

    /// Runs one control tick: reads the backend's current pressure,
    /// computes the next valve command toward `setpoint_pa`, writes it,
    /// and advances the backend by `dt_s`. Returns the pressure reading
    /// this tick was computed from (i.e. *before* `dt_s` of advancement).
    pub fn run_tick(&mut self, setpoint_pa: f32, dt_s: f32) -> f32 {
        let measurement = self.backend.read_pressure_pa();
        let command = self.controller.step(setpoint_pa, measurement, dt_s);
        self.backend.write_valve_command(command);
        self.backend.step(dt_s);
        measurement
    }

    /// Runs `ticks` consecutive control ticks at a fixed setpoint and
    /// period, returning the final pressure reading (see
    /// [`Self::run_tick`]'s return value).
    pub fn run_ticks(&mut self, setpoint_pa: f32, dt_s: f32, ticks: u32) -> f32 {
        let mut last = 0.0;
        for _ in 0..ticks {
            last = self.run_tick(setpoint_pa, dt_s);
        }
        last
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulated::SimulatedBackend;

    fn gains() -> PidGains {
        PidGains {
            kp: 2.0e-8,
            ki: 4.0e-8,
            kd: 0.0,
        }
    }

    #[test]
    fn the_pid_loop_converges_pressure_to_the_setpoint() {
        let backend = SimulatedBackend::new(0.05, 2.5e7);
        let mut harness = HilHarness::new(backend, gains(), -1.0, 1.0);

        let setpoint_pa = 1.0e7;
        let final_reading = harness.run_ticks(setpoint_pa, 0.001, 60_000); // 60s at 1kHz

        assert!(
            (final_reading - setpoint_pa).abs() < 1.0e6,
            "expected convergence near {setpoint_pa} Pa, got {final_reading} Pa"
        );
    }

    #[test]
    fn a_lower_setpoint_converges_to_a_lower_pressure() {
        let backend = SimulatedBackend::new(0.05, 2.5e7);
        let mut harness = HilHarness::new(backend, gains(), -1.0, 1.0);

        let final_reading = harness.run_ticks(3.0e6, 0.001, 60_000);
        assert!(final_reading < 6.0e6, "got {final_reading} Pa");
    }
}
