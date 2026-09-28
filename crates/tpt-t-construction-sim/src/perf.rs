// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Real-time performance validation: spec.txt §8 requires the simulator
//! run fast enough that a developer can test a 12-hour shift in about an
//! hour of wall-clock time (a 10x+ speedup over real time). Rather than
//! actually simulating 12 hours in a test (43_200s of sim time would make
//! this test itself take real wall-clock minutes even at a large
//! speedup), [`measure_realtime_speedup`] times a representative batch of
//! ticks and extrapolates: the vehicle step function's cost is
//! per-tick-constant (no branching on elapsed sim time), so a short
//! measurement's ticks-per-second is the same figure a 12-hour run would
//! see.

use crate::vehicle::SimVehicle;
use std::time::Instant;

/// The result of timing a batch of simulation ticks.
#[derive(Debug, Clone, Copy)]
pub struct RealTimeReport {
    pub simulated_seconds: f64,
    pub wall_seconds: f64,
}

impl RealTimeReport {
    /// How many seconds of simulated time elapse per second of wall-clock
    /// time (e.g. `12.0` means a 12-hour shift takes 1 wall-clock hour).
    pub fn speedup_factor(&self) -> f64 {
        if self.wall_seconds <= 0.0 {
            f64::INFINITY
        } else {
            self.simulated_seconds / self.wall_seconds
        }
    }

    /// Projected wall-clock seconds to simulate a full 12-hour shift at
    /// this measured rate.
    pub fn projected_wall_seconds_for_full_shift(&self) -> f64 {
        const SHIFT_SECONDS: f64 = 12.0 * 3600.0;
        SHIFT_SECONDS / self.speedup_factor()
    }
}

/// Runs `ticks` steps of a [`SimVehicle::reference_haul_truck`] at `dt_s`
/// per tick under full throttle, timing how long that takes on this
/// machine, and reports the resulting real-time speedup.
pub fn measure_realtime_speedup(ticks: u32, dt_s: f32) -> RealTimeReport {
    let mut vehicle = SimVehicle::reference_haul_truck();
    let start = Instant::now();
    for _ in 0..ticks {
        vehicle.step(1.0, dt_s);
        std::hint::black_box(&vehicle);
    }
    let wall_seconds = start.elapsed().as_secs_f64();
    RealTimeReport {
        simulated_seconds: ticks as f64 * dt_s as f64,
        wall_seconds,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Validates the >=10x real-time requirement: at a 100Hz sim tick
    /// rate (matching the 1kHz/100Hz control-loop rates elsewhere in this
    /// workspace, downsampled to what a physics step actually needs),
    /// a 12-hour shift must project to well under an hour of wall-clock
    /// time on ordinary CI hardware.
    #[test]
    fn twelve_hour_shift_projects_to_under_one_hour_wall_clock() {
        // 200_000 ticks at 100Hz = 2000 simulated seconds, enough to
        // amortize measurement noise while keeping the test itself fast.
        let report = measure_realtime_speedup(200_000, 0.01);
        assert!(
            report.speedup_factor() >= 10.0,
            "real-time speedup was only {:.1}x (need >= 10x): {:?}",
            report.speedup_factor(),
            report
        );
        assert!(
            report.projected_wall_seconds_for_full_shift() <= 3600.0,
            "projected {:.0}s wall-clock for a 12h shift (need <= 3600s): {:?}",
            report.projected_wall_seconds_for_full_shift(),
            report
        );
    }
}
