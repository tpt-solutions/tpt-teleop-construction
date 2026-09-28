// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The output side of the control loop: a hardware-independent interface
//! for driving an electro-hydraulic proportional valve's PWM signal
//! directly (spec.txt §4.1's "direct PWM control", no PLC/CANopen
//! middleman in between).
//!
//! [`ValveDriver`] is the seam between this crate's control logic and
//! actual hardware: today, only [`NullValveDriver`] exists (a recording
//! stand-in for tests and for `tpt-t-construction-sim`-driven development).
//! The real GPIO/PWM-peripheral implementation is Phase 12 (hardware
//! bring-up) — deliberately not built until there's a target board to
//! bring up against.

/// Drives a single proportional valve's PWM duty cycle.
///
/// `duty` is normalized `-1.0` (full flow toward port B) to `1.0` (full
/// flow toward port A), matching the sign convention used throughout this
/// workspace's hydraulic modeling (see `tpt-t-construction-sim`'s
/// `SpoolValve`).
pub trait ValveDriver {
    fn write_duty(&mut self, duty: f32);
}

/// A [`ValveDriver`] that writes nowhere but remembers the last value
/// written, clamped to `[-1, 1]` exactly as real hardware would saturate
/// an out-of-range PWM command. Used in tests and wherever the control
/// loop needs to run without real hardware attached.
#[derive(Debug, Clone, Copy, Default)]
pub struct NullValveDriver {
    last_duty: f32,
}

impl NullValveDriver {
    pub fn new() -> Self {
        NullValveDriver { last_duty: 0.0 }
    }

    pub fn last_duty(&self) -> f32 {
        self.last_duty
    }
}

impl ValveDriver for NullValveDriver {
    fn write_duty(&mut self, duty: f32) {
        self.last_duty = duty.clamp(-1.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_last_written_duty() {
        let mut driver = NullValveDriver::new();
        driver.write_duty(0.3);
        assert_eq!(driver.last_duty(), 0.3);
        driver.write_duty(-0.7);
        assert_eq!(driver.last_duty(), -0.7);
    }

    #[test]
    fn clamps_out_of_range_duty() {
        let mut driver = NullValveDriver::new();
        driver.write_duty(5.0);
        assert_eq!(driver.last_duty(), 1.0);
        driver.write_duty(-5.0);
        assert_eq!(driver.last_duty(), -1.0);
    }
}
