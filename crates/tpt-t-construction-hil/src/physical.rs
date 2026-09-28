// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The physical [`HardwareBackend`] — today, always unavailable.
//!
//! A real physical bring-up needs a CAN interface or proportional-valve
//! driver board wired to an actual machine, plus a pressure transducer
//! to read back from — none of which exist in this sandboxed workspace
//! (todo.md Phase 12: "Physical hydraulic valve/PWM driver bring-up on
//! target machine", "Hardware-in-loop testing harness"). Rather than
//! fake a connection or `unimplemented!()`-panic the first time someone
//! calls a method on it, [`PhysicalBackend::connect`] itself fails
//! honestly with [`HilError::NoHardwareAttached`] — the harness this
//! crate provides is real and runnable today against
//! [`crate::simulated::SimulatedBackend`]; only this one backend is a
//! documented stub waiting on hardware that doesn't exist yet.

use crate::backend::{HardwareBackend, HilError};

/// A backend that would talk to real hardware, if any were attached.
#[derive(Debug)]
pub struct PhysicalBackend {
    _unreachable: (),
}

impl PhysicalBackend {
    /// Attempts to connect to a physical valve/pressure-transducer
    /// interface at `device_path` (e.g. a CAN interface name or serial
    /// device). Always returns `Err(HilError::NoHardwareAttached)` in
    /// this workspace — see the module docs.
    pub fn connect(_device_path: &str) -> Result<Self, HilError> {
        Err(HilError::NoHardwareAttached)
    }
}

impl HardwareBackend for PhysicalBackend {
    fn write_valve_command(&mut self, _duty: f32) {
        unreachable!("PhysicalBackend::connect always fails, so no instance can exist to call this")
    }

    fn read_pressure_pa(&self) -> f32 {
        unreachable!("PhysicalBackend::connect always fails, so no instance can exist to call this")
    }

    fn step(&mut self, _dt_s: f32) {
        unreachable!("PhysicalBackend::connect always fails, so no instance can exist to call this")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn connecting_to_physical_hardware_always_fails_honestly() {
        assert_eq!(
            PhysicalBackend::connect("/dev/can0").unwrap_err(),
            HilError::NoHardwareAttached
        );
    }
}
