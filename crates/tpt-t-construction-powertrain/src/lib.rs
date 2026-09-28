// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Diesel engine and transmission control: torque arbitration across
//! operator/engine/traction/thermal limits, a throttle-and-speed
//! production shift schedule, and thermal/emissions-based engine
//! derating (spec.txt §3).
//!
//! `tpt-t-construction-sim::powertrain` is the physics *plant* (torque
//! curves, RPM-threshold shifting as a simulation simplification) this
//! crate's *controller* logic is developed and tested against; this crate
//! contains no plant model of its own.

mod derate;
mod shift_schedule;
mod torque;

pub use derate::{DerateConfig, ThermalLimit};
pub use shift_schedule::{ShiftPoint, ShiftSchedule};
pub use torque::arbitrate_torque_nm;
