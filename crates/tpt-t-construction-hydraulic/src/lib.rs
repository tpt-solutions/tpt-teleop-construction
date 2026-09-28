// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Proportional hydraulic valve control: the 1kHz `SCHED_FIFO` PID loop,
//! pressure compensation, flow sharing, load-sensing, zero-allocation
//! sensor/control data storage, and vibration filtering that
//! `tpt-t-construction-excavator`, `-haul`, `-dozer`, `-loader`, and
//! `-drill` all build implement control on top of (spec.txt §4.1).
//!
//! This crate is the *controller*: it decides what PWM duty cycle each
//! valve should receive. `tpt-t-construction-sim`'s `hydraulics` module is
//! the *plant* this controller is tested against in development (valve
//! lag, cylinder pressure dynamics); the real electro-hydraulic hardware
//! is what `tpt-t-construction-teleop`/Phase 12's `ValveDriver`
//! implementation will drive in the field.

mod compensation;
mod filter;
mod pid;
mod realtime;
mod sensors;
mod valve;

pub use compensation::{load_sense_pressure, pressure_compensated_flow_fraction, share_flow};
pub use filter::Biquad;
pub use pid::{PidController, PidGains};
pub use realtime::{pin_and_elevate, LoopBudget, RealtimeError};
pub use sensors::{FrameFull, SensorFrame};
pub use valve::{NullValveDriver, ValveDriver};
