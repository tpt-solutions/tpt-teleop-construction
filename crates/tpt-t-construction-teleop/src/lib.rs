// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Adapter crate to tpt-teleop-domain-bridge: teleoperation engage/disengage, control command translation, safety state machine wiring.

mod adapter;
mod control_command;
mod dti;
mod fault_handover;
mod implement_switch;
mod safety_fsm;
mod valve_mapping;
mod vibration_filter;

pub use adapter::ExcavatorTeleopAdapter;
pub use control_command::{ButtonState, DecodedControlCommand, RawControlCommand};
pub use dti::{DomainState, DomainTelemetry, DomainTeleopInterface, SensorFeed, TeleopError};
pub use fault_handover::handover_response;
pub use implement_switch::{ImplementMode, ImplementSwitch};
pub use safety_fsm::{HandoverFsm, HandoverState, HandoverTransitionError};
pub use valve_mapping::{compensated_valve_duties, ValveMap};
pub use vibration_filter::JoystickFilterBank;
