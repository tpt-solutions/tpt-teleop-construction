// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Teleoperation adapter (Phase 10): the universal handover state
//! machine (`safety_fsm`, `AUTONOMOUS <-> REQUESTING_TELEOP <->
//! TELEOP_ACTIVE <-> RETURNING_TO_AUTONOMY <-> EMERGENCY_STOP`),
//! fault-driven handover triggers (`fault_handover`), diesel-vibration
//! joystick filtering (`vibration_filter`), `ControlCommand` -> hydraulic
//! valve duty translation (`valve_mapping`), bucket/hammer implement
//! switching (`implement_switch`), and a local Domain Teleoperation
//! Interface (`dti`) plus a concrete `ExcavatorTeleopAdapter`
//! (`adapter`) implementing it.
//!
//! `tpt-teleop-domain-bridge`/`tpt-teleop-core` live in the separate
//! sister `tpt-teleop` repository, not this workspace, so this crate
//! defines its own wire format (`control_command`) and DTI trait rather
//! than importing them — see `control_command`'s module docs for the
//! specific reconciliation against bridge spec.txt's aspirational
//! `ControlCommand`.

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
