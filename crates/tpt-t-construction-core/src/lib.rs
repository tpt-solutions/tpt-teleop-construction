// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Central event loop, machine state machine, lock-free message bus, and
//! shift-start self-test framework shared by every `tpt-t-construction-*`
//! implement crate (excavator, haul truck, dozer, ...).

mod bus;
mod event_loop;
mod messages;
mod self_test;
mod state;

pub use bus::{new_bus, BusReceiver, BusSender};
pub use event_loop::{MachineController, TickError};
pub use messages::{Fault, FaultSeverity, MachineEvent, TelemetrySample};
pub use self_test::{SelfTestCheck, SelfTestOutcome, SelfTestReport, SelfTestSuite};
pub use state::{MachineFsm, MachineState, TransitionError};
