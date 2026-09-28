// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Central event loop, machine state machine, lock-free message bus,
//! shift-start self-test framework, and multi-machine excavator/haul-truck
//! dispatch coordination (spec.txt §4.6, §9) shared by every
//! `tpt-t-construction-*` implement crate (excavator, haul truck, dozer,
//! ...).

mod bus;
mod coordination_loop;
mod dispatch;
mod event_loop;
mod messages;
mod self_test;
mod state;
mod truck_arrival;

pub use bus::{new_bus, BusReceiver, BusSender};
pub use coordination_loop::{compute_swing_adjustment, CoordinationLoop, SwingAdjustment};
pub use dispatch::{DispatchQueue, TruckEta};
pub use event_loop::{MachineController, TickError};
pub use messages::{Fault, FaultSeverity, MachineEvent, TelemetrySample};
pub use self_test::{SelfTestCheck, SelfTestOutcome, SelfTestReport, SelfTestSuite};
pub use state::{MachineFsm, MachineState, TransitionError};
pub use truck_arrival::predicted_arrival_s;
