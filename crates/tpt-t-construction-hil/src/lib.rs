// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Hardware-in-loop (HIL) test harness (todo.md Phase 12: "Hardware-in-loop
//! testing harness"). The [`HardwareBackend`] trait is the seam: the real
//! Phase 3 [`tpt_t_construction_hydraulic::PidController`] drives whatever
//! implements it via [`HilHarness`], today [`simulated::SimulatedBackend`]
//! (wired to `tpt-t-construction-sim`'s plant models), and — once the
//! physical valve/transducer hardware [`physical::PhysicalBackend`]
//! documents as missing actually exists — the same harness code, unchanged,
//! against real hardware.
//!
//! What this crate is *not*: a claim that hardware-in-loop testing has
//! actually been performed. It hasn't, and can't be, without the hardware
//! Phase 12's other checklist items bring up. What exists here is the
//! harness itself, validated against the one backend this workspace can
//! honestly provide.

mod backend;
mod harness;
mod physical;
mod simulated;

pub use backend::{HardwareBackend, HilError};
pub use harness::HilHarness;
pub use physical::PhysicalBackend;
pub use simulated::SimulatedBackend;
