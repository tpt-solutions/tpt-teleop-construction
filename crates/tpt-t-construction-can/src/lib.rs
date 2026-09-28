// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Zero-alloc CAN/J1939 parsing (todo.md Phase 12: "Zero-alloc CAN/J1939
//! parser implementation") — the one item in Phase 12 that's genuinely
//! software, not hardware, bring-up: decoding a J1939 29-bit identifier
//! into a Parameter Group Number, extracting known engine signals from a
//! frame's payload, and reassembling multi-packet broadcast messages,
//! all without allocating.
//!
//! This is a real-CAN-bus consumer, not a simulated one: it operates on
//! [`CanFrame`] values a caller reads from an actual CAN controller (or,
//! for now, from `tpt-t-construction-sim`'s synthetic data, since no
//! physical hardware is available in this workspace — see Phase 12's
//! other, hardware-blocked items in `todo.md`).

mod frame;
mod id;
mod signal;
mod spn;
mod transport;

pub use frame::CanFrame;
pub use id::J1939Id;
pub use signal::{extract_unsigned, not_available_sentinel, scale};
pub use spn::{engine_coolant_temp_c, engine_speed_rpm, SpnValue, PGN_EEC1, PGN_ET1};
pub use transport::{BamReassembler, TransportError, TP_CM_PGN, TP_DT_PGN};
