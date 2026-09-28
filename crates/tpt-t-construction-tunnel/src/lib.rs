// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Tunnel boring machine (TBM) and roadheader control: advance rate
//! optimization, cutter head torque management, and segment installation
//! coordination (spec.txt §6).

mod advance_rate;
mod cutter_head_torque;
mod segment_installation;

pub use advance_rate::max_sustainable_advance_rate_m_s;
pub use cutter_head_torque::torque_limited_advance_rate_m_s;
pub use segment_installation::{can_advance, SegmentRing};
