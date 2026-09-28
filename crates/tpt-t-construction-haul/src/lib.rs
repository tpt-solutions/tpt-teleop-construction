// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Autonomous haul truck coordination: haul road navigation and edge
//! drop-off avoidance, speed management on downgrades/curves/low
//! visibility, dump-point/crusher queueing, and per-shift tonnage
//! tracking (spec.txt §5.2).

mod queue;
mod road;
mod speed;
mod tonnage;

pub use queue::{estimated_wait_s, DumpQueue};
pub use road::{HaulRoad, NearestSegment, Point2, RoadSegment};
pub use speed::{
    combined_speed_limit_m_s, curve_speed_limit_m_s, downgrade_speed_limit_m_s,
    visibility_speed_limit_m_s, SpeedLimitParams,
};
pub use tonnage::ShiftTonnage;
