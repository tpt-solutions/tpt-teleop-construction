// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Operator fatigue monitoring (for teleoperation or supervised
//! autonomy): camera-based eye tracking (PERCLOS), steering pattern
//! analysis, and mandatory rest enforcement (spec.txt §7).

mod eye_tracking;
mod rest_enforcement;
mod steering_pattern;

pub use eye_tracking::PerclosMonitor;
pub use rest_enforcement::RestEnforcer;
pub use steering_pattern::SteeringActivityMonitor;
