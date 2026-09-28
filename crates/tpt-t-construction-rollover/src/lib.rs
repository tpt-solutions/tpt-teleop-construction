// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Rollover protection system (ROPS) monitoring: real-time tilt angle
//! estimation, center-of-gravity estimation, and automatic brake
//! application as the rollover threshold is approached (spec.txt §7).

mod brake;
mod cog_estimation;
mod tilt_estimation;

pub use brake::{
    critical_roll_angle_rad, rollover_margin_rad, should_apply_brakes, static_stability_factor,
};
pub use cog_estimation::effective_cog_height_m;
pub use tilt_estimation::{
    pitch_from_accelerometer, roll_from_accelerometer, ComplementaryTiltFilter,
};
