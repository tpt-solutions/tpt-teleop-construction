// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Autonomous dozer control: slope grading via real-time blade position
//! feedback, real-time cut/fill volume tracking, and material pushing to
//! stockpiles/hoppers/crushers (spec.txt §5.3).

mod blade;
mod cutfill_tracker;
mod push_path;

pub use blade::{
    compute_corner_targets, corner_errors_m, BladeCornerTargets, BladeState, GradePlane,
};
pub use cutfill_tracker::PassTracker;
pub use push_path::{blade_load_fraction, is_overloaded, PushPlan};
