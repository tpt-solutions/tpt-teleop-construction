// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Headless simulator: 6DOF vehicle dynamics, tire/track-soil and rollover
//! physics, hydraulic system dynamics, diesel engine/transmission curves,
//! synthetic LiDAR/radar returns, and proximity-detection scenario
//! scripting (spec.txt §8). No rendering here: `tpt-t-construction-viz` is
//! the separate egui visualizer binary that displays this crate's output.

pub mod hydraulics;
pub mod lidar;
pub mod math;
pub mod perf;
pub mod powertrain;
pub mod rigid_body;
pub mod scenario;
pub mod soil;
pub mod vehicle;

pub use math::{Quat, Vec3};
pub use rigid_body::RigidBody6Dof;
pub use vehicle::SimVehicle;
