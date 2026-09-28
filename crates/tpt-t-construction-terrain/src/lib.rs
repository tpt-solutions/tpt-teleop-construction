// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Real-time 3D terrain modeling: incremental LiDAR surface
//! reconstruction, slope stability analysis, bearing capacity estimation,
//! and dynamic obstacle detection (spec.txt §3).

mod bearing;
mod grid;
mod obstacle;
mod slope;

pub use bearing::{
    bearing_capacity_factors, bearing_factor_of_safety, ultimate_bearing_capacity_pa,
    BearingCapacityFactors,
};
pub use grid::TerrainGrid;
pub use obstacle::{detect_obstacles, ObstacleCandidate};
pub use slope::{infinite_slope_factor_of_safety, slope_angle_rad};
