// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Underground mine navigation: LiDAR SLAM (a custom EKF fusing LiDAR,
//! wheel odometry, and IMU), 2D ICP scan matching, UWB beacon
//! integration, ventilation-aware routing, and ground-support rig
//! coordination (spec.txt §6).

mod ekf;
mod ground_support;
mod icp;
mod matrix3;
mod uwb;
mod ventilation;

pub use ekf::Ekf2D;
pub use ground_support::{is_safe_to_proceed, GroundSupportRig, RigStatus};
pub use icp::{
    compute_rigid_transform_2d, icp_align, nearest_neighbor_correspondences, RigidTransform2D,
};
pub use matrix3::{Mat3, Vec3f};
pub use uwb::{multilateration_2d, Beacon, RangeMeasurement};
pub use ventilation::{shortest_safe_route, Edge, VentilationGraph};
