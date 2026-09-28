// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Material tracking and volume calculation: cut/fill volume estimation,
//! zero-copy slab-allocated point cloud storage, on-board payload
//! weighing, and material type classification (spec.txt §3).

mod material;
mod point_cloud;
mod scale;
mod volume;

pub use material::{bulk_density_kg_m3, classify_by_density, DensityBands, MaterialType};
pub use point_cloud::{new_point_cloud_channel, CloudFull, Point3, PointCloud};
pub use scale::{LoadCell, PayloadScale, SettlingDetector};
pub use volume::{cut_fill_volumes, CutFillResult};
