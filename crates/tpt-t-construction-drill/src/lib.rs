// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Automated drilling rigs: blast hole pattern execution, rock hardness
//! detection via drill rate/torque, and collar positioning via LiDAR/GNSS
//! (spec.txt §5.5).

mod collar;
mod hardness;
mod pattern;

pub use collar::{collar_error_m, is_fix_trustworthy, is_within_tolerance, PositionFix};
pub use hardness::{classify_hardness, specific_energy_j_m3, HardnessBands, RockHardness};
pub use pattern::{BlastHole, BlastPattern, PatternExecutor, Point2};
