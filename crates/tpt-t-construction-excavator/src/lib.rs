// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Autonomous excavator control: trench digging to exact depth/width,
//! truck loading with minimal swing cycle time, grading to a designed
//! slope, boom-arm-bucket kinematics with hydraulic cylinder coordination,
//! and obstacle avoidance (spec.txt §5.1).

mod cylinder_coordination;
mod kinematics;
mod obstacle_avoidance;
mod swing_cycle;
mod trench;

pub use cylinder_coordination::{CylinderCoordination, CylinderMount, JointRates};
pub use kinematics::{
    forward_kinematics, inverse_kinematics, JointAngles, LinkLengths, Point3, UnreachableTarget,
};
pub use obstacle_avoidance::{nearest_zone_clearance_m, path_clear_of_obstacles, KeepOutZone};
pub use swing_cycle::{estimate_cycle_time, SwingCycle, SwingPhase};
pub use trench::{GradePlane, TrenchPlan};
