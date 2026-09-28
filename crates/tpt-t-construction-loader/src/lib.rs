// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Wheel/front-end loader control: bucket digging and lifting kinematics,
//! the dig/lift/carry/dump cycle, and truck loading with payload weighing
//! (spec.txt §5.4).

mod cycle;
mod kinematics;
mod truck_loading;

pub use cycle::{LoadCycle, LoadPhase};
pub use kinematics::{
    boom_angle_for_lift_height, boom_tip_position, bucket_joint_for_absolute_angle,
    forward_kinematics, JointAngles, LinkLengths, UnreachableTarget,
};
pub use truck_loading::TruckLoadTarget;
