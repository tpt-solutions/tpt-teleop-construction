// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Boom-arm-bucket kinematics (spec.txt §5.1): the standard geometric
//! forward/inverse kinematics for a 3-link planar excavator arm (boom,
//! arm/stick, bucket) mounted on a rotating swing turret.
//!
//! The digging plane is treated as 2D (radial distance `r` from the swing
//! axis, and height `z`); the swing angle then rotates that plane about
//! the vertical axis to place the bucket tip in 3D machine space. Inverse
//! kinematics fixes the elbow ("arm") joint to its "curled forward"
//! solution, matching how an excavator's arm cylinder is mounted — the
//! other geometric solution requires the arm to fold back through the
//! boom, which is not physically realizable.

/// A point in the machine's local frame, meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Link lengths, meters, pivot to pivot.
#[derive(Debug, Clone, Copy)]
pub struct LinkLengths {
    pub boom_m: f32,
    pub arm_m: f32,
    pub bucket_m: f32,
}

/// The four joint angles: swing (rotation about the vertical axis) plus
/// the three digging-plane joints, all radians.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointAngles {
    pub swing_rad: f32,
    pub boom_rad: f32,
    pub arm_rad: f32,
    pub bucket_rad: f32,
}

/// The requested bucket tip target cannot be reached by this link
/// geometry (too far away, or too close in for the links to fold into).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnreachableTarget;

/// Computes the bucket tooth tip position for a given set of joint
/// angles.
pub fn forward_kinematics(links: &LinkLengths, joints: &JointAngles) -> Point3 {
    let theta1 = joints.boom_rad;
    let theta12 = joints.boom_rad + joints.arm_rad;
    let theta123 = theta12 + joints.bucket_rad;

    let r =
        links.boom_m * theta1.cos() + links.arm_m * theta12.cos() + links.bucket_m * theta123.cos();
    let z =
        links.boom_m * theta1.sin() + links.arm_m * theta12.sin() + links.bucket_m * theta123.sin();

    Point3 {
        x: r * joints.swing_rad.cos(),
        y: r * joints.swing_rad.sin(),
        z,
    }
}

/// Solves for the joint angles that place the bucket tip at radial
/// distance `target_r_m` and height `target_z_m` from the swing axis, at
/// swing angle `swing_rad`, with the bucket link held at absolute angle
/// `bucket_absolute_angle_rad` (e.g. teeth angled down for digging).
///
/// Fixing the bucket's *absolute* angle rather than its joint angle
/// relative to the arm is what makes this system fully determined: three
/// unknowns (boom, arm, bucket joint angles), three constraints (target
/// r, target z, bucket absolute angle).
pub fn inverse_kinematics(
    links: &LinkLengths,
    swing_rad: f32,
    target_r_m: f32,
    target_z_m: f32,
    bucket_absolute_angle_rad: f32,
) -> Result<JointAngles, UnreachableTarget> {
    // Subtract the bucket link's own contribution to find where the
    // wrist (arm-bucket joint) must be, reducing this to the classic
    // 2-link planar arm IK problem for the boom and arm.
    let wrist_r = target_r_m - links.bucket_m * bucket_absolute_angle_rad.cos();
    let wrist_z = target_z_m - links.bucket_m * bucket_absolute_angle_rad.sin();

    let l1 = links.boom_m;
    let l2 = links.arm_m;
    let dist_sq = wrist_r * wrist_r + wrist_z * wrist_z;
    let dist = dist_sq.sqrt();

    if dist > l1 + l2 || dist < (l1 - l2).abs() {
        return Err(UnreachableTarget);
    }

    // Law of cosines for the elbow angle; the "+" root is the
    // arm-curled-forward solution (arm bends toward the machine, as an
    // excavator's arm cylinder geometry requires).
    let cos_theta2 = ((dist_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)).clamp(-1.0, 1.0);
    let theta2 = cos_theta2.acos();
    let theta1 = wrist_z.atan2(wrist_r) - (l2 * theta2.sin()).atan2(l1 + l2 * theta2.cos());
    let theta3 = bucket_absolute_angle_rad - theta1 - theta2;

    Ok(JointAngles {
        swing_rad,
        boom_rad: theta1,
        arm_rad: theta2,
        bucket_rad: theta3,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn typical_links() -> LinkLengths {
        LinkLengths {
            boom_m: 6.0,
            arm_m: 3.0,
            bucket_m: 1.5,
        }
    }

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() <= eps
    }

    #[test]
    fn forward_kinematics_places_a_straight_arm_at_full_reach() {
        let links = typical_links();
        let joints = JointAngles {
            swing_rad: 0.0,
            boom_rad: 0.0,
            arm_rad: 0.0,
            bucket_rad: 0.0,
        };
        let tip = forward_kinematics(&links, &joints);
        // Fully extended along the swing-zero direction: r = sum of link lengths.
        assert!(approx_eq(tip.x, 10.5, 1e-4));
        assert!(approx_eq(tip.y, 0.0, 1e-4));
        assert!(approx_eq(tip.z, 0.0, 1e-4));
    }

    #[test]
    fn inverse_then_forward_recovers_the_target() {
        let links = typical_links();
        for (target_r, target_z) in [(7.0, -2.0), (5.0, 1.0), (8.5, -4.0), (4.0, 3.0)] {
            let joints = inverse_kinematics(&links, 0.3, target_r, target_z, -0.5)
                .unwrap_or_else(|_| panic!("target ({target_r}, {target_z}) should be reachable"));
            let tip = forward_kinematics(&links, &joints);
            let r = (tip.x * tip.x + tip.y * tip.y).sqrt();
            assert!(
                approx_eq(r, target_r, 1e-3),
                "r mismatch: {r} vs {target_r}"
            );
            assert!(
                approx_eq(tip.z, target_z, 1e-3),
                "z mismatch: {} vs {target_z}",
                tip.z
            );
        }
    }

    #[test]
    fn swing_angle_rotates_the_tip_about_the_vertical_axis() {
        let links = typical_links();
        let joints =
            inverse_kinematics(&links, std::f32::consts::FRAC_PI_2, 7.0, -1.0, -0.3).unwrap();
        let tip = forward_kinematics(&links, &joints);
        // Swung 90 degrees: reach should land almost entirely on Y, not X.
        assert!(tip.x.abs() < 1e-3);
        assert!(tip.y > 6.0);
    }

    #[test]
    fn a_target_beyond_full_reach_is_unreachable() {
        let links = typical_links();
        assert_eq!(
            inverse_kinematics(&links, 0.0, 100.0, 0.0, 0.0),
            Err(UnreachableTarget)
        );
    }

    #[test]
    fn a_target_too_close_in_is_unreachable() {
        let links = typical_links();
        // Closer than |boom - arm| leaves no way to fold the two links in.
        assert_eq!(
            inverse_kinematics(&links, 0.0, 0.1, 0.0, 0.0),
            Err(UnreachableTarget)
        );
    }
}
