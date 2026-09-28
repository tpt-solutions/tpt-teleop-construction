// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Bucket digging and lifting kinematics (spec.txt §5.3/5.4's loader
//! bullet): unlike `tpt-t-construction-excavator`'s boom-arm-bucket arm,
//! a wheel loader's lift arm is a single rigid link rotating about one
//! chassis-mounted pivot — reach isn't independently controllable by a
//! second "stick" joint, it's a consequence of lift angle (and of driving
//! the machine forward/back). So there's no 2-link position IK here:
//! given a target lift height, there's exactly one boom angle that
//! reaches it (on the boom's arc), and the bucket's own curl/rack angle
//! is commanded independently for its absolute orientation.

/// Link lengths, meters, pivot to pivot.
#[derive(Debug, Clone, Copy)]
pub struct LinkLengths {
    pub boom_m: f32,
    pub bucket_m: f32,
}

/// `boom_rad`: lift angle from horizontal. `bucket_rad`: bucket joint
/// angle *relative to the boom*, not absolute.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointAngles {
    pub boom_rad: f32,
    pub bucket_rad: f32,
}

/// The requested lift height is beyond the boom's physical arc (further
/// than fully raised, or lower than fully lowered).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnreachableTarget;

/// The boom tip's position (x forward, z up) in the machine's local
/// vertical plane, relative to the boom pivot.
pub fn boom_tip_position(links: &LinkLengths, boom_rad: f32) -> (f32, f32) {
    (links.boom_m * boom_rad.cos(), links.boom_m * boom_rad.sin())
}

/// The bucket tooth tip's position for a given set of joint angles.
pub fn forward_kinematics(links: &LinkLengths, joints: &JointAngles) -> (f32, f32) {
    let (boom_x, boom_z) = boom_tip_position(links, joints.boom_rad);
    let theta_total = joints.boom_rad + joints.bucket_rad;
    (
        boom_x + links.bucket_m * theta_total.cos(),
        boom_z + links.bucket_m * theta_total.sin(),
    )
}

/// The boom angle that puts the boom tip at `target_height_m` above the
/// pivot (the only degree of freedom a single-link boom has).
pub fn boom_angle_for_lift_height(
    links: &LinkLengths,
    target_height_m: f32,
) -> Result<f32, UnreachableTarget> {
    let ratio = target_height_m / links.boom_m;
    if !(-1.0..=1.0).contains(&ratio) {
        return Err(UnreachableTarget);
    }
    Ok(ratio.asin())
}

/// The bucket joint angle (relative to the boom) needed to hold the
/// bucket at absolute angle `bucket_absolute_angle_rad` (e.g. `0` =
/// level, for carrying a load without spilling) given the current boom
/// angle.
pub fn bucket_joint_for_absolute_angle(boom_rad: f32, bucket_absolute_angle_rad: f32) -> f32 {
    bucket_absolute_angle_rad - boom_rad
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() <= eps
    }

    #[test]
    fn boom_tip_at_zero_angle_is_straight_out() {
        let links = LinkLengths {
            boom_m: 2.0,
            bucket_m: 0.8,
        };
        let (x, z) = boom_tip_position(&links, 0.0);
        assert!(approx_eq(x, 2.0, 1e-4));
        assert!(approx_eq(z, 0.0, 1e-4));
    }

    #[test]
    fn boom_tip_fully_raised_is_straight_up() {
        let links = LinkLengths {
            boom_m: 2.0,
            bucket_m: 0.8,
        };
        let (x, z) = boom_tip_position(&links, FRAC_PI_2);
        assert!(approx_eq(x, 0.0, 1e-4));
        assert!(approx_eq(z, 2.0, 1e-4));
    }

    #[test]
    fn boom_angle_for_height_round_trips_through_boom_tip_position() {
        let links = LinkLengths {
            boom_m: 2.5,
            bucket_m: 0.8,
        };
        for target_height in [-1.0, 0.0, 1.0, 2.0] {
            let angle = boom_angle_for_lift_height(&links, target_height).unwrap();
            let (_, z) = boom_tip_position(&links, angle);
            assert!(approx_eq(z, target_height, 1e-3));
        }
    }

    #[test]
    fn heights_beyond_the_booms_arc_are_unreachable() {
        let links = LinkLengths {
            boom_m: 2.0,
            bucket_m: 0.8,
        };
        assert_eq!(
            boom_angle_for_lift_height(&links, 3.0),
            Err(UnreachableTarget)
        );
        assert_eq!(
            boom_angle_for_lift_height(&links, -3.0),
            Err(UnreachableTarget)
        );
    }

    #[test]
    fn bucket_joint_holds_absolute_angle_as_boom_moves() {
        // To keep the bucket level (absolute angle 0) as the boom lifts,
        // the joint angle must counter-rotate by exactly the boom angle.
        let boom_rad = 0.9;
        let joint = bucket_joint_for_absolute_angle(boom_rad, 0.0);
        assert!(approx_eq(joint, -boom_rad, 1e-6));
    }

    #[test]
    fn forward_kinematics_combines_boom_and_bucket_contributions() {
        let links = LinkLengths {
            boom_m: 2.0,
            bucket_m: 1.0,
        };
        let joints = JointAngles {
            boom_rad: 0.0,
            bucket_rad: 0.0,
        };
        let (x, z) = forward_kinematics(&links, &joints);
        assert!(approx_eq(x, 3.0, 1e-4));
        assert!(approx_eq(z, 0.0, 1e-4));
    }
}
