// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Hydraulic cylinder coordination (spec.txt §5.1): each of
//! [`crate::kinematics::JointAngles`]'s three joints is actually driven by
//! a linear hydraulic cylinder mounted off-axis from the joint's pivot,
//! not a direct rotary actuator — so commanding a joint angle (or angular
//! rate) means first converting it to the cylinder length (or extension
//! rate) that produces it. That conversion is the classic "cylinder
//! actuating a revolute joint via two fixed moment arms" geometry, solved
//! with the law of cosines on the triangle formed by the two mounting
//! arms and the cylinder itself.

/// One cylinder's mounting geometry: it connects a base-end pivot
/// `arm1_m` from the joint axis to a rod-end pivot `arm2_m` from the
/// joint axis, and `angle_offset_rad` is the constant offset between the
/// joint's own zero angle and the included angle of those two arms (a
/// property of how the cylinder is physically mounted, not something the
/// controller chooses).
#[derive(Debug, Clone, Copy)]
pub struct CylinderMount {
    pub arm1_m: f32,
    pub arm2_m: f32,
    pub angle_offset_rad: f32,
}

impl CylinderMount {
    /// The cylinder length that produces `joint_angle_rad`, via the law
    /// of cosines: `L^2 = r1^2 + r2^2 - 2*r1*r2*cos(theta)`.
    pub fn cylinder_length_m(&self, joint_angle_rad: f32) -> f32 {
        let theta = joint_angle_rad + self.angle_offset_rad;
        (self.arm1_m * self.arm1_m + self.arm2_m * self.arm2_m
            - 2.0 * self.arm1_m * self.arm2_m * theta.cos())
        .max(0.0)
        .sqrt()
    }

    /// The joint angle that produces a given cylinder length (the
    /// inverse of [`Self::cylinder_length_m`]), or `None` if
    /// `length_m` is physically impossible for this mount (shorter than
    /// `|arm1 - arm2|` or longer than `arm1 + arm2`).
    pub fn joint_angle_for_length(&self, length_m: f32) -> Option<f32> {
        if length_m < (self.arm1_m - self.arm2_m).abs() || length_m > self.arm1_m + self.arm2_m {
            return None;
        }
        let cos_theta = ((self.arm1_m.powi(2) + self.arm2_m.powi(2) - length_m.powi(2))
            / (2.0 * self.arm1_m * self.arm2_m))
            .clamp(-1.0, 1.0);
        Some(cos_theta.acos() - self.angle_offset_rad)
    }

    /// The cylinder extension rate (m/s) that results from the joint
    /// rotating at `joint_angular_velocity_rad_s` while at
    /// `joint_angle_rad`, via `dL/dtheta = (arm1*arm2*sin(theta)) / L`
    /// (the analytic derivative of the law-of-cosines relationship) —
    /// the "Jacobian" relating joint-space and cylinder-space rates for
    /// this single degree of freedom.
    pub fn cylinder_velocity_m_s(
        &self,
        joint_angle_rad: f32,
        joint_angular_velocity_rad_s: f32,
    ) -> f32 {
        let theta = joint_angle_rad + self.angle_offset_rad;
        let length = self.cylinder_length_m(joint_angle_rad);
        if length <= 1e-6 {
            return 0.0;
        }
        let dl_dtheta = (self.arm1_m * self.arm2_m * theta.sin()) / length;
        dl_dtheta * joint_angular_velocity_rad_s
    }
}

/// Angular rates for the three digging-plane joints, matching
/// [`crate::kinematics::JointAngles`] minus swing (swing is turret
/// rotation, not part of this cylinder-coordination geometry).
#[derive(Debug, Clone, Copy)]
pub struct JointRates {
    pub boom_rad_s: f32,
    pub arm_rad_s: f32,
    pub bucket_rad_s: f32,
}

/// The three cylinders (boom, arm, bucket) and their mounting geometry.
#[derive(Debug, Clone, Copy)]
pub struct CylinderCoordination {
    pub boom: CylinderMount,
    pub arm: CylinderMount,
    pub bucket: CylinderMount,
}

impl CylinderCoordination {
    /// Converts current joint angles and a commanded joint angular
    /// velocity for each into the three cylinders' required extension
    /// rates (m/s) — what the hydraulic control loop (Phase 3's
    /// `tpt-t-construction-hydraulic`) actually needs to drive each valve
    /// toward, so all three cylinders arrive in sync rather than each
    /// finishing its own motion at a different time.
    pub fn cylinder_velocities_m_s(
        &self,
        joints: &crate::kinematics::JointAngles,
        rates: &JointRates,
    ) -> (f32, f32, f32) {
        (
            self.boom
                .cylinder_velocity_m_s(joints.boom_rad, rates.boom_rad_s),
            self.arm
                .cylinder_velocity_m_s(joints.arm_rad, rates.arm_rad_s),
            self.bucket
                .cylinder_velocity_m_s(joints.bucket_rad, rates.bucket_rad_s),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() <= eps
    }

    fn typical_mount() -> CylinderMount {
        CylinderMount {
            arm1_m: 0.6,
            arm2_m: 0.5,
            angle_offset_rad: 1.2,
        }
    }

    #[test]
    fn length_matches_law_of_cosines_at_a_known_angle() {
        let mount = CylinderMount {
            arm1_m: 1.0,
            arm2_m: 1.0,
            angle_offset_rad: 0.0,
        };
        // theta = pi/2: isoceles right triangle, L = sqrt(2).
        let length = mount.cylinder_length_m(std::f32::consts::FRAC_PI_2);
        assert!(approx_eq(length, std::f32::consts::SQRT_2, 1e-4));
    }

    #[test]
    fn inverse_recovers_the_original_joint_angle() {
        let mount = typical_mount();
        for angle in [-0.5_f32, -0.1, 0.2, 0.6] {
            let length = mount.cylinder_length_m(angle);
            let recovered = mount.joint_angle_for_length(length).unwrap();
            assert!(
                approx_eq(recovered, angle, 1e-3),
                "angle {angle} vs recovered {recovered}"
            );
        }
    }

    #[test]
    fn length_outside_the_physical_range_is_unreachable() {
        let mount = typical_mount();
        let max_length = mount.arm1_m + mount.arm2_m;
        assert_eq!(mount.joint_angle_for_length(max_length + 0.1), None);
        let min_length = (mount.arm1_m - mount.arm2_m).abs();
        assert_eq!(mount.joint_angle_for_length(min_length - 0.05), None);
    }

    #[test]
    fn cylinder_velocity_matches_a_finite_difference_estimate() {
        let mount = typical_mount();
        let angle = 0.3_f32;
        let dt = 1e-4;
        let l0 = mount.cylinder_length_m(angle);
        let l1 = mount.cylinder_length_m(angle + dt);
        let numerical_rate = (l1 - l0) / dt;
        let analytic_rate = mount.cylinder_velocity_m_s(angle, 1.0);
        assert!(
            approx_eq(numerical_rate, analytic_rate, 1e-2),
            "numerical {numerical_rate} vs analytic {analytic_rate}"
        );
    }

    #[test]
    fn zero_joint_rate_gives_zero_cylinder_velocity() {
        let mount = typical_mount();
        assert_eq!(mount.cylinder_velocity_m_s(0.3, 0.0), 0.0);
    }

    #[test]
    fn coordination_combines_all_three_cylinders_independently() {
        let coordination = CylinderCoordination {
            boom: typical_mount(),
            arm: typical_mount(),
            bucket: typical_mount(),
        };
        let joints = crate::kinematics::JointAngles {
            swing_rad: 0.0,
            boom_rad: 0.1,
            arm_rad: 0.2,
            bucket_rad: -0.1,
        };
        let rates = JointRates {
            boom_rad_s: 0.5,
            arm_rad_s: 0.0,
            bucket_rad_s: -0.3,
        };
        let (boom_v, arm_v, bucket_v) = coordination.cylinder_velocities_m_s(&joints, &rates);
        assert_ne!(boom_v, 0.0);
        assert_eq!(arm_v, 0.0);
        assert_ne!(bucket_v, 0.0);
    }
}
