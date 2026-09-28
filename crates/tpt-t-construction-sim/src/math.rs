// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Minimal `f32` vector/quaternion math, hand-rolled rather than pulling in
//! `glam`/`nalgebra`: the sim's needs (a 3-vector, a rotation quaternion,
//! and the handful of ops below) don't justify a general-purpose linear
//! algebra dependency, in keeping with the workspace's no-bloat policy
//! (spec.txt §7).

use std::ops::{Add, AddAssign, Mul, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }

    pub fn dot(self, rhs: Vec3) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn cross(self, rhs: Vec3) -> Vec3 {
        Vec3::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn normalized(self) -> Vec3 {
        let len = self.length();
        if len <= f32::EPSILON {
            Vec3::ZERO
        } else {
            self * (1.0 / len)
        }
    }

    /// Component-wise product, used for diagonal-inertia-tensor math
    /// (`inertia_diag * omega` instead of a full 3x3 matrix multiply).
    pub fn hadamard(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }

    /// Component-wise division; used to invert a diagonal inertia tensor.
    pub fn div_components(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x / rhs.x, self.y / rhs.y, self.z / rhs.z)
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Vec3) {
        *self = *self + rhs;
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, rhs: Vec3) -> Vec3 {
        Vec3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, rhs: f32) -> Vec3 {
        Vec3::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul for Quat {
    type Output = Quat;

    /// Hamilton product (quaternion composition): `self * rhs` applies
    /// `rhs`'s rotation first, then `self`'s.
    fn mul(self, rhs: Quat) -> Quat {
        Quat {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

/// A unit quaternion representing a body's orientation relative to the
/// world frame (Hamilton convention, `w + xi + yj + zk`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn from_axis_angle(axis: Vec3, angle_rad: f32) -> Quat {
        let axis = axis.normalized();
        let half = angle_rad * 0.5;
        let s = half.sin();
        Quat {
            w: half.cos(),
            x: axis.x * s,
            y: axis.y * s,
            z: axis.z * s,
        }
    }

    pub fn length(self) -> f32 {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn normalized(self) -> Quat {
        let len = self.length();
        if len <= f32::EPSILON {
            Quat::IDENTITY
        } else {
            let inv = 1.0 / len;
            Quat {
                w: self.w * inv,
                x: self.x * inv,
                y: self.y * inv,
                z: self.z * inv,
            }
        }
    }

    /// Rotates `v` from body frame into world frame.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        // v' = q * (0, v) * q_conjugate, expanded to avoid allocating an
        // intermediate quaternion multiply chain.
        let qv = Vec3::new(self.x, self.y, self.z);
        let uv = qv.cross(v);
        let uuv = qv.cross(uv);
        v + (uv * self.w + uuv) * 2.0
    }

    /// Advances orientation by one integration step given a body-frame
    /// angular velocity (rad/s), via the standard quaternion kinematic
    /// equation `dq/dt = 0.5 * q * (0, omega)`, then re-normalizes to
    /// correct for the integrator's first-order drift off the unit sphere.
    pub fn integrate(self, angular_velocity_body: Vec3, dt_s: f32) -> Quat {
        let omega = Quat {
            w: 0.0,
            x: angular_velocity_body.x,
            y: angular_velocity_body.y,
            z: angular_velocity_body.z,
        };
        let dq = self * omega;
        Quat {
            w: self.w + dq.w * 0.5 * dt_s,
            x: self.x + dq.x * 0.5 * dt_s,
            y: self.y + dq.y * 0.5 * dt_s,
            z: self.z + dq.z * 0.5 * dt_s,
        }
        .normalized()
    }

    /// Roll (x), pitch (y), yaw (z) in radians, aerospace (Z-Y-X) convention.
    pub fn to_euler(self) -> (f32, f32, f32) {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);

        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            std::f32::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() <= eps
    }

    #[test]
    fn vec3_cross_and_dot() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(x.cross(y), Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(x.dot(y), 0.0);
        assert_eq!(x.dot(x), 1.0);
    }

    #[test]
    fn quat_identity_rotation_is_noop() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let rotated = Quat::IDENTITY.rotate(v);
        assert!(approx_eq(rotated.x, v.x, 1e-6));
        assert!(approx_eq(rotated.y, v.y, 1e-6));
        assert!(approx_eq(rotated.z, v.z, 1e-6));
    }

    #[test]
    fn quat_90_degree_yaw_rotates_x_to_y() {
        let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
        let rotated = q.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(approx_eq(rotated.x, 0.0, 1e-5));
        assert!(approx_eq(rotated.y, 1.0, 1e-5));
        assert!(approx_eq(rotated.z, 0.0, 1e-5));
    }

    #[test]
    fn quat_to_euler_roundtrips_yaw() {
        let yaw = 0.7_f32;
        let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw);
        let (roll, pitch, recovered_yaw) = q.to_euler();
        assert!(approx_eq(roll, 0.0, 1e-5));
        assert!(approx_eq(pitch, 0.0, 1e-5));
        assert!(approx_eq(recovered_yaw, yaw, 1e-5));
    }

    #[test]
    fn quat_integrate_stays_unit_length() {
        let mut q = Quat::IDENTITY;
        for _ in 0..1000 {
            q = q.integrate(Vec3::new(0.1, 0.2, -0.05), 0.01);
        }
        assert!(approx_eq(q.length(), 1.0, 1e-4));
    }
}
