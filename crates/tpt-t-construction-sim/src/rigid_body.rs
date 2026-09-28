// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Generic 6DOF rigid body dynamics: the core integrator every machine
//! model (haul truck chassis, excavator upper structure, dozer) builds on
//! by feeding in world-frame force and body-frame torque each tick.
//!
//! Rotational dynamics follow Euler's rigid body equations with a diagonal
//! (principal-axis) inertia tensor; translational dynamics are Newton's
//! second law. Both are integrated with semi-implicit ("symplectic")
//! Euler, which is unconditionally stable for the stiff, lightly-damped
//! systems this workspace simulates (unlike explicit Euler) while costing
//! no more per step.

use crate::math::{Quat, Vec3};

/// The dynamic state of one 6DOF rigid body.
#[derive(Debug, Clone, Copy)]
pub struct RigidBody6Dof {
    pub position: Vec3,
    /// Body-to-world orientation.
    pub orientation: Quat,
    /// Linear velocity, world frame.
    pub linear_velocity: Vec3,
    /// Angular velocity, body frame (the natural frame for Euler's
    /// equations, since the inertia tensor is constant there).
    pub angular_velocity: Vec3,
    pub mass_kg: f32,
    /// Principal moments of inertia (body frame axes assumed aligned with
    /// principal axes, which holds well enough for the box-ish chassis of
    /// heavy equipment).
    pub inertia_diag_kg_m2: Vec3,
}

impl RigidBody6Dof {
    pub fn stationary(mass_kg: f32, inertia_diag_kg_m2: Vec3) -> Self {
        RigidBody6Dof {
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            mass_kg,
            inertia_diag_kg_m2,
        }
    }

    /// Advances the body one step under a world-frame applied force and a
    /// body-frame applied torque (e.g. from tire/track-soil contact,
    /// gravity, hydraulic cylinder reaction loads).
    pub fn step(&mut self, force_world: Vec3, torque_body: Vec3, dt_s: f32) {
        // Translation: a = F / m.
        let linear_accel = force_world * (1.0 / self.mass_kg);
        self.linear_velocity += linear_accel * dt_s;
        self.position += self.linear_velocity * dt_s;

        // Rotation: Euler's equation, I*alpha = torque - omega x (I*omega),
        // solved for alpha with I diagonal so the cross term and the
        // inverse are both plain component-wise vector ops.
        let angular_momentum = self.inertia_diag_kg_m2.hadamard(self.angular_velocity);
        let gyroscopic = self.angular_velocity.cross(angular_momentum);
        let angular_accel = (torque_body - gyroscopic).div_components(self.inertia_diag_kg_m2);
        self.angular_velocity += angular_accel * dt_s;
        self.orientation = self.orientation.integrate(self.angular_velocity, dt_s);
    }

    /// Roll, pitch, yaw in radians.
    pub fn roll_pitch_yaw(&self) -> (f32, f32, f32) {
        self.orientation.to_euler()
    }

    /// Velocity of a point offset `r_body` from the center of mass
    /// (body frame), expressed in the world frame. Used to find the
    /// contact-point velocity at each wheel/track shoe for traction
    /// calculations.
    pub fn point_velocity_world(&self, r_body: Vec3) -> Vec3 {
        let omega_cross_r = self.angular_velocity.cross(r_body);
        self.linear_velocity + self.orientation.rotate(omega_cross_r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_body_under_gravity_falls() {
        let mut body = RigidBody6Dof::stationary(1000.0, Vec3::new(500.0, 500.0, 500.0));
        let gravity = Vec3::new(0.0, 0.0, -9.81 * body.mass_kg);
        for _ in 0..100 {
            body.step(gravity, Vec3::ZERO, 0.01);
        }
        // After 1s of free fall, v = g*t = -9.81 m/s.
        assert!((body.linear_velocity.z - (-9.81)).abs() < 1e-3);
        assert!(body.position.z < 0.0);
    }

    #[test]
    fn no_force_no_torque_is_static_equilibrium() {
        let mut body = RigidBody6Dof::stationary(1000.0, Vec3::new(500.0, 500.0, 500.0));
        body.step(Vec3::ZERO, Vec3::ZERO, 0.01);
        assert_eq!(body.linear_velocity, Vec3::ZERO);
        assert_eq!(body.position, Vec3::ZERO);
        assert_eq!(body.angular_velocity, Vec3::ZERO);
    }

    #[test]
    fn pure_torque_spins_up_about_its_axis() {
        let mut body = RigidBody6Dof::stationary(1000.0, Vec3::new(200.0, 200.0, 400.0));
        // Yaw torque only.
        for _ in 0..500 {
            body.step(Vec3::ZERO, Vec3::new(0.0, 0.0, 100.0), 0.01);
        }
        assert!(body.angular_velocity.z > 0.0);
        assert!(body.angular_velocity.x.abs() < 1e-6);
        assert!(body.angular_velocity.y.abs() < 1e-6);
        let (_, _, yaw) = body.roll_pitch_yaw();
        assert!(yaw.is_finite());
    }

    #[test]
    fn point_velocity_matches_rigid_rotation_at_offset() {
        let mut body = RigidBody6Dof::stationary(1000.0, Vec3::new(500.0, 500.0, 500.0));
        body.angular_velocity = Vec3::new(0.0, 0.0, 1.0); // 1 rad/s yaw
        let r = Vec3::new(2.0, 0.0, 0.0); // 2m ahead of CoM, body frame
        let v = body.point_velocity_world(r);
        // omega x r = (0,0,1) x (2,0,0) = (0,2,0)
        assert!((v.x - 0.0).abs() < 1e-5);
        assert!((v.y - 2.0).abs() < 1e-5);
    }
}
