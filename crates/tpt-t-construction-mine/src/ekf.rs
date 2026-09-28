// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Custom Extended Kalman Filter fusing wheel odometry, IMU yaw rate, and
//! LiDAR/UWB pose corrections for GPS-denied underground localization
//! (spec.txt §4.3, §6). State is the machine's planar pose `(x, y,
//! theta)`; underground mine tunnels are navigated in 2D, so a full 6DOF
//! pose estimate isn't needed the way it would be on open, uneven terrain.
//!
//! - [`Ekf2D::predict`] is the motion model: wheel odometry gives linear
//!   speed, the IMU gyro gives yaw rate directly (more reliable than
//!   differencing odometry alone in a skidding/slipping track vehicle).
//! - [`Ekf2D::update_position`] corrects `(x, y)` from a UWB range fix.
//! - [`Ekf2D::update_pose`] corrects the full `(x, y, theta)` from a
//!   LiDAR scan-matched pose (see [`crate::icp`]).

use crate::matrix3::{self, Mat3, Vec3f};
use std::f32::consts::PI;

/// Wraps an angle to `(-pi, pi]`, needed everywhere a `theta` innovation
/// is computed: without it, a true heading near `+pi` and an estimate
/// near `-pi` would appear to differ by nearly a full turn instead of a
/// small angle.
fn wrap_angle(angle_rad: f32) -> f32 {
    let mut a = (angle_rad + PI) % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a - PI
}

/// The filter's belief: a pose estimate and its covariance.
#[derive(Debug, Clone)]
pub struct Ekf2D {
    /// `[x_m, y_m, theta_rad]`.
    pub state: Vec3f,
    pub covariance: Mat3,
}

impl Ekf2D {
    pub fn new(initial_state: Vec3f, initial_covariance: Mat3) -> Self {
        Ekf2D {
            state: initial_state,
            covariance: initial_covariance,
        }
    }

    /// Propagates the state forward under a unicycle motion model driven
    /// by wheel-odometry linear speed and IMU-measured yaw rate, and
    /// grows the covariance by `process_noise` plus the motion model's
    /// own linearized uncertainty.
    pub fn predict(
        &mut self,
        linear_velocity_m_s: f32,
        angular_velocity_rad_s: f32,
        dt_s: f32,
        process_noise: &Mat3,
    ) {
        let theta = self.state[2];
        let dx = linear_velocity_m_s * theta.cos() * dt_s;
        let dy = linear_velocity_m_s * theta.sin() * dt_s;
        let dtheta = angular_velocity_rad_s * dt_s;

        self.state = [
            self.state[0] + dx,
            self.state[1] + dy,
            wrap_angle(self.state[2] + dtheta),
        ];

        // Jacobian of the motion model with respect to the state.
        let f: Mat3 = [
            [1.0, 0.0, -linear_velocity_m_s * theta.sin() * dt_s],
            [0.0, 1.0, linear_velocity_m_s * theta.cos() * dt_s],
            [0.0, 0.0, 1.0],
        ];
        let fp = matrix3::mul(&f, &self.covariance);
        let fpft = matrix3::mul(&fp, &matrix3::transpose(&f));
        self.covariance = matrix3::add(&fpft, process_noise);
    }

    /// Corrects `(x, y)` from an absolute position fix (e.g. UWB
    /// multilateration, see [`crate::uwb`]), leaving heading uncertainty
    /// untouched by this measurement.
    pub fn update_position(&mut self, measured_xy: [f32; 2], measurement_noise: [[f32; 2]; 2]) {
        let p = self.covariance;
        let innovation = [
            measured_xy[0] - self.state[0],
            measured_xy[1] - self.state[1],
        ];

        // S = H*P*H^T + R, where H selects the x,y rows/columns of P.
        let s = [
            [
                p[0][0] + measurement_noise[0][0],
                p[0][1] + measurement_noise[0][1],
            ],
            [
                p[1][0] + measurement_noise[1][0],
                p[1][1] + measurement_noise[1][1],
            ],
        ];
        let det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
        if det.abs() < 1e-12 {
            return;
        }
        let s_inv = [
            [s[1][1] / det, -s[0][1] / det],
            [-s[1][0] / det, s[0][0] / det],
        ];

        // K = P*H^T*S^-1; P*H^T is P's first two columns.
        let mut k = [[0.0f32; 2]; 3];
        for i in 0..3 {
            for j in 0..2 {
                k[i][j] = p[i][0] * s_inv[0][j] + p[i][1] * s_inv[1][j];
            }
        }

        for (state_i, k_row) in self.state.iter_mut().zip(k.iter()) {
            *state_i += k_row[0] * innovation[0] + k_row[1] * innovation[1];
        }
        self.state[2] = wrap_angle(self.state[2]);

        // P = (I - K*H)*P; K*H has K's two columns in positions 0,1 and
        // zero in position 2, since H only picks out x and y.
        let mut kh = [[0.0f32; 3]; 3];
        for (i, row) in kh.iter_mut().enumerate() {
            row[0] = k[i][0];
            row[1] = k[i][1];
        }
        let i_minus_kh = matrix3::sub(&matrix3::identity(), &kh);
        self.covariance = matrix3::mul(&i_minus_kh, &self.covariance);
    }

    /// Corrects the full pose from a LiDAR scan-matched pose estimate
    /// (see [`crate::icp::icp_align`] composed onto a prior pose).
    pub fn update_pose(&mut self, measured_pose: Vec3f, measurement_noise: &Mat3) {
        let innovation: Vec3f = [
            measured_pose[0] - self.state[0],
            measured_pose[1] - self.state[1],
            wrap_angle(measured_pose[2] - self.state[2]),
        ];

        let s = matrix3::add(&self.covariance, measurement_noise);
        let Some(s_inv) = matrix3::inverse(&s) else {
            return;
        };
        let k = matrix3::mul(&self.covariance, &s_inv);
        let correction = matrix3::vec_mul(&k, &innovation);

        for (state_i, corr_i) in self.state.iter_mut().zip(correction.iter()) {
            *state_i += corr_i;
        }
        self.state[2] = wrap_angle(self.state[2]);

        let i_minus_k = matrix3::sub(&matrix3::identity(), &k);
        self.covariance = matrix3::mul(&i_minus_k, &self.covariance);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small_noise() -> Mat3 {
        [[0.001, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 0.0001]]
    }

    #[test]
    fn predict_moves_straight_ahead_when_facing_zero_heading() {
        let mut ekf = Ekf2D::new([0.0, 0.0, 0.0], matrix3::identity());
        ekf.predict(2.0, 0.0, 1.0, &small_noise());
        assert!((ekf.state[0] - 2.0).abs() < 1e-4);
        assert!(ekf.state[1].abs() < 1e-4);
    }

    #[test]
    fn predict_with_zero_velocity_holds_position() {
        let mut ekf = Ekf2D::new([5.0, -3.0, 0.7], matrix3::identity());
        ekf.predict(0.0, 0.0, 1.0, &small_noise());
        assert!((ekf.state[0] - 5.0).abs() < 1e-4);
        assert!((ekf.state[1] - -3.0).abs() < 1e-4);
        assert!((ekf.state[2] - 0.7).abs() < 1e-4);
    }

    #[test]
    fn predict_grows_uncertainty() {
        let mut ekf = Ekf2D::new([0.0, 0.0, 0.0], matrix3::identity());
        let before = ekf.covariance[0][0];
        ekf.predict(1.0, 0.1, 1.0, &small_noise());
        assert!(ekf.covariance[0][0] >= before);
    }

    #[test]
    fn position_update_pulls_estimate_toward_the_measurement() {
        let mut ekf = Ekf2D::new([0.0, 0.0, 0.0], matrix3::identity());
        ekf.update_position([10.0, 10.0], [[0.01, 0.0], [0.0, 0.01]]);
        assert!(ekf.state[0] > 5.0);
        assert!(ekf.state[1] > 5.0);
    }

    #[test]
    fn position_update_shrinks_position_uncertainty() {
        let mut ekf = Ekf2D::new([0.0, 0.0, 0.0], matrix3::identity());
        let before = ekf.covariance[0][0];
        ekf.update_position([1.0, 1.0], [[0.01, 0.0], [0.0, 0.01]]);
        assert!(ekf.covariance[0][0] < before);
    }

    #[test]
    fn precise_repeated_position_updates_converge_close_to_measurement() {
        let mut ekf = Ekf2D::new([0.0, 0.0, 0.0], matrix3::identity());
        for _ in 0..20 {
            ekf.update_position([3.0, 4.0], [[1e-4, 0.0], [0.0, 1e-4]]);
        }
        assert!((ekf.state[0] - 3.0).abs() < 0.05);
        assert!((ekf.state[1] - 4.0).abs() < 0.05);
    }

    #[test]
    fn pose_update_wraps_heading_innovation_across_the_pi_boundary() {
        let mut ekf = Ekf2D::new([0.0, 0.0, PI - 0.05], matrix3::identity());
        // Measured heading just past -pi, which is very close to the
        // estimate's PI - 0.05 going the "short way" around.
        ekf.update_pose([0.0, 0.0, -PI + 0.05], &small_noise());
        // The corrected heading should stay near +/- pi, not jump toward 0.
        assert!(ekf.state[2].abs() > PI / 2.0);
    }

    #[test]
    fn pose_update_converges_with_repeated_precise_measurements() {
        let mut ekf = Ekf2D::new([0.0, 0.0, 0.0], matrix3::identity());
        let target: Vec3f = [2.0, -1.0, 0.3];
        for _ in 0..20 {
            ekf.update_pose(
                target,
                &[[1e-4, 0.0, 0.0], [0.0, 1e-4, 0.0], [0.0, 0.0, 1e-5]],
            );
        }
        assert!((ekf.state[0] - target[0]).abs() < 0.05);
        assert!((ekf.state[1] - target[1]).abs() < 0.05);
        assert!((ekf.state[2] - target[2]).abs() < 0.05);
    }
}
