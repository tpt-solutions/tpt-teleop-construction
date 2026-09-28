// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Real-time tilt angle estimation (spec.txt §7): a gyro alone drifts
//! over time, and an accelerometer alone is corrupted by every bump and
//! vibration spike (spec.txt's "10g shaking from diesel engines" —
//! exactly the noise `tpt-t-construction-hydraulic`'s vibration filter
//! exists to reject on the position-sensing side); the standard answer is
//! a complementary filter that trusts the gyro's short-term rate of
//! change and the accelerometer's long-term gravity-vector reference,
//! blended by a single time-constant parameter.

/// Roll angle (radians) from an accelerometer reading alone, via the
/// gravity vector's projection onto the y-z plane. Only meaningful when
/// the machine isn't significantly accelerating (gravity dominates the
/// reading) — see [`ComplementaryTiltFilter`] for the fused estimate that
/// works during motion too.
pub fn roll_from_accelerometer(ay: f32, az: f32) -> f32 {
    ay.atan2(az)
}

/// Pitch angle (radians) from an accelerometer reading alone.
pub fn pitch_from_accelerometer(ax: f32, ay: f32, az: f32) -> f32 {
    (-ax).atan2((ay * ay + az * az).sqrt())
}

/// A complementary filter fusing gyro angular rate (fast, drifts) with
/// accelerometer-derived tilt (slow, noisy) into one real-time roll/pitch
/// estimate.
#[derive(Debug, Clone, Copy)]
pub struct ComplementaryTiltFilter {
    pub roll_rad: f32,
    pub pitch_rad: f32,
    /// Blend factor in `(0, 1)`: closer to `1` trusts the gyro (and
    /// drifts slower), closer to `0` trusts the accelerometer (and
    /// rejects vibration less).
    alpha: f32,
}

impl ComplementaryTiltFilter {
    pub fn new(alpha: f32) -> Self {
        assert!((0.0..1.0).contains(&alpha), "alpha must be in (0, 1)");
        ComplementaryTiltFilter {
            roll_rad: 0.0,
            pitch_rad: 0.0,
            alpha,
        }
    }

    /// Advances the filter by `dt_s` given the gyro's roll/pitch rates
    /// (rad/s) and the accelerometer reading `[ax, ay, az]` (m/s^2).
    pub fn update(
        &mut self,
        gyro_roll_rate_rad_s: f32,
        gyro_pitch_rate_rad_s: f32,
        accel: [f32; 3],
        dt_s: f32,
    ) {
        let accel_roll = roll_from_accelerometer(accel[1], accel[2]);
        let accel_pitch = pitch_from_accelerometer(accel[0], accel[1], accel[2]);

        self.roll_rad = self.alpha * (self.roll_rad + gyro_roll_rate_rad_s * dt_s)
            + (1.0 - self.alpha) * accel_roll;
        self.pitch_rad = self.alpha * (self.pitch_rad + gyro_pitch_rate_rad_s * dt_s)
            + (1.0 - self.alpha) * accel_pitch;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() <= eps
    }

    #[test]
    fn level_accelerometer_reads_zero_tilt() {
        assert!(approx_eq(roll_from_accelerometer(0.0, 9.81), 0.0, 1e-4));
        assert!(approx_eq(
            pitch_from_accelerometer(0.0, 0.0, 9.81),
            0.0,
            1e-4
        ));
    }

    #[test]
    fn accelerometer_on_its_side_reads_90_degrees_roll() {
        assert!(approx_eq(
            roll_from_accelerometer(9.81, 0.0),
            FRAC_PI_2,
            1e-3
        ));
    }

    #[test]
    fn filter_tracks_a_steady_gyro_rotation_with_a_level_accelerometer() {
        // Rotating at a constant rate with an accelerometer that always
        // reads "level" (unrealistic in isolation, but isolates the
        // gyro-integration path) should closely follow gyro*t after a
        // reasonably high alpha's contribution keeps the accel term from
        // dominating over many samples.
        let mut filter = ComplementaryTiltFilter::new(0.98);
        let rate = 0.2_f32;
        let dt = 0.01;
        for _ in 0..100 {
            filter.update(rate, 0.0, [0.0, 0.0, 9.81], dt);
        }
        // Pure integration would give rate*1.0s = 0.2 rad; the 2% accel
        // pull toward zero each step means it settles a bit under that.
        assert!(filter.roll_rad > 0.05 && filter.roll_rad < 0.2);
    }

    #[test]
    fn filter_converges_to_a_sustained_tilt_when_stationary() {
        let mut filter = ComplementaryTiltFilter::new(0.9);
        let tilted_accel = [0.0, 9.81 * FRAC_PI_2.sin(), 9.81 * FRAC_PI_2.cos()];
        for _ in 0..500 {
            filter.update(0.0, 0.0, tilted_accel, 0.01);
        }
        assert!(approx_eq(filter.roll_rad, FRAC_PI_2, 0.05));
    }

    #[test]
    #[should_panic(expected = "alpha must be in")]
    fn alpha_out_of_range_panics() {
        ComplementaryTiltFilter::new(1.5);
    }
}
