// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Steering pattern analysis (spec.txt §7): an alert operator makes
//! frequent small corrective steering inputs; a drowsy or distracted one
//! makes far fewer, drifting instead of correcting. Steering reversal
//! rate (direction changes per minute) is a standard proxy for this in
//! driver-monitoring research — this tracks it from a stream of steering
//! angle samples.

/// Tracks steering direction reversals over time to estimate activity
/// level.
#[derive(Debug, Clone, Copy)]
pub struct SteeringActivityMonitor {
    reversal_count: u32,
    elapsed_s: f32,
    last_angle_rad: f32,
    last_direction: i8,
    /// Angle changes smaller than this are treated as noise, not a
    /// deliberate steering input, so sensor jitter doesn't get counted as
    /// "activity".
    deadband_rad: f32,
}

impl SteeringActivityMonitor {
    pub fn new(deadband_rad: f32) -> Self {
        SteeringActivityMonitor {
            reversal_count: 0,
            elapsed_s: 0.0,
            last_angle_rad: 0.0,
            last_direction: 0,
            deadband_rad,
        }
    }

    /// Feeds in the current steering angle (radians) after `dt_s` has
    /// elapsed since the last update, counting a reversal whenever the
    /// direction of travel changes.
    pub fn update(&mut self, steering_angle_rad: f32, dt_s: f32) {
        self.elapsed_s += dt_s;
        let delta = steering_angle_rad - self.last_angle_rad;
        let direction: i8 = if delta > self.deadband_rad {
            1
        } else if delta < -self.deadband_rad {
            -1
        } else {
            0
        };

        if direction != 0 {
            if self.last_direction != 0 && direction != self.last_direction {
                self.reversal_count += 1;
            }
            self.last_direction = direction;
        }
        self.last_angle_rad = steering_angle_rad;
    }

    pub fn reversal_count(&self) -> u32 {
        self.reversal_count
    }

    /// Reversals per minute over the monitor's whole observation period
    /// so far; `0.0` before any time has elapsed.
    pub fn reversals_per_minute(&self) -> f32 {
        if self.elapsed_s <= 0.0 {
            0.0
        } else {
            self.reversal_count as f32 / self.elapsed_s * 60.0
        }
    }

    /// Whether observed steering activity is low enough to suggest
    /// fatigue/distraction: fewer reversals per minute than
    /// `min_reversals_per_minute`, evaluated only once at least a minute
    /// of data has accumulated (a rate estimate from a few seconds is too
    /// noisy to act on).
    pub fn indicates_fatigue(&self, min_reversals_per_minute: f32) -> bool {
        self.elapsed_s >= 60.0 && self.reversals_per_minute() < min_reversals_per_minute
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_updates_reports_zero_rate() {
        let monitor = SteeringActivityMonitor::new(0.01);
        assert_eq!(monitor.reversals_per_minute(), 0.0);
        assert!(!monitor.indicates_fatigue(5.0));
    }

    #[test]
    fn oscillating_steering_counts_reversals() {
        let mut monitor = SteeringActivityMonitor::new(0.01);
        let pattern = [0.1, 0.2, 0.1, 0.2, 0.1, 0.2];
        for angle in pattern {
            monitor.update(angle, 1.0);
        }
        assert!(monitor.reversal_count() >= 4);
    }

    #[test]
    fn tiny_jitter_within_the_deadband_is_not_counted() {
        let mut monitor = SteeringActivityMonitor::new(0.05);
        for angle in [0.0, 0.001, -0.001, 0.002, -0.002] {
            monitor.update(angle, 1.0);
        }
        assert_eq!(monitor.reversal_count(), 0);
    }

    #[test]
    fn monotonic_steering_has_no_reversals() {
        let mut monitor = SteeringActivityMonitor::new(0.01);
        for angle in [0.0, 0.1, 0.2, 0.3, 0.4] {
            monitor.update(angle, 1.0);
        }
        assert_eq!(monitor.reversal_count(), 0);
    }

    #[test]
    fn low_activity_over_a_full_minute_indicates_fatigue() {
        let mut monitor = SteeringActivityMonitor::new(0.01);
        // Perfectly steady steering for a full minute: zero reversals.
        for _ in 0..60 {
            monitor.update(0.3, 1.0);
        }
        assert!(monitor.indicates_fatigue(5.0));
    }

    #[test]
    fn active_steering_over_a_full_minute_does_not_indicate_fatigue() {
        let mut monitor = SteeringActivityMonitor::new(0.01);
        for i in 0..120 {
            let angle = if i % 2 == 0 { 0.1 } else { 0.2 };
            monitor.update(angle, 0.5);
        }
        assert!(!monitor.indicates_fatigue(5.0));
    }

    #[test]
    fn fatigue_check_ignored_before_a_full_minute_of_data() {
        let mut monitor = SteeringActivityMonitor::new(0.01);
        for _ in 0..10 {
            monitor.update(0.3, 1.0); // 10 seconds of steady steering
        }
        assert!(!monitor.indicates_fatigue(5.0));
    }
}
