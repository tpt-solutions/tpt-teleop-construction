// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Truck arrival prediction from GPS position/speed (spec.txt §4.6): a
//! straight-line distance-over-speed estimate, with a floor on the speed
//! used so a truck that's currently stopped (traffic, a queued dump
//! point ahead) doesn't project an infinite ETA — it's still approaching,
//! just slower than its cruising speed right now.

/// Predicted time (s) for a truck to cover `distance_m` at
/// `current_speed_m_s`, using at least `min_speed_m_s` in the
/// calculation so a momentarily stopped or crawling truck still yields a
/// finite, useful estimate rather than approaching infinity.
pub fn predicted_arrival_s(distance_m: f32, current_speed_m_s: f32, min_speed_m_s: f32) -> f32 {
    let speed = current_speed_m_s.max(min_speed_m_s).max(f32::EPSILON);
    distance_m.max(0.0) / speed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_simple_distance_over_speed() {
        assert!((predicted_arrival_s(100.0, 10.0, 1.0) - 10.0).abs() < 1e-4);
    }

    #[test]
    fn stopped_truck_uses_the_minimum_speed_floor() {
        let eta = predicted_arrival_s(100.0, 0.0, 2.0);
        assert!((eta - 50.0).abs() < 1e-4);
    }

    #[test]
    fn zero_distance_is_zero_eta() {
        assert_eq!(predicted_arrival_s(0.0, 10.0, 1.0), 0.0);
    }

    #[test]
    fn negative_distance_is_clamped_to_zero() {
        assert_eq!(predicted_arrival_s(-50.0, 10.0, 1.0), 0.0);
    }
}
