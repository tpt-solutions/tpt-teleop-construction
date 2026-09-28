// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Cutter head torque management (spec.txt §6): derates the desired
//! advance rate whenever the torque that rate would demand exceeds the
//! cutter head drive's rated maximum, rather than letting the drive
//! stall or trip an overcurrent protection.

/// The advance rate (m/s) actually commanded after torque limiting:
/// `desired_rate_m_s` unchanged if `predicted_torque_nm` is within
/// `max_torque_nm`, otherwise scaled down proportionally so the
/// (approximately torque-proportional-to-rate) demand lands back at the
/// limit.
pub fn torque_limited_advance_rate_m_s(
    desired_rate_m_s: f32,
    predicted_torque_nm: f32,
    max_torque_nm: f32,
) -> f32 {
    if predicted_torque_nm <= max_torque_nm || predicted_torque_nm <= 0.0 {
        desired_rate_m_s
    } else {
        desired_rate_m_s * (max_torque_nm / predicted_torque_nm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn within_torque_limit_leaves_rate_unchanged() {
        assert_eq!(
            torque_limited_advance_rate_m_s(0.05, 8_000.0, 10_000.0),
            0.05
        );
    }

    #[test]
    fn exceeding_torque_limit_scales_rate_down_proportionally() {
        let limited = torque_limited_advance_rate_m_s(0.05, 20_000.0, 10_000.0);
        assert!((limited - 0.025).abs() < 1e-6);
    }

    #[test]
    fn exactly_at_the_limit_leaves_rate_unchanged() {
        assert_eq!(
            torque_limited_advance_rate_m_s(0.05, 10_000.0, 10_000.0),
            0.05
        );
    }

    #[test]
    fn zero_or_negative_predicted_torque_leaves_rate_unchanged() {
        assert_eq!(torque_limited_advance_rate_m_s(0.05, 0.0, 10_000.0), 0.05);
    }
}
