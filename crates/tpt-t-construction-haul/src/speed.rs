// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Speed management: automatic reduction on downgrades, curves, and
//! low-visibility sections (spec.txt §5.2). Each limit is computed
//! independently from first principles (grade-based brake derating,
//! friction-limited curve speed, stopping-distance-limited visibility
//! speed); the truck's actual commanded speed is always the tightest
//! (lowest) of them, the same "lowest limit wins" rule used throughout
//! this workspace.

/// Tunable parameters for the physics-based speed limits below.
#[derive(Debug, Clone, Copy)]
pub struct SpeedLimitParams {
    /// Maximum lateral acceleration (m/s^2) allowed through a curve
    /// before it's treated as unsafe — a fraction of the tire/track-soil
    /// friction limit, not the absolute traction ceiling.
    pub max_lateral_accel_m_s2: f32,
    /// Perception + decision reaction time (s) budgeted before braking
    /// begins, once a hazard within the visible distance is detected.
    pub reaction_time_s: f32,
    /// Maximum braking deceleration (m/s^2) the truck can reliably
    /// achieve (loaded, on the expected surface).
    pub max_decel_m_s2: f32,
}

/// Reduces `base_limit_m_s` on a downgrade (`grade_percent < 0`) to manage
/// brake heat and control on a loaded descent; grade is unrestricted
/// (returns `base_limit_m_s` unchanged) on flat ground or an upgrade.
pub fn downgrade_speed_limit_m_s(base_limit_m_s: f32, grade_percent: f32) -> f32 {
    if grade_percent >= 0.0 {
        return base_limit_m_s;
    }
    let steepness = -grade_percent;
    let derate = (steepness * 0.03).min(0.6);
    base_limit_m_s * (1.0 - derate)
}

/// The friction-limited maximum speed through a curve of `radius_m`,
/// from `v = sqrt(a_lat_max * r)`.
pub fn curve_speed_limit_m_s(radius_m: f32, max_lateral_accel_m_s2: f32) -> f32 {
    (max_lateral_accel_m_s2 * radius_m.max(0.0)).sqrt()
}

/// The maximum speed at which the truck can still stop within
/// `visibility_m`, accounting for `reaction_time_s` of travel before
/// braking begins plus `v^2 / (2 * max_decel_m_s2)` of braking distance.
/// Solves `visibility = v*reaction_time + v^2/(2*decel)` for `v`.
pub fn visibility_speed_limit_m_s(
    visibility_m: f32,
    reaction_time_s: f32,
    max_decel_m_s2: f32,
) -> f32 {
    if visibility_m <= 0.0 {
        return 0.0;
    }
    let a = 1.0 / (2.0 * max_decel_m_s2);
    let b = reaction_time_s;
    let c = -visibility_m;
    let discriminant = b * b - 4.0 * a * c;
    (-b + discriminant.sqrt()) / (2.0 * a)
}

/// Combines the base road-section limit with grade, curve, and
/// visibility limits (each optional — a straight, unobstructed section
/// has no curve or visibility constraint to apply), returning the lowest.
pub fn combined_speed_limit_m_s(
    base_limit_m_s: f32,
    grade_percent: f32,
    curve_radius_m: Option<f32>,
    visibility_m: Option<f32>,
    params: &SpeedLimitParams,
) -> f32 {
    let mut limit = downgrade_speed_limit_m_s(base_limit_m_s, grade_percent);
    if let Some(radius_m) = curve_radius_m {
        limit = limit.min(curve_speed_limit_m_s(
            radius_m,
            params.max_lateral_accel_m_s2,
        ));
    }
    if let Some(visibility_m) = visibility_m {
        limit = limit.min(visibility_speed_limit_m_s(
            visibility_m,
            params.reaction_time_s,
            params.max_decel_m_s2,
        ));
    }
    limit
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_or_upgrade_does_not_reduce_speed() {
        assert_eq!(downgrade_speed_limit_m_s(15.0, 0.0), 15.0);
        assert_eq!(downgrade_speed_limit_m_s(15.0, 5.0), 15.0);
    }

    #[test]
    fn steeper_downgrade_reduces_speed_more() {
        let gentle = downgrade_speed_limit_m_s(15.0, -3.0);
        let steep = downgrade_speed_limit_m_s(15.0, -10.0);
        assert!(steep < gentle);
        assert!(gentle < 15.0);
    }

    #[test]
    fn tighter_curves_require_lower_speed() {
        let tight = curve_speed_limit_m_s(20.0, 3.0);
        let wide = curve_speed_limit_m_s(200.0, 3.0);
        assert!(tight < wide);
    }

    #[test]
    fn zero_radius_curve_has_zero_speed_limit() {
        assert_eq!(curve_speed_limit_m_s(0.0, 3.0), 0.0);
    }

    #[test]
    fn stopping_distance_at_the_computed_visibility_limit_matches_visibility() {
        let visibility = 80.0;
        let reaction_time = 1.5;
        let decel = 3.0;
        let v = visibility_speed_limit_m_s(visibility, reaction_time, decel);
        let stopping_distance = v * reaction_time + v * v / (2.0 * decel);
        assert!((stopping_distance - visibility).abs() < 1e-2);
    }

    #[test]
    fn zero_visibility_means_zero_speed() {
        assert_eq!(visibility_speed_limit_m_s(0.0, 1.5, 3.0), 0.0);
    }

    #[test]
    fn combined_limit_is_the_tightest_of_every_applicable_constraint() {
        let params = SpeedLimitParams {
            max_lateral_accel_m_s2: 2.0,
            reaction_time_s: 1.5,
            max_decel_m_s2: 3.0,
        };
        let limit = combined_speed_limit_m_s(20.0, -8.0, Some(15.0), Some(30.0), &params);
        assert!(limit <= downgrade_speed_limit_m_s(20.0, -8.0));
        assert!(limit <= curve_speed_limit_m_s(15.0, params.max_lateral_accel_m_s2));
        assert!(
            limit
                <= visibility_speed_limit_m_s(30.0, params.reaction_time_s, params.max_decel_m_s2)
        );
    }

    #[test]
    fn no_curve_or_visibility_constraint_leaves_only_grade_limit() {
        let params = SpeedLimitParams {
            max_lateral_accel_m_s2: 2.0,
            reaction_time_s: 1.5,
            max_decel_m_s2: 3.0,
        };
        let limit = combined_speed_limit_m_s(20.0, 0.0, None, None, &params);
        assert_eq!(limit, 20.0);
    }
}
