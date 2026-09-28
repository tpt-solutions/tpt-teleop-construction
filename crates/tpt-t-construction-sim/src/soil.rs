// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Tire/track-soil interaction and rollover stability physics.
//!
//! The traction model is a simplified Coulomb-friction model with slip
//! saturation (not a full terramechanics/Bekker model): enough to give
//! `tpt-t-construction-sim` a plausible acceleration/braking envelope per
//! soil type without pulling in a dedicated terramechanics dependency.
//! Rollover uses the standard static stability factor used for real
//! off-highway vehicles: `SSF = track_width / (2 * CoG_height)`.

/// Soil/surface classification, each with its own peak friction
/// coefficient against a steel track or rubber tire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SoilType {
    Dirt,
    Mud,
    Rock,
    Gravel,
}

impl SoilType {
    /// Peak (unsaturated-slip) friction coefficient. Mud is deliberately
    /// the lowest: it's the classic case where a machine can spin its
    /// tracks and get no traction, which the slip-saturation curve below
    /// is meant to reproduce.
    pub fn peak_friction_coeff(self) -> f32 {
        match self {
            SoilType::Dirt => 0.6,
            SoilType::Mud => 0.35,
            SoilType::Rock => 0.8,
            SoilType::Gravel => 0.5,
        }
    }
}

/// Slip ratio magnitude (0 = pure rolling, 1 = full spin/lockup) at which
/// friction utilization saturates. Beyond this, further slip buys no more
/// traction (indeed real tires lose grip past their peak-slip point, but
/// modeling that falloff isn't needed for this sim's fidelity target).
const SLIP_SATURATION: f32 = 0.2;

/// Longitudinal slip ratio between a contact patch's surface speed and the
/// vehicle's ground speed at that patch: `0` is pure rolling, `+1` is full
/// wheel spin under power, `-1` is full lockup under braking.
pub fn slip_ratio(wheel_surface_speed_m_s: f32, ground_speed_m_s: f32) -> f32 {
    let denom = wheel_surface_speed_m_s
        .abs()
        .max(ground_speed_m_s.abs())
        .max(0.01);
    (wheel_surface_speed_m_s - ground_speed_m_s) / denom
}

/// Longitudinal traction force (N) a contact patch can deliver, given its
/// normal load, the soil it's on, and its current slip ratio. Positive
/// slip (wheel spin) yields positive (driving) force; negative slip
/// (lockup) yields negative (braking) force.
pub fn traction_force_n(normal_load_n: f32, slip_ratio: f32, soil: SoilType) -> f32 {
    let utilization = (slip_ratio.abs() / SLIP_SATURATION).min(1.0);
    soil.peak_friction_coeff() * normal_load_n.max(0.0) * utilization * slip_ratio.signum()
}

/// The static stability factor of a vehicle: half its track/gauge width
/// divided by its center-of-gravity height. This is the lateral
/// acceleration, in g, at which a rigid vehicle's rollover moment equals
/// its restoring moment — the standard NHTSA-style rollover metric for
/// off-highway equipment.
pub fn static_stability_factor(track_width_m: f32, cg_height_m: f32) -> f32 {
    (track_width_m * 0.5) / cg_height_m
}

/// The roll angle (radians, magnitude) at which a stationary vehicle on a
/// slope would tip, for the same track width / CoG height geometry.
pub fn critical_static_roll_angle_rad(track_width_m: f32, cg_height_m: f32) -> f32 {
    static_stability_factor(track_width_m, cg_height_m).atan()
}

/// Whether the current combination of lateral acceleration (in g) and
/// static roll/tilt angle (radians) has crossed the rollover threshold,
/// with `margin_g` subtracted from the theoretical limit as a safety
/// margin for `tpt-t-construction-rollover` to trigger braking before the
/// machine actually reaches the tipping point (Phase 7).
pub fn rollover_imminent(
    lateral_accel_g: f32,
    roll_angle_rad: f32,
    track_width_m: f32,
    cg_height_m: f32,
    margin_g: f32,
) -> bool {
    let ssf = static_stability_factor(track_width_m, cg_height_m);
    // The slope's own tilt reduces the lateral acceleration margin
    // available before tipping in that direction.
    let effective_limit = (ssf - roll_angle_rad.abs().tan()).max(0.0);
    lateral_accel_g.abs() >= (effective_limit - margin_g).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_rolling_has_zero_slip_and_zero_traction() {
        assert_eq!(slip_ratio(5.0, 5.0), 0.0);
        assert_eq!(traction_force_n(10_000.0, 0.0, SoilType::Dirt), 0.0);
    }

    #[test]
    fn wheel_spin_yields_positive_traction_up_to_saturation() {
        let full_spin = slip_ratio(10.0, 0.0); // wheel moving, vehicle stationary
        assert_eq!(full_spin, 1.0);
        let force_at_saturation = traction_force_n(10_000.0, SLIP_SATURATION, SoilType::Dirt);
        let force_beyond_saturation = traction_force_n(10_000.0, 1.0, SoilType::Dirt);
        assert_eq!(force_at_saturation, force_beyond_saturation);
        assert!(force_at_saturation > 0.0);
    }

    #[test]
    fn braking_slip_yields_negative_force() {
        let slip = slip_ratio(0.0, 10.0); // wheel locked, vehicle still moving
        assert!(slip < 0.0);
        assert!(traction_force_n(10_000.0, slip, SoilType::Dirt) < 0.0);
    }

    #[test]
    fn mud_gives_less_traction_than_rock_at_equal_slip() {
        let mud = traction_force_n(10_000.0, 0.1, SoilType::Mud);
        let rock = traction_force_n(10_000.0, 0.1, SoilType::Rock);
        assert!(mud < rock);
    }

    #[test]
    fn wider_track_and_lower_cg_are_more_stable() {
        let narrow_tall = static_stability_factor(2.0, 2.0);
        let wide_low = static_stability_factor(4.0, 1.0);
        assert!(wide_low > narrow_tall);
    }

    #[test]
    fn flat_ground_rollover_matches_ssf_within_margin() {
        let track_width = 3.0;
        let cg_height = 1.5;
        let ssf = static_stability_factor(track_width, cg_height); // 1.0 g
        assert!(!rollover_imminent(
            ssf - 0.2,
            0.0,
            track_width,
            cg_height,
            0.05
        ));
        assert!(rollover_imminent(
            ssf - 0.01,
            0.0,
            track_width,
            cg_height,
            0.05
        ));
    }

    #[test]
    fn tilted_ground_lowers_the_rollover_threshold() {
        let track_width = 3.0;
        let cg_height = 1.5;
        let lateral_accel = static_stability_factor(track_width, cg_height) - 0.1;
        assert!(!rollover_imminent(
            lateral_accel,
            0.0,
            track_width,
            cg_height,
            0.0
        ));
        // The same lateral acceleration on a machine already tilted toward
        // the same side is closer to tipping.
        assert!(rollover_imminent(
            lateral_accel,
            critical_static_roll_angle_rad(track_width, cg_height) * 0.9,
            track_width,
            cg_height,
            0.0
        ));
    }
}
