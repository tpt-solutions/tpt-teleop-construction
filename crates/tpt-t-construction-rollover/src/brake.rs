// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Automatic brake application on rollover threshold (spec.txt §7): the
//! same static-stability-factor geometry `tpt-t-construction-sim::soil`
//! uses to model rollover in the physics plant, implemented here as the
//! real-time monitor deciding whether to intervene, fed by
//! [`crate::tilt_estimation`]'s tilt estimate and
//! [`crate::cog_estimation`]'s effective CoG height rather than
//! simulation ground truth.

/// The static stability factor: half the track/gauge width divided by
/// the center-of-gravity height. This is the roll angle's tangent at
/// which a rigid vehicle's rollover moment equals its restoring moment.
pub fn static_stability_factor(track_width_m: f32, cog_height_m: f32) -> f32 {
    (track_width_m * 0.5) / cog_height_m
}

/// The critical roll angle (radians, magnitude) at which the machine
/// would tip, for the given track width and CoG height.
pub fn critical_roll_angle_rad(track_width_m: f32, cog_height_m: f32) -> f32 {
    static_stability_factor(track_width_m, cog_height_m).atan()
}

/// Remaining margin (radians) before the critical tip angle: positive
/// means still stable, zero or negative means at or past the threshold.
pub fn rollover_margin_rad(roll_angle_rad: f32, track_width_m: f32, cog_height_m: f32) -> f32 {
    critical_roll_angle_rad(track_width_m, cog_height_m) - roll_angle_rad.abs()
}

/// Whether the automatic brakes should apply now: the current roll angle
/// has closed to within `margin_rad` of the critical tip angle.
pub fn should_apply_brakes(
    roll_angle_rad: f32,
    track_width_m: f32,
    cog_height_m: f32,
    margin_rad: f32,
) -> bool {
    rollover_margin_rad(roll_angle_rad, track_width_m, cog_height_m) <= margin_rad
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wider_track_or_lower_cog_increases_stability() {
        let narrow_tall = static_stability_factor(2.0, 2.0);
        let wide_low = static_stability_factor(4.0, 1.0);
        assert!(wide_low > narrow_tall);
    }

    #[test]
    fn upright_machine_has_full_margin() {
        let margin = rollover_margin_rad(0.0, 3.0, 1.5);
        assert!(margin > 0.0);
    }

    #[test]
    fn margin_shrinks_as_roll_angle_approaches_critical() {
        let track_width = 3.0;
        let cog_height = 1.5;
        let critical = critical_roll_angle_rad(track_width, cog_height);
        let far = rollover_margin_rad(critical * 0.2, track_width, cog_height);
        let near = rollover_margin_rad(critical * 0.9, track_width, cog_height);
        assert!(near < far);
        assert!(near > 0.0);
    }

    #[test]
    fn brakes_apply_once_within_margin_of_the_critical_angle() {
        let track_width = 3.0;
        let cog_height = 1.5;
        let critical = critical_roll_angle_rad(track_width, cog_height);
        assert!(!should_apply_brakes(
            critical * 0.5,
            track_width,
            cog_height,
            0.05
        ));
        assert!(should_apply_brakes(
            critical * 0.99,
            track_width,
            cog_height,
            0.05
        ));
    }

    #[test]
    fn brakes_apply_regardless_of_roll_direction() {
        let track_width = 3.0;
        let cog_height = 1.5;
        let critical = critical_roll_angle_rad(track_width, cog_height);
        assert!(should_apply_brakes(
            -critical * 0.99,
            track_width,
            cog_height,
            0.05
        ));
    }

    #[test]
    fn a_higher_cog_from_a_raised_payload_reduces_the_critical_angle() {
        let low_cog_critical = critical_roll_angle_rad(3.0, 1.5);
        let high_cog_critical = critical_roll_angle_rad(3.0, 3.0);
        assert!(high_cog_critical < low_cog_critical);
    }
}
