// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Material pushing to stockpiles, hoppers, or crushers (spec.txt §5.3):
//! a straight push path from the current blade load's position to the
//! target dump location, plus a blade-load check so the pushing logic
//! knows when it's carrying more material than the blade can hold without
//! spilling over the top or around the sides.

/// A straight push from `start` to `target`, both in the machine's local
/// 2D frame, meters.
#[derive(Debug, Clone, Copy)]
pub struct PushPlan {
    pub start: (f32, f32),
    pub target: (f32, f32),
}

impl PushPlan {
    pub fn distance_m(&self) -> f32 {
        let dx = self.target.0 - self.start.0;
        let dy = self.target.1 - self.start.1;
        (dx * dx + dy * dy).sqrt()
    }

    /// Heading (radians) from `start` toward `target`.
    pub fn heading_rad(&self) -> f32 {
        (self.target.1 - self.start.1).atan2(self.target.0 - self.start.0)
    }

    /// The position at `fraction` of the way from `start` (`0.0`) to
    /// `target` (`1.0`); values outside `[0, 1]` extrapolate rather than
    /// clamp, since a caller checking "one step past the target" is a
    /// valid use (e.g. overshoot detection).
    pub fn position_at(&self, fraction: f32) -> (f32, f32) {
        (
            self.start.0 + (self.target.0 - self.start.0) * fraction,
            self.start.1 + (self.target.1 - self.start.1) * fraction,
        )
    }
}

/// Fraction of blade capacity currently carried; `> 1.0` indicates the
/// blade is overloaded and material is likely spilling over the top or
/// around the ends rather than being pushed forward.
pub fn blade_load_fraction(carried_volume_m3: f32, blade_capacity_m3: f32) -> f32 {
    if blade_capacity_m3 <= 0.0 {
        0.0
    } else {
        carried_volume_m3 / blade_capacity_m3
    }
}

/// Whether the blade is carrying more material than its rated capacity.
pub fn is_overloaded(carried_volume_m3: f32, blade_capacity_m3: f32) -> bool {
    blade_load_fraction(carried_volume_m3, blade_capacity_m3) > 1.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    #[test]
    fn distance_matches_straight_line_length() {
        let plan = PushPlan {
            start: (0.0, 0.0),
            target: (3.0, 4.0),
        };
        assert!((plan.distance_m() - 5.0).abs() < 1e-4);
    }

    #[test]
    fn heading_points_from_start_toward_target() {
        let plan = PushPlan {
            start: (0.0, 0.0),
            target: (0.0, 10.0),
        };
        assert!((plan.heading_rad() - FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn position_at_interpolates_along_the_push() {
        let plan = PushPlan {
            start: (0.0, 0.0),
            target: (10.0, 0.0),
        };
        assert_eq!(plan.position_at(0.0), (0.0, 0.0));
        assert_eq!(plan.position_at(0.5), (5.0, 0.0));
        assert_eq!(plan.position_at(1.0), (10.0, 0.0));
    }

    #[test]
    fn load_fraction_and_overload_check_agree() {
        assert!(!is_overloaded(8.0, 10.0));
        assert!(is_overloaded(12.0, 10.0));
        assert!((blade_load_fraction(5.0, 10.0) - 0.5).abs() < 1e-4);
    }

    #[test]
    fn zero_capacity_reports_zero_fraction_not_a_division_error() {
        assert_eq!(blade_load_fraction(5.0, 0.0), 0.0);
    }
}
