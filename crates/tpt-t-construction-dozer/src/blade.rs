// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Slope grading via real-time blade position feedback (spec.txt §5.3):
//! a dozer blade has independent left and right corners, so maintaining a
//! designed slope (e.g. 3:1) means computing each corner's own target
//! height from the design plane and its own error — a pure left/right
//! lift correction, distinct from `tpt-t-construction-excavator`'s single
//! bucket-tip grade error.

/// A planar design surface: a reference point and a slope (rise per unit
/// run) in each horizontal direction, the same convention as
/// `tpt-t-construction-excavator::GradePlane`.
#[derive(Debug, Clone, Copy)]
pub struct GradePlane {
    pub reference_x: f32,
    pub reference_y: f32,
    pub reference_z: f32,
    pub slope_x: f32,
    pub slope_y: f32,
}

impl GradePlane {
    pub fn design_height_m(&self, x: f32, y: f32) -> f32 {
        self.reference_z
            + (x - self.reference_x) * self.slope_x
            + (y - self.reference_y) * self.slope_y
    }
}

/// The dozer's current position, heading, and blade geometry/state.
#[derive(Debug, Clone, Copy)]
pub struct BladeState {
    pub center_x: f32,
    pub center_y: f32,
    /// Heading, radians, machine-forward direction (0 = +x axis).
    pub heading_rad: f32,
    pub blade_width_m: f32,
    pub left_height_m: f32,
    pub right_height_m: f32,
}

/// Design height each blade corner should be at to match the grade plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BladeCornerTargets {
    pub left_target_m: f32,
    pub right_target_m: f32,
}

fn corner_positions(blade: &BladeState) -> ((f32, f32), (f32, f32)) {
    let half_width = blade.blade_width_m / 2.0;
    // Perpendicular to heading: rotate the forward vector 90 degrees.
    let perp_x = -blade.heading_rad.sin();
    let perp_y = blade.heading_rad.cos();
    let left = (
        blade.center_x + perp_x * half_width,
        blade.center_y + perp_y * half_width,
    );
    let right = (
        blade.center_x - perp_x * half_width,
        blade.center_y - perp_y * half_width,
    );
    (left, right)
}

/// The design height at each blade corner's current position.
pub fn compute_corner_targets(plane: &GradePlane, blade: &BladeState) -> BladeCornerTargets {
    let (left, right) = corner_positions(blade);
    BladeCornerTargets {
        left_target_m: plane.design_height_m(left.0, left.1),
        right_target_m: plane.design_height_m(right.0, right.1),
    }
}

/// `(left_error_m, right_error_m)`: each corner's actual height minus its
/// design target. Positive means that corner is too high (needs cutting);
/// negative means too low (needs fill) — same sign convention as
/// `tpt-t-construction-excavator::GradePlane::grade_error_m`.
pub fn corner_errors_m(plane: &GradePlane, blade: &BladeState) -> (f32, f32) {
    let targets = compute_corner_targets(plane, blade);
    (
        blade.left_height_m - targets.left_target_m,
        blade.right_height_m - targets.right_target_m,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_plane() -> GradePlane {
        GradePlane {
            reference_x: 0.0,
            reference_y: 0.0,
            reference_z: 10.0,
            slope_x: 0.0,
            slope_y: 0.0,
        }
    }

    #[test]
    fn flat_plane_gives_equal_corner_targets() {
        let blade = BladeState {
            center_x: 5.0,
            center_y: 5.0,
            heading_rad: 0.0,
            blade_width_m: 4.0,
            left_height_m: 10.0,
            right_height_m: 10.0,
        };
        let targets = compute_corner_targets(&flat_plane(), &blade);
        assert!((targets.left_target_m - 10.0).abs() < 1e-4);
        assert!((targets.right_target_m - 10.0).abs() < 1e-4);
    }

    #[test]
    fn cross_slope_gives_different_corner_targets_when_traveling_along_it() {
        // Slope rises in y; heading along +x means the blade's
        // left/right corners sit at different y, and so at different
        // design heights.
        let plane = GradePlane {
            reference_x: 0.0,
            reference_y: 0.0,
            reference_z: 10.0,
            slope_x: 0.0,
            slope_y: 1.0 / 3.0,
        };
        let blade = BladeState {
            center_x: 0.0,
            center_y: 0.0,
            heading_rad: 0.0,
            blade_width_m: 4.0,
            left_height_m: 10.0,
            right_height_m: 10.0,
        };
        let targets = compute_corner_targets(&plane, &blade);
        assert!((targets.left_target_m - targets.right_target_m).abs() > 1.0);
    }

    #[test]
    fn corner_errors_are_zero_when_blade_matches_design() {
        let plane = flat_plane();
        let blade = BladeState {
            center_x: 0.0,
            center_y: 0.0,
            heading_rad: 0.5,
            blade_width_m: 4.0,
            left_height_m: 10.0,
            right_height_m: 10.0,
        };
        let (left_err, right_err) = corner_errors_m(&plane, &blade);
        assert!(left_err.abs() < 1e-4);
        assert!(right_err.abs() < 1e-4);
    }

    #[test]
    fn corner_error_signals_cut_vs_fill_per_corner() {
        let plane = flat_plane();
        let blade = BladeState {
            center_x: 0.0,
            center_y: 0.0,
            heading_rad: 0.0,
            blade_width_m: 4.0,
            left_height_m: 10.5, // too high: cut
            right_height_m: 9.5, // too low: fill
        };
        let (left_err, right_err) = corner_errors_m(&plane, &blade);
        assert!(left_err > 0.0);
        assert!(right_err < 0.0);
    }
}
