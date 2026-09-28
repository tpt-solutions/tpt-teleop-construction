// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Collar positioning via LiDAR/GNSS (spec.txt §5.5): checking a
//! positioning fix's own accuracy is good enough to trust before using it
//! to place the drill collar, and then checking the collar actually
//! landed within tolerance of the design position.

/// A position fix from either sensor (LiDAR-based local positioning or
/// GNSS), carrying its own estimated accuracy so the caller can judge
/// whether to trust it.
#[derive(Debug, Clone, Copy)]
pub struct PositionFix {
    pub x: f32,
    pub y: f32,
    pub accuracy_m: f32,
}

/// Distance (m) between a fix and the design collar position.
pub fn collar_error_m(target: (f32, f32), fix: &PositionFix) -> f32 {
    let dx = fix.x - target.0;
    let dy = fix.y - target.1;
    (dx * dx + dy * dy).sqrt()
}

/// Whether `fix` places the collar within `tolerance_m` of `target`.
pub fn is_within_tolerance(target: (f32, f32), fix: &PositionFix, tolerance_m: f32) -> bool {
    collar_error_m(target, fix) <= tolerance_m
}

/// Whether a fix's own estimated accuracy is good enough to be trusted
/// for collar positioning at all — a GNSS fix degraded by multipath or a
/// LiDAR fix taken in a feature-poor area might report a large
/// `accuracy_m` that should reject its use outright rather than accepting
/// its position estimate at face value.
pub fn is_fix_trustworthy(fix: &PositionFix, max_accuracy_m: f32) -> bool {
    fix.accuracy_m <= max_accuracy_m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_fix_has_zero_error() {
        let fix = PositionFix {
            x: 10.0,
            y: 10.0,
            accuracy_m: 0.02,
        };
        assert_eq!(collar_error_m((10.0, 10.0), &fix), 0.0);
    }

    #[test]
    fn error_matches_straight_line_distance() {
        let fix = PositionFix {
            x: 3.0,
            y: 4.0,
            accuracy_m: 0.02,
        };
        assert!((collar_error_m((0.0, 0.0), &fix) - 5.0).abs() < 1e-4);
    }

    #[test]
    fn tolerance_check_matches_error_comparison() {
        let fix = PositionFix {
            x: 0.05,
            y: 0.0,
            accuracy_m: 0.02,
        };
        assert!(is_within_tolerance((0.0, 0.0), &fix, 0.1));
        assert!(!is_within_tolerance((0.0, 0.0), &fix, 0.01));
    }

    #[test]
    fn trustworthiness_gates_on_the_fixs_own_reported_accuracy() {
        let good_fix = PositionFix {
            x: 0.0,
            y: 0.0,
            accuracy_m: 0.1,
        };
        let degraded_fix = PositionFix {
            x: 0.0,
            y: 0.0,
            accuracy_m: 5.0,
        };
        assert!(is_fix_trustworthy(&good_fix, 0.5));
        assert!(!is_fix_trustworthy(&degraded_fix, 0.5));
    }
}
