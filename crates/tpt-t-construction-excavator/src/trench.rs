// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Trench digging to an exact depth/width, and grading to a designed
//! slope (spec.txt §5.1). Both boil down to the same question at
//! different points: "how far off is the ground under the bucket from
//! where the design says it should be", which is what feeds the
//! hydraulic control loop's setpoint.

use crate::kinematics::Point3;

/// A trench's design geometry: a straight run from `start` to `end` at
/// constant `depth_m` below the original ground surface and `width_m`
/// wide.
#[derive(Debug, Clone, Copy)]
pub struct TrenchPlan {
    pub start: Point3,
    pub end: Point3,
    pub depth_m: f32,
    pub width_m: f32,
}

impl TrenchPlan {
    fn length_m(&self) -> f32 {
        let dx = self.end.x - self.start.x;
        let dy = self.end.y - self.start.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// One evenly-spaced dig-pass position along the trench centerline,
    /// `0.0` at `start` and `1.0` at `end`, at the design depth.
    fn centerline_point(&self, t: f32) -> Point3 {
        Point3 {
            x: self.start.x + (self.end.x - self.start.x) * t,
            y: self.start.y + (self.end.y - self.start.y) * t,
            z: self.start.z.min(self.end.z) - self.depth_m,
        }
    }

    /// A sequence of `num_passes` evenly-spaced target points along the
    /// trench centerline at design depth, for the excavator to swing to
    /// and dig at in turn. `num_passes` should be chosen so consecutive
    /// passes overlap by less than the bucket width (i.e. roughly
    /// `length / bucket_width_m` passes) — that sizing decision belongs
    /// to the caller, which knows the bucket geometry; this just lays out
    /// however many passes it's asked for.
    pub fn dig_passes(&self, num_passes: usize) -> Vec<Point3> {
        if num_passes == 0 {
            return Vec::new();
        }
        if num_passes == 1 {
            return vec![self.centerline_point(0.5)];
        }
        (0..num_passes)
            .map(|i| self.centerline_point(i as f32 / (num_passes - 1) as f32))
            .collect()
    }

    /// How far off `actual_depth_m` (measured depth below original grade
    /// at the bucket's current position) is from the design depth.
    /// Positive means still too shallow (needs more digging); negative
    /// means over-dug.
    pub fn depth_error_m(&self, actual_depth_m: f32) -> f32 {
        self.depth_m - actual_depth_m
    }

    pub fn length(&self) -> f32 {
        self.length_m()
    }
}

/// A planar grading surface defined by a reference point and a slope
/// ratio (rise over run) in each horizontal direction — e.g. a 3:1 (H:V)
/// slope is `1.0 / 3.0` rise per unit run.
#[derive(Debug, Clone, Copy)]
pub struct GradePlane {
    pub reference: Point3,
    pub slope_x: f32,
    pub slope_y: f32,
}

impl GradePlane {
    /// The design height at horizontal position `(x, y)`.
    pub fn design_height_m(&self, x: f32, y: f32) -> f32 {
        self.reference.z
            + (x - self.reference.x) * self.slope_x
            + (y - self.reference.y) * self.slope_y
    }

    /// How far `actual_height_m` at `(x, y)` is from the design surface.
    /// Positive means the ground is too high there (needs cutting);
    /// negative means too low (needs fill).
    pub fn grade_error_m(&self, x: f32, y: f32, actual_height_m: f32) -> f32 {
        actual_height_m - self.design_height_m(x, y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_trench() -> TrenchPlan {
        TrenchPlan {
            start: Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            end: Point3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            depth_m: 1.5,
            width_m: 0.6,
        }
    }

    #[test]
    fn dig_passes_span_start_to_end_at_design_depth() {
        let trench = flat_trench();
        let passes = trench.dig_passes(3);
        assert_eq!(passes.len(), 3);
        assert!((passes[0].x - 0.0).abs() < 1e-4);
        assert!((passes[2].x - 10.0).abs() < 1e-4);
        for p in &passes {
            assert!((p.z - -1.5).abs() < 1e-4);
        }
    }

    #[test]
    fn single_pass_targets_the_midpoint() {
        let trench = flat_trench();
        let passes = trench.dig_passes(1);
        assert_eq!(passes.len(), 1);
        assert!((passes[0].x - 5.0).abs() < 1e-4);
    }

    #[test]
    fn zero_passes_is_an_empty_plan() {
        assert!(flat_trench().dig_passes(0).is_empty());
    }

    #[test]
    fn depth_error_is_positive_when_too_shallow() {
        let trench = flat_trench();
        assert!((trench.depth_error_m(1.0) - 0.5).abs() < 1e-4);
    }

    #[test]
    fn depth_error_is_negative_when_over_dug() {
        let trench = flat_trench();
        assert!(trench.depth_error_m(2.0) < 0.0);
    }

    #[test]
    fn grade_plane_interpolates_along_both_axes() {
        let plane = GradePlane {
            reference: Point3 {
                x: 0.0,
                y: 0.0,
                z: 10.0,
            },
            slope_x: -1.0 / 3.0,
            slope_y: 0.0,
        };
        assert!((plane.design_height_m(3.0, 0.0) - 9.0).abs() < 1e-4);
        assert!((plane.design_height_m(-3.0, 0.0) - 11.0).abs() < 1e-4);
    }

    #[test]
    fn grade_error_signals_cut_vs_fill() {
        let plane = GradePlane {
            reference: Point3 {
                x: 0.0,
                y: 0.0,
                z: 10.0,
            },
            slope_x: 0.0,
            slope_y: 0.0,
        };
        assert!(plane.grade_error_m(0.0, 0.0, 10.5) > 0.0); // too high: cut
        assert!(plane.grade_error_m(0.0, 0.0, 9.5) < 0.0); // too low: fill
    }
}
