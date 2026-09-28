// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Blast hole pattern execution (spec.txt §5.5): each hole is drilled to
//! an exact depth and angle from a surveyed collar position, and the
//! pattern is executed hole-by-hole in sequence.

/// A 2D surface position, meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}

/// One blast hole's design: where it starts, how deep, and at what angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlastHole {
    pub collar: Point2,
    pub depth_m: f32,
    /// Angle from vertical, radians (`0` = straight down).
    pub angle_from_vertical_rad: f32,
    /// Compass direction (radians, 0 = +x axis) the hole leans toward,
    /// meaningless when `angle_from_vertical_rad == 0`.
    pub azimuth_rad: f32,
}

impl BlastHole {
    /// The hole bottom's 3D position (x, y, z) relative to the collar,
    /// `z` negative downward.
    pub fn bottom_position(&self) -> (f32, f32, f32) {
        let horizontal_m = self.depth_m * self.angle_from_vertical_rad.sin();
        let vertical_m = -self.depth_m * self.angle_from_vertical_rad.cos();
        let dx = horizontal_m * self.azimuth_rad.cos();
        let dy = horizontal_m * self.azimuth_rad.sin();
        (self.collar.x + dx, self.collar.y + dy, vertical_m)
    }
}

/// A full blast pattern: every hole to be drilled.
#[derive(Debug, Clone, Default)]
pub struct BlastPattern {
    pub holes: Vec<BlastHole>,
}

/// Executes a [`BlastPattern`] one hole at a time.
#[derive(Debug, Clone)]
pub struct PatternExecutor {
    pattern: BlastPattern,
    current_index: usize,
}

impl PatternExecutor {
    pub fn new(pattern: BlastPattern) -> Self {
        PatternExecutor {
            pattern,
            current_index: 0,
        }
    }

    pub fn current_hole(&self) -> Option<&BlastHole> {
        self.pattern.holes.get(self.current_index)
    }

    pub fn remaining(&self) -> usize {
        self.pattern.holes.len().saturating_sub(self.current_index)
    }

    pub fn is_complete(&self) -> bool {
        self.current_index >= self.pattern.holes.len()
    }

    /// Marks the current hole done and moves to the next one. Returns
    /// `true` if there is a next hole to drill, `false` if the pattern is
    /// now complete. Calling this again once already complete is a no-op
    /// that keeps returning `false`, rather than panicking on an
    /// already-finished pattern.
    pub fn advance(&mut self) -> bool {
        if self.is_complete() {
            return false;
        }
        self.current_index += 1;
        !self.is_complete()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vertical_hole_bottom_is_straight_down_from_collar() {
        let hole = BlastHole {
            collar: Point2 { x: 5.0, y: 5.0 },
            depth_m: 10.0,
            angle_from_vertical_rad: 0.0,
            azimuth_rad: 0.0,
        };
        let (x, y, z) = hole.bottom_position();
        assert!((x - 5.0).abs() < 1e-4);
        assert!((y - 5.0).abs() < 1e-4);
        assert!((z - -10.0).abs() < 1e-4);
    }

    #[test]
    fn angled_hole_offsets_horizontally_by_depth_times_sine() {
        let hole = BlastHole {
            collar: Point2 { x: 0.0, y: 0.0 },
            depth_m: 10.0,
            angle_from_vertical_rad: std::f32::consts::FRAC_PI_6, // 30 degrees
            azimuth_rad: 0.0,
        };
        let (x, _, z) = hole.bottom_position();
        assert!((x - 10.0 * 0.5).abs() < 1e-3); // sin(30deg) = 0.5
        assert!(z < 0.0);
    }

    fn three_hole_pattern() -> BlastPattern {
        BlastPattern {
            holes: vec![
                BlastHole {
                    collar: Point2 { x: 0.0, y: 0.0 },
                    depth_m: 8.0,
                    angle_from_vertical_rad: 0.0,
                    azimuth_rad: 0.0,
                },
                BlastHole {
                    collar: Point2 { x: 3.0, y: 0.0 },
                    depth_m: 8.0,
                    angle_from_vertical_rad: 0.0,
                    azimuth_rad: 0.0,
                },
                BlastHole {
                    collar: Point2 { x: 6.0, y: 0.0 },
                    depth_m: 8.0,
                    angle_from_vertical_rad: 0.0,
                    azimuth_rad: 0.0,
                },
            ],
        }
    }

    #[test]
    fn executor_starts_at_the_first_hole() {
        let executor = PatternExecutor::new(three_hole_pattern());
        assert_eq!(
            executor.current_hole().unwrap().collar,
            Point2 { x: 0.0, y: 0.0 }
        );
        assert_eq!(executor.remaining(), 3);
        assert!(!executor.is_complete());
    }

    #[test]
    fn advances_through_every_hole_then_completes() {
        let mut executor = PatternExecutor::new(three_hole_pattern());
        assert!(executor.advance());
        assert_eq!(
            executor.current_hole().unwrap().collar,
            Point2 { x: 3.0, y: 0.0 }
        );
        assert!(executor.advance());
        assert_eq!(
            executor.current_hole().unwrap().collar,
            Point2 { x: 6.0, y: 0.0 }
        );
        assert!(!executor.advance());
        assert!(executor.is_complete());
        assert_eq!(executor.current_hole(), None);
    }

    #[test]
    fn advancing_past_completion_stays_complete() {
        let mut executor = PatternExecutor::new(three_hole_pattern());
        for _ in 0..10 {
            executor.advance();
        }
        assert!(executor.is_complete());
        assert_eq!(executor.remaining(), 0);
    }

    #[test]
    fn empty_pattern_starts_complete() {
        let mut executor = PatternExecutor::new(BlastPattern::default());
        assert!(executor.is_complete());
        assert!(!executor.advance());
    }
}
