// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Dynamic obstacle detection (spec.txt §3) via terrain-grid differencing:
//! anything sitting above (or a hole dug into) the previously mapped
//! static surface by more than a threshold is flagged as a candidate
//! obstacle. This is a first-pass "what changed" detector suited to
//! catching anything not part of the known terrain — full object
//! tracking and human/vehicle/equipment classification is
//! `tpt-t-construction-safety`'s job (Phase 7), fed by candidates like
//! these.

use crate::grid::TerrainGrid;

/// A grid cell whose live-scan height disagrees with the stored static
/// baseline by more than the detection threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstacleCandidate {
    pub col: usize,
    pub row: usize,
    /// `live - baseline`: positive means something is sitting above the
    /// known surface, negative means the surface dropped (a new
    /// excavation, a washout, ...).
    pub height_delta_m: f32,
}

/// Compares a live scan against a static baseline grid (both must share
/// identical dimensions) and returns every cell where they disagree by at
/// least `threshold_m`, in row-major order. A cell unsampled in either
/// grid is skipped rather than reported: no baseline to compare against
/// is not evidence of a new obstacle.
///
/// # Panics
///
/// Panics if `baseline` and `live` have different dimensions — comparing
/// grids of two different areas is a caller bug, not a runtime condition.
pub fn detect_obstacles(
    baseline: &TerrainGrid,
    live: &TerrainGrid,
    threshold_m: f32,
) -> Vec<ObstacleCandidate> {
    assert_eq!(
        baseline.width(),
        live.width(),
        "grids must cover the same area"
    );
    assert_eq!(
        baseline.depth(),
        live.depth(),
        "grids must cover the same area"
    );

    let mut obstacles = Vec::new();
    for row in 0..baseline.depth() {
        for col in 0..baseline.width() {
            if let (Some(base_h), Some(live_h)) =
                (baseline.height_at(col, row), live.height_at(col, row))
            {
                let delta = live_h - base_h;
                if delta.abs() >= threshold_m {
                    obstacles.push(ObstacleCandidate {
                        col,
                        row,
                        height_delta_m: delta,
                    });
                }
            }
        }
    }
    obstacles
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat_grid(width: usize, depth: usize, height_m: f32) -> TerrainGrid {
        let mut grid = TerrainGrid::new(width, depth, 1.0);
        for row in 0..depth {
            for col in 0..width {
                let x = col as f32 - width as f32 / 2.0;
                let y = row as f32 - depth as f32 / 2.0;
                grid.integrate_point(x, y, height_m);
            }
        }
        grid
    }

    #[test]
    fn identical_grids_have_no_obstacles() {
        let baseline = flat_grid(4, 4, 0.0);
        let live = flat_grid(4, 4, 0.0);
        assert!(detect_obstacles(&baseline, &live, 0.1).is_empty());
    }

    #[test]
    fn an_elevated_cell_is_flagged() {
        let baseline = flat_grid(4, 4, 0.0);
        let mut live = flat_grid(4, 4, 0.0);
        live.integrate_point(0.0, 0.0, 2.0); // note: averages with the existing sample
        let obstacles = detect_obstacles(&baseline, &live, 0.5);
        assert_eq!(obstacles.len(), 1);
        assert!(obstacles[0].height_delta_m > 0.5);
    }

    #[test]
    fn sub_threshold_noise_is_not_flagged() {
        let baseline = flat_grid(4, 4, 0.0);
        let live = flat_grid(4, 4, 0.05);
        assert!(detect_obstacles(&baseline, &live, 0.5).is_empty());
    }

    #[test]
    fn a_negative_delta_excavation_is_flagged_too() {
        let baseline = flat_grid(4, 4, 0.0);
        let live = flat_grid(4, 4, -1.0);
        let obstacles = detect_obstacles(&baseline, &live, 0.5);
        assert_eq!(obstacles.len(), 16);
        assert!(obstacles.iter().all(|o| o.height_delta_m < 0.0));
    }

    #[test]
    #[should_panic(expected = "same area")]
    fn mismatched_dimensions_panics() {
        let baseline = flat_grid(4, 4, 0.0);
        let live = flat_grid(5, 5, 0.0);
        detect_obstacles(&baseline, &live, 0.5);
    }
}
