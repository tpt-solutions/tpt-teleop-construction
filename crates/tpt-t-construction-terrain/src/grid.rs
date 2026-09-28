// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Real-time 3D terrain modeling from streaming LiDAR returns (spec.txt
//! §3: "real-time 3D terrain modeling, LiDAR-based surface
//! reconstruction"). [`TerrainGrid`] is a fixed-size height grid that
//! incrementally reconstructs the surface as individual point returns
//! arrive, rather than requiring a full point cloud in memory at once:
//! each new point folds into its cell via a running mean, so a noisy
//! LiDAR return nudges the surface estimate instead of overwriting it.

/// A regular-grid terrain height model, `width x depth` cells of
/// `resolution_m` meters each, centered on the origin in the XY plane —
/// the same layout convention as `tpt-t-construction-sim::lidar::Heightmap`,
/// so synthetic scans from the simulator map directly onto this grid.
#[derive(Debug, Clone)]
pub struct TerrainGrid {
    width: usize,
    depth: usize,
    resolution_m: f32,
    heights_m: Vec<f32>,
    hit_counts: Vec<u32>,
}

impl TerrainGrid {
    pub fn new(width: usize, depth: usize, resolution_m: f32) -> Self {
        assert!(width > 0 && depth > 0, "grid dimensions must be positive");
        assert!(resolution_m > 0.0, "resolution_m must be positive");
        TerrainGrid {
            width,
            depth,
            resolution_m,
            heights_m: vec![0.0; width * depth],
            hit_counts: vec![0; width * depth],
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn depth(&self) -> usize {
        self.depth
    }

    pub fn resolution_m(&self) -> f32 {
        self.resolution_m
    }

    fn index(&self, col: usize, row: usize) -> Option<usize> {
        if col < self.width && row < self.depth {
            Some(row * self.width + col)
        } else {
            None
        }
    }

    /// The reconstructed height at `(col, row)`, or `None` if that cell is
    /// out of range or has never received a LiDAR return.
    pub fn height_at(&self, col: usize, row: usize) -> Option<f32> {
        let idx = self.index(col, row)?;
        if self.hit_counts[idx] == 0 {
            None
        } else {
            Some(self.heights_m[idx])
        }
    }

    pub fn hit_count_at(&self, col: usize, row: usize) -> u32 {
        self.index(col, row)
            .map(|idx| self.hit_counts[idx])
            .unwrap_or(0)
    }

    /// Maps a world-space `(x, y)` position to the grid cell containing
    /// it, or `None` if it falls outside the grid's mapped extent.
    pub fn cell_of(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        let half_w = self.width as f32 * self.resolution_m * 0.5;
        let half_d = self.depth as f32 * self.resolution_m * 0.5;
        if x < -half_w || x >= half_w || y < -half_d || y >= half_d {
            return None;
        }
        let col = ((x + half_w) / self.resolution_m) as usize;
        let row = ((y + half_d) / self.resolution_m) as usize;
        Some((col.min(self.width - 1), row.min(self.depth - 1)))
    }

    /// Folds one LiDAR return at world position `(x, y, z)` into the grid.
    /// Points outside the grid's mapped extent are silently dropped —
    /// real-time ingestion from a spinning LiDAR unit will routinely see
    /// returns from outside today's region of interest, and that's not an
    /// error condition worth interrupting the stream over.
    ///
    /// Repeated returns into the same cell are combined via a running
    /// mean (`new = old + (z - old) / n`), so transient sensor noise
    /// averages out rather than the most recent point always winning.
    pub fn integrate_point(&mut self, x: f32, y: f32, z: f32) {
        let Some((col, row)) = self.cell_of(x, y) else {
            return;
        };
        let idx = self
            .index(col, row)
            .expect("cell_of only returns in-range cells");
        let n = self.hit_counts[idx];
        if n == 0 {
            self.heights_m[idx] = z;
        } else {
            self.heights_m[idx] += (z - self.heights_m[idx]) / (n as f32 + 1.0);
        }
        self.hit_counts[idx] = n + 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unsampled_cell_has_no_height() {
        let grid = TerrainGrid::new(4, 4, 1.0);
        assert_eq!(grid.height_at(0, 0), None);
        assert_eq!(grid.hit_count_at(0, 0), 0);
    }

    #[test]
    fn out_of_range_cell_is_none_not_a_panic() {
        let grid = TerrainGrid::new(4, 4, 1.0);
        assert_eq!(grid.height_at(100, 100), None);
    }

    #[test]
    fn single_point_sets_that_cells_height() {
        let mut grid = TerrainGrid::new(4, 4, 1.0);
        grid.integrate_point(0.0, 0.0, 5.0);
        let (col, row) = grid.cell_of(0.0, 0.0).unwrap();
        assert_eq!(grid.height_at(col, row), Some(5.0));
        assert_eq!(grid.hit_count_at(col, row), 1);
    }

    #[test]
    fn repeated_points_average_via_running_mean() {
        let mut grid = TerrainGrid::new(4, 4, 1.0);
        grid.integrate_point(0.0, 0.0, 10.0);
        grid.integrate_point(0.0, 0.0, 20.0);
        let (col, row) = grid.cell_of(0.0, 0.0).unwrap();
        assert_eq!(grid.height_at(col, row), Some(15.0));
        grid.integrate_point(0.0, 0.0, 30.0);
        // mean of 10, 20, 30 = 20
        assert!((grid.height_at(col, row).unwrap() - 20.0).abs() < 1e-4);
    }

    #[test]
    fn points_outside_the_grid_extent_are_dropped_silently() {
        let mut grid = TerrainGrid::new(4, 4, 1.0);
        grid.integrate_point(1000.0, 1000.0, 5.0);
        for row in 0..4 {
            for col in 0..4 {
                assert_eq!(grid.height_at(col, row), None);
            }
        }
    }
}
