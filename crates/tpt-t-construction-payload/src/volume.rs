// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Cut/fill volume calculation from paired terrain height samples
//! (spec.txt §4.2: "<10 milliseconds per scan").
//!
//! `core::simd` ("portable-simd") remains a nightly-only feature
//! (rust-lang/rust#86656) as of this toolchain, and this workspace's CI
//! targets stable Rust, so explicit SIMD intrinsics aren't an option
//! here. Instead, the summation below is written across `LANES`
//! independent accumulators that touch non-overlapping elements each
//! iteration — a standard pattern LLVM's auto-vectorizer recognizes and
//! packs into SIMD instructions on the target CPU, without the crate
//! itself depending on any unstable API. Should `core::simd` stabilize,
//! this is the one function that would switch to it directly.

const LANES: usize = 8;

/// Cut volume (material removed, m^3), fill volume (material added, m^3),
/// and their difference, all as positive-or-negative-signed magnitudes:
/// `net_m3 == cut_m3 - fill_m3`... except sign convention here follows
/// [`CutFillResult`]'s field docs directly, so read those rather than
/// inferring one from the other.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CutFillResult {
    /// Total volume where the surface dropped (`previous > current`),
    /// i.e. material removed. Always `>= 0`.
    pub cut_m3: f32,
    /// Total volume where the surface rose (`current > previous`), i.e.
    /// material added. Always `>= 0`.
    pub fill_m3: f32,
    /// `sum(current - previous) * cell_area_m2`: positive means net
    /// material was added over the scanned area, negative means net
    /// material was removed.
    pub net_m3: f32,
}

/// Computes cut and fill volumes between two same-length height-sample
/// arrays (e.g. two `tpt-t-construction-terrain::TerrainGrid` snapshots'
/// backing cells, taken 5 seconds apart per spec.txt §8), given the
/// world-space area each sample represents.
///
/// # Panics
///
/// Panics if `previous_heights_m` and `current_heights_m` differ in
/// length — comparing two scans of different shapes is a caller bug.
pub fn cut_fill_volumes(
    previous_heights_m: &[f32],
    current_heights_m: &[f32],
    cell_area_m2: f32,
) -> CutFillResult {
    assert_eq!(
        previous_heights_m.len(),
        current_heights_m.len(),
        "previous and current height arrays must be the same length"
    );

    let mut cut_lanes = [0.0f32; LANES];
    let mut fill_lanes = [0.0f32; LANES];
    let mut net_lanes = [0.0f32; LANES];

    let mut prev_chunks = previous_heights_m.chunks_exact(LANES);
    let mut curr_chunks = current_heights_m.chunks_exact(LANES);
    for (prev_chunk, curr_chunk) in prev_chunks.by_ref().zip(curr_chunks.by_ref()) {
        for lane in 0..LANES {
            let delta = curr_chunk[lane] - prev_chunk[lane];
            net_lanes[lane] += delta;
            if delta > 0.0 {
                fill_lanes[lane] += delta;
            } else {
                cut_lanes[lane] -= delta;
            }
        }
    }

    let mut cut_m2: f32 = cut_lanes.iter().sum();
    let mut fill_m2: f32 = fill_lanes.iter().sum();
    let mut net_m2: f32 = net_lanes.iter().sum();

    for (&prev, &curr) in prev_chunks.remainder().iter().zip(curr_chunks.remainder()) {
        let delta = curr - prev;
        net_m2 += delta;
        if delta > 0.0 {
            fill_m2 += delta;
        } else {
            cut_m2 -= delta;
        }
    }

    CutFillResult {
        cut_m3: cut_m2 * cell_area_m2,
        fill_m3: fill_m2 * cell_area_m2,
        net_m3: net_m2 * cell_area_m2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn identical_scans_have_zero_cut_and_fill() {
        let heights = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = cut_fill_volumes(&heights, &heights, 1.0);
        assert_eq!(
            result,
            CutFillResult {
                cut_m3: 0.0,
                fill_m3: 0.0,
                net_m3: 0.0,
            }
        );
    }

    #[test]
    fn a_single_lowered_cell_registers_as_cut() {
        let previous = vec![5.0; 10];
        let mut current = previous.clone();
        current[3] = 2.0; // dropped by 3.0
        let result = cut_fill_volumes(&previous, &current, 2.0);
        assert!((result.cut_m3 - 6.0).abs() < 1e-4); // 3.0m drop * 2.0m^2 cell
        assert_eq!(result.fill_m3, 0.0);
        assert!((result.net_m3 - -6.0).abs() < 1e-4);
    }

    #[test]
    fn a_single_raised_cell_registers_as_fill() {
        let previous = vec![5.0; 10];
        let mut current = previous.clone();
        current[7] = 8.0; // raised by 3.0
        let result = cut_fill_volumes(&previous, &current, 2.0);
        assert!((result.fill_m3 - 6.0).abs() < 1e-4);
        assert_eq!(result.cut_m3, 0.0);
    }

    #[test]
    fn cut_and_fill_accumulate_independently_of_each_other() {
        // One cell cut, one cell filled, net should reflect the difference
        // but cut/fill should each reflect their own gross magnitude.
        let previous = vec![0.0, 0.0];
        let current = vec![-2.0, 3.0];
        let result = cut_fill_volumes(&previous, &current, 1.0);
        assert!((result.cut_m3 - 2.0).abs() < 1e-4);
        assert!((result.fill_m3 - 3.0).abs() < 1e-4);
        assert!((result.net_m3 - 1.0).abs() < 1e-4);
    }

    #[test]
    fn handles_lengths_not_a_multiple_of_the_lane_width() {
        // 5 elements deliberately doesn't divide evenly by LANES (8), to
        // exercise the scalar remainder path.
        let previous = vec![0.0; 5];
        let mut current = vec![0.0; 5];
        current[4] = 1.0;
        let result = cut_fill_volumes(&previous, &current, 1.0);
        assert!((result.fill_m3 - 1.0).abs() < 1e-4);
    }

    #[test]
    #[should_panic(expected = "same length")]
    fn mismatched_lengths_panics() {
        cut_fill_volumes(&[0.0, 1.0], &[0.0], 1.0);
    }

    #[test]
    fn a_realistic_scan_computes_in_well_under_ten_milliseconds() {
        // A generous stand-in for one LiDAR scan's worth of terrain cells
        // (spec.txt §4.2 targets <10ms per scan taken every 5 seconds).
        const CELLS: usize = 500 * 500;
        let previous: Vec<f32> = (0..CELLS).map(|i| (i % 100) as f32 * 0.01).collect();
        let current: Vec<f32> = previous.iter().map(|&h| h + 0.05).collect();

        let start = Instant::now();
        let result = cut_fill_volumes(&previous, &current, 0.05 * 0.05);
        let elapsed = start.elapsed();

        assert!(result.fill_m3 > 0.0);
        assert!(
            elapsed.as_millis() < 10,
            "cut/fill volume calc over {CELLS} cells took {elapsed:?}, budget is 10ms"
        );
    }
}
