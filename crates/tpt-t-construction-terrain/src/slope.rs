// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Slope stability analysis (spec.txt §3): a local slope angle computed
//! from the reconstructed terrain grid, and the classic infinite-slope
//! factor-of-safety equation used throughout geotechnical practice for a
//! long, uniform slope where the failure surface runs parallel to the
//! ground surface at some depth `H`.

use crate::grid::TerrainGrid;

/// The local slope angle (radians, always non-negative) at `(col, row)`,
/// from a central-difference gradient over the grid's 4-neighborhood.
/// Returns `None` if `(col, row)` or any of its four neighbors haven't
/// been sampled yet (including grid edges, which have no neighbor on one
/// side) — a slope estimate from a partial neighborhood would be
/// misleading rather than merely imprecise.
pub fn slope_angle_rad(grid: &TerrainGrid, col: usize, row: usize) -> Option<f32> {
    let h_left = grid.height_at(col.checked_sub(1)?, row)?;
    let h_right = grid.height_at(col.checked_add(1)?, row)?;
    let h_down = grid.height_at(col, row.checked_sub(1)?)?;
    let h_up = grid.height_at(col, row.checked_add(1)?)?;

    let two_res = 2.0 * grid.resolution_m();
    let dz_dx = (h_right - h_left) / two_res;
    let dz_dy = (h_up - h_down) / two_res;
    let gradient_magnitude = (dz_dx * dz_dx + dz_dy * dz_dy).sqrt();
    Some(gradient_magnitude.atan())
}

/// The infinite-slope factor of safety: the ratio of resisting shear
/// strength to driving shear stress along a failure plane parallel to the
/// surface at depth `depth_m`, per Mohr-Coulomb shear strength
/// (`c + sigma_n * tan(phi)`) resolved onto that plane.
///
/// `FS < 1.0` means the slope is predicted to fail; `FS >= 1.0` means it
/// holds, with larger values indicating more margin. A perfectly flat
/// slope (`slope_angle_rad == 0`) has no driving shear stress at all and
/// is reported as infinitely stable rather than triggering a division by
/// zero.
pub fn infinite_slope_factor_of_safety(
    slope_angle_rad: f32,
    cohesion_pa: f32,
    friction_angle_rad: f32,
    unit_weight_n_m3: f32,
    depth_m: f32,
) -> f32 {
    let beta = slope_angle_rad;
    let normal_stress_pa = unit_weight_n_m3 * depth_m * beta.cos() * beta.cos();
    let shear_stress_pa = unit_weight_n_m3 * depth_m * beta.sin() * beta.cos();
    if shear_stress_pa <= 0.0 {
        return f32::INFINITY;
    }
    (cohesion_pa + normal_stress_pa * friction_angle_rad.tan()) / shear_stress_pa
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_4;

    #[test]
    fn flat_neighborhood_has_zero_slope() {
        let mut grid = TerrainGrid::new(5, 5, 1.0);
        for row in 0..5 {
            for col in 0..5 {
                let (x, y) = ((col as f32 - 2.0), (row as f32 - 2.0));
                grid.integrate_point(x, y, 10.0);
            }
        }
        let angle = slope_angle_rad(&grid, 2, 2).unwrap();
        assert!(angle.abs() < 1e-4);
    }

    #[test]
    fn none_when_neighborhood_is_incomplete() {
        let mut grid = TerrainGrid::new(5, 5, 1.0);
        grid.integrate_point(0.0, 0.0, 10.0); // only the center cell sampled
        assert_eq!(slope_angle_rad(&grid, 2, 2), None);
        // Corner cells have no full 4-neighborhood at all, regardless of sampling.
        assert_eq!(slope_angle_rad(&grid, 0, 0), None);
    }

    #[test]
    fn a_45_degree_ramp_reports_pi_over_4() {
        let mut grid = TerrainGrid::new(5, 5, 1.0);
        for row in 0..5 {
            for col in 0..5 {
                let x = col as f32 - 2.0;
                let y = row as f32 - 2.0;
                grid.integrate_point(x, y, x); // height rises 1m per 1m of x: 45 degrees
            }
        }
        let angle = slope_angle_rad(&grid, 2, 2).unwrap();
        assert!((angle - FRAC_PI_4).abs() < 1e-3);
    }

    #[test]
    fn flat_ground_is_infinitely_stable() {
        let fs = infinite_slope_factor_of_safety(0.0, 5_000.0, 0.5, 18_000.0, 2.0);
        assert!(fs.is_infinite());
    }

    #[test]
    fn cohesionless_soil_matches_the_classic_tan_phi_over_tan_beta_identity() {
        let beta = 0.4_f32;
        let phi = 0.6_f32;
        let fs = infinite_slope_factor_of_safety(beta, 0.0, phi, 18_000.0, 2.0);
        let expected = phi.tan() / beta.tan();
        assert!((fs - expected).abs() < 1e-3);
    }

    #[test]
    fn more_cohesion_or_less_steep_slope_increases_safety_factor() {
        let base = infinite_slope_factor_of_safety(0.5, 5_000.0, 0.5, 18_000.0, 2.0);
        let more_cohesion = infinite_slope_factor_of_safety(0.5, 20_000.0, 0.5, 18_000.0, 2.0);
        let gentler = infinite_slope_factor_of_safety(0.2, 5_000.0, 0.5, 18_000.0, 2.0);
        assert!(more_cohesion > base);
        assert!(gentler > base);
    }
}
