// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! TBM/roadheader advance rate optimization (spec.txt §6): the fastest
//! the cutter head can safely advance is set by how much cutting power is
//! available and how much energy the rock in front of it takes to remove
//! per unit volume — the same Teale specific-energy concept
//! `tpt-t-construction-drill` uses for rock hardness (Teale, R. (1965),
//! "The concept of specific energy in rock drilling"), rearranged here to
//! solve for rate instead of reporting hardness:
//!
//! `SE = power / (face_area * advance_rate)` rearranges to
//! `advance_rate = power / (SE * face_area)`.

/// The maximum advance rate (m/s) sustainable at `available_power_w`
/// cutting rock of `specific_energy_j_m3` across a `face_area_m2` cutting
/// face. Returns `0.0` if the specific energy or face area is
/// non-positive — there's no rate at which "no energy needed" or "no
/// face to cut" is a meaningful ratio.
pub fn max_sustainable_advance_rate_m_s(
    available_power_w: f32,
    specific_energy_j_m3: f32,
    face_area_m2: f32,
) -> f32 {
    if specific_energy_j_m3 <= 0.0 || face_area_m2 <= 0.0 {
        return 0.0;
    }
    available_power_w / (specific_energy_j_m3 * face_area_m2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn more_power_allows_a_faster_advance_rate() {
        let slow = max_sustainable_advance_rate_m_s(500_000.0, 80e6, 20.0);
        let fast = max_sustainable_advance_rate_m_s(1_000_000.0, 80e6, 20.0);
        assert!(fast > slow);
        assert!((fast - 2.0 * slow).abs() < 1e-6);
    }

    #[test]
    fn harder_rock_reduces_the_sustainable_rate() {
        let soft_rock = max_sustainable_advance_rate_m_s(1_000_000.0, 20e6, 20.0);
        let hard_rock = max_sustainable_advance_rate_m_s(1_000_000.0, 150e6, 20.0);
        assert!(hard_rock < soft_rock);
    }

    #[test]
    fn larger_face_area_reduces_the_sustainable_rate_at_equal_power() {
        let small_face = max_sustainable_advance_rate_m_s(1_000_000.0, 80e6, 10.0);
        let large_face = max_sustainable_advance_rate_m_s(1_000_000.0, 80e6, 40.0);
        assert!(large_face < small_face);
    }

    #[test]
    fn non_positive_specific_energy_or_area_gives_zero_rate() {
        assert_eq!(
            max_sustainable_advance_rate_m_s(1_000_000.0, 0.0, 20.0),
            0.0
        );
        assert_eq!(
            max_sustainable_advance_rate_m_s(1_000_000.0, 80e6, 0.0),
            0.0
        );
    }
}
