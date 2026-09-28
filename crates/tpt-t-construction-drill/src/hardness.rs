// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Rock hardness detection via drill rate and torque (spec.txt §5.5),
//! using Teale's specific energy concept (Teale, R. (1965), "The concept
//! of specific energy in rock drilling", *Int. J. Rock Mech. Mining Sci.*
//! 2(1)): the mechanical energy expended per unit volume of rock removed.
//! Harder rock takes more energy to remove the same volume, so specific
//! energy is a direct, physically grounded hardness proxy from
//! measurements the drill rig already has (thrust, torque, rotation
//! speed, penetration rate) — no separate hardness sensor needed.

use std::f32::consts::PI;

/// Specific energy (J/m^3) expended drilling at the given thrust, bit
/// geometry, torque, rotation speed, and penetration rate:
///
/// `SE = thrust/area + (2*pi*N*T) / (area*ROP)`
///
/// Returns `0.0` if the bit isn't actually advancing (`penetration_rate_m_s
/// <= 0`) or has no cross-section — there's no meaningful "energy per
/// volume removed" when no volume is being removed.
pub fn specific_energy_j_m3(
    thrust_n: f32,
    bit_area_m2: f32,
    torque_nm: f32,
    rotation_speed_rev_s: f32,
    penetration_rate_m_s: f32,
) -> f32 {
    if bit_area_m2 <= 0.0 || penetration_rate_m_s <= 0.0 {
        return 0.0;
    }
    thrust_n / bit_area_m2
        + (2.0 * PI * rotation_speed_rev_s * torque_nm) / (bit_area_m2 * penetration_rate_m_s)
}

/// A qualitative hardness classification, for blast design and drill-rate
/// derating decisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RockHardness {
    Soft,
    Medium,
    Hard,
    VeryHard,
}

/// Specific-energy thresholds separating [`RockHardness`] categories.
/// Representative, not a substitute for site-specific calibration against
/// core samples.
#[derive(Debug, Clone, Copy)]
pub struct HardnessBands {
    pub soft_max_j_m3: f32,
    pub medium_max_j_m3: f32,
    pub hard_max_j_m3: f32,
}

impl Default for HardnessBands {
    fn default() -> Self {
        HardnessBands {
            soft_max_j_m3: 20e6,
            medium_max_j_m3: 60e6,
            hard_max_j_m3: 150e6,
        }
    }
}

pub fn classify_hardness(specific_energy_j_m3: f32, bands: &HardnessBands) -> RockHardness {
    if specific_energy_j_m3 <= bands.soft_max_j_m3 {
        RockHardness::Soft
    } else if specific_energy_j_m3 <= bands.medium_max_j_m3 {
        RockHardness::Medium
    } else if specific_energy_j_m3 <= bands.hard_max_j_m3 {
        RockHardness::Hard
    } else {
        RockHardness::VeryHard
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_penetration_rate_is_zero_specific_energy() {
        assert_eq!(specific_energy_j_m3(10_000.0, 0.01, 500.0, 5.0, 0.0), 0.0);
    }

    #[test]
    fn zero_bit_area_is_zero_specific_energy() {
        assert_eq!(specific_energy_j_m3(10_000.0, 0.0, 500.0, 5.0, 0.01), 0.0);
    }

    #[test]
    fn slower_penetration_at_equal_torque_means_higher_specific_energy() {
        let fast = specific_energy_j_m3(10_000.0, 0.01, 500.0, 5.0, 0.02);
        let slow = specific_energy_j_m3(10_000.0, 0.01, 500.0, 5.0, 0.005);
        assert!(
            slow > fast,
            "drilling slower for the same torque should read as harder rock"
        );
    }

    #[test]
    fn more_torque_at_equal_penetration_rate_means_higher_specific_energy() {
        let low_torque = specific_energy_j_m3(10_000.0, 0.01, 300.0, 5.0, 0.01);
        let high_torque = specific_energy_j_m3(10_000.0, 0.01, 900.0, 5.0, 0.01);
        assert!(high_torque > low_torque);
    }

    #[test]
    fn classifies_each_band() {
        let bands = HardnessBands::default();
        assert_eq!(classify_hardness(10e6, &bands), RockHardness::Soft);
        assert_eq!(classify_hardness(40e6, &bands), RockHardness::Medium);
        assert_eq!(classify_hardness(100e6, &bands), RockHardness::Hard);
        assert_eq!(classify_hardness(200e6, &bands), RockHardness::VeryHard);
    }
}
