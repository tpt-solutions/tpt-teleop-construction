// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Bearing capacity estimation (spec.txt §3): how much pressure the
//! ground under a machine's tracks/outriggers/footing can take before
//! shear failure, via the classic Terzaghi bearing capacity equation with
//! Meyerhof's closed-form approximation for the `Nγ` factor (Terzaghi's
//! own `Nγ` has no closed form and was originally read off a chart).
//!
//! `qu = c*Nc + q*Nq + 0.5*gamma*B*Ngamma`
//!
//! where `c` is soil cohesion, `q` is the surcharge (overburden pressure)
//! at the footing depth, `gamma` is soil unit weight, and `B` is footing
//! width.

use std::f32::consts::FRAC_PI_4;

/// Terzaghi bearing capacity factors for a given soil friction angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BearingCapacityFactors {
    pub nc: f32,
    pub nq: f32,
    pub ngamma: f32,
}

/// Computes the three bearing capacity factors for friction angle `phi`
/// (radians). `Nc` uses Prandtl's exact undrained (`phi == 0`) value of
/// `5.14` at the origin, where the general formula's `cot(phi)` term is
/// undefined.
pub fn bearing_capacity_factors(friction_angle_rad: f32) -> BearingCapacityFactors {
    let phi = friction_angle_rad;
    let nq = (std::f32::consts::PI * phi.tan()).exp() * (FRAC_PI_4 + phi / 2.0).tan().powi(2);
    let nc = if phi.abs() < 1e-6 {
        5.14
    } else {
        (nq - 1.0) / phi.tan()
    };
    // Meyerhof (1963) approximation, the standard closed-form stand-in for
    // Terzaghi's originally chart-derived Ngamma.
    let ngamma = (nq - 1.0) * (1.4 * phi).tan();
    BearingCapacityFactors { nc, nq, ngamma }
}

/// The ultimate bearing capacity (Pa) of a strip footing/track of width
/// `footing_width_m`, at surcharge (overburden) pressure `surcharge_pa`,
/// on soil with the given cohesion and unit weight.
pub fn ultimate_bearing_capacity_pa(
    cohesion_pa: f32,
    surcharge_pa: f32,
    unit_weight_n_m3: f32,
    footing_width_m: f32,
    factors: &BearingCapacityFactors,
) -> f32 {
    cohesion_pa * factors.nc
        + surcharge_pa * factors.nq
        + 0.5 * unit_weight_n_m3 * footing_width_m * factors.ngamma
}

/// Factor of safety comparing ultimate bearing capacity to the pressure a
/// machine actually applies (its weight divided by track/footing contact
/// area). `FS < 1.0` means the ground is predicted to fail under that
/// load.
pub fn bearing_factor_of_safety(
    ultimate_bearing_capacity_pa: f32,
    applied_pressure_pa: f32,
) -> f32 {
    if applied_pressure_pa <= 0.0 {
        f32::INFINITY
    } else {
        ultimate_bearing_capacity_pa / applied_pressure_pa
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_friction_angle_matches_the_classic_undrained_values() {
        let factors = bearing_capacity_factors(0.0);
        assert!((factors.nq - 1.0).abs() < 1e-3);
        assert!((factors.nc - 5.14).abs() < 1e-3);
        assert!(factors.ngamma.abs() < 1e-3);
    }

    #[test]
    fn factors_increase_with_friction_angle() {
        let low = bearing_capacity_factors(0.2);
        let mid = bearing_capacity_factors(0.4);
        let high = bearing_capacity_factors(0.6);
        assert!(low.nq < mid.nq && mid.nq < high.nq);
        assert!(low.nc < mid.nc && mid.nc < high.nc);
        assert!(low.ngamma < mid.ngamma && mid.ngamma < high.ngamma);
    }

    #[test]
    fn ultimate_capacity_combines_all_three_terms() {
        let factors = BearingCapacityFactors {
            nc: 5.14,
            nq: 1.0,
            ngamma: 0.0,
        };
        // With Nq=1 and Ngamma=0, only cohesion and surcharge contribute.
        let qu = ultimate_bearing_capacity_pa(10_000.0, 20_000.0, 18_000.0, 3.0, &factors);
        assert!((qu - (10_000.0 * 5.14 + 20_000.0)).abs() < 1.0);
    }

    #[test]
    fn wider_footing_increases_capacity_via_the_ngamma_term() {
        let factors = bearing_capacity_factors(0.5);
        let narrow = ultimate_bearing_capacity_pa(5_000.0, 10_000.0, 18_000.0, 1.0, &factors);
        let wide = ultimate_bearing_capacity_pa(5_000.0, 10_000.0, 18_000.0, 4.0, &factors);
        assert!(wide > narrow);
    }

    #[test]
    fn factor_of_safety_flags_overloaded_ground() {
        assert!(bearing_factor_of_safety(100_000.0, 200_000.0) < 1.0);
        assert!(bearing_factor_of_safety(300_000.0, 100_000.0) > 1.0);
    }
}
