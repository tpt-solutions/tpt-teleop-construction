// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Center-of-gravity estimation (spec.txt §7): a raised, loaded bucket
//! shifts the machine's effective center of gravity upward and forward
//! from its unloaded value, which is exactly what changes its rollover
//! threshold moment to moment — the standard composite-body weighted
//! average of the empty machine's own CoG and the payload's position.

/// The effective center-of-gravity height (m, above ground) of the
/// machine-plus-payload system: the mass-weighted average of the empty
/// machine's own CoG height and the payload's height. Returns the empty
/// machine's CoG height unchanged if the total mass is non-positive
/// (nothing to average against).
pub fn effective_cog_height_m(
    empty_mass_kg: f32,
    empty_cog_height_m: f32,
    payload_mass_kg: f32,
    payload_height_m: f32,
) -> f32 {
    let total_mass_kg = empty_mass_kg + payload_mass_kg;
    if total_mass_kg <= 0.0 {
        return empty_cog_height_m;
    }
    (empty_mass_kg * empty_cog_height_m + payload_mass_kg * payload_height_m) / total_mass_kg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_payload_leaves_cog_at_the_empty_machine_value() {
        let cog = effective_cog_height_m(30_000.0, 1.5, 0.0, 3.0);
        assert!((cog - 1.5).abs() < 1e-4);
    }

    #[test]
    fn a_high_heavy_payload_raises_the_effective_cog() {
        let empty_cog = effective_cog_height_m(30_000.0, 1.5, 0.0, 0.0);
        let loaded_cog = effective_cog_height_m(30_000.0, 1.5, 10_000.0, 4.0);
        assert!(loaded_cog > empty_cog);
    }

    #[test]
    fn matches_the_weighted_average_formula_exactly() {
        // Equal masses at 1.0m and 3.0m should average to exactly 2.0m.
        let cog = effective_cog_height_m(10_000.0, 1.0, 10_000.0, 3.0);
        assert!((cog - 2.0).abs() < 1e-4);
    }

    #[test]
    fn zero_total_mass_falls_back_to_empty_cog_without_dividing_by_zero() {
        let cog = effective_cog_height_m(0.0, 1.5, 0.0, 5.0);
        assert_eq!(cog, 1.5);
    }
}
