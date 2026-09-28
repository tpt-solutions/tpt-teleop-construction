// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Material type classification (spec.txt §3): a bulk-density-based
//! classifier distinguishing dirt, rock, and ore from a bucket/bed's
//! payload weight and the volume it occupies — the two signals this
//! crate already has on hand from [`crate::scale`] and
//! [`crate::volume`], without requiring a camera or spectrometer model.
//! Vibration/acoustic signatures and drill parameters are additional
//! signals a real system would fuse in; this is the density-based
//! primary signal spec.txt calls out.

/// A material classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaterialType {
    Dirt,
    Rock,
    Ore,
    /// The volume was zero, negative, or otherwise not usable to derive a
    /// density from — not enough information to classify.
    Unknown,
}

/// The density thresholds separating [`MaterialType`] categories.
/// Defaults are representative bulk densities (loose/broken material, not
/// solid rock density), not a certified assay for any specific site.
#[derive(Debug, Clone, Copy)]
pub struct DensityBands {
    /// At or below this density: [`MaterialType::Dirt`].
    pub dirt_max_kg_m3: f32,
    /// Above `dirt_max_kg_m3` and at or below this: [`MaterialType::Rock`].
    /// Above this: [`MaterialType::Ore`].
    pub rock_max_kg_m3: f32,
}

impl Default for DensityBands {
    fn default() -> Self {
        DensityBands {
            dirt_max_kg_m3: 2_000.0,
            rock_max_kg_m3: 2_800.0,
        }
    }
}

/// Bulk density (kg/m^3) from a payload's mass and the volume it
/// occupies. Returns `0.0` for a non-positive volume rather than
/// dividing by zero or returning a negative/infinite density.
pub fn bulk_density_kg_m3(mass_kg: f32, volume_m3: f32) -> f32 {
    if volume_m3 <= 0.0 {
        0.0
    } else {
        mass_kg / volume_m3
    }
}

/// Classifies a bulk density against [`DensityBands`].
pub fn classify_by_density(bulk_density_kg_m3: f32, bands: &DensityBands) -> MaterialType {
    if bulk_density_kg_m3 <= 0.0 {
        MaterialType::Unknown
    } else if bulk_density_kg_m3 <= bands.dirt_max_kg_m3 {
        MaterialType::Dirt
    } else if bulk_density_kg_m3 <= bands.rock_max_kg_m3 {
        MaterialType::Rock
    } else {
        MaterialType::Ore
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_or_negative_volume_yields_zero_density() {
        assert_eq!(bulk_density_kg_m3(1000.0, 0.0), 0.0);
        assert_eq!(bulk_density_kg_m3(1000.0, -5.0), 0.0);
    }

    #[test]
    fn density_divides_mass_by_volume() {
        assert!((bulk_density_kg_m3(3600.0, 2.0) - 1800.0).abs() < 1e-4);
    }

    #[test]
    fn classifies_each_band_correctly() {
        let bands = DensityBands::default();
        assert_eq!(classify_by_density(1_600.0, &bands), MaterialType::Dirt);
        assert_eq!(classify_by_density(2_400.0, &bands), MaterialType::Rock);
        assert_eq!(classify_by_density(3_200.0, &bands), MaterialType::Ore);
    }

    #[test]
    fn boundaries_are_inclusive_toward_the_lighter_category() {
        let bands = DensityBands::default();
        assert_eq!(
            classify_by_density(bands.dirt_max_kg_m3, &bands),
            MaterialType::Dirt
        );
        assert_eq!(
            classify_by_density(bands.rock_max_kg_m3, &bands),
            MaterialType::Rock
        );
    }

    #[test]
    fn non_positive_density_is_unknown() {
        let bands = DensityBands::default();
        assert_eq!(classify_by_density(0.0, &bands), MaterialType::Unknown);
        assert_eq!(classify_by_density(-100.0, &bands), MaterialType::Unknown);
    }
}
