// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Thermal and emissions-based engine derating (spec.txt §3: "engine
//! derating based on thermal and emission constraints"). Rather than
//! either ignoring an overheat condition or cutting power abruptly at a
//! hard limit, each monitored temperature ramps the available torque down
//! linearly between a "starting to worry" threshold and a "must not
//! exceed" threshold, giving the operator/autonomy stack a graceful power
//! reduction instead of a sudden cutoff.

/// A single temperature limit pair: `warn_c` is where derating begins,
/// `shutdown_c` is where it reaches zero.
#[derive(Debug, Clone, Copy)]
pub struct ThermalLimit {
    pub warn_c: f32,
    pub shutdown_c: f32,
}

impl ThermalLimit {
    /// Fraction (`0.0..=1.0`) of rated torque still available at
    /// `temp_c`: `1.0` at or below `warn_c`, ramping linearly down to
    /// `0.0` at or above `shutdown_c`.
    pub fn derate_factor(&self, temp_c: f32) -> f32 {
        if temp_c <= self.warn_c {
            1.0
        } else if temp_c >= self.shutdown_c {
            0.0
        } else {
            1.0 - (temp_c - self.warn_c) / (self.shutdown_c - self.warn_c)
        }
    }
}

/// The full set of thermal/emissions limits the powertrain monitors.
/// Defaults are representative diesel heavy-equipment limits, not a
/// specific engine's certified values.
#[derive(Debug, Clone, Copy)]
pub struct DerateConfig {
    pub coolant: ThermalLimit,
    pub exhaust: ThermalLimit,
}

impl Default for DerateConfig {
    fn default() -> Self {
        DerateConfig {
            coolant: ThermalLimit {
                warn_c: 100.0,
                shutdown_c: 110.0,
            },
            exhaust: ThermalLimit {
                warn_c: 650.0,
                shutdown_c: 700.0,
            },
        }
    }
}

impl DerateConfig {
    /// The combined derate factor across every monitored limit: the
    /// worst (lowest) individual factor wins, matching
    /// [`crate::torque::arbitrate_torque_nm`]'s "lowest limit wins" rule
    /// — one overheating system is enough to cut power, regardless of how
    /// healthy the others are.
    pub fn combined_derate_factor(&self, coolant_temp_c: f32, exhaust_temp_c: f32) -> f32 {
        self.coolant
            .derate_factor(coolant_temp_c)
            .min(self.exhaust.derate_factor(exhaust_temp_c))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_torque_below_warn_threshold() {
        let limit = ThermalLimit {
            warn_c: 100.0,
            shutdown_c: 110.0,
        };
        assert_eq!(limit.derate_factor(90.0), 1.0);
        assert_eq!(limit.derate_factor(100.0), 1.0);
    }

    #[test]
    fn zero_torque_at_or_above_shutdown_threshold() {
        let limit = ThermalLimit {
            warn_c: 100.0,
            shutdown_c: 110.0,
        };
        assert_eq!(limit.derate_factor(110.0), 0.0);
        assert_eq!(limit.derate_factor(150.0), 0.0);
    }

    #[test]
    fn linear_ramp_between_thresholds() {
        let limit = ThermalLimit {
            warn_c: 100.0,
            shutdown_c: 110.0,
        };
        assert!((limit.derate_factor(105.0) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn combined_factor_takes_the_worst_of_coolant_and_exhaust() {
        let config = DerateConfig::default();
        // Coolant is fine, exhaust is deep into its derate band.
        let factor = config.combined_derate_factor(80.0, 675.0);
        assert!((factor - 0.5).abs() < 1e-6);
    }

    #[test]
    fn nominal_conditions_give_full_torque() {
        let config = DerateConfig::default();
        assert_eq!(config.combined_derate_factor(85.0, 500.0), 1.0);
    }
}
