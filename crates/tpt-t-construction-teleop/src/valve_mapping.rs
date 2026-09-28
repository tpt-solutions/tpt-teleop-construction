// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Translates a [`crate::control_command::DecodedControlCommand`]'s
//! primary axes into hydraulic proportional valve commands (todo.md Phase
//! 10: "Translate ControlCommand -> hydraulic proportional valve PWM via
//! tpt-t-construction-hydraulic (pressure comp, flow sharing)"), reusing
//! Phase 3's [`pressure_compensated_flow_fraction`] and [`share_flow`]
//! rather than re-deriving the compensation math here.

use tpt_t_construction_hydraulic::{pressure_compensated_flow_fraction, share_flow};

/// One valve-duty fraction (`-1.0..=1.0`, sign is direction) per
/// excavator-style implement axis. This is the direct axis-to-actuator
/// mapping — boom/arm/bucket/swing — before pressure compensation and
/// flow sharing are applied; see [`compensated_valve_duties`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ValveMap {
    pub boom: f32,
    pub arm: f32,
    pub bucket: f32,
    pub swing: f32,
}

impl ValveMap {
    /// Maps `primary_axes` (boom, arm, bucket, swing — see
    /// [`crate::control_command::DecodedControlCommand::primary_axes`])
    /// directly onto valve duties, clamped to the valid `-1.0..=1.0`
    /// commanded-flow-fraction range.
    pub fn from_primary_axes(primary_axes: &[f32; 4]) -> Self {
        ValveMap {
            boom: primary_axes[0].clamp(-1.0, 1.0),
            arm: primary_axes[1].clamp(-1.0, 1.0),
            bucket: primary_axes[2].clamp(-1.0, 1.0),
            swing: primary_axes[3].clamp(-1.0, 1.0),
        }
    }

    fn as_array(self) -> [f32; 4] {
        [self.boom, self.arm, self.bucket, self.swing]
    }

    fn from_array(a: [f32; 4]) -> Self {
        ValveMap {
            boom: a[0],
            arm: a[1],
            bucket: a[2],
            swing: a[3],
        }
    }
}

/// Applies pressure compensation (per axis, against that axis's own load
/// pressure) and then flow sharing (across all four axes, against the
/// pump's total available flow) to a raw [`ValveMap`], returning the
/// duties the valves should actually be commanded to.
///
/// `load_pressures_pa` and `pump_flow_capacity_m3_s` mirror
/// [`tpt_t_construction_hydraulic::pressure_compensated_flow_fraction`]
/// and [`tpt_t_construction_hydraulic::share_flow`]'s own parameters —
/// this function only sequences those two Phase-3 primitives across the
/// four implement axes, matching bridge spec.txt's intent that teleop
/// commands ride the same compensation/sharing path as autonomous ones.
pub fn compensated_valve_duties(
    raw: ValveMap,
    supply_pressure_pa: f32,
    load_pressures_pa: [f32; 4],
    min_margin_pa: f32,
    pump_flow_capacity_m3_s: f32,
    max_axis_flow_m3_s: f32,
) -> ValveMap {
    let raw_axes = raw.as_array();
    let mut demands_m3_s = [0.0f32; 4];
    for i in 0..4 {
        let fraction = pressure_compensated_flow_fraction(
            raw_axes[i].abs(),
            supply_pressure_pa,
            load_pressures_pa[i],
            min_margin_pa,
        );
        demands_m3_s[i] = fraction * max_axis_flow_m3_s;
    }

    let mut shared_m3_s = [0.0f32; 4];
    share_flow(&demands_m3_s, pump_flow_capacity_m3_s, &mut shared_m3_s);

    let mut out = [0.0f32; 4];
    for i in 0..4 {
        let signed_fraction = if max_axis_flow_m3_s > 0.0 {
            (shared_m3_s[i] / max_axis_flow_m3_s) * raw_axes[i].signum()
        } else {
            0.0
        };
        out[i] = signed_fraction;
    }
    ValveMap::from_array(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axes_map_directly_when_within_range() {
        let map = ValveMap::from_primary_axes(&[0.5, -0.5, 1.0, -1.0]);
        assert_eq!(
            map,
            ValveMap {
                boom: 0.5,
                arm: -0.5,
                bucket: 1.0,
                swing: -1.0,
            }
        );
    }

    #[test]
    fn axes_are_clamped_to_valid_range() {
        let map = ValveMap::from_primary_axes(&[2.0, -2.0, 0.0, 0.0]);
        assert_eq!(map.boom, 1.0);
        assert_eq!(map.arm, -1.0);
    }

    #[test]
    fn ample_pressure_and_flow_pass_commands_through_unchanged() {
        let raw = ValveMap {
            boom: 0.5,
            arm: -0.3,
            bucket: 0.2,
            swing: 0.0,
        };
        let out = compensated_valve_duties(raw, 300e5, [100e5; 4], 20e5, 1.0, 0.01);
        assert!((out.boom - 0.5).abs() < 1e-4);
        assert!((out.arm - -0.3).abs() < 1e-4);
        assert!((out.bucket - 0.2).abs() < 1e-4);
        assert_eq!(out.swing, 0.0);
    }

    #[test]
    fn constrained_pump_flow_scales_all_axes_down_proportionally() {
        let raw = ValveMap {
            boom: 1.0,
            arm: 1.0,
            bucket: 1.0,
            swing: 1.0,
        };
        // Each axis demands max_axis_flow_m3_s = 0.01, total demand 0.04,
        // but the pump can only supply 0.02 -> every axis should be
        // scaled to half.
        let out = compensated_valve_duties(raw, 300e5, [50e5; 4], 20e5, 0.02, 0.01);
        for duty in out.as_array() {
            assert!((duty - 0.5).abs() < 1e-4, "duty={duty}");
        }
    }

    #[test]
    fn load_at_supply_pressure_zeroes_that_axis_only() {
        let raw = ValveMap {
            boom: 1.0,
            arm: 1.0,
            bucket: 0.0,
            swing: 0.0,
        };
        let out = compensated_valve_duties(raw, 200e5, [200e5, 50e5, 0.0, 0.0], 20e5, 1.0, 0.01);
        assert_eq!(out.boom, 0.0);
        assert!(out.arm > 0.0);
    }

    #[test]
    fn direction_sign_is_preserved() {
        let raw = ValveMap {
            boom: -0.8,
            arm: 0.0,
            bucket: 0.0,
            swing: 0.0,
        };
        let out = compensated_valve_duties(raw, 300e5, [100e5; 4], 20e5, 1.0, 0.01);
        assert!(out.boom < 0.0);
    }
}
