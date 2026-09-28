// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Pressure compensation, flow sharing, and load-sensing logic (spec.txt
//! §3's "pressure compensation, flow sharing, and load-sensing logic").
//!
//! Every function here is a pure calculation over caller-provided
//! slices/buffers — no allocation, so the 1kHz loop can call these every
//! tick with fixed, pre-allocated arrays.

/// The fraction (`0.0..=1.0`) of a valve's commanded flow that can actually
/// be delivered given the current supply/load pressure margin.
///
/// A real pressure compensator holds flow constant across a range of load
/// pressures by trimming its own orifice as the margin shrinks; once the
/// margin drops below `min_margin_pa`, delivered flow is throttled down
/// proportionally to how much margin remains, reaching zero once the load
/// pressure meets or exceeds supply (the actuator can no longer move).
pub fn pressure_compensated_flow_fraction(
    demanded_fraction: f32,
    supply_pressure_pa: f32,
    load_pressure_pa: f32,
    min_margin_pa: f32,
) -> f32 {
    let demanded_fraction = demanded_fraction.clamp(0.0, 1.0);
    let margin_pa = supply_pressure_pa - load_pressure_pa;
    if margin_pa >= min_margin_pa {
        demanded_fraction
    } else if margin_pa <= 0.0 {
        0.0
    } else {
        demanded_fraction * (margin_pa / min_margin_pa)
    }
}

/// Splits `available_m3_s` of pump flow across simultaneous actuator
/// demands, writing each actuator's actually-delivered flow into
/// `out_m3_s` (same length as `demands_m3_s`, no allocation).
///
/// If total demand fits within what's available, every actuator gets
/// exactly what it asked for. Otherwise every actuator's share is scaled
/// down by the same factor (`available / total_demand`) — proportional
/// flow sharing, so no single actuator starves the others but none gets
/// more than its fair share of a constrained pump either.
///
/// # Panics
///
/// Panics if `out_m3_s` is not the same length as `demands_m3_s` — a
/// caller bug, not a runtime condition.
pub fn share_flow(demands_m3_s: &[f32], available_m3_s: f32, out_m3_s: &mut [f32]) {
    assert_eq!(
        demands_m3_s.len(),
        out_m3_s.len(),
        "out_m3_s must have one slot per demand"
    );
    let total_demand_m3_s: f32 = demands_m3_s.iter().sum();
    if total_demand_m3_s <= available_m3_s || total_demand_m3_s <= 0.0 {
        out_m3_s.copy_from_slice(demands_m3_s);
    } else {
        let scale = available_m3_s / total_demand_m3_s;
        for (out, &demand) in out_m3_s.iter_mut().zip(demands_m3_s) {
            *out = demand * scale;
        }
    }
}

/// The pump compensator setpoint a load-sensing system should hold:
/// the highest current actuator load pressure, plus `margin_pa` to keep
/// enough pressure drop across every valve for it to still control flow.
/// Returns `margin_pa` alone if `load_pressures_pa` is empty (no load,
/// pump only needs to hold standby pressure).
pub fn load_sense_pressure(load_pressures_pa: &[f32], margin_pa: f32) -> f32 {
    load_pressures_pa.iter().copied().fold(0.0_f32, f32::max) + margin_pa
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_flow_delivered_with_ample_margin() {
        let fraction = pressure_compensated_flow_fraction(0.8, 300e5, 100e5, 20e5);
        assert_eq!(fraction, 0.8);
    }

    #[test]
    fn zero_flow_when_load_meets_or_exceeds_supply() {
        assert_eq!(
            pressure_compensated_flow_fraction(1.0, 200e5, 200e5, 20e5),
            0.0
        );
        assert_eq!(
            pressure_compensated_flow_fraction(1.0, 150e5, 200e5, 20e5),
            0.0
        );
    }

    #[test]
    fn flow_throttles_down_as_margin_shrinks() {
        let half_margin = pressure_compensated_flow_fraction(1.0, 210e5, 200e5, 20e5);
        assert!((half_margin - 0.5).abs() < 1e-4);
    }

    #[test]
    fn share_flow_gives_full_demand_when_supply_is_ample() {
        let demands = [0.001, 0.002, 0.0015];
        let mut out = [0.0; 3];
        share_flow(&demands, 0.01, &mut out);
        assert_eq!(out, demands);
    }

    #[test]
    fn share_flow_scales_down_proportionally_when_constrained() {
        let demands = [0.002, 0.004, 0.002]; // total 0.008
        let mut out = [0.0; 3];
        share_flow(&demands, 0.004, &mut out); // half of total available
        for (o, d) in out.iter().zip(demands) {
            assert!((o / d - 0.5).abs() < 1e-6);
        }
        let total_out: f32 = out.iter().sum();
        assert!((total_out - 0.004).abs() < 1e-6);
    }

    #[test]
    #[should_panic(expected = "one slot per demand")]
    fn share_flow_panics_on_mismatched_lengths() {
        let demands = [0.001, 0.002];
        let mut out = [0.0; 3];
        share_flow(&demands, 0.01, &mut out);
    }

    #[test]
    fn load_sense_pressure_tracks_the_highest_load_plus_margin() {
        let loads = [50e5, 220e5, 100e5];
        assert_eq!(load_sense_pressure(&loads, 20e5), 240e5);
    }

    #[test]
    fn load_sense_pressure_with_no_actuators_is_just_the_margin() {
        assert_eq!(load_sense_pressure(&[], 20e5), 20e5);
    }
}
