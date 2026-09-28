// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Torque arbitration: the single point where every independent limit on
//! how much torque the engine may deliver right now — the operator's own
//! request, the engine's own capability at its current RPM, a traction
//! controller's slip limit, and thermal/emissions derating — gets
//! combined into the one number actually commanded.
//!
//! The rule is always "the lowest limit wins": no subsystem's request can
//! override another subsystem's safety limit. That's deliberate
//! defense-in-depth, the same principle spec.txt applies to proximity
//! detection and rollover protection — a limit only ever tightens the
//! envelope, never loosens what another limit already tightened.

/// Combines every independent torque limit into the torque (N*m) actually
/// commanded to the engine.
///
/// - `requested_nm`: what the operator/autonomy stack is asking for
///   (never allowed to go negative here; braking/retarding is a separate
///   path, not a negative torque request).
/// - `engine_max_nm`: the engine's own torque curve limit at its current
///   RPM (see `tpt-t-construction-sim`'s `EngineTorqueCurve` for the
///   plant-side equivalent).
/// - `traction_limit_nm`: the most torque the wheels/tracks can currently
///   put to the ground without exceeding the traction controller's slip
///   limit.
/// - `derate_factor`: `0.0..=1.0`, from [`crate::derate`] — how much of
///   the engine's rated capacity remains available under current thermal
///   conditions.
pub fn arbitrate_torque_nm(
    requested_nm: f32,
    engine_max_nm: f32,
    traction_limit_nm: f32,
    derate_factor: f32,
) -> f32 {
    let requested_nm = requested_nm.max(0.0);
    let engine_capacity_nm = engine_max_nm.max(0.0) * derate_factor.clamp(0.0, 1.0);
    let traction_limit_nm = traction_limit_nm.max(0.0);
    requested_nm.min(engine_capacity_nm).min(traction_limit_nm)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passes_through_request_when_nothing_else_binds() {
        assert_eq!(arbitrate_torque_nm(500.0, 2000.0, 2000.0, 1.0), 500.0);
    }

    #[test]
    fn engine_capacity_caps_an_overreaching_request() {
        assert_eq!(arbitrate_torque_nm(3000.0, 2000.0, 5000.0, 1.0), 2000.0);
    }

    #[test]
    fn traction_limit_caps_even_when_engine_has_headroom() {
        assert_eq!(arbitrate_torque_nm(3000.0, 5000.0, 800.0, 1.0), 800.0);
    }

    #[test]
    fn derate_factor_scales_down_engine_capacity() {
        assert_eq!(arbitrate_torque_nm(3000.0, 2000.0, 5000.0, 0.5), 1000.0);
    }

    #[test]
    fn negative_request_clamps_to_zero_rather_than_going_negative() {
        assert_eq!(arbitrate_torque_nm(-100.0, 2000.0, 2000.0, 1.0), 0.0);
    }

    #[test]
    fn out_of_range_derate_factor_is_clamped() {
        assert_eq!(arbitrate_torque_nm(3000.0, 2000.0, 5000.0, 1.5), 2000.0);
        assert_eq!(arbitrate_torque_nm(3000.0, 2000.0, 5000.0, -0.5), 0.0);
    }
}
