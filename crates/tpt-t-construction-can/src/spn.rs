// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Known Suspect Parameter Number (SPN) decoders for the two engine
//! signals this workspace's telemetry/powertrain crates already track
//! (`engine_rpm` in `tpt-t-construction-telemetry::record::TelemetryRecord`
//! and `tpt-t-construction-core::messages::TelemetrySample`): a real
//! deployment would read these off the machine's J1939 bus instead of a
//! simulated value. This is a representative pair of decoders, not a
//! full SAE J1939-71 SPN database — every decoder here follows the same
//! two-step pattern ([`crate::signal::extract_unsigned`] then
//! [`crate::signal::scale`]), so adding another known SPN is
//! straightforward but out of scope until a caller actually needs one.

use crate::signal::{extract_unsigned, not_available_sentinel, scale};

/// Electronic Engine Controller 1 — carries SPN 190 (Engine Speed).
pub const PGN_EEC1: u32 = 61444;
/// Engine Temperature 1 — carries SPN 110 (Engine Coolant Temperature).
pub const PGN_ET1: u32 = 65262;

/// A decoded SPN value, distinguishing a real reading from the bus
/// explicitly reporting the parameter as unavailable (the sender is
/// present but doesn't have this value, e.g. a sensor fault) — a caller
/// should not treat `NotAvailable` as zero.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpnValue {
    Value(f32),
    NotAvailable,
}

/// SPN 190, Engine Speed: bytes 4-5 (0-indexed 3-4) of a PGN 61444
/// (EEC1) payload, little-endian, 0.125 rpm/bit, no offset.
///
/// Returns `None` (not even `NotAvailable`) if `payload` is too short to
/// contain the field at all — a malformed frame, distinct from a
/// well-formed frame explicitly reporting no data.
pub fn engine_speed_rpm(payload: &[u8]) -> Option<SpnValue> {
    let raw = extract_unsigned(payload, 3, 2)?;
    if raw == not_available_sentinel(2) {
        return Some(SpnValue::NotAvailable);
    }
    Some(SpnValue::Value(scale(raw, 0.125, 0.0)))
}

/// SPN 110, Engine Coolant Temperature: byte 1 (0-indexed 0) of a PGN
/// 65262 (ET1) payload, 1 degC/bit, -40 degC offset.
pub fn engine_coolant_temp_c(payload: &[u8]) -> Option<SpnValue> {
    let raw = extract_unsigned(payload, 0, 1)?;
    if raw == not_available_sentinel(1) {
        return Some(SpnValue::NotAvailable);
    }
    Some(SpnValue::Value(scale(raw, 1.0, -40.0)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_known_engine_speed_reading() {
        // 2500 rpm = 20000 raw = 0x4E20, little-endian bytes [0x20, 0x4E]
        // at byte offset 3-4 of an 8-byte EEC1 payload.
        let payload = [0xFF, 0xFF, 0xFF, 0x20, 0x4E, 0xFF, 0xFF, 0xFF];
        assert_eq!(engine_speed_rpm(&payload), Some(SpnValue::Value(2500.0)));
    }

    #[test]
    fn engine_speed_not_available_sentinel_is_reported_explicitly() {
        let payload = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(engine_speed_rpm(&payload), Some(SpnValue::NotAvailable));
    }

    #[test]
    fn engine_speed_returns_none_for_a_truncated_payload() {
        let payload = [0x00, 0x00, 0x00];
        assert_eq!(engine_speed_rpm(&payload), None);
    }

    #[test]
    fn decodes_a_known_coolant_temperature_reading() {
        // 85 degC = raw 125 = 0x7D at byte 0.
        let payload = [0x7D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(engine_coolant_temp_c(&payload), Some(SpnValue::Value(85.0)));
    }

    #[test]
    fn coolant_temperature_handles_the_below_zero_offset() {
        // raw 0 -> -40 degC, the coldest representable reading.
        let payload = [0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(
            engine_coolant_temp_c(&payload),
            Some(SpnValue::Value(-40.0))
        );
    }

    #[test]
    fn known_pgns_match_the_sae_assigned_numbers() {
        assert_eq!(PGN_EEC1, 61444);
        assert_eq!(PGN_ET1, 65262);
    }
}
