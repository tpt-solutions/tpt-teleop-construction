// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The fixed-size telemetry record logged at 100Hz (spec.txt §8). A
//! plain `#[repr(C)]` struct of fixed-width numeric fields has a
//! constant, predictable in-memory layout, so converting to/from bytes is
//! a direct reinterpretation — zero-copy in both directions, no
//! serialization format or parser needed. This is a different (and
//! simpler) approach than `tpt-t-construction-core`'s `rkyv`-archived
//! `TelemetrySample`: that type is a generic, extensible bus/wire
//! message; this one is the actual fixed 100Hz-logged record, where a
//! raw fixed-stride binary layout is exactly what a real telemetry logger
//! writes to disk.

/// One 100Hz sample of the machine's key parameters.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TelemetryRecord {
    pub timestamp_us: u64,
    pub hydraulic_pressure_pa: f32,
    pub engine_rpm: f32,
    pub ground_speed_m_s: f32,
    pub payload_kg: f32,
}

impl TelemetryRecord {
    /// The on-disk size of one record, in bytes.
    pub const SIZE: usize = std::mem::size_of::<TelemetryRecord>();

    /// Reinterprets this record as its raw bytes, with no copying beyond
    /// the returned array itself.
    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        // SAFETY: `TelemetryRecord` is `#[repr(C)]` and contains only
        // fixed-width numeric fields (no padding-sensitive niches, no
        // pointers), so every bit pattern of its `SIZE` bytes is a valid
        // `TelemetryRecord` and vice versa; reading it byte-for-byte via
        // `transmute_copy` cannot produce an invalid value.
        unsafe { std::mem::transmute_copy(&self) }
    }

    /// Reconstructs a record from exactly `Self::SIZE` bytes previously
    /// produced by [`Self::to_bytes`] (or read from the same offset in a
    /// log file this crate wrote).
    ///
    /// # Panics
    ///
    /// Panics if `bytes.len() != Self::SIZE`.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(
            bytes.len(),
            Self::SIZE,
            "TelemetryRecord::from_bytes needs exactly SIZE bytes"
        );
        // SAFETY: `bytes` is exactly `SIZE` bytes (checked above), and
        // `read_unaligned` tolerates any alignment of `bytes.as_ptr()`,
        // so this is a sound reinterpretation of the same POD layout
        // `to_bytes` produced.
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const TelemetryRecord) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> TelemetryRecord {
        TelemetryRecord {
            timestamp_us: 123_456_789,
            hydraulic_pressure_pa: 2.1e7,
            engine_rpm: 1850.0,
            ground_speed_m_s: 6.3,
            payload_kg: 42_000.0,
        }
    }

    #[test]
    fn round_trips_through_bytes() {
        let record = sample();
        let bytes = record.to_bytes();
        let recovered = TelemetryRecord::from_bytes(&bytes);
        assert_eq!(record, recovered);
    }

    #[test]
    fn size_is_constant_across_different_values() {
        let a = sample();
        let mut b = sample();
        b.engine_rpm = 999.0;
        b.payload_kg = 0.0;
        assert_eq!(a.to_bytes().len(), b.to_bytes().len());
    }

    #[test]
    #[should_panic(expected = "exactly SIZE bytes")]
    fn from_bytes_rejects_wrong_length() {
        TelemetryRecord::from_bytes(&[0u8; 3]);
    }
}
