// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Generic byte-aligned signal extraction and physical-unit scaling —
//! the two steps every J1939 SPN decode in [`crate::spn`] is built from.
//! Scope note: this handles the common byte-aligned case (a signal that
//! starts on a byte boundary and spans a whole number of bytes,
//! little-endian, which is the large majority of J1939 SPNs); arbitrary
//! bit-offset/bit-length signals (a handful of packed status/enum SPNs)
//! are out of scope here and would need a bit-level variant added
//! alongside this one, not in place of it.

/// Extracts an unsigned integer (little-endian, up to 4 bytes) starting
/// at `start_byte` of `data`. Returns `None` if `length_bytes` is `0`,
/// greater than `4`, or the requested range runs past the end of `data`
/// — never panics on malformed/short input, since a corrupted or
/// truncated CAN frame is exactly the kind of input this has to survive.
pub fn extract_unsigned(data: &[u8], start_byte: usize, length_bytes: usize) -> Option<u32> {
    if length_bytes == 0 || length_bytes > 4 {
        return None;
    }
    if start_byte.checked_add(length_bytes)? > data.len() {
        return None;
    }
    let mut value: u32 = 0;
    for i in 0..length_bytes {
        value |= (data[start_byte + i] as u32) << (8 * i);
    }
    Some(value)
}

/// The all-ones sentinel J1939 uses for "parameter not available" at a
/// given byte length (e.g. `0xFFFF` for a 2-byte field).
pub fn not_available_sentinel(length_bytes: usize) -> u32 {
    if length_bytes >= 4 {
        u32::MAX
    } else {
        (1u32 << (8 * length_bytes)) - 1
    }
}

/// Converts a raw extracted integer to its physical value via J1939's
/// standard `physical = raw * resolution + offset` scaling.
pub fn scale(raw: u32, resolution: f32, offset: f32) -> f32 {
    raw as f32 * resolution + offset
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extracts_a_single_byte() {
        assert_eq!(extract_unsigned(&[0x11, 0x22, 0x33], 1, 1), Some(0x22));
    }

    #[test]
    fn extracts_multiple_bytes_little_endian() {
        // bytes [0x20, 0x4E] as a little-endian u16 -> 0x4E20
        assert_eq!(extract_unsigned(&[0x20, 0x4E], 0, 2), Some(0x4E20));
    }

    #[test]
    fn out_of_range_start_returns_none() {
        assert_eq!(extract_unsigned(&[0x01, 0x02], 1, 2), None);
    }

    #[test]
    fn zero_or_oversized_length_returns_none() {
        assert_eq!(extract_unsigned(&[0x01], 0, 0), None);
        assert_eq!(extract_unsigned(&[0u8; 8], 0, 5), None);
    }

    #[test]
    fn not_available_sentinel_matches_byte_length() {
        assert_eq!(not_available_sentinel(1), 0xFF);
        assert_eq!(not_available_sentinel(2), 0xFFFF);
        assert_eq!(not_available_sentinel(4), 0xFFFF_FFFF);
    }

    #[test]
    fn scale_applies_resolution_and_offset() {
        assert_eq!(scale(125, 1.0, -40.0), 85.0);
        assert!((scale(20000, 0.125, 0.0) - 2500.0).abs() < 1e-3);
    }
}
