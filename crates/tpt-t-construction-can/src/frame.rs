// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A raw CAN frame as it comes off the bus: a 29-bit extended identifier
//! (J1939 always uses the extended format, never the 11-bit standard
//! one) and up to 8 data bytes. This crate never allocates to represent
//! or parse one — [`CanFrame`] is `Copy`, and every function operating
//! on it borrows or copies fixed-size data.

/// One classic CAN 2.0B frame: a 29-bit extended identifier plus up to 8
/// bytes of data (`dlc` is the *valid* prefix length of `data`; bytes
/// past `dlc` are unspecified, not zeroed).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    /// The 29-bit extended CAN identifier (only the low 29 bits are
    /// meaningful; higher bits are ignored by every function here).
    pub id: u32,
    /// Data length code: the number of valid bytes in `data`, `0..=8`.
    pub dlc: u8,
    pub data: [u8; 8],
}

impl CanFrame {
    /// The valid portion of `data`, as a slice of length `dlc`.
    ///
    /// # Panics
    ///
    /// Panics if `dlc > 8` — a malformed frame, not a runtime condition
    /// a real CAN controller can produce.
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_is_exactly_dlc_bytes() {
        let frame = CanFrame {
            id: 0x18FEF100,
            dlc: 3,
            data: [1, 2, 3, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA],
        };
        assert_eq!(frame.payload(), &[1, 2, 3]);
    }

    #[test]
    fn zero_length_payload_is_empty() {
        let frame = CanFrame {
            id: 0,
            dlc: 0,
            data: [0; 8],
        };
        assert!(frame.payload().is_empty());
    }
}
