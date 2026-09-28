// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! J1939's 29-bit identifier layout (SAE J1939-21) and Parameter Group
//! Number (PGN) extraction. Bit layout, MSB to LSB within the 29 bits:
//!
//! ```text
//! bits 28-26: Priority (3 bits, 0 = highest)
//! bit  25:    Reserved (R / "EDP", almost always 0)
//! bit  24:    Data Page (DP)
//! bits 23-16: PDU Format (PF)
//! bits 15-8:  PDU Specific (PS) — a destination address if PF < 240
//!             ("PDU1", peer-to-peer), or a Group Extension that's part
//!             of the PGN itself if PF >= 240 ("PDU2", broadcast-only)
//! bits 7-0:   Source Address (SA)
//! ```

/// A decoded J1939 29-bit identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct J1939Id {
    pub priority: u8,
    pub reserved: bool,
    pub data_page: bool,
    pub pdu_format: u8,
    pub pdu_specific: u8,
    pub source_address: u8,
}

impl J1939Id {
    /// Decodes the low 29 bits of a raw extended CAN identifier.
    pub fn from_raw(raw_id: u32) -> Self {
        J1939Id {
            priority: ((raw_id >> 26) & 0x7) as u8,
            reserved: (raw_id >> 25) & 0x1 != 0,
            data_page: (raw_id >> 24) & 0x1 != 0,
            pdu_format: ((raw_id >> 16) & 0xFF) as u8,
            pdu_specific: ((raw_id >> 8) & 0xFF) as u8,
            source_address: (raw_id & 0xFF) as u8,
        }
    }

    /// The Parameter Group Number this identifier addresses.
    ///
    /// PDU1 format (`pdu_format < 240`, peer-to-peer): the PGN excludes
    /// `pdu_specific` (that field is the destination address instead).
    /// PDU2 format (`pdu_format >= 240`, broadcast-only): `pdu_specific`
    /// is a Group Extension and *is* part of the PGN.
    pub fn pgn(&self) -> u32 {
        let dp = (self.data_page as u32) << 16;
        let pf = (self.pdu_format as u32) << 8;
        if self.pdu_format < 240 {
            dp | pf
        } else {
            dp | pf | self.pdu_specific as u32
        }
    }

    /// The destination address for a PDU1 (peer-to-peer) message, or
    /// `None` for a PDU2 (broadcast-only) message, where the same bits
    /// are a Group Extension instead of an address.
    pub fn destination_address(&self) -> Option<u8> {
        if self.pdu_format < 240 {
            Some(self.pdu_specific)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_pdu2_broadcast_identifier() {
        // Engine Speed (EEC1), PGN 61444 (0xF004), priority 3, source 0.
        // 29-bit id: priority=3 (011) R=0 DP=0 PF=0xF0 PS=0x04 SA=0x00
        let raw = (0b011u32 << 26) | (0xF0 << 16) | (0x04 << 8);
        let id = J1939Id::from_raw(raw);
        assert_eq!(id.priority, 3);
        assert!(!id.reserved);
        assert!(!id.data_page);
        assert_eq!(id.pdu_format, 0xF0);
        assert_eq!(id.pdu_specific, 0x04);
        assert_eq!(id.source_address, 0x00);
        assert_eq!(id.pgn(), 61444);
        assert_eq!(id.destination_address(), None);
    }

    #[test]
    fn decodes_a_pdu1_peer_to_peer_identifier_with_destination_address() {
        // PF = 0xE8 (232, < 240) => PDU1, PS is a destination address,
        // not part of the PGN.
        let raw = (0b110u32 << 26) | (0xE8 << 16) | (0x7B << 8) | 0x11;
        let id = J1939Id::from_raw(raw);
        assert_eq!(id.priority, 6);
        assert_eq!(id.pdu_format, 0xE8);
        assert_eq!(id.destination_address(), Some(0x7B));
        // PGN excludes PS entirely for PDU1.
        assert_eq!(id.pgn(), (0xE8u32) << 8);
    }

    #[test]
    fn data_page_bit_shifts_the_pgn() {
        let raw_dp0 = (0xF0u32 << 16) | (0x04 << 8);
        let raw_dp1 = (1u32 << 24) | (0xF0 << 16) | (0x04 << 8);
        assert_eq!(J1939Id::from_raw(raw_dp0).pgn(), 61444);
        assert_eq!(J1939Id::from_raw(raw_dp1).pgn(), 61444 + (1 << 16));
    }

    #[test]
    fn source_address_round_trips() {
        let raw = 0xABu32;
        assert_eq!(J1939Id::from_raw(raw).source_address, 0xAB);
    }
}
