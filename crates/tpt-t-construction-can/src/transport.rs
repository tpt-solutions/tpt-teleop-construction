// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! J1939 Transport Protocol (SAE J1939-21 §5.10) reassembly for messages
//! longer than one CAN frame's 8 data bytes (up to 1785 bytes across up
//! to 255 packets). This implements the **Broadcast Announce Message
//! (BAM)** variant only — the connectionless, one-to-many transfer used
//! for most broadcast PGNs — via [`BamReassembler`], a fixed-capacity,
//! zero-allocation reassembly buffer sized by the caller's `MAX_BYTES`
//! const generic, matching the rest of this workspace's zero-alloc,
//! fixed-capacity container convention (`tpt-t-construction-hydraulic::SensorFrame<N>`,
//! `tpt-t-construction-payload::PointCloud<N>`, and others).
//!
//! **Scope note**: the connection-mode transport (RTS/CTS, peer-to-peer,
//! flow-controlled) is a different state machine with its own
//! back-pressure and abort handling, and isn't implemented here —
//! [`TransportError::UnsupportedControlByte`] is returned for any TP.CM
//! control byte other than BAM's `0x20`, rather than silently
//! misinterpreting it.

use crate::signal::extract_unsigned;

/// PGN of the Transport Protocol Connection Management message (carries
/// the BAM announcement this module handles).
pub const TP_CM_PGN: u32 = 60416;
/// PGN of the Transport Protocol Data Transfer message (carries each
/// 7-byte chunk of the reassembled payload).
pub const TP_DT_PGN: u32 = 60160;

const BAM_CONTROL_BYTE: u8 = 0x20;

/// Why a [`BamReassembler`] call was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    /// The TP.CM payload was shorter than the 8 bytes a control message
    /// always carries.
    TruncatedControlMessage,
    /// A TP.CM control byte other than BAM (`0x20`) — see the module
    /// docs' scope note.
    UnsupportedControlByte(u8),
    /// The announced total message size exceeds this reassembler's fixed
    /// buffer capacity.
    MessageTooLargeForBuffer { requested: usize, capacity: usize },
    /// A TP.DT arrived with no BAM announcement having been processed
    /// first (or after the previous transfer already completed).
    NoTransferInProgress,
    /// The TP.DT payload was empty (no sequence-number byte at all).
    TruncatedDataMessage,
    /// A TP.DT's sequence number wasn't the next one expected — a
    /// dropped or reordered packet. The partial transfer is abandoned;
    /// the next BAM restarts it.
    OutOfOrderSequence { expected: u8, received: u8 },
}

/// Reassembles one BAM multi-packet transfer at a time into a
/// fixed-capacity, caller-sized buffer. Construct with
/// [`BamReassembler::new`], feed the PGN-60416 control message to
/// [`Self::on_control_message`], then each PGN-60160 data message to
/// [`Self::on_data_message`] — the latter returns the complete
/// reassembled payload once every packet has arrived.
pub struct BamReassembler<const MAX_BYTES: usize> {
    buffer: [u8; MAX_BYTES],
    total_size: usize,
    next_sequence: u8,
    bytes_received: usize,
    target_pgn: u32,
    in_progress: bool,
}

impl<const MAX_BYTES: usize> Default for BamReassembler<MAX_BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_BYTES: usize> BamReassembler<MAX_BYTES> {
    pub fn new() -> Self {
        BamReassembler {
            buffer: [0u8; MAX_BYTES],
            total_size: 0,
            next_sequence: 1,
            bytes_received: 0,
            target_pgn: 0,
            in_progress: false,
        }
    }

    /// The PGN of the message currently being (or most recently)
    /// reassembled, as announced by the last accepted BAM.
    pub fn target_pgn(&self) -> u32 {
        self.target_pgn
    }

    /// Starts a new reassembly from a TP.CM (PGN 60416) payload. Any
    /// transfer already in progress is discarded — a new BAM always
    /// means a new transfer, never a resumption.
    pub fn on_control_message(&mut self, payload: &[u8]) -> Result<(), TransportError> {
        if payload.len() < 8 {
            return Err(TransportError::TruncatedControlMessage);
        }
        let control_byte = payload[0];
        if control_byte != BAM_CONTROL_BYTE {
            return Err(TransportError::UnsupportedControlByte(control_byte));
        }
        // SAFETY of the `.unwrap()`s below: the length check above
        // guarantees `payload` is at least 8 bytes, and every
        // `extract_unsigned` call here requests a range within that.
        let total_size = extract_unsigned(payload, 1, 2).unwrap() as usize;
        let pgn = extract_unsigned(payload, 5, 3).unwrap();

        if total_size > MAX_BYTES {
            return Err(TransportError::MessageTooLargeForBuffer {
                requested: total_size,
                capacity: MAX_BYTES,
            });
        }

        self.total_size = total_size;
        self.target_pgn = pgn;
        self.next_sequence = 1;
        self.bytes_received = 0;
        self.in_progress = true;
        Ok(())
    }

    /// Feeds one TP.DT (PGN 60160) payload. Returns `Ok(Some(bytes))`
    /// with the complete reassembled message once the last packet
    /// arrives, `Ok(None)` while more packets are still expected.
    pub fn on_data_message(&mut self, payload: &[u8]) -> Result<Option<&[u8]>, TransportError> {
        if !self.in_progress {
            return Err(TransportError::NoTransferInProgress);
        }
        if payload.is_empty() {
            return Err(TransportError::TruncatedDataMessage);
        }
        let sequence = payload[0];
        if sequence != self.next_sequence {
            self.in_progress = false;
            return Err(TransportError::OutOfOrderSequence {
                expected: self.next_sequence,
                received: sequence,
            });
        }

        let chunk = &payload[1..];
        let remaining = self.total_size - self.bytes_received;
        let take = remaining.min(chunk.len()).min(7);
        self.buffer[self.bytes_received..self.bytes_received + take]
            .copy_from_slice(&chunk[..take]);
        self.bytes_received += take;
        self.next_sequence = self.next_sequence.wrapping_add(1);

        if self.bytes_received >= self.total_size {
            self.in_progress = false;
            Ok(Some(&self.buffer[..self.total_size]))
        } else {
            Ok(None)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bam_control_message(total_size: u16, total_packets: u8, pgn: u32) -> [u8; 8] {
        let size_bytes = total_size.to_le_bytes();
        let pgn_bytes = pgn.to_le_bytes();
        [
            BAM_CONTROL_BYTE,
            size_bytes[0],
            size_bytes[1],
            total_packets,
            0xFF,
            pgn_bytes[0],
            pgn_bytes[1],
            pgn_bytes[2],
        ]
    }

    #[test]
    fn reassembles_a_message_spanning_three_packets() {
        // 16 bytes across ceil(16/7) = 3 packets.
        let mut reassembler = BamReassembler::<32>::new();
        reassembler
            .on_control_message(&bam_control_message(16, 3, 65280))
            .unwrap();
        assert_eq!(reassembler.target_pgn(), 65280);

        assert_eq!(
            reassembler
                .on_data_message(&[1, 0, 1, 2, 3, 4, 5, 6])
                .unwrap(),
            None
        );
        assert_eq!(
            reassembler
                .on_data_message(&[2, 7, 8, 9, 10, 11, 12, 13])
                .unwrap(),
            None
        );
        let complete = reassembler
            .on_data_message(&[3, 14, 15, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
            .unwrap()
            .unwrap();
        assert_eq!(complete, (0u8..16).collect::<Vec<u8>>().as_slice());
    }

    #[test]
    fn rejects_a_message_larger_than_the_buffer() {
        let mut reassembler = BamReassembler::<8>::new();
        let err = reassembler
            .on_control_message(&bam_control_message(16, 3, 0))
            .unwrap_err();
        assert_eq!(
            err,
            TransportError::MessageTooLargeForBuffer {
                requested: 16,
                capacity: 8
            }
        );
    }

    #[test]
    fn rejects_a_non_bam_control_byte() {
        let mut reassembler = BamReassembler::<32>::new();
        let mut msg = bam_control_message(16, 3, 0);
        msg[0] = 0x10; // RTS, not BAM
        let err = reassembler.on_control_message(&msg).unwrap_err();
        assert_eq!(err, TransportError::UnsupportedControlByte(0x10));
    }

    #[test]
    fn data_message_without_a_prior_bam_is_rejected() {
        let mut reassembler = BamReassembler::<32>::new();
        let err = reassembler
            .on_data_message(&[1, 0, 0, 0, 0, 0, 0, 0])
            .unwrap_err();
        assert_eq!(err, TransportError::NoTransferInProgress);
    }

    #[test]
    fn out_of_order_sequence_is_detected_and_abandons_the_transfer() {
        let mut reassembler = BamReassembler::<32>::new();
        reassembler
            .on_control_message(&bam_control_message(16, 3, 0))
            .unwrap();
        reassembler
            .on_data_message(&[1, 0, 1, 2, 3, 4, 5, 6])
            .unwrap();
        let err = reassembler
            .on_data_message(&[3, 7, 8, 9, 10, 11, 12, 13]) // skipped sequence 2
            .unwrap_err();
        assert_eq!(
            err,
            TransportError::OutOfOrderSequence {
                expected: 2,
                received: 3
            }
        );
        // The abandoned transfer no longer accepts further data.
        assert_eq!(
            reassembler
                .on_data_message(&[2, 0, 0, 0, 0, 0, 0, 0])
                .unwrap_err(),
            TransportError::NoTransferInProgress
        );
    }

    #[test]
    fn a_new_bam_discards_a_transfer_already_in_progress() {
        let mut reassembler = BamReassembler::<32>::new();
        reassembler
            .on_control_message(&bam_control_message(16, 3, 111))
            .unwrap();
        reassembler
            .on_data_message(&[1, 0, 1, 2, 3, 4, 5, 6])
            .unwrap();

        // A fresh BAM for a different, shorter message resets everything.
        reassembler
            .on_control_message(&bam_control_message(4, 1, 222))
            .unwrap();
        assert_eq!(reassembler.target_pgn(), 222);
        let complete = reassembler
            .on_data_message(&[1, 0xAA, 0xBB, 0xCC, 0xDD, 0xFF, 0xFF, 0xFF])
            .unwrap()
            .unwrap();
        assert_eq!(complete, &[0xAA, 0xBB, 0xCC, 0xDD]);
    }
}
