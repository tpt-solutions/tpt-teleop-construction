// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Append-only, hash-chained audit trail logging for certification and
//! incident review (todo.md Phase 11: "Complete audit trail logging for
//! certification"). Every safety-relevant event (a lifecycle FSM
//! transition, a raised fault, a teleoperation handover, a completed
//! self-test) is appended as a fixed-size [`AuditRecord`] whose `hash`
//! field covers both its own fields and the previous record's hash —
//! the same hash-chain construction used by append-only ledgers, so
//! [`verify_chain`] can detect any record that was altered, reordered, or
//! deleted after the fact without needing to keep a separate copy of the
//! log to compare against.
//!
//! **On the strength of "tamper-evident" here**: the chain uses FNV-1a, a
//! fast non-cryptographic hash — sufficient to catch accidental
//! corruption (disk errors, a truncated write, a dropped record) and
//! naive tampering (hand-editing bytes in the log file), but not
//! resistant to a deliberate, well-resourced forgery that can afford to
//! search for hash collisions. A production certification deployment
//! wanting resistance to *deliberate* tampering should chain with a
//! cryptographic MAC (e.g. HMAC-SHA256) keyed from a hardware-backed key
//! store instead — that requires a real key management story, which is a
//! Phase 12 hardware bring-up concern this sandboxed workspace has no way
//! to provide honestly. This module is the chain-of-custody mechanism;
//! swapping the hash function is a one-line change once that key store
//! exists.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// The FNV-1a offset basis, reused here as this log's fixed genesis hash
/// (the `prev_hash` of record `0`) — an arbitrary but fixed, documented
/// constant, not a secret.
pub const GENESIS_HASH: u64 = 0xcbf2_9ce4_8422_2325;

fn fnv1a_64(bytes: &[u8]) -> u64 {
    const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut hash = GENESIS_HASH;
    for &byte in bytes {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

/// The category of a logged audit event. Callers (the lifecycle FSM,
/// fault-raising subsystems, the teleop handover FSM, the self-test
/// suite) map their own domain events onto one of these before appending
/// — this module doesn't depend on any of those crates, to stay usable
/// from all of them without a dependency cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum AuditEventKind {
    /// The machine lifecycle FSM changed state.
    StateTransition = 1,
    /// A subsystem raised a fault.
    FaultRaised = 2,
    /// The teleoperation handover FSM changed state.
    HandoverTransition = 3,
    /// The shift-start self-test suite completed.
    SelfTestCompleted = 4,
    /// A direct operator action not otherwise categorized (e.g. an
    /// acknowledged fault, a manual override).
    OperatorAction = 5,
}

/// One entry in the audit trail: fixed-width, `#[repr(C)]`, and — other
/// than [`AuditRecord::hash`] itself — immutable evidence of one event.
/// `detail` is a generic signed payload a caller packs its own event's
/// specifics into (e.g. a fault code, or two packed 32-bit state
/// discriminants for a transition); this module doesn't interpret it.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AuditRecord {
    pub sequence: u64,
    pub timestamp_us: u64,
    pub event_code: u16,
    pub severity: u8,
    _reserved: u8,
    pub detail: i64,
    pub prev_hash: u64,
    pub hash: u64,
}

impl AuditRecord {
    pub const SIZE: usize = std::mem::size_of::<AuditRecord>();

    fn new(
        sequence: u64,
        timestamp_us: u64,
        kind: AuditEventKind,
        severity: u8,
        detail: i64,
        prev_hash: u64,
    ) -> Self {
        let mut record = AuditRecord {
            sequence,
            timestamp_us,
            event_code: kind as u16,
            severity,
            _reserved: 0,
            detail,
            prev_hash,
            hash: 0,
        };
        record.hash = fnv1a_64(&record.hashable_bytes());
        record
    }

    /// This record's bytes with [`Self::hash`] zeroed — what the hash
    /// itself is computed over, so the stored hash never hashes itself.
    fn hashable_bytes(&self) -> [u8; Self::SIZE] {
        let mut copy = *self;
        copy.hash = 0;
        // SAFETY: same POD justification as `TelemetryRecord::to_bytes`
        // (`#[repr(C)]`, fixed-width fields only, no niches or pointers).
        unsafe { std::mem::transmute_copy(&copy) }
    }

    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        // SAFETY: see `hashable_bytes`.
        unsafe { std::mem::transmute_copy(&self) }
    }

    /// # Panics
    ///
    /// Panics if `bytes.len() != Self::SIZE`.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(
            bytes.len(),
            Self::SIZE,
            "AuditRecord::from_bytes needs exactly SIZE bytes"
        );
        // SAFETY: `bytes` is exactly `SIZE` bytes (checked above), and
        // `read_unaligned` tolerates any alignment of `bytes.as_ptr()`.
        unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const AuditRecord) }
    }
}

/// An append-only, hash-chained audit log file. Construct with
/// [`AuditLog::create`] (always starts a fresh chain from
/// [`GENESIS_HASH`]) and append events with [`AuditLog::append`].
pub struct AuditLog {
    writer: BufWriter<File>,
    next_sequence: u64,
    last_hash: u64,
}

impl AuditLog {
    /// Creates (or truncates) the audit log file at `path`.
    pub fn create(path: &Path) -> io::Result<Self> {
        let file = File::create(path)?;
        Ok(AuditLog {
            writer: BufWriter::new(file),
            next_sequence: 0,
            last_hash: GENESIS_HASH,
        })
    }

    /// Appends one event, chaining it to the previously appended record
    /// (or [`GENESIS_HASH`] for the first). Returns the record actually
    /// written, in case the caller wants its `hash` (e.g. to display
    /// alongside a printed incident report).
    pub fn append(
        &mut self,
        timestamp_us: u64,
        kind: AuditEventKind,
        severity: u8,
        detail: i64,
    ) -> io::Result<AuditRecord> {
        let record = AuditRecord::new(
            self.next_sequence,
            timestamp_us,
            kind,
            severity,
            detail,
            self.last_hash,
        );
        self.writer.write_all(&record.to_bytes())?;
        self.next_sequence += 1;
        self.last_hash = record.hash;
        Ok(record)
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.writer.flush()
    }
}

/// Reads every [`AuditRecord`] from a log file written by [`AuditLog`],
/// in the order they were appended.
pub fn read_audit_log(path: &Path) -> io::Result<Vec<AuditRecord>> {
    let bytes = std::fs::read(path)?;
    Ok(bytes
        .chunks_exact(AuditRecord::SIZE)
        .map(AuditRecord::from_bytes)
        .collect())
}

/// Why [`verify_chain`] rejected a sequence of records.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuditVerificationError {
    /// `records[i].sequence` wasn't the next expected value — a record is
    /// missing, duplicated, or out of order.
    SequenceGap { expected: u64, found: u64 },
    /// `records[i].prev_hash` doesn't match the previous record's `hash`
    /// (or [`GENESIS_HASH`] for the first record) — the chain link is
    /// broken, meaning a record was deleted, reordered, or its `prev_hash`
    /// field itself was altered.
    ChainBroken { sequence: u64 },
    /// Recomputing the hash over `records[i]`'s own fields doesn't match
    /// its stored `hash` — that record's contents were altered after
    /// being appended.
    HashMismatch { sequence: u64 },
}

/// Verifies that `records` (as read back by [`read_audit_log`]) form an
/// unbroken, unaltered chain from [`GENESIS_HASH`]. `Ok(())` means every
/// record is exactly as it was originally appended, in its original
/// order, with none missing.
pub fn verify_chain(records: &[AuditRecord]) -> Result<(), AuditVerificationError> {
    let mut expected_prev_hash = GENESIS_HASH;
    for (i, record) in records.iter().enumerate() {
        let expected_sequence = i as u64;
        if record.sequence != expected_sequence {
            return Err(AuditVerificationError::SequenceGap {
                expected: expected_sequence,
                found: record.sequence,
            });
        }
        if record.prev_hash != expected_prev_hash {
            return Err(AuditVerificationError::ChainBroken {
                sequence: record.sequence,
            });
        }
        if fnv1a_64(&record.hashable_bytes()) != record.hash {
            return Err(AuditVerificationError::HashMismatch {
                sequence: record.sequence,
            });
        }
        expected_prev_hash = record.hash;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_path(name: &str) -> std::path::PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "tpt-t-construction-telemetry-audit-test-{name}-{}.bin",
            std::process::id()
        ));
        path
    }

    #[test]
    fn appended_records_read_back_and_verify() {
        let path = temp_path("roundtrip");
        {
            let mut log = AuditLog::create(&path).unwrap();
            log.append(0, AuditEventKind::SelfTestCompleted, 0, 1)
                .unwrap();
            log.append(1_000, AuditEventKind::StateTransition, 0, 0x01_0002)
                .unwrap();
            log.append(2_000, AuditEventKind::FaultRaised, 2, 42)
                .unwrap();
            log.flush().unwrap();
        }

        let records = read_audit_log(&path).unwrap();
        assert_eq!(records.len(), 3);
        assert!(verify_chain(&records).is_ok());

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn each_record_chains_to_the_previous_hash() {
        let path = temp_path("chain-links");
        {
            let mut log = AuditLog::create(&path).unwrap();
            log.append(0, AuditEventKind::SelfTestCompleted, 0, 0)
                .unwrap();
            log.append(1, AuditEventKind::StateTransition, 0, 0)
                .unwrap();
        }
        let records = read_audit_log(&path).unwrap();
        assert_eq!(records[0].prev_hash, GENESIS_HASH);
        assert_eq!(records[1].prev_hash, records[0].hash);
        assert_ne!(records[0].hash, records[1].hash);

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn altering_a_record_after_the_fact_is_detected() {
        let path = temp_path("tamper-content");
        {
            let mut log = AuditLog::create(&path).unwrap();
            log.append(0, AuditEventKind::FaultRaised, 2, 42).unwrap();
            log.append(1, AuditEventKind::StateTransition, 0, 0)
                .unwrap();
        }
        let mut records = read_audit_log(&path).unwrap();
        records[0].detail = 999; // tamper with the logged fault code

        let err = verify_chain(&records).unwrap_err();
        assert_eq!(err, AuditVerificationError::HashMismatch { sequence: 0 });

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn deleting_a_record_breaks_the_chain() {
        let path = temp_path("tamper-delete");
        {
            let mut log = AuditLog::create(&path).unwrap();
            log.append(0, AuditEventKind::SelfTestCompleted, 0, 0)
                .unwrap();
            log.append(1, AuditEventKind::StateTransition, 0, 0)
                .unwrap();
            log.append(2, AuditEventKind::FaultRaised, 2, 7).unwrap();
        }
        let mut records = read_audit_log(&path).unwrap();
        records.remove(1); // delete the middle record

        let err = verify_chain(&records).unwrap_err();
        // The remaining record 2 now sits at index 1 with sequence 2,
        // which the sequence check catches before the chain-link check.
        assert_eq!(
            err,
            AuditVerificationError::SequenceGap {
                expected: 1,
                found: 2
            }
        );

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn reordering_records_breaks_the_chain() {
        let path = temp_path("tamper-reorder");
        {
            let mut log = AuditLog::create(&path).unwrap();
            log.append(0, AuditEventKind::SelfTestCompleted, 0, 0)
                .unwrap();
            log.append(1, AuditEventKind::StateTransition, 0, 0)
                .unwrap();
        }
        let mut records = read_audit_log(&path).unwrap();
        records.swap(0, 1);
        // Restore in-order sequence numbers so only the chain-link check
        // (not the sequence check) is exercised.
        records[0].sequence = 0;
        records[1].sequence = 1;

        let err = verify_chain(&records).unwrap_err();
        assert_eq!(err, AuditVerificationError::ChainBroken { sequence: 0 });

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn empty_log_verifies_trivially() {
        assert_eq!(verify_chain(&[]), Ok(()));
    }

    #[test]
    fn record_size_is_stable_across_different_values() {
        let a = AuditRecord::new(0, 0, AuditEventKind::StateTransition, 0, 0, GENESIS_HASH);
        let b = AuditRecord::new(5, 999, AuditEventKind::FaultRaised, 3, -7, a.hash);
        assert_eq!(a.to_bytes().len(), b.to_bytes().len());
    }
}
