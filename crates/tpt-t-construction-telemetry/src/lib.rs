// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! High-frequency telemetry logging: 100Hz machine parameter capture to
//! local storage, and post-shift analysis export for OEM warranty claim
//! support (spec.txt §8).

mod audit;
mod export;
mod logger;
mod record;

pub use audit::{
    read_audit_log, verify_chain, AuditEventKind, AuditLog, AuditRecord, AuditVerificationError,
    GENESIS_HASH,
};
pub use export::{export_csv, read_all_records, summarize_shift, ShiftSummary};
pub use logger::TelemetryLogger;
pub use record::TelemetryRecord;
