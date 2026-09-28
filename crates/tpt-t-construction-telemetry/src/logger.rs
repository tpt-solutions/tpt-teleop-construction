// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! 100Hz logging of machine parameters to local storage (spec.txt §8).
//! Appends each [`TelemetryRecord`] to a file as its raw fixed-size byte
//! representation, buffered so a 100Hz write rate doesn't mean 100
//! syscalls per second. The actual NVMe-specific performance work
//! (`O_DIRECT`, `io_uring`, wear-leveling-aware write patterns) is a
//! hardware bring-up concern for Phase 12, once there's a real target
//! device to tune against; this is the portable logging mechanism itself.

use crate::record::TelemetryRecord;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

/// An append-only, buffered log of fixed-size [`TelemetryRecord`]s.
pub struct TelemetryLogger {
    writer: BufWriter<File>,
}

impl TelemetryLogger {
    /// Creates (or truncates) the log file at `path` for writing.
    pub fn create(path: &Path) -> io::Result<Self> {
        let file = File::create(path)?;
        Ok(TelemetryLogger {
            writer: BufWriter::new(file),
        })
    }

    /// Appends one record to the log.
    pub fn log(&mut self, record: &TelemetryRecord) -> io::Result<()> {
        self.writer.write_all(&record.to_bytes())
    }

    /// Flushes buffered writes to the underlying file. The logger also
    /// flushes on drop via `BufWriter`, but a caller that wants to
    /// guarantee data is written before, say, a post-shift export reads
    /// it back should call this explicitly.
    pub fn flush(&mut self) -> io::Result<()> {
        self.writer.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::export::read_all_records;

    fn temp_path(name: &str) -> std::path::PathBuf {
        let mut path = std::env::temp_dir();
        path.push(format!(
            "tpt-t-construction-telemetry-test-{name}-{}.bin",
            std::process::id()
        ));
        path
    }

    #[test]
    fn logged_records_read_back_identically() {
        let path = temp_path("roundtrip");
        let records = [
            TelemetryRecord {
                timestamp_us: 0,
                hydraulic_pressure_pa: 1.0e7,
                engine_rpm: 800.0,
                ground_speed_m_s: 0.0,
                payload_kg: 0.0,
            },
            TelemetryRecord {
                timestamp_us: 10_000,
                hydraulic_pressure_pa: 2.0e7,
                engine_rpm: 1500.0,
                ground_speed_m_s: 5.5,
                payload_kg: 40_000.0,
            },
        ];

        {
            let mut logger = TelemetryLogger::create(&path).unwrap();
            for record in &records {
                logger.log(record).unwrap();
            }
            logger.flush().unwrap();
        }

        let read_back = read_all_records(&path).unwrap();
        assert_eq!(read_back, records);

        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn creating_the_logger_again_truncates_the_file() {
        let path = temp_path("truncate");
        {
            let mut logger = TelemetryLogger::create(&path).unwrap();
            logger
                .log(&TelemetryRecord {
                    timestamp_us: 0,
                    hydraulic_pressure_pa: 0.0,
                    engine_rpm: 0.0,
                    ground_speed_m_s: 0.0,
                    payload_kg: 0.0,
                })
                .unwrap();
            logger.flush().unwrap();
        }
        {
            let _logger = TelemetryLogger::create(&path).unwrap();
        }
        let read_back = read_all_records(&path).unwrap();
        assert!(read_back.is_empty());

        std::fs::remove_file(&path).ok();
    }
}
