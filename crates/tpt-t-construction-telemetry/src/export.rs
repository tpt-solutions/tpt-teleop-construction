// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Post-shift analysis export and OEM warranty claim support (spec.txt
//! §8): reading a logged shift's records back, summarizing them, and
//! exporting a portable CSV any OEM warranty system can ingest as
//! evidence of usage and operating conditions.

use crate::record::TelemetryRecord;
use std::io;
use std::path::Path;

/// Reads every [`TelemetryRecord`] from a log file written by
/// [`crate::logger::TelemetryLogger`], in the order they were logged.
pub fn read_all_records(path: &Path) -> io::Result<Vec<TelemetryRecord>> {
    let bytes = std::fs::read(path)?;
    Ok(bytes
        .chunks_exact(TelemetryRecord::SIZE)
        .map(TelemetryRecord::from_bytes)
        .collect())
}

/// Summary statistics for a shift's worth of records — the kind of
/// evidence an OEM warranty claim needs (peak pressures, total distance,
/// duration) without having to hand over the raw log.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShiftSummary {
    pub sample_count: usize,
    pub duration_s: f64,
    pub max_hydraulic_pressure_pa: f32,
    pub min_hydraulic_pressure_pa: f32,
    pub avg_engine_rpm: f32,
    pub total_distance_m: f64,
    pub max_payload_kg: f32,
}

/// Summarizes a shift's records, or `None` if `records` is empty (there's
/// no shift to summarize).
pub fn summarize_shift(records: &[TelemetryRecord]) -> Option<ShiftSummary> {
    let first = records.first()?;
    let last = records.last()?;
    let duration_s = (last.timestamp_us.saturating_sub(first.timestamp_us)) as f64 / 1_000_000.0;

    let mut max_pressure = f32::MIN;
    let mut min_pressure = f32::MAX;
    let mut rpm_sum = 0.0f64;
    let mut max_payload = f32::MIN;
    let mut total_distance_m = 0.0f64;

    for window in records.windows(2) {
        let dt_s = (window[1]
            .timestamp_us
            .saturating_sub(window[0].timestamp_us)) as f64
            / 1_000_000.0;
        total_distance_m += window[0].ground_speed_m_s as f64 * dt_s;
    }

    for record in records {
        max_pressure = max_pressure.max(record.hydraulic_pressure_pa);
        min_pressure = min_pressure.min(record.hydraulic_pressure_pa);
        rpm_sum += record.engine_rpm as f64;
        max_payload = max_payload.max(record.payload_kg);
    }

    Some(ShiftSummary {
        sample_count: records.len(),
        duration_s,
        max_hydraulic_pressure_pa: max_pressure,
        min_hydraulic_pressure_pa: min_pressure,
        avg_engine_rpm: (rpm_sum / records.len() as f64) as f32,
        total_distance_m,
        max_payload_kg: max_payload,
    })
}

/// Exports records as CSV text: a header row followed by one row per
/// record, in a format any spreadsheet or OEM warranty intake system can
/// read directly.
pub fn export_csv(records: &[TelemetryRecord]) -> String {
    let mut csv =
        String::from("timestamp_us,hydraulic_pressure_pa,engine_rpm,ground_speed_m_s,payload_kg\n");
    for record in records {
        csv.push_str(&format!(
            "{},{},{},{},{}\n",
            record.timestamp_us,
            record.hydraulic_pressure_pa,
            record.engine_rpm,
            record.ground_speed_m_s,
            record.payload_kg
        ));
    }
    csv
}

#[cfg(test)]
mod tests {
    use super::*;

    fn records() -> Vec<TelemetryRecord> {
        vec![
            TelemetryRecord {
                timestamp_us: 0,
                hydraulic_pressure_pa: 1.0e7,
                engine_rpm: 1000.0,
                ground_speed_m_s: 2.0,
                payload_kg: 0.0,
            },
            TelemetryRecord {
                timestamp_us: 1_000_000,
                hydraulic_pressure_pa: 2.0e7,
                engine_rpm: 1500.0,
                ground_speed_m_s: 4.0,
                payload_kg: 40_000.0,
            },
            TelemetryRecord {
                timestamp_us: 2_000_000,
                hydraulic_pressure_pa: 1.5e7,
                engine_rpm: 1200.0,
                ground_speed_m_s: 0.0,
                payload_kg: 40_000.0,
            },
        ]
    }

    #[test]
    fn empty_records_have_no_summary() {
        assert_eq!(summarize_shift(&[]), None);
    }

    #[test]
    fn summary_reports_min_max_and_averages() {
        let summary = summarize_shift(&records()).unwrap();
        assert_eq!(summary.sample_count, 3);
        assert!((summary.duration_s - 2.0).abs() < 1e-9);
        assert_eq!(summary.max_hydraulic_pressure_pa, 2.0e7);
        assert_eq!(summary.min_hydraulic_pressure_pa, 1.0e7);
        assert!((summary.avg_engine_rpm - 1233.333).abs() < 0.5);
        assert_eq!(summary.max_payload_kg, 40_000.0);
    }

    #[test]
    fn summary_integrates_distance_from_speed_over_time() {
        let summary = summarize_shift(&records()).unwrap();
        // 2.0 m/s for 1s + 4.0 m/s for 1s (trapezoid via left-Riemann sum) = 6.0m
        assert!((summary.total_distance_m - 6.0).abs() < 1e-6);
    }

    #[test]
    fn csv_export_has_a_header_and_one_row_per_record() {
        let csv = export_csv(&records());
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines.len(), 4); // header + 3 records
        assert!(lines[0].starts_with("timestamp_us,"));
        assert!(lines[1].contains("1000"));
    }

    #[test]
    fn csv_export_of_no_records_is_just_the_header() {
        let csv = export_csv(&[]);
        assert_eq!(csv.lines().count(), 1);
    }
}
