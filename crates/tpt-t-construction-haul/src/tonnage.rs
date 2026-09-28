// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Per-shift tonnage tracking (spec.txt §5.2): accumulates settled
//! payload weights (from `tpt-t-construction-payload`'s on-board scale
//! and settling detector — this module takes their output as a plain
//! `f32`, not a direct dependency, since all it needs from that pipeline
//! is "here is one confirmed, settled load weight") into a running total
//! for the shift.

/// Running tonnage and load-count totals for one shift.
#[derive(Debug, Clone, Copy, Default)]
pub struct ShiftTonnage {
    total_kg: f64,
    load_count: u32,
}

impl ShiftTonnage {
    pub fn new() -> Self {
        Self::default()
    }

    /// Records one settled, confirmed load. `weight_kg` should already be
    /// a stable reading (e.g. `SettlingDetector::settled_weight_kg`), not
    /// a raw in-motion sample.
    pub fn record_load(&mut self, weight_kg: f32) {
        self.total_kg += weight_kg as f64;
        self.load_count += 1;
    }

    pub fn load_count(&self) -> u32 {
        self.load_count
    }

    pub fn total_tonnes(&self) -> f64 {
        self.total_kg / 1000.0
    }

    /// Mean load weight (kg) so far this shift, `0.0` if no loads have
    /// been recorded yet.
    pub fn average_load_kg(&self) -> f64 {
        if self.load_count == 0 {
            0.0
        } else {
            self.total_kg / self.load_count as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_shift_has_zero_totals() {
        let shift = ShiftTonnage::new();
        assert_eq!(shift.load_count(), 0);
        assert_eq!(shift.total_tonnes(), 0.0);
        assert_eq!(shift.average_load_kg(), 0.0);
    }

    #[test]
    fn records_accumulate_total_and_count() {
        let mut shift = ShiftTonnage::new();
        shift.record_load(42_000.0);
        shift.record_load(41_000.0);
        shift.record_load(43_500.0);
        assert_eq!(shift.load_count(), 3);
        assert!((shift.total_tonnes() - 126.5).abs() < 1e-6);
    }

    #[test]
    fn average_load_divides_total_by_count() {
        let mut shift = ShiftTonnage::new();
        shift.record_load(40_000.0);
        shift.record_load(60_000.0);
        assert!((shift.average_load_kg() - 50_000.0).abs() < 1e-6);
    }
}
