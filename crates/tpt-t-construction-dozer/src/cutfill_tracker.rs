// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Real-time cut/fill volume tracking over a dozing pass (spec.txt §5.3).
//! Builds directly on `tpt-t-construction-payload`'s cut/fill volume
//! calculation (the same computation spec.txt §4.2 calls out — this
//! isn't a second implementation, just this crate's use of it), adding
//! the running accumulation across every scan taken during one pass.

use tpt_t_construction_payload::{cut_fill_volumes, CutFillResult};

/// Accumulates cut/fill volumes across every scan-to-scan comparison
/// taken during one dozing pass.
#[derive(Debug, Clone, Copy, Default)]
pub struct PassTracker {
    cumulative_cut_m3: f32,
    cumulative_fill_m3: f32,
}

impl PassTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// Computes the cut/fill volume between two height-sample scans and
    /// folds it into this pass's running totals, returning that scan's
    /// own (non-cumulative) result.
    pub fn record_scan(
        &mut self,
        previous_heights_m: &[f32],
        current_heights_m: &[f32],
        cell_area_m2: f32,
    ) -> CutFillResult {
        let result = cut_fill_volumes(previous_heights_m, current_heights_m, cell_area_m2);
        self.cumulative_cut_m3 += result.cut_m3;
        self.cumulative_fill_m3 += result.fill_m3;
        result
    }

    pub fn cumulative_cut_m3(&self) -> f32 {
        self.cumulative_cut_m3
    }

    pub fn cumulative_fill_m3(&self) -> f32 {
        self.cumulative_fill_m3
    }

    pub fn cumulative_net_m3(&self) -> f32 {
        self.cumulative_fill_m3 - self.cumulative_cut_m3
    }

    /// Clears accumulated totals, e.g. when starting a new pass.
    pub fn reset(&mut self) {
        self.cumulative_cut_m3 = 0.0;
        self.cumulative_fill_m3 = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_tracker_starts_at_zero() {
        let tracker = PassTracker::new();
        assert_eq!(tracker.cumulative_cut_m3(), 0.0);
        assert_eq!(tracker.cumulative_fill_m3(), 0.0);
        assert_eq!(tracker.cumulative_net_m3(), 0.0);
    }

    #[test]
    fn accumulates_across_multiple_scans() {
        let mut tracker = PassTracker::new();
        // First scan: one cell cut by 1.0m over 1.0m^2.
        tracker.record_scan(&[5.0, 5.0], &[4.0, 5.0], 1.0);
        // Second scan: the other cell filled by 2.0m.
        tracker.record_scan(&[4.0, 5.0], &[4.0, 7.0], 1.0);
        assert!((tracker.cumulative_cut_m3() - 1.0).abs() < 1e-4);
        assert!((tracker.cumulative_fill_m3() - 2.0).abs() < 1e-4);
        assert!((tracker.cumulative_net_m3() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn reset_clears_accumulated_totals() {
        let mut tracker = PassTracker::new();
        tracker.record_scan(&[5.0], &[3.0], 1.0);
        tracker.reset();
        assert_eq!(tracker.cumulative_cut_m3(), 0.0);
        assert_eq!(tracker.cumulative_fill_m3(), 0.0);
    }

    #[test]
    fn record_scan_returns_that_scans_own_result_not_the_cumulative_total() {
        let mut tracker = PassTracker::new();
        tracker.record_scan(&[5.0], &[3.0], 1.0);
        let second = tracker.record_scan(&[3.0], &[2.0], 1.0);
        assert!((second.cut_m3 - 1.0).abs() < 1e-4);
        assert!((tracker.cumulative_cut_m3() - 3.0).abs() < 1e-4);
    }
}
