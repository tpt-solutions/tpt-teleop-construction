// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! On-board scale payload weighing (spec.txt §3): converting raw load
//! cell counts into a calibrated weight, and deciding when that reading
//! is stable enough to log as the shift's recorded tonnage rather than a
//! transient mid-swing/mid-dump reading.

/// One load cell's calibration: a linear counts-to-weight conversion with
/// a tare offset.
#[derive(Debug, Clone, Copy)]
pub struct LoadCell {
    pub tare_kg: f32,
    pub scale_factor_kg_per_count: f32,
}

impl LoadCell {
    pub fn weight_kg(&self, raw_counts: i32) -> f32 {
        raw_counts as f32 * self.scale_factor_kg_per_count - self.tare_kg
    }
}

/// A fixed set of `N` load cells (e.g. one per suspension/hydraulic
/// mounting point) whose readings sum to the total payload weight.
#[derive(Debug, Clone, Copy)]
pub struct PayloadScale<const N: usize> {
    cells: [LoadCell; N],
}

impl<const N: usize> PayloadScale<N> {
    pub fn new(cells: [LoadCell; N]) -> Self {
        PayloadScale { cells }
    }

    pub fn total_weight_kg(&self, raw_counts: &[i32; N]) -> f32 {
        self.cells
            .iter()
            .zip(raw_counts)
            .map(|(cell, &counts)| cell.weight_kg(counts))
            .sum()
    }
}

/// Tracks a rolling window of weight readings and reports whether they've
/// settled (low variance) enough to log as a final value — the same
/// judgment call a real scale controller must make between "still
/// swinging/dumping" and "at rest, log this number".
#[derive(Debug, Clone)]
pub struct SettlingDetector<const WINDOW: usize> {
    history: [f32; WINDOW],
    filled: usize,
    next_slot: usize,
}

impl<const WINDOW: usize> SettlingDetector<WINDOW> {
    pub fn new() -> Self {
        assert!(
            WINDOW >= 2,
            "a settling window needs at least 2 samples to have a variance"
        );
        SettlingDetector {
            history: [0.0; WINDOW],
            filled: 0,
            next_slot: 0,
        }
    }

    pub fn push(&mut self, weight_kg: f32) {
        self.history[self.next_slot] = weight_kg;
        self.next_slot = (self.next_slot + 1) % WINDOW;
        self.filled = (self.filled + 1).min(WINDOW);
    }

    fn mean(&self) -> f32 {
        self.history[..self.filled].iter().sum::<f32>() / self.filled as f32
    }

    fn std_dev(&self) -> f32 {
        let mean = self.mean();
        let variance = self.history[..self.filled]
            .iter()
            .map(|v| (v - mean).powi(2))
            .sum::<f32>()
            / self.filled as f32;
        variance.sqrt()
    }

    /// Whether the window is full and its standard deviation is within
    /// `max_std_dev_kg` — i.e. the reading has held steady for a full
    /// window's worth of samples.
    pub fn is_settled(&self, max_std_dev_kg: f32) -> bool {
        self.filled == WINDOW && self.std_dev() <= max_std_dev_kg
    }

    /// The mean of the current window, meaningful once [`Self::is_settled`]
    /// reports `true`.
    pub fn settled_weight_kg(&self) -> f32 {
        self.mean()
    }
}

impl<const WINDOW: usize> Default for SettlingDetector<WINDOW> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_cell_applies_scale_factor_and_tare() {
        let cell = LoadCell {
            tare_kg: 50.0,
            scale_factor_kg_per_count: 0.1,
        };
        assert!((cell.weight_kg(1000) - 50.0).abs() < 1e-4);
    }

    #[test]
    fn payload_scale_sums_every_cell() {
        let scale = PayloadScale::new([
            LoadCell {
                tare_kg: 0.0,
                scale_factor_kg_per_count: 1.0,
            },
            LoadCell {
                tare_kg: 0.0,
                scale_factor_kg_per_count: 1.0,
            },
        ]);
        assert_eq!(scale.total_weight_kg(&[1000, 2000]), 3000.0);
    }

    #[test]
    fn not_settled_until_window_is_full() {
        let mut detector: SettlingDetector<5> = SettlingDetector::new();
        for _ in 0..4 {
            detector.push(1000.0);
            assert!(!detector.is_settled(1.0));
        }
    }

    #[test]
    fn constant_readings_settle_at_the_exact_value() {
        let mut detector: SettlingDetector<5> = SettlingDetector::new();
        for _ in 0..5 {
            detector.push(42_000.0);
        }
        assert!(detector.is_settled(0.01));
        assert!((detector.settled_weight_kg() - 42_000.0).abs() < 1e-3);
    }

    #[test]
    fn noisy_readings_within_tolerance_still_settle() {
        let mut detector: SettlingDetector<6> = SettlingDetector::new();
        for w in [41_995.0, 42_005.0, 41_998.0, 42_002.0, 42_000.0, 41_999.0] {
            detector.push(w);
        }
        assert!(detector.is_settled(10.0));
    }

    #[test]
    fn large_swings_are_not_settled() {
        let mut detector: SettlingDetector<4> = SettlingDetector::new();
        for w in [10_000.0, 40_000.0, 15_000.0, 38_000.0] {
            detector.push(w);
        }
        assert!(!detector.is_settled(100.0));
    }

    #[test]
    fn window_slides_so_only_the_most_recent_samples_count() {
        let mut detector: SettlingDetector<3> = SettlingDetector::new();
        detector.push(100_000.0); // a wild early transient
        detector.push(5_000.0);
        detector.push(5_000.0);
        detector.push(5_000.0); // pushes the transient out of the window
        assert!(detector.is_settled(0.1));
        assert!((detector.settled_weight_kg() - 5_000.0).abs() < 1e-3);
    }
}
