// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Camera-based eye tracking for drowsiness detection (spec.txt §7), via
//! PERCLOS ("PERcentage of eye CLOSure"): the fraction of a rolling time
//! window during which the eyes are at least 80% closed. PERCLOS is the
//! standard drowsiness metric used in real driver/operator monitoring
//! systems (originally developed for the FHWA's drowsy-driving research
//! in the 1990s), not something invented for this crate — this module
//! computes it from a stream of per-frame eye-openness measurements a
//! camera-based eye tracker would produce (`1.0` = fully open, `0.0` =
//! fully closed).

/// Rolling-window PERCLOS computation over up to `WINDOW` eye-openness
/// samples, stack-allocated (no heap).
#[derive(Debug, Clone)]
pub struct PerclosMonitor<const WINDOW: usize> {
    samples: [f32; WINDOW],
    next_slot: usize,
    filled: usize,
}

/// The openness fraction at or below which an eye is considered "closed"
/// for PERCLOS purposes (the standard P80 definition: at least 80%
/// closed, i.e. openness at or below 20%).
const CLOSED_THRESHOLD: f32 = 0.2;

impl<const WINDOW: usize> PerclosMonitor<WINDOW> {
    pub fn new() -> Self {
        assert!(WINDOW > 0, "PerclosMonitor window must be non-empty");
        PerclosMonitor {
            samples: [1.0; WINDOW],
            next_slot: 0,
            filled: 0,
        }
    }

    pub fn push(&mut self, eye_openness_fraction: f32) {
        self.samples[self.next_slot] = eye_openness_fraction.clamp(0.0, 1.0);
        self.next_slot = (self.next_slot + 1) % WINDOW;
        self.filled = (self.filled + 1).min(WINDOW);
    }

    /// The fraction of the current window's samples that read as closed.
    /// `0.0` if no samples have been pushed yet.
    pub fn perclos(&self) -> f32 {
        if self.filled == 0 {
            return 0.0;
        }
        let closed_count = self.samples[..self.filled]
            .iter()
            .filter(|&&v| v <= CLOSED_THRESHOLD)
            .count();
        closed_count as f32 / self.filled as f32
    }

    /// Whether the window is full and its PERCLOS meets or exceeds
    /// `threshold` (a typical drowsiness alarm threshold is around
    /// `0.15`-`0.2`, i.e. eyes closed 15-20% of the time).
    pub fn is_drowsy(&self, threshold: f32) -> bool {
        self.filled == WINDOW && self.perclos() >= threshold
    }
}

impl<const WINDOW: usize> Default for PerclosMonitor<WINDOW> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wide_open_eyes_have_zero_perclos() {
        let mut monitor: PerclosMonitor<10> = PerclosMonitor::new();
        for _ in 0..10 {
            monitor.push(1.0);
        }
        assert_eq!(monitor.perclos(), 0.0);
        assert!(!monitor.is_drowsy(0.1));
    }

    #[test]
    fn constantly_closed_eyes_have_full_perclos() {
        let mut monitor: PerclosMonitor<10> = PerclosMonitor::new();
        for _ in 0..10 {
            monitor.push(0.0);
        }
        assert_eq!(monitor.perclos(), 1.0);
        assert!(monitor.is_drowsy(0.5));
    }

    #[test]
    fn partial_closure_gives_a_proportional_perclos() {
        let mut monitor: PerclosMonitor<10> = PerclosMonitor::new();
        for _ in 0..3 {
            monitor.push(0.0); // closed
        }
        for _ in 0..7 {
            monitor.push(1.0); // open
        }
        assert!((monitor.perclos() - 0.3).abs() < 1e-4);
    }

    #[test]
    fn not_drowsy_until_the_window_fills() {
        let mut monitor: PerclosMonitor<10> = PerclosMonitor::new();
        for _ in 0..5 {
            monitor.push(0.0);
        }
        // All samples so far are "closed", but the window isn't full yet.
        assert!(!monitor.is_drowsy(0.1));
    }

    #[test]
    fn old_samples_fall_out_of_the_rolling_window() {
        let mut monitor: PerclosMonitor<5> = PerclosMonitor::new();
        for _ in 0..5 {
            monitor.push(0.0); // fully closed
        }
        assert_eq!(monitor.perclos(), 1.0);
        for _ in 0..5 {
            monitor.push(1.0); // fully open, overwrites the closed samples
        }
        assert_eq!(monitor.perclos(), 0.0);
    }

    #[test]
    fn partially_closed_at_the_threshold_boundary_counts_as_closed() {
        let mut monitor: PerclosMonitor<4> = PerclosMonitor::new();
        for _ in 0..4 {
            monitor.push(0.2); // exactly at the P80 boundary
        }
        assert_eq!(monitor.perclos(), 1.0);
    }
}
