// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Hydraulic pressure, engine hours, and undercarriage wear monitoring
//! (spec.txt §8): the non-vibration side of predictive maintenance —
//! simple, direct trend and accumulator tracking rather than frequency
//! analysis.

/// Accumulates total engine operating hours and reports service-interval
/// status.
#[derive(Debug, Clone, Copy, Default)]
pub struct EngineHoursMeter {
    total_hours: f64,
}

impl EngineHoursMeter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn total_hours(&self) -> f64 {
        self.total_hours
    }

    pub fn accumulate(&mut self, dt_hours: f64) {
        self.total_hours += dt_hours;
    }

    /// Hours remaining until the next scheduled service, assuming
    /// service occurs every `service_interval_hours`. A plain
    /// `total_hours % service_interval_hours` wraps an exact multiple of
    /// the interval back around to reporting a full interval remaining
    /// (remainder `0.0` looks the same as "just started"), so landing on
    /// or within a small epsilon of a boundary is treated as `0.0` hours
    /// remaining — due now — instead.
    pub fn hours_until_next_service(&self, service_interval_hours: f64) -> f64 {
        if service_interval_hours <= 0.0 {
            return 0.0;
        }
        let remainder = self.total_hours % service_interval_hours;
        if self.total_hours > 0.0 && remainder <= 1e-6 {
            0.0
        } else {
            service_interval_hours - remainder
        }
    }

    /// Whether service is due now.
    pub fn is_service_due(&self, service_interval_hours: f64) -> bool {
        self.hours_until_next_service(service_interval_hours) <= 1e-6
    }
}

/// Tracks a rolling baseline of hydraulic pressure readings and flags an
/// abnormal drop — a classic sign of internal leakage or pump wear.
#[derive(Debug, Clone)]
pub struct PressureTrendMonitor<const WINDOW: usize> {
    samples: [f32; WINDOW],
    next_slot: usize,
    filled: usize,
}

impl<const WINDOW: usize> PressureTrendMonitor<WINDOW> {
    pub fn new() -> Self {
        assert!(WINDOW > 0, "PressureTrendMonitor window must be non-empty");
        PressureTrendMonitor {
            samples: [0.0; WINDOW],
            next_slot: 0,
            filled: 0,
        }
    }

    pub fn push(&mut self, pressure_pa: f32) {
        self.samples[self.next_slot] = pressure_pa;
        self.next_slot = (self.next_slot + 1) % WINDOW;
        self.filled = (self.filled + 1).min(WINDOW);
    }

    pub fn baseline_pa(&self) -> f32 {
        if self.filled == 0 {
            0.0
        } else {
            self.samples[..self.filled].iter().sum::<f32>() / self.filled as f32
        }
    }

    /// Whether `current_pa` has dropped by at least `drop_fraction_threshold`
    /// below the established baseline. Requires a full window of history
    /// before reporting anything, so a cold start doesn't false-positive.
    pub fn is_abnormally_low(&self, current_pa: f32, drop_fraction_threshold: f32) -> bool {
        self.filled == WINDOW && current_pa < self.baseline_pa() * (1.0 - drop_fraction_threshold)
    }
}

impl<const WINDOW: usize> Default for PressureTrendMonitor<WINDOW> {
    fn default() -> Self {
        Self::new()
    }
}

/// Tracks undercarriage wear via cumulative distance traveled against a
/// rated wear life.
#[derive(Debug, Clone, Copy)]
pub struct UndercarriageWear {
    cumulative_distance_m: f64,
    rated_life_distance_m: f64,
}

impl UndercarriageWear {
    pub fn new(rated_life_distance_m: f64) -> Self {
        UndercarriageWear {
            cumulative_distance_m: 0.0,
            rated_life_distance_m,
        }
    }

    pub fn accumulate_distance(&mut self, distance_m: f64) {
        self.cumulative_distance_m += distance_m;
    }

    pub fn cumulative_distance_m(&self) -> f64 {
        self.cumulative_distance_m
    }

    /// Fraction of rated wear life remaining, clamped to `[0, 1]`.
    pub fn remaining_life_fraction(&self) -> f32 {
        if self.rated_life_distance_m <= 0.0 {
            return 0.0;
        }
        (1.0 - (self.cumulative_distance_m / self.rated_life_distance_m)).clamp(0.0, 1.0) as f32
    }

    pub fn needs_replacement(&self, threshold_fraction: f32) -> bool {
        self.remaining_life_fraction() <= threshold_fraction
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn service_is_due_exactly_at_the_interval() {
        let mut meter = EngineHoursMeter::new();
        meter.accumulate(249.9);
        assert!(!meter.is_service_due(250.0));
        meter.accumulate(0.1);
        assert!(meter.is_service_due(250.0));
    }

    #[test]
    fn hours_until_service_counts_down_within_an_interval() {
        let mut meter = EngineHoursMeter::new();
        meter.accumulate(300.0); // 50 hours into the second 250h interval
        assert!((meter.hours_until_next_service(250.0) - 200.0).abs() < 1e-6);
    }

    #[test]
    fn pressure_monitor_is_not_abnormal_before_the_window_fills() {
        let mut monitor: PressureTrendMonitor<10> = PressureTrendMonitor::new();
        for _ in 0..5 {
            monitor.push(200e5);
        }
        assert!(!monitor.is_abnormally_low(50e5, 0.2));
    }

    #[test]
    fn pressure_monitor_flags_a_significant_drop() {
        let mut monitor: PressureTrendMonitor<10> = PressureTrendMonitor::new();
        for _ in 0..10 {
            monitor.push(200e5);
        }
        assert!(!monitor.is_abnormally_low(190e5, 0.2)); // within tolerance
        assert!(monitor.is_abnormally_low(150e5, 0.2)); // 25% drop
    }

    #[test]
    fn undercarriage_wear_tracks_remaining_life() {
        let mut wear = UndercarriageWear::new(10_000.0);
        wear.accumulate_distance(2_500.0);
        assert!((wear.remaining_life_fraction() - 0.75).abs() < 1e-4);
        assert!(!wear.needs_replacement(0.1));
    }

    #[test]
    fn undercarriage_wear_flags_replacement_near_end_of_life() {
        let mut wear = UndercarriageWear::new(10_000.0);
        wear.accumulate_distance(9_500.0);
        assert!(wear.needs_replacement(0.1));
    }

    #[test]
    fn undercarriage_wear_clamps_beyond_rated_life() {
        let mut wear = UndercarriageWear::new(10_000.0);
        wear.accumulate_distance(20_000.0);
        assert_eq!(wear.remaining_life_fraction(), 0.0);
    }
}
