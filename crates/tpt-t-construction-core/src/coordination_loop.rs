// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The 10Hz swing-cycle adjustment coordination loop (spec.txt §4.6,
//! §9): ties [`crate::dispatch::DispatchQueue`] and
//! [`crate::truck_arrival::predicted_arrival_s`] together into the
//! decision an excavator's controller actually needs each tick —
//! "given who's coming and how long my own dig/dump cycle takes, should I
//! dwell before starting the next swing so I finish loading exactly as
//! the next truck arrives, instead of either the truck or I sitting
//! idle."

use crate::dispatch::{DispatchQueue, TruckEta};
use crate::truck_arrival::predicted_arrival_s;

/// A recommended adjustment to the excavator's swing cycle timing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SwingAdjustment {
    /// Extra time (s) to dwell before starting the next dig/swing pass,
    /// so the excavator finishes loading right as the truck arrives
    /// rather than early (leaving the excavator idle) or late (leaving
    /// the truck idle). `0.0` if the truck will arrive at or before the
    /// cycle would naturally finish anyway.
    pub recommended_dwell_s: f32,
}

/// Computes the dwell adjustment for a cycle that would otherwise take
/// `natural_cycle_time_s`, given the next truck arrives in `truck_eta_s`.
pub fn compute_swing_adjustment(natural_cycle_time_s: f32, truck_eta_s: f32) -> SwingAdjustment {
    SwingAdjustment {
        recommended_dwell_s: (truck_eta_s - natural_cycle_time_s).max(0.0),
    }
}

/// Owns the dispatch queue for one dig face and produces a swing
/// adjustment recommendation each tick — the "10Hz... coordination loop"
/// spec.txt calls for, though the cadence at which a caller invokes
/// [`CoordinationLoop::tick`] is up to that caller; this type itself is
/// just the zero-allocation per-tick computation.
#[derive(Debug, Clone)]
pub struct CoordinationLoop<const N: usize> {
    dispatch: DispatchQueue<N>,
}

impl<const N: usize> CoordinationLoop<N> {
    pub fn new() -> Self {
        CoordinationLoop {
            dispatch: DispatchQueue::new(),
        }
    }

    /// Reports (or updates) one truck's GPS-derived position/speed as a
    /// predicted ETA into the dispatch queue.
    pub fn report_truck_position(
        &mut self,
        truck_id: u32,
        distance_m: f32,
        speed_m_s: f32,
        min_speed_m_s: f32,
    ) -> bool {
        let eta_s = predicted_arrival_s(distance_m, speed_m_s, min_speed_m_s);
        self.dispatch.update_or_insert(truck_id, eta_s)
    }

    /// Removes a truck from the queue once it's been loaded.
    pub fn complete_load(&mut self, truck_id: u32) -> bool {
        self.dispatch.remove(truck_id)
    }

    /// One coordination tick: the truck the excavator should plan around
    /// next, and the recommended dwell adjustment for `natural_cycle_time_s`.
    /// `None` if no trucks are currently tracked.
    pub fn tick(&self, natural_cycle_time_s: f32) -> Option<(TruckEta, SwingAdjustment)> {
        let next = self.dispatch.next_truck()?;
        Some((
            next,
            compute_swing_adjustment(natural_cycle_time_s, next.eta_s),
        ))
    }

    pub fn queue_len(&self) -> usize {
        self.dispatch.len()
    }
}

impl<const N: usize> Default for CoordinationLoop<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn no_trucks_means_no_tick_result() {
        let loop_: CoordinationLoop<4> = CoordinationLoop::new();
        assert_eq!(loop_.tick(60.0), None);
    }

    #[test]
    fn truck_arriving_after_the_natural_cycle_gets_a_dwell_recommendation() {
        assert_eq!(
            compute_swing_adjustment(60.0, 90.0),
            SwingAdjustment {
                recommended_dwell_s: 30.0
            }
        );
    }

    #[test]
    fn truck_arriving_before_the_natural_cycle_needs_no_dwell() {
        assert_eq!(
            compute_swing_adjustment(60.0, 40.0),
            SwingAdjustment {
                recommended_dwell_s: 0.0
            }
        );
    }

    #[test]
    fn tick_plans_around_the_soonest_truck() {
        let mut loop_: CoordinationLoop<4> = CoordinationLoop::new();
        loop_.report_truck_position(1, 900.0, 10.0, 1.0); // eta 90s
        loop_.report_truck_position(2, 200.0, 10.0, 1.0); // eta 20s
        let (truck, adjustment) = loop_.tick(60.0).unwrap();
        assert_eq!(truck.truck_id, 2);
        assert_eq!(adjustment.recommended_dwell_s, 0.0); // 20s < 60s cycle
    }

    #[test]
    fn completing_a_load_removes_it_from_future_ticks() {
        let mut loop_: CoordinationLoop<4> = CoordinationLoop::new();
        loop_.report_truck_position(1, 100.0, 10.0, 1.0);
        assert!(loop_.complete_load(1));
        assert_eq!(loop_.tick(60.0), None);
    }

    #[test]
    fn a_full_tick_cycle_runs_well_within_a_10hz_100ms_budget() {
        let mut loop_: CoordinationLoop<8> = CoordinationLoop::new();
        for i in 0..8 {
            loop_.report_truck_position(i, 100.0 * (i + 1) as f32, 8.0, 1.0);
        }
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = loop_.tick(60.0);
        }
        let elapsed = start.elapsed();
        assert!(
            elapsed.as_millis() < 100,
            "1000 coordination ticks took {elapsed:?}, budget is 100ms for one 10Hz tick"
        );
    }
}
