// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Truck loading with payload weighing (spec.txt §5.4): tracking a
//! target truck's accumulated load across bucket passes and deciding when
//! it's full — including stopping *before* a pass that would overfill it,
//! not just detecting overfill after the fact. `bucket_weight_kg` values
//! come from `tpt-t-construction-payload`'s on-board scale and settling
//! detector, taken as plain `f32`s here for the same reason
//! `tpt-t-construction-haul::ShiftTonnage` does.

/// Tracks one truck's accumulated load against its rated capacity.
#[derive(Debug, Clone, Copy)]
pub struct TruckLoadTarget {
    pub capacity_kg: f32,
    loaded_kg: f32,
    passes: u32,
}

impl TruckLoadTarget {
    pub fn new(capacity_kg: f32) -> Self {
        TruckLoadTarget {
            capacity_kg,
            loaded_kg: 0.0,
            passes: 0,
        }
    }

    pub fn loaded_kg(&self) -> f32 {
        self.loaded_kg
    }

    pub fn passes(&self) -> u32 {
        self.passes
    }

    /// Records one bucket pass's settled weight into the truck's total.
    pub fn record_pass(&mut self, bucket_weight_kg: f32) {
        self.loaded_kg += bucket_weight_kg;
        self.passes += 1;
    }

    pub fn remaining_capacity_kg(&self) -> f32 {
        (self.capacity_kg - self.loaded_kg).max(0.0)
    }

    pub fn is_full(&self) -> bool {
        self.loaded_kg >= self.capacity_kg
    }

    /// Whether one more pass of `average_bucket_weight_kg` would push the
    /// truck over capacity — the check that should stop loading *before*
    /// the next pass, rather than detecting overfill only after it
    /// already happened.
    pub fn would_overfill_on_next_pass(&self, average_bucket_weight_kg: f32) -> bool {
        self.loaded_kg + average_bucket_weight_kg > self.capacity_kg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_truck_is_not_full() {
        let truck = TruckLoadTarget::new(40_000.0);
        assert!(!truck.is_full());
        assert_eq!(truck.remaining_capacity_kg(), 40_000.0);
    }

    #[test]
    fn passes_accumulate_toward_capacity() {
        let mut truck = TruckLoadTarget::new(40_000.0);
        truck.record_pass(9_000.0);
        truck.record_pass(9_500.0);
        assert_eq!(truck.passes(), 2);
        assert!((truck.loaded_kg() - 18_500.0).abs() < 1e-3);
        assert!(!truck.is_full());
    }

    #[test]
    fn reaching_capacity_reports_full() {
        let mut truck = TruckLoadTarget::new(20_000.0);
        truck.record_pass(20_000.0);
        assert!(truck.is_full());
        assert_eq!(truck.remaining_capacity_kg(), 0.0);
    }

    #[test]
    fn overfilling_still_reports_full_and_clamps_remaining_to_zero() {
        let mut truck = TruckLoadTarget::new(20_000.0);
        truck.record_pass(25_000.0);
        assert!(truck.is_full());
        assert_eq!(truck.remaining_capacity_kg(), 0.0);
    }

    #[test]
    fn would_overfill_check_prevents_a_pass_that_exceeds_capacity() {
        let mut truck = TruckLoadTarget::new(40_000.0);
        truck.record_pass(35_000.0);
        assert!(truck.would_overfill_on_next_pass(9_000.0));
        assert!(!truck.would_overfill_on_next_pass(4_000.0));
    }
}
