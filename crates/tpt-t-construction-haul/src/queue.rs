// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Dump-point/crusher queueing (spec.txt §5.2): trucks reserve a queue
//! slot by their predicted arrival time (ETA) rather than strictly by who
//! physically arrives first, so a truck that's farther away but will
//! clearly arrive sooner (lighter load, shorter route) doesn't get stuck
//! behind one that's already committed to a later slot — the same
//! ETA-based ordering spec.txt §4.6 uses for excavator/haul-truck
//! coordination, applied here to the dump end of the cycle.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// A truck's reserved dump slot: earliest ETA is served first, ties
/// broken by truck ID for determinism (never leaves the ordering
/// ambiguous between two trucks with identical ETAs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct QueuedTruck {
    eta_ms: u64,
    truck_id: u32,
}

/// A FIFO-by-ETA queue for one dump point or crusher.
#[derive(Debug, Default)]
pub struct DumpQueue {
    heap: BinaryHeap<Reverse<QueuedTruck>>,
}

impl DumpQueue {
    pub fn new() -> Self {
        DumpQueue {
            heap: BinaryHeap::new(),
        }
    }

    /// Reserves a slot for `truck_id`, predicted to arrive at `eta_ms`
    /// (milliseconds since some shared shift-start epoch).
    pub fn request_slot(&mut self, truck_id: u32, eta_ms: u64) {
        self.heap.push(Reverse(QueuedTruck { eta_ms, truck_id }));
    }

    /// Removes and returns the truck with the earliest ETA, or `None` if
    /// the queue is empty.
    pub fn next_to_dump(&mut self) -> Option<u32> {
        self.heap.pop().map(|Reverse(t)| t.truck_id)
    }

    pub fn len(&self) -> usize {
        self.heap.len()
    }

    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }
}

/// Estimated wait (seconds) for a truck at `position_in_queue` (`0` =
/// next to dump) behind trucks each taking `dump_duration_s` to clear the
/// dump point.
pub fn estimated_wait_s(position_in_queue: usize, dump_duration_s: f32) -> f32 {
    position_in_queue as f32 * dump_duration_s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_queue_has_nothing_to_dump() {
        let mut queue = DumpQueue::new();
        assert!(queue.is_empty());
        assert_eq!(queue.next_to_dump(), None);
    }

    #[test]
    fn earliest_eta_is_served_first_regardless_of_request_order() {
        let mut queue = DumpQueue::new();
        queue.request_slot(3, 5_000);
        queue.request_slot(1, 1_000);
        queue.request_slot(2, 3_000);
        assert_eq!(queue.next_to_dump(), Some(1));
        assert_eq!(queue.next_to_dump(), Some(2));
        assert_eq!(queue.next_to_dump(), Some(3));
        assert_eq!(queue.next_to_dump(), None);
    }

    #[test]
    fn tied_etas_break_ties_deterministically_by_truck_id() {
        let mut queue = DumpQueue::new();
        queue.request_slot(9, 1_000);
        queue.request_slot(2, 1_000);
        assert_eq!(queue.next_to_dump(), Some(2));
        assert_eq!(queue.next_to_dump(), Some(9));
    }

    #[test]
    fn queue_length_tracks_pending_reservations() {
        let mut queue = DumpQueue::new();
        queue.request_slot(1, 1_000);
        queue.request_slot(2, 2_000);
        assert_eq!(queue.len(), 2);
        queue.next_to_dump();
        assert_eq!(queue.len(), 1);
    }

    #[test]
    fn estimated_wait_scales_with_queue_position() {
        assert_eq!(estimated_wait_s(0, 90.0), 0.0);
        assert_eq!(estimated_wait_s(3, 90.0), 270.0);
    }
}
