// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Excavator/haul-truck queue management (spec.txt §4.6, §9): tracking
//! every truck currently assigned to this dig face by predicted arrival
//! time, so the excavator's swing cycle can be timed to have the next
//! truck loaded the moment it arrives rather than either truck or
//! excavator sitting idle.
//!
//! Backed by a fixed-capacity array rather than a `BinaryHeap` — a real
//! dig face has a handful of trucks in rotation at most, and a linear
//! scan over `N` slots with no heap allocation at all fits this crate's
//! zero-allocation coordination-loop requirement more directly than a
//! heap that reallocates as it grows.

/// One truck's predicted arrival time at the dig face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TruckEta {
    pub truck_id: u32,
    pub eta_s: f32,
}

/// A fixed-capacity set of at most `N` trucks currently queued (or
/// en route) for this dig face, ordered implicitly by ETA at query time.
#[derive(Debug, Clone)]
pub struct DispatchQueue<const N: usize> {
    entries: [Option<TruckEta>; N],
}

impl<const N: usize> DispatchQueue<N> {
    pub fn new() -> Self {
        DispatchQueue { entries: [None; N] }
    }

    /// Records or updates a truck's ETA. Returns `false` without effect
    /// if the truck isn't already tracked and there's no free slot.
    pub fn update_or_insert(&mut self, truck_id: u32, eta_s: f32) -> bool {
        for slot in self.entries.iter_mut().flatten() {
            if slot.truck_id == truck_id {
                slot.eta_s = eta_s;
                return true;
            }
        }
        for slot in &mut self.entries {
            if slot.is_none() {
                *slot = Some(TruckEta { truck_id, eta_s });
                return true;
            }
        }
        false
    }

    /// Removes a truck from the queue (e.g. once it's been loaded),
    /// freeing its slot. Returns `true` if the truck was found.
    pub fn remove(&mut self, truck_id: u32) -> bool {
        for slot in &mut self.entries {
            if slot.is_some_and(|e| e.truck_id == truck_id) {
                *slot = None;
                return true;
            }
        }
        false
    }

    /// The truck with the soonest ETA, if any are tracked.
    pub fn next_truck(&self) -> Option<TruckEta> {
        self.entries
            .iter()
            .flatten()
            .copied()
            .min_by(|a, b| a.eta_s.partial_cmp(&b.eta_s).unwrap())
    }

    pub fn len(&self) -> usize {
        self.entries.iter().filter(|e| e.is_some()).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub const fn capacity(&self) -> usize {
        N
    }
}

impl<const N: usize> Default for DispatchQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_queue_has_no_next_truck() {
        let queue: DispatchQueue<4> = DispatchQueue::new();
        assert!(queue.is_empty());
        assert_eq!(queue.next_truck(), None);
    }

    #[test]
    fn next_truck_is_whichever_has_the_soonest_eta() {
        let mut queue: DispatchQueue<4> = DispatchQueue::new();
        queue.update_or_insert(1, 120.0);
        queue.update_or_insert(2, 45.0);
        queue.update_or_insert(3, 90.0);
        assert_eq!(
            queue.next_truck(),
            Some(TruckEta {
                truck_id: 2,
                eta_s: 45.0
            })
        );
    }

    #[test]
    fn updating_an_existing_truck_does_not_use_a_new_slot() {
        let mut queue: DispatchQueue<2> = DispatchQueue::new();
        queue.update_or_insert(1, 100.0);
        queue.update_or_insert(1, 50.0);
        assert_eq!(queue.len(), 1);
        assert_eq!(
            queue.next_truck(),
            Some(TruckEta {
                truck_id: 1,
                eta_s: 50.0
            })
        );
    }

    #[test]
    fn queue_rejects_inserts_past_capacity() {
        let mut queue: DispatchQueue<2> = DispatchQueue::new();
        assert!(queue.update_or_insert(1, 10.0));
        assert!(queue.update_or_insert(2, 20.0));
        assert!(!queue.update_or_insert(3, 30.0));
        assert_eq!(queue.len(), 2);
    }

    #[test]
    fn removing_a_truck_frees_its_slot_for_reuse() {
        let mut queue: DispatchQueue<1> = DispatchQueue::new();
        queue.update_or_insert(1, 10.0);
        assert!(queue.remove(1));
        assert!(queue.is_empty());
        assert!(queue.update_or_insert(2, 20.0));
    }

    #[test]
    fn removing_an_untracked_truck_reports_false() {
        let mut queue: DispatchQueue<4> = DispatchQueue::new();
        assert!(!queue.remove(99));
    }
}
