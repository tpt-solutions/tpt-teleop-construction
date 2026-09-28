// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The lock-free, zero-allocation message bus every subsystem publishes
//! [`MachineEvent`]s onto. Built directly on `tpt-teleop-ring`'s bounded
//! MPMC ring: many subsystem threads (hydraulic, safety, powertrain, ...)
//! each hold a cloned [`BusSender`], while
//! [`crate::event_loop::MachineController`] holds the single [`BusReceiver`].

use crate::messages::MachineEvent;
use tpt_teleop_ring::{mpmc, MpmcFull, Receiver, Sender};

/// Default bus capacity, rounded up to a power of two by `tpt-teleop-ring`.
/// Sized generously relative to the 10Hz-100Hz event rates subsystems are
/// expected to publish at (see spec.txt §4.6, §8): full only under a real
/// event storm.
pub const DEFAULT_BUS_CAPACITY: usize = 1024;

/// The publish half of the bus. Cheaply [`Clone`]-able; hand one to every
/// subsystem that needs to raise events or request transitions.
pub struct BusSender {
    inner: Sender<MachineEvent>,
}

/// The single consume half of the bus, owned by
/// [`crate::event_loop::MachineController`].
pub struct BusReceiver {
    inner: Receiver<MachineEvent>,
}

/// A publish attempt failed because the bus is full (the consumer has
/// fallen behind). Carries the event back so the caller can decide how to
/// handle backpressure.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BusFull(pub MachineEvent);

/// Creates a new bus with [`DEFAULT_BUS_CAPACITY`] slots.
pub fn new_bus() -> (BusSender, BusReceiver) {
    new_bus_with_capacity(DEFAULT_BUS_CAPACITY)
}

/// Creates a new bus with an explicit capacity (rounded up to a power of two).
pub fn new_bus_with_capacity(capacity: usize) -> (BusSender, BusReceiver) {
    let (tx, rx) = mpmc::<MachineEvent>(capacity);
    (BusSender { inner: tx }, BusReceiver { inner: rx })
}

impl Clone for BusSender {
    fn clone(&self) -> Self {
        BusSender {
            inner: self.inner.clone(),
        }
    }
}

impl BusSender {
    /// Publishes an event without blocking. Fails with [`BusFull`] if every
    /// subsystem publishing concurrently has outpaced the event loop.
    pub fn publish(&self, event: MachineEvent) -> Result<(), BusFull> {
        self.inner
            .push(event)
            .map_err(|MpmcFull(event)| BusFull(event))
    }

    /// Capacity of the underlying ring (always a power of two, at least 2).
    pub fn capacity(&self) -> usize {
        self.inner.capacity()
    }
}

impl BusReceiver {
    /// Drains at most one pending event without blocking.
    pub fn poll(&self) -> Option<MachineEvent> {
        self.inner.pop()
    }

    /// Number of events currently queued.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::MachineState;

    #[test]
    fn publish_and_poll_roundtrip() {
        let (tx, rx) = new_bus();
        assert!(rx.poll().is_none());
        tx.publish(MachineEvent::RequestTransition {
            next: MachineState::Moving,
        })
        .unwrap();
        assert_eq!(rx.len(), 1);
        let event = rx.poll().unwrap();
        assert_eq!(
            event,
            MachineEvent::RequestTransition {
                next: MachineState::Moving
            }
        );
        assert!(rx.is_empty());
    }

    #[test]
    fn multiple_senders_share_one_bus() {
        let (tx, rx) = new_bus_with_capacity(4);
        let tx2 = tx.clone();
        tx.publish(MachineEvent::SelfTestCompleted { passed: true })
            .unwrap();
        tx2.publish(MachineEvent::SelfTestCompleted { passed: false })
            .unwrap();
        assert_eq!(rx.len(), 2);
    }

    #[test]
    fn reports_full_with_event_intact() {
        let (tx, _rx) = new_bus_with_capacity(1);
        for _ in 0..tx.capacity() {
            tx.publish(MachineEvent::SelfTestCompleted { passed: true })
                .unwrap();
        }
        let event = MachineEvent::SelfTestCompleted { passed: false };
        assert_eq!(tx.publish(event), Err(BusFull(event)));
    }
}
