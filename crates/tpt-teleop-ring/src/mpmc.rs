// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A bounded, lock-free, zero-allocation-after-construction multi-producer
//! multi-consumer ring buffer (Dmitry Vyukov's bounded MPMC queue).
//!
//! Used with multiple producer handles and exactly one consumer handle, this
//! is the workspace's MPSC ring: e.g. every implement subsystem (hydraulic,
//! powertrain, safety) pushing state-machine events onto
//! `tpt-t-construction-core`'s single event-loop consumer.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use crate::util::{next_power_of_two, CachePadded};

struct Slot<T> {
    /// Generation sequence number. A slot is ready to enqueue into when
    /// `sequence == index`, and ready to dequeue from when
    /// `sequence == index + 1`.
    sequence: AtomicUsize,
    value: UnsafeCell<MaybeUninit<T>>,
}

struct Shared<T> {
    buf: Box<[Slot<T>]>,
    mask: usize,
    enqueue_pos: CachePadded<AtomicUsize>,
    dequeue_pos: CachePadded<AtomicUsize>,
}

// SAFETY: every slot is claimed by exactly one enqueuer and read by exactly
// one dequeuer, gated by the `sequence` handoff (Acquire load pairs with the
// Release store that follows the write/read), so concurrent producers and
// consumers never touch the same slot's `UnsafeCell` at once. `T: Send` is
// required because a value produced on one thread is read and dropped on
// another.
unsafe impl<T: Send> Send for Shared<T> {}
unsafe impl<T: Send> Sync for Shared<T> {}

impl<T> Shared<T> {
    fn capacity(&self) -> usize {
        self.mask + 1
    }
}

impl<T> Drop for Shared<T> {
    fn drop(&mut self) {
        let mut pos = *self.dequeue_pos.get_mut();
        let end = *self.enqueue_pos.get_mut();
        while pos != end {
            let slot = &self.buf[pos & self.mask];
            unsafe {
                (*slot.value.get()).assume_init_drop();
            }
            pos = pos.wrapping_add(1);
        }
    }
}

fn new_shared<T>(capacity: usize) -> Arc<Shared<T>> {
    // A capacity of 1 makes the enqueue-ready and dequeue-ready sequence
    // numbers for the sole slot alias (both read as `pos + 1` once occupied),
    // so a producer can never tell "slot holds an unread item" apart from
    // "slot is free" and can corrupt an unread item. Floor at 2, the
    // smallest capacity where every slot's enqueue/dequeue sequence values
    // stay distinct.
    let capacity = next_power_of_two(capacity).max(2);
    let mut buf = Vec::with_capacity(capacity);
    for i in 0..capacity {
        buf.push(Slot {
            sequence: AtomicUsize::new(i),
            value: UnsafeCell::new(MaybeUninit::uninit()),
        });
    }
    Arc::new(Shared {
        buf: buf.into_boxed_slice(),
        mask: capacity - 1,
        enqueue_pos: CachePadded(AtomicUsize::new(0)),
        dequeue_pos: CachePadded(AtomicUsize::new(0)),
    })
}

/// Error returned when the ring is full (on push) or empty (on pop, via
/// `None` instead — pop has no data to hand back on failure).
#[derive(Debug, PartialEq, Eq)]
pub struct Full<T>(pub T);

/// A clonable producer handle onto a bounded MPMC ring. Cloning is cheap
/// (an `Arc` bump) and every clone may push concurrently from any thread.
pub struct Sender<T> {
    shared: Arc<Shared<T>>,
}

impl<T> Clone for Sender<T> {
    fn clone(&self) -> Self {
        Sender {
            shared: self.shared.clone(),
        }
    }
}

/// A clonable consumer handle onto a bounded MPMC ring. Cloning is cheap
/// (an `Arc` bump) and every clone may pop concurrently from any thread.
/// For the common MPSC case, simply keep a single `Receiver` alive.
pub struct Receiver<T> {
    shared: Arc<Shared<T>>,
}

impl<T> Clone for Receiver<T> {
    fn clone(&self) -> Self {
        Receiver {
            shared: self.shared.clone(),
        }
    }
}

/// Creates a bounded MPMC ring buffer and returns a `(Sender, Receiver)`
/// pair. Both ends are clonable: clone `Sender` for each producer thread,
/// clone `Receiver` for each consumer thread (or keep just one for MPSC
/// use). `capacity` is rounded up to the next power of two.
pub fn mpmc<T>(capacity: usize) -> (Sender<T>, Receiver<T>) {
    let shared = new_shared(capacity);
    (
        Sender {
            shared: shared.clone(),
        },
        Receiver { shared },
    )
}

impl<T> Sender<T> {
    pub fn capacity(&self) -> usize {
        self.shared.capacity()
    }

    /// Attempts to push a value without blocking. Returns `Err(Full(value))`
    /// if every slot is currently occupied.
    pub fn push(&self, value: T) -> Result<(), Full<T>> {
        let mask = self.shared.mask;
        loop {
            let pos = self.shared.enqueue_pos.load(Ordering::Relaxed);
            let slot = &self.shared.buf[pos & mask];
            let seq = slot.sequence.load(Ordering::Acquire);
            let diff = seq as isize - pos as isize;
            if diff == 0 {
                if self
                    .shared
                    .enqueue_pos
                    .compare_exchange_weak(
                        pos,
                        pos.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    unsafe {
                        (*slot.value.get()).write(value);
                    }
                    slot.sequence.store(pos.wrapping_add(1), Ordering::Release);
                    return Ok(());
                }
                // Lost the race for this slot to another producer; retry.
            } else if diff < 0 {
                // Consumer(s) haven't freed this slot yet: the ring is full.
                return Err(Full(value));
            }
            // else: another producer has already claimed and published this
            // slot's generation (diff > 0); reload `enqueue_pos` and retry.
        }
    }

    pub fn len(&self) -> usize {
        let enqueue = self.shared.enqueue_pos.load(Ordering::Relaxed);
        let dequeue = self.shared.dequeue_pos.load(Ordering::Relaxed);
        enqueue.wrapping_sub(dequeue)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<T> Receiver<T> {
    pub fn capacity(&self) -> usize {
        self.shared.capacity()
    }

    /// Attempts to pop a value without blocking. Returns `None` if the ring
    /// is currently empty.
    pub fn pop(&self) -> Option<T> {
        let mask = self.shared.mask;
        loop {
            let pos = self.shared.dequeue_pos.load(Ordering::Relaxed);
            let slot = &self.shared.buf[pos & mask];
            let seq = slot.sequence.load(Ordering::Acquire);
            let diff = seq as isize - (pos.wrapping_add(1)) as isize;
            if diff == 0 {
                if self
                    .shared
                    .dequeue_pos
                    .compare_exchange_weak(
                        pos,
                        pos.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    let value = unsafe { (*slot.value.get()).assume_init_read() };
                    slot.sequence
                        .store(pos.wrapping_add(mask).wrapping_add(1), Ordering::Release);
                    return Some(value);
                }
                // Lost the race for this slot to another consumer; retry.
            } else if diff < 0 {
                // No producer has published this generation yet: empty.
                return None;
            }
        }
    }

    pub fn len(&self) -> usize {
        let enqueue = self.shared.enqueue_pos.load(Ordering::Relaxed);
        let dequeue = self.shared.dequeue_pos.load(Ordering::Relaxed);
        enqueue.wrapping_sub(dequeue)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;
    use std::thread;

    #[test]
    fn push_pop_roundtrip() {
        let (tx, rx) = mpmc::<u32>(4);
        assert!(rx.pop().is_none());
        tx.push(1).unwrap();
        tx.push(2).unwrap();
        assert_eq!(rx.pop(), Some(1));
        assert_eq!(rx.pop(), Some(2));
        assert!(rx.pop().is_none());
    }

    #[test]
    fn reports_full() {
        let (tx, _rx) = mpmc::<u32>(3);
        assert_eq!(tx.capacity(), 4);
        for i in 0..4 {
            tx.push(i).unwrap();
        }
        assert_eq!(tx.push(99), Err(Full(99)));
    }

    #[test]
    fn capacity_one_is_floored_to_two() {
        let (tx, rx) = mpmc::<u32>(1);
        assert_eq!(tx.capacity(), 2);
        tx.push(1).unwrap();
        tx.push(2).unwrap();
        assert_eq!(tx.push(3), Err(Full(3)));
        assert_eq!(rx.pop(), Some(1));
        assert_eq!(rx.pop(), Some(2));
        assert_eq!(rx.pop(), None);
    }

    #[test]
    fn multi_producer_single_consumer_stress() {
        const PRODUCERS: usize = 4;
        const PER_PRODUCER: u32 = 50_000;
        let (tx, rx) = mpmc::<u32>(256);

        let handles: Vec<_> = (0..PRODUCERS)
            .map(|_| {
                let tx = tx.clone();
                thread::spawn(move || {
                    let mut sent = 0u32;
                    while sent < PER_PRODUCER {
                        if tx.push(sent).is_ok() {
                            sent += 1;
                        } else {
                            thread::yield_now();
                        }
                    }
                })
            })
            .collect();
        drop(tx);

        let total = PRODUCERS as u32 * PER_PRODUCER;
        let mut received = 0u32;
        let mut seen: HashSet<u32> = HashSet::new();
        while received < total {
            if let Some(v) = rx.pop() {
                assert!(v < PER_PRODUCER);
                received += 1;
                seen.insert(v);
            } else {
                thread::yield_now();
            }
        }
        for h in handles {
            h.join().unwrap();
        }
        assert_eq!(received, total);
        // Every producer counted 0..PER_PRODUCER independently, so the
        // distinct values received should span exactly that range.
        assert_eq!(seen.len() as u32, PER_PRODUCER);
    }
}
