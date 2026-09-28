// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A bounded, lock-free, zero-allocation-after-construction single-producer
//! single-consumer ring buffer.
//!
//! Intended for the highest-rate hot paths in this workspace (1kHz hydraulic
//! control loops handing samples to a telemetry writer, a sensor-fusion
//! thread streaming point clouds to a consumer) where there is exactly one
//! writer and exactly one reader and neither can be allowed to block or
//! allocate.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use crate::util::{next_power_of_two, CachePadded};

struct Shared<T> {
    buf: Box<[UnsafeCell<MaybeUninit<T>>]>,
    mask: usize,
    /// Next slot index the producer will write. Written only by the
    /// producer, read by both.
    head: CachePadded<AtomicUsize>,
    /// Next slot index the consumer will read. Written only by the
    /// consumer, read by both.
    tail: CachePadded<AtomicUsize>,
}

// SAFETY: `Shared<T>` is only ever mutated through the disjoint slot ranges
// owned by the single producer (indices in `[tail, head)` complement) and
// single consumer (indices in `[tail, head)`), coordinated by the
// acquire/release fences on `head`/`tail` below. `T: Send` is required
// because a value produced on one thread is read and dropped on another.
unsafe impl<T: Send> Send for Shared<T> {}
unsafe impl<T: Send> Sync for Shared<T> {}

impl<T> Shared<T> {
    fn capacity(&self) -> usize {
        self.mask + 1
    }
}

impl<T> Drop for Shared<T> {
    fn drop(&mut self) {
        // Drop any values still sitting in the buffer between tail and head.
        let mut tail = *self.tail.get_mut();
        let head = *self.head.get_mut();
        while tail != head {
            let slot = &self.buf[tail & self.mask];
            unsafe {
                (*slot.get()).assume_init_drop();
            }
            tail = tail.wrapping_add(1);
        }
    }
}

/// The write half of an [`SpscRing`]. Not [`Clone`]: only one producer may
/// exist per ring, which is what makes the lock-free push path sound.
pub struct Producer<T> {
    shared: Arc<Shared<T>>,
}

/// The read half of an [`SpscRing`]. Not [`Clone`]: only one consumer may
/// exist per ring, which is what makes the lock-free pop path sound.
pub struct Consumer<T> {
    shared: Arc<Shared<T>>,
}

/// Error returned by [`Producer::push`] when the ring is full. Carries the
/// value back so the caller can decide how to handle backpressure (drop the
/// oldest telemetry sample, spin, count an overrun, ...).
#[derive(Debug, PartialEq, Eq)]
pub struct Full<T>(pub T);

/// Creates a bounded SPSC ring buffer and splits it into its producer and
/// consumer halves. `capacity` is rounded up to the next power of two.
pub fn spsc<T>(capacity: usize) -> (Producer<T>, Consumer<T>) {
    let capacity = next_power_of_two(capacity);
    let mut buf = Vec::with_capacity(capacity);
    for _ in 0..capacity {
        buf.push(UnsafeCell::new(MaybeUninit::uninit()));
    }
    let shared = Arc::new(Shared {
        buf: buf.into_boxed_slice(),
        mask: capacity - 1,
        head: CachePadded(AtomicUsize::new(0)),
        tail: CachePadded(AtomicUsize::new(0)),
    });
    (
        Producer {
            shared: shared.clone(),
        },
        Consumer { shared },
    )
}

impl<T> Producer<T> {
    /// Capacity of the underlying ring (always a power of two).
    pub fn capacity(&self) -> usize {
        self.shared.capacity()
    }

    /// Attempts to push a value without blocking. Returns `Err(Full(value))`
    /// if the consumer hasn't kept up and the ring is full.
    pub fn push(&mut self, value: T) -> Result<(), Full<T>> {
        let head = self.shared.head.load(Ordering::Relaxed);
        // Acquire: synchronizes-with the consumer's Release store to `tail`
        // in `pop`, so we observe every slot it has finished freeing.
        let tail = self.shared.tail.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= self.shared.capacity() {
            return Err(Full(value));
        }
        let slot = &self.shared.buf[head & self.shared.mask];
        unsafe {
            (*slot.get()).write(value);
        }
        // Release: publishes the write above to the consumer's next
        // Acquire load of `head`.
        self.shared
            .head
            .store(head.wrapping_add(1), Ordering::Release);
        Ok(())
    }

    /// Number of items currently queued, as observed at this instant.
    pub fn len(&self) -> usize {
        let head = self.shared.head.load(Ordering::Relaxed);
        let tail = self.shared.tail.load(Ordering::Acquire);
        head.wrapping_sub(tail)
    }

    pub fn is_full(&self) -> bool {
        self.len() >= self.shared.capacity()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<T> Consumer<T> {
    /// Capacity of the underlying ring (always a power of two).
    pub fn capacity(&self) -> usize {
        self.shared.capacity()
    }

    /// Attempts to pop a value without blocking. Returns `None` if the ring
    /// is currently empty.
    pub fn pop(&mut self) -> Option<T> {
        let tail = self.shared.tail.load(Ordering::Relaxed);
        // Acquire: synchronizes-with the producer's Release store to `head`
        // in `push`, so we observe every slot it has finished writing.
        let head = self.shared.head.load(Ordering::Acquire);
        if tail == head {
            return None;
        }
        let slot = &self.shared.buf[tail & self.shared.mask];
        let value = unsafe { (*slot.get()).assume_init_read() };
        // Release: publishes the free-up of this slot to the producer's
        // next Acquire load of `tail`.
        self.shared
            .tail
            .store(tail.wrapping_add(1), Ordering::Release);
        Some(value)
    }

    /// Number of items currently queued, as observed at this instant.
    pub fn len(&self) -> usize {
        let tail = self.shared.tail.load(Ordering::Relaxed);
        let head = self.shared.head.load(Ordering::Acquire);
        head.wrapping_sub(tail)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn push_pop_roundtrip() {
        let (mut p, mut c) = spsc::<u32>(4);
        assert!(c.pop().is_none());
        p.push(1).unwrap();
        p.push(2).unwrap();
        assert_eq!(c.pop(), Some(1));
        assert_eq!(c.pop(), Some(2));
        assert!(c.pop().is_none());
    }

    #[test]
    fn capacity_rounds_up_and_reports_full() {
        let (mut p, _c) = spsc::<u32>(3);
        assert_eq!(p.capacity(), 4);
        for i in 0..4 {
            p.push(i).unwrap();
        }
        assert!(p.is_full());
        assert_eq!(p.push(99), Err(Full(99)));
    }

    #[test]
    fn wraps_around_many_cycles() {
        let (mut p, mut c) = spsc::<u32>(4);
        for round in 0..1000u32 {
            p.push(round).unwrap();
            assert_eq!(c.pop(), Some(round));
        }
    }

    #[test]
    fn drops_undelivered_values() {
        use std::sync::atomic::AtomicUsize as ACount;
        static DROPPED: ACount = ACount::new(0);
        #[derive(Debug)]
        struct Counted;
        impl Drop for Counted {
            fn drop(&mut self) {
                DROPPED.fetch_add(1, Ordering::SeqCst);
            }
        }
        {
            let (mut p, _c) = spsc::<Counted>(4);
            p.push(Counted).unwrap();
            p.push(Counted).unwrap();
        }
        assert_eq!(DROPPED.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn cross_thread_stress() {
        const N: u32 = 200_000;
        let (mut p, mut c) = spsc::<u32>(256);
        let producer = thread::spawn(move || {
            let mut i = 0;
            while i < N {
                if p.push(i).is_ok() {
                    i += 1;
                } else {
                    thread::yield_now();
                }
            }
        });
        let consumer = thread::spawn(move || {
            let mut expected = 0;
            while expected < N {
                if let Some(v) = c.pop() {
                    assert_eq!(v, expected);
                    expected += 1;
                } else {
                    thread::yield_now();
                }
            }
        });
        producer.join().unwrap();
        consumer.join().unwrap();
    }
}
