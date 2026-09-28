// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Zero-allocation, fixed-capacity storage for sensor and control data
//! (spec.txt §4.1: "zero-allocation: all sensor readings and control
//! outputs use pre-allocated static arrays"). `SensorFrame<N>` is a
//! stack-allocated `[f32; N]` with a running length — a `Vec`-like push
//! API without ever touching the heap, so it's safe to construct and fill
//! from inside the 1kHz `SCHED_FIFO` loop.

/// Error returned by [`SensorFrame::push`] when the frame's fixed capacity
/// `N` has already been reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameFull;

/// A fixed-capacity, stack-allocated frame of up to `N` `f32` channel
/// readings (pressures, positions, currents, ...), timestamped once per
/// frame.
#[derive(Debug, Clone, Copy)]
pub struct SensorFrame<const N: usize> {
    pub timestamp_us: u64,
    values: [f32; N],
    len: usize,
}

impl<const N: usize> SensorFrame<N> {
    pub fn new(timestamp_us: u64) -> Self {
        SensorFrame {
            timestamp_us,
            values: [0.0; N],
            len: 0,
        }
    }

    /// Appends one reading. Fails with [`FrameFull`] once `N` readings
    /// have already been pushed — the frame never grows past its
    /// pre-allocated capacity.
    pub fn push(&mut self, value: f32) -> Result<(), FrameFull> {
        if self.len >= N {
            return Err(FrameFull);
        }
        self.values[self.len] = value;
        self.len += 1;
        Ok(())
    }

    pub fn as_slice(&self) -> &[f32] {
        &self.values[..self.len]
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub const fn capacity(&self) -> usize {
        N
    }

    pub fn is_full(&self) -> bool {
        self.len == N
    }

    /// Empties the frame (for reuse on the next tick) without touching
    /// its backing storage.
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_frame_is_empty() {
        let frame: SensorFrame<4> = SensorFrame::new(1000);
        assert!(frame.is_empty());
        assert_eq!(frame.capacity(), 4);
        assert_eq!(frame.as_slice(), &[] as &[f32]);
    }

    #[test]
    fn push_appends_in_order_up_to_capacity() {
        let mut frame: SensorFrame<3> = SensorFrame::new(0);
        frame.push(1.0).unwrap();
        frame.push(2.0).unwrap();
        frame.push(3.0).unwrap();
        assert!(frame.is_full());
        assert_eq!(frame.as_slice(), &[1.0, 2.0, 3.0]);
        assert_eq!(frame.push(4.0), Err(FrameFull));
    }

    #[test]
    fn clear_resets_length_but_keeps_capacity() {
        let mut frame: SensorFrame<2> = SensorFrame::new(0);
        frame.push(1.0).unwrap();
        frame.clear();
        assert!(frame.is_empty());
        assert_eq!(frame.capacity(), 2);
        frame.push(9.0).unwrap();
        assert_eq!(frame.as_slice(), &[9.0]);
    }

    #[test]
    fn frame_is_stack_sized_with_no_heap_indirection() {
        // A `SensorFrame<N>` is exactly a `[f32; N]` plus two `usize`s in
        // size — no `Vec`/`Box` pointer chasing to the heap.
        assert_eq!(
            std::mem::size_of::<SensorFrame<16>>(),
            std::mem::size_of::<[f32; 16]>()
                + std::mem::size_of::<u64>()
                + std::mem::size_of::<usize>()
        );
    }
}
