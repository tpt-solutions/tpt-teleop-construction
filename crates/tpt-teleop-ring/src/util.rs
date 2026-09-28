// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

use std::ops::{Deref, DerefMut};

/// Pads a value out to a full cache line (64 bytes on essentially every
/// target this workspace ships to) so producer- and consumer-owned atomics
/// never share a cache line and false-share under contention.
#[repr(align(64))]
pub(crate) struct CachePadded<T>(pub(crate) T);

impl<T> Deref for CachePadded<T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.0
    }
}

impl<T> DerefMut for CachePadded<T> {
    fn deref_mut(&mut self) -> &mut T {
        &mut self.0
    }
}

/// Rounds `capacity` up to the next power of two, with a floor of `1`.
///
/// Power-of-two capacities let index arithmetic use a bitmask (`index &
/// mask`) instead of a modulo, which matters on the hot push/pop path.
pub(crate) fn next_power_of_two(capacity: usize) -> usize {
    capacity.max(1).next_power_of_two()
}
