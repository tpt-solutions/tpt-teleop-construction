// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Segment installation coordination (spec.txt §6): a TBM lines the
//! tunnel with precast concrete segments bolted together into a ring
//! behind the cutter head. Segments in a ring must be installed in
//! sequence (typically bottom-up, with a final wedge-shaped "key"
//! segment locking the ring), and the TBM must not resume advancing
//! until the current ring is fully installed — advancing prematurely
//! would push against an incomplete, structurally unsound ring.

/// One tunnel lining ring's installation progress. Segments are tracked
/// by position index (`0..num_segments`) and must be installed in that
/// order.
#[derive(Debug, Clone)]
pub struct SegmentRing {
    num_segments: usize,
    installed_count: usize,
}

impl SegmentRing {
    pub fn new(num_segments: usize) -> Self {
        SegmentRing {
            num_segments,
            installed_count: 0,
        }
    }

    pub fn num_segments(&self) -> usize {
        self.num_segments
    }

    pub fn installed_count(&self) -> usize {
        self.installed_count
    }

    pub fn is_complete(&self) -> bool {
        self.installed_count >= self.num_segments
    }

    /// Installs the next segment in sequence, returning its index, or
    /// `None` if the ring is already complete.
    pub fn install_next(&mut self) -> Option<usize> {
        if self.is_complete() {
            return None;
        }
        let index = self.installed_count;
        self.installed_count += 1;
        Some(index)
    }
}

/// Whether the TBM may resume advancing: only once the current ring is
/// fully installed.
pub fn can_advance(ring: &SegmentRing) -> bool {
    ring.is_complete()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_ring_is_not_complete() {
        let ring = SegmentRing::new(6);
        assert!(!ring.is_complete());
        assert_eq!(ring.installed_count(), 0);
        assert!(!can_advance(&ring));
    }

    #[test]
    fn installs_segments_in_order() {
        let mut ring = SegmentRing::new(3);
        assert_eq!(ring.install_next(), Some(0));
        assert_eq!(ring.install_next(), Some(1));
        assert_eq!(ring.install_next(), Some(2));
        assert!(ring.is_complete());
    }

    #[test]
    fn installing_past_completion_returns_none() {
        let mut ring = SegmentRing::new(1);
        assert_eq!(ring.install_next(), Some(0));
        assert_eq!(ring.install_next(), None);
        assert_eq!(ring.installed_count(), 1);
    }

    #[test]
    fn cannot_advance_until_every_segment_is_installed() {
        let mut ring = SegmentRing::new(2);
        ring.install_next();
        assert!(!can_advance(&ring));
        ring.install_next();
        assert!(can_advance(&ring));
    }

    #[test]
    fn a_zero_segment_ring_starts_complete() {
        let ring = SegmentRing::new(0);
        assert!(ring.is_complete());
        assert!(can_advance(&ring));
    }
}
