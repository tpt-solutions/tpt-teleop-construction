// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Zero-copy, slab-allocated point cloud storage (spec.txt §4.2:
//! "LiDAR point clouds are stored in slab-allocated ring buffers. No
//! intermediate allocations.").
//!
//! [`PointCloud<N>`] is the "slab" element: a fixed-capacity, stack-sized
//! buffer of up to `N` points with no heap indirection of its own (the
//! same fixed-capacity-array pattern as
//! `tpt-t-construction-hydraulic::SensorFrame`, applied to 3D points
//! instead of scalar sensor channels). [`new_point_cloud_channel`] moves
//! whole `PointCloud<N>` values through `tpt-teleop-ring`'s lock-free
//! SPSC ring — the ring's backing storage *is* the slab, pre-allocated
//! once at construction, and a scan handoff is a move into an existing
//! slot rather than a new allocation.

use tpt_teleop_ring::{spsc, Consumer, Producer};

/// A single `(x, y, z)` LiDAR return, meters, in the machine's local frame.
pub type Point3 = [f32; 3];

/// Error returned by [`PointCloud::push`] when the cloud's fixed capacity
/// `N` has already been reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CloudFull;

/// A fixed-capacity, stack-allocated set of up to `N` points from one
/// scan, timestamped once per cloud.
#[derive(Debug, Clone, Copy)]
pub struct PointCloud<const N: usize> {
    pub timestamp_us: u64,
    points: [Point3; N],
    len: usize,
}

impl<const N: usize> PointCloud<N> {
    pub fn new(timestamp_us: u64) -> Self {
        PointCloud {
            timestamp_us,
            points: [[0.0; 3]; N],
            len: 0,
        }
    }

    pub fn push(&mut self, point: Point3) -> Result<(), CloudFull> {
        if self.len >= N {
            return Err(CloudFull);
        }
        self.points[self.len] = point;
        self.len += 1;
        Ok(())
    }

    pub fn as_slice(&self) -> &[Point3] {
        &self.points[..self.len]
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

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl<const N: usize> Default for PointCloud<N> {
    fn default() -> Self {
        PointCloud::new(0)
    }
}

/// Creates a bounded producer/consumer channel of `PointCloud<N>` values,
/// backed by `tpt-teleop-ring`'s lock-free SPSC ring: the scanning thread
/// holds the [`Producer`] and fills clouds into pre-allocated slots, the
/// processing thread (e.g. this crate's cut/fill volume calculation) holds
/// the [`Consumer`] and reads them out, with no allocation on either side
/// once the channel itself is constructed.
pub fn new_point_cloud_channel<const N: usize>(
    capacity: usize,
) -> (Producer<PointCloud<N>>, Consumer<PointCloud<N>>) {
    spsc(capacity)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_cloud_is_empty() {
        let cloud: PointCloud<4> = PointCloud::new(1000);
        assert!(cloud.is_empty());
        assert_eq!(cloud.capacity(), 4);
    }

    #[test]
    fn push_up_to_capacity_then_reports_full() {
        let mut cloud: PointCloud<2> = PointCloud::new(0);
        cloud.push([1.0, 2.0, 3.0]).unwrap();
        cloud.push([4.0, 5.0, 6.0]).unwrap();
        assert_eq!(cloud.push([7.0, 8.0, 9.0]), Err(CloudFull));
        assert_eq!(cloud.as_slice(), &[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    }

    #[test]
    fn clear_resets_length() {
        let mut cloud: PointCloud<2> = PointCloud::new(0);
        cloud.push([1.0, 1.0, 1.0]).unwrap();
        cloud.clear();
        assert!(cloud.is_empty());
    }

    #[test]
    fn moves_zero_copy_through_the_ring_channel() {
        const SCAN_POINTS: usize = 32;
        let (mut tx, mut rx) = new_point_cloud_channel::<SCAN_POINTS>(4);

        let mut scan: PointCloud<SCAN_POINTS> = PointCloud::new(42);
        for i in 0..SCAN_POINTS {
            scan.push([i as f32, 0.0, 0.0]).unwrap();
        }
        tx.push(scan).unwrap();

        let received = rx.pop().expect("scan should be available");
        assert_eq!(received.timestamp_us, 42);
        assert_eq!(received.len(), SCAN_POINTS);
        assert_eq!(received.as_slice()[10], [10.0, 0.0, 0.0]);
    }

    #[test]
    fn point_cloud_is_stack_sized_with_no_heap_indirection() {
        assert_eq!(
            std::mem::size_of::<PointCloud<16>>(),
            std::mem::size_of::<[Point3; 16]>()
                + std::mem::size_of::<u64>()
                + std::mem::size_of::<usize>()
        );
    }
}
