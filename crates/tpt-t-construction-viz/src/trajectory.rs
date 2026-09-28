// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A bounded history of a machine's position and payload, for the
//! "machine trajectories and payload weights" view (spec.txt §8). Pure
//! data structure, no GUI dependency: `main.rs` reads it to feed
//! `egui_plot`.

use std::collections::VecDeque;

/// One recorded sample of a machine's ground position and current
/// payload.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrajectorySample {
    pub x_m: f32,
    pub y_m: f32,
    pub payload_kg: f32,
}

/// A fixed-capacity FIFO history of [`TrajectorySample`]s: oldest samples
/// fall off once `capacity` is reached, so the view always shows the most
/// recent stretch of a shift rather than growing without bound.
#[derive(Debug, Clone)]
pub struct TrajectoryBuffer {
    capacity: usize,
    samples: VecDeque<TrajectorySample>,
}

impl TrajectoryBuffer {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "TrajectoryBuffer capacity must be positive");
        TrajectoryBuffer {
            capacity,
            samples: VecDeque::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, sample: TrajectorySample) {
        if self.samples.len() == self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    pub fn latest(&self) -> Option<TrajectorySample> {
        self.samples.back().copied()
    }

    pub fn iter(&self) -> impl Iterator<Item = &TrajectorySample> {
        self.samples.iter()
    }

    /// The path as `[x, y]` points, in oldest-to-newest order, ready to
    /// hand to `egui_plot::PlotPoints`/`Line`.
    pub fn path_points(&self) -> Vec<[f64; 2]> {
        self.samples
            .iter()
            .map(|s| [s.x_m as f64, s.y_m as f64])
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(x: f32, y: f32) -> TrajectorySample {
        TrajectorySample {
            x_m: x,
            y_m: y,
            payload_kg: 0.0,
        }
    }

    #[test]
    fn empty_buffer_has_no_latest() {
        let buf = TrajectoryBuffer::new(4);
        assert!(buf.is_empty());
        assert_eq!(buf.latest(), None);
    }

    #[test]
    fn oldest_samples_drop_once_over_capacity() {
        let mut buf = TrajectoryBuffer::new(3);
        for i in 0..5 {
            buf.push(sample(i as f32, 0.0));
        }
        assert_eq!(buf.len(), 3);
        let xs: Vec<f32> = buf.iter().map(|s| s.x_m).collect();
        assert_eq!(xs, vec![2.0, 3.0, 4.0]);
    }

    #[test]
    fn latest_returns_most_recent_push() {
        let mut buf = TrajectoryBuffer::new(4);
        buf.push(sample(1.0, 1.0));
        buf.push(sample(2.0, 2.0));
        assert_eq!(buf.latest(), Some(sample(2.0, 2.0)));
    }

    #[test]
    fn path_points_are_oldest_to_newest() {
        let mut buf = TrajectoryBuffer::new(4);
        buf.push(sample(0.0, 0.0));
        buf.push(sample(1.0, 1.0));
        assert_eq!(buf.path_points(), vec![[0.0, 0.0], [1.0, 1.0]]);
    }
}
