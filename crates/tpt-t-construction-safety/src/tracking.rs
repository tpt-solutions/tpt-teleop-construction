// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Object tracking (spec.txt §4.4, §5.6): assigning a persistent identity
//! to each [`crate::perception::FusedObject`] across frames via
//! nearest-neighbor association, so the safety pipeline can reason about
//! "this same human has been in the slowdown zone for 3 seconds" rather
//! than treating every frame's detections as brand new.

use crate::perception::FusedObject;

/// A tracked object's persistent state.
#[derive(Debug, Clone)]
pub struct Track {
    pub id: u32,
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    /// Consecutive frames without a matching detection; the track is
    /// dropped once this exceeds the manager's configured limit.
    missed_frames: u32,
}

impl Track {
    pub fn missed_frames(&self) -> u32 {
        self.missed_frames
    }
}

fn distance(a: [f32; 2], b: [f32; 2]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// Maintains a set of [`Track`]s across successive frames of fused
/// detections.
#[derive(Debug, Default)]
pub struct TrackManager {
    tracks: Vec<Track>,
    next_id: u32,
    gating_distance_m: f32,
    max_missed_frames: u32,
}

impl TrackManager {
    pub fn new(gating_distance_m: f32, max_missed_frames: u32) -> Self {
        TrackManager {
            tracks: Vec::new(),
            next_id: 0,
            gating_distance_m,
            max_missed_frames,
        }
    }

    pub fn tracks(&self) -> &[Track] {
        &self.tracks
    }

    /// Associates `detections` with existing tracks by greedy nearest-
    /// distance matching (each track claims its closest unclaimed
    /// detection within the gating distance), advances unmatched tracks'
    /// missed-frame counts (dropping any that exceed the limit), and
    /// starts a new track for every detection nothing claimed.
    pub fn update(&mut self, detections: &[FusedObject]) {
        let mut claimed = vec![false; detections.len()];

        for track in &mut self.tracks {
            let best = detections
                .iter()
                .enumerate()
                .filter(|(i, _)| !claimed[*i])
                .map(|(i, d)| (i, distance(track.position, d.position)))
                .filter(|(_, dist)| *dist <= self.gating_distance_m)
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

            match best {
                Some((i, _)) => {
                    claimed[i] = true;
                    track.position = detections[i].position;
                    track.velocity = detections[i].velocity;
                    track.missed_frames = 0;
                }
                None => {
                    track.missed_frames += 1;
                }
            }
        }

        self.tracks
            .retain(|t| t.missed_frames <= self.max_missed_frames);

        for (i, detection) in detections.iter().enumerate() {
            if !claimed[i] {
                self.tracks.push(Track {
                    id: self.next_id,
                    position: detection.position,
                    velocity: detection.velocity,
                    missed_frames: 0,
                });
                self.next_id += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::perception::SensorKind;

    fn fused(x: f32, y: f32) -> FusedObject {
        FusedObject {
            position: [x, y],
            velocity: [0.0, 0.0],
            confidence: 1.0,
            contributing_sensors: vec![SensorKind::Lidar],
        }
    }

    #[test]
    fn a_new_detection_starts_a_new_track() {
        let mut manager = TrackManager::new(1.0, 2);
        manager.update(&[fused(0.0, 0.0)]);
        assert_eq!(manager.tracks().len(), 1);
        assert_eq!(manager.tracks()[0].id, 0);
    }

    #[test]
    fn a_matching_detection_keeps_the_same_track_id() {
        let mut manager = TrackManager::new(1.0, 2);
        manager.update(&[fused(0.0, 0.0)]);
        let id = manager.tracks()[0].id;
        manager.update(&[fused(0.3, 0.1)]); // small move, within gating distance
        assert_eq!(manager.tracks().len(), 1);
        assert_eq!(manager.tracks()[0].id, id);
    }

    #[test]
    fn a_far_away_detection_starts_a_separate_track() {
        let mut manager = TrackManager::new(1.0, 2);
        manager.update(&[fused(0.0, 0.0)]);
        manager.update(&[fused(50.0, 50.0)]);
        // The far detection can't match the existing track, so it starts
        // a new one while the old one racks up a missed frame.
        assert_eq!(manager.tracks().len(), 2);
    }

    #[test]
    fn a_track_survives_brief_gaps_but_not_a_long_one() {
        let mut manager = TrackManager::new(1.0, 2);
        manager.update(&[fused(0.0, 0.0)]);
        let id = manager.tracks()[0].id;
        manager.update(&[]); // missed frame 1
        assert_eq!(manager.tracks().len(), 1);
        manager.update(&[]); // missed frame 2
        assert_eq!(manager.tracks().len(), 1);
        assert_eq!(manager.tracks()[0].id, id);
        manager.update(&[]); // missed frame 3, exceeds max_missed_frames=2
        assert!(manager.tracks().is_empty());
    }

    #[test]
    fn each_track_claims_at_most_one_detection_per_frame() {
        let mut manager = TrackManager::new(5.0, 2);
        manager.update(&[fused(0.0, 0.0)]);
        // Two detections both within gating distance of the one track;
        // only one should be claimed, the other becomes a new track.
        manager.update(&[fused(0.1, 0.0), fused(0.2, 0.0)]);
        assert_eq!(manager.tracks().len(), 2);
    }
}
