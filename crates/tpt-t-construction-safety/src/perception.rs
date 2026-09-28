// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! 360-degree LiDAR/radar/camera fusion (spec.txt §4.4, §7): each sensor
//! reports its own detections independently, and this module merges
//! detections that plausibly refer to the same real-world object into a
//! single fused estimate — greedy single-link clustering by proximity,
//! which is enough when detections from a true positive are much closer
//! together than the gap to the next real object (the normal case for
//! sensors covering the same scene at the same instant).

/// Which sensor produced a [`Detection`], kept so a [`FusedObject`] can
/// report which modalities agreed on it — useful downstream for
/// confidence weighting (e.g. LiDAR+radar agreement is stronger evidence
/// than camera alone).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorKind {
    Lidar,
    Radar,
    Camera,
}

/// One sensor's independent detection of an object.
#[derive(Debug, Clone, Copy)]
pub struct Detection {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    pub confidence: f32,
    pub sensor: SensorKind,
}

/// A fused estimate combining one or more sensors' detections of what is
/// judged to be the same object.
#[derive(Debug, Clone)]
pub struct FusedObject {
    pub position: [f32; 2],
    pub velocity: [f32; 2],
    pub confidence: f32,
    pub contributing_sensors: Vec<SensorKind>,
}

fn distance(a: [f32; 2], b: [f32; 2]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// Fuses raw detections from any number of sensors into merged objects:
/// any two detections within `association_radius_m` of each other (by
/// transitive chaining — single-link clustering) are folded into one
/// [`FusedObject`], whose position/velocity are the confidence-weighted
/// average of its members.
pub fn fuse_detections(detections: &[Detection], association_radius_m: f32) -> Vec<FusedObject> {
    let n = detections.len();
    if n == 0 {
        return Vec::new();
    }

    // Union-find over detection indices to cluster by transitive proximity.
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], x: usize) -> usize {
        if parent[x] != x {
            parent[x] = find(parent, parent[x]);
        }
        parent[x]
    }
    for i in 0..n {
        for j in (i + 1)..n {
            if distance(detections[i].position, detections[j].position) <= association_radius_m {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj {
                    parent[ri] = rj;
                }
            }
        }
    }

    let mut clusters: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for i in 0..n {
        let root = find(&mut parent, i);
        clusters.entry(root).or_default().push(i);
    }

    clusters
        .into_values()
        .map(|members| {
            let total_confidence: f32 = members.iter().map(|&i| detections[i].confidence).sum();
            let weight = |c: f32| {
                if total_confidence > 0.0 {
                    c / total_confidence
                } else {
                    1.0 / members.len() as f32
                }
            };

            let mut position = [0.0f32; 2];
            let mut velocity = [0.0f32; 2];
            let mut sensors = Vec::new();
            for &i in &members {
                let d = &detections[i];
                let w = weight(d.confidence);
                position[0] += d.position[0] * w;
                position[1] += d.position[1] * w;
                velocity[0] += d.velocity[0] * w;
                velocity[1] += d.velocity[1] * w;
                if !sensors.contains(&d.sensor) {
                    sensors.push(d.sensor);
                }
            }

            FusedObject {
                position,
                velocity,
                confidence: members
                    .iter()
                    .map(|&i| detections[i].confidence)
                    .fold(0.0f32, f32::max),
                contributing_sensors: sensors,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn no_detections_yields_no_objects() {
        assert!(fuse_detections(&[], 1.0).is_empty());
    }

    #[test]
    fn distant_detections_stay_separate_objects() {
        let detections = [
            Detection {
                position: [0.0, 0.0],
                velocity: [0.0, 0.0],
                confidence: 0.9,
                sensor: SensorKind::Lidar,
            },
            Detection {
                position: [100.0, 0.0],
                velocity: [0.0, 0.0],
                confidence: 0.9,
                sensor: SensorKind::Radar,
            },
        ];
        assert_eq!(fuse_detections(&detections, 1.0).len(), 2);
    }

    #[test]
    fn nearby_detections_from_different_sensors_fuse_into_one_object() {
        let detections = [
            Detection {
                position: [10.0, 0.0],
                velocity: [1.0, 0.0],
                confidence: 0.8,
                sensor: SensorKind::Lidar,
            },
            Detection {
                position: [10.2, 0.1],
                velocity: [1.1, 0.0],
                confidence: 0.6,
                sensor: SensorKind::Radar,
            },
        ];
        let fused = fuse_detections(&detections, 1.0);
        assert_eq!(fused.len(), 1);
        assert_eq!(fused[0].contributing_sensors.len(), 2);
    }

    #[test]
    fn fused_position_is_confidence_weighted() {
        let detections = [
            Detection {
                position: [0.0, 0.0],
                velocity: [0.0, 0.0],
                confidence: 3.0,
                sensor: SensorKind::Lidar,
            },
            Detection {
                position: [10.0, 0.0],
                velocity: [0.0, 0.0],
                confidence: 1.0,
                sensor: SensorKind::Camera,
            },
        ];
        let fused = fuse_detections(&detections, 20.0);
        assert_eq!(fused.len(), 1);
        // Weighted toward the higher-confidence (lidar) detection: 3/4 * 0 + 1/4 * 10 = 2.5
        assert!((fused[0].position[0] - 2.5).abs() < 1e-3);
    }

    #[test]
    fn transitive_chain_merges_into_a_single_cluster() {
        // A-B and B-C are each within radius, but A-C alone would not be;
        // single-link clustering should still merge all three.
        let detections = [
            Detection {
                position: [0.0, 0.0],
                velocity: [0.0, 0.0],
                confidence: 1.0,
                sensor: SensorKind::Lidar,
            },
            Detection {
                position: [0.9, 0.0],
                velocity: [0.0, 0.0],
                confidence: 1.0,
                sensor: SensorKind::Radar,
            },
            Detection {
                position: [1.8, 0.0],
                velocity: [0.0, 0.0],
                confidence: 1.0,
                sensor: SensorKind::Camera,
            },
        ];
        assert_eq!(fuse_detections(&detections, 1.0).len(), 1);
    }

    #[test]
    fn fusion_of_a_busy_scene_completes_in_well_under_fifty_milliseconds() {
        // A generous stand-in for a busy proximity scene (spec.txt §4.4's
        // <50ms perception pipeline budget).
        let detections: Vec<Detection> = (0..300)
            .map(|i| Detection {
                position: [i as f32 * 2.0, (i % 7) as f32],
                velocity: [0.0, 0.0],
                confidence: 0.7,
                sensor: SensorKind::Lidar,
            })
            .collect();

        let start = Instant::now();
        let fused = fuse_detections(&detections, 1.0);
        let elapsed = start.elapsed();

        assert!(!fused.is_empty());
        assert!(
            elapsed.as_millis() < 50,
            "fusion took {elapsed:?}, budget is 50ms"
        );
    }
}
