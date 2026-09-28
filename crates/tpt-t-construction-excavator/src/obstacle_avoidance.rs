// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Obstacle avoidance for the boom-arm-bucket path (spec.txt §5.1):
//! detecting and avoiding underground utilities, boulders, and other
//! machines. Modeled as spherical keep-out zones (a conservative bounding
//! volume around a utility line's known position, a boulder, another
//! machine's envelope) that a planned bucket-tip path must clear by a
//! minimum standoff distance.
//!
//! This checks a *planned* path before commanding it; `tpt-t-construction-
//! safety` (Phase 7) is the independent, always-on system that reacts to
//! obstacles detected in real time regardless of what was planned.

use crate::kinematics::Point3;

/// A spherical keep-out zone: an obstacle's position and the radius
/// around it that must not be entered.
#[derive(Debug, Clone, Copy)]
pub struct KeepOutZone {
    pub center: Point3,
    pub radius_m: f32,
}

fn distance(a: Point3, b: Point3) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Whether every point along `path` clears every keep-out zone by at
/// least `min_standoff_m` beyond the zone's own radius.
///
/// Checking only the path's waypoints (not the straight-line segments
/// between them) is deliberately conservative for callers that supply a
/// densely-sampled path (as [`crate::trench::TrenchPlan::dig_passes`]
/// does for its centerline); a caller planning long, sparse segments
/// should sample more points along each segment before calling this.
pub fn path_clear_of_obstacles(
    path: &[Point3],
    zones: &[KeepOutZone],
    min_standoff_m: f32,
) -> bool {
    path.iter().all(|&point| {
        zones
            .iter()
            .all(|zone| distance(point, zone.center) >= zone.radius_m + min_standoff_m)
    })
}

/// The nearest keep-out zone to any point on `path`, and the clearance
/// (which may be negative, meaning the path already violates that zone)
/// at the closest approach — useful for reporting *why* a path was
/// rejected, not just that it was.
pub fn nearest_zone_clearance_m(path: &[Point3], zones: &[KeepOutZone]) -> Option<f32> {
    path.iter()
        .flat_map(|&point| {
            zones
                .iter()
                .map(move |zone| distance(point, zone.center) - zone.radius_m)
        })
        .fold(None, |closest, clearance| {
            Some(closest.map_or(clearance, |c: f32| c.min(clearance)))
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_path() -> Vec<Point3> {
        (0..10)
            .map(|i| Point3 {
                x: i as f32,
                y: 0.0,
                z: -1.0,
            })
            .collect()
    }

    #[test]
    fn clear_path_with_no_zones() {
        assert!(path_clear_of_obstacles(&straight_path(), &[], 0.5));
    }

    #[test]
    fn distant_zone_does_not_block_the_path() {
        let zones = [KeepOutZone {
            center: Point3 {
                x: 100.0,
                y: 100.0,
                z: 0.0,
            },
            radius_m: 1.0,
        }];
        assert!(path_clear_of_obstacles(&straight_path(), &zones, 0.3));
    }

    #[test]
    fn a_zone_straddling_the_path_blocks_it() {
        let zones = [KeepOutZone {
            center: Point3 {
                x: 5.0,
                y: 0.0,
                z: -1.0,
            },
            radius_m: 0.5,
        }];
        assert!(!path_clear_of_obstacles(&straight_path(), &zones, 0.3));
    }

    #[test]
    fn standoff_margin_is_enforced_beyond_the_bare_radius() {
        // The path passes exactly `radius_m` away from the zone center at
        // its closest point — clear with zero standoff, blocked once a
        // margin is required.
        let zones = [KeepOutZone {
            center: Point3 {
                x: 5.0,
                y: 1.0,
                z: -1.0,
            },
            radius_m: 1.0,
        }];
        assert!(path_clear_of_obstacles(&straight_path(), &zones, 0.0));
        assert!(!path_clear_of_obstacles(&straight_path(), &zones, 0.1));
    }

    #[test]
    fn reports_the_tightest_clearance_across_the_whole_path() {
        let zones = [
            KeepOutZone {
                center: Point3 {
                    x: 2.0,
                    y: 0.0,
                    z: -1.0,
                },
                radius_m: 0.5,
            },
            KeepOutZone {
                center: Point3 {
                    x: 8.0,
                    y: 0.0,
                    z: -1.0,
                },
                radius_m: 0.5,
            },
        ];
        let clearance = nearest_zone_clearance_m(&straight_path(), &zones).unwrap();
        assert!((clearance - -0.5).abs() < 1e-4);
    }

    #[test]
    fn empty_zones_have_no_clearance_value() {
        assert_eq!(nearest_zone_clearance_m(&straight_path(), &[]), None);
    }
}
