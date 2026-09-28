// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Haul road navigation and edge drop-off avoidance (spec.txt §5.2): a
//! haul road is modeled as a polyline of straight segments, each with its
//! own width, and a truck's lateral position is checked against a safe
//! corridor inset from the road's physical edges — the margin a real
//! haul road holds against a berm or a drop-off, not the full paved width.

/// A 2D point in the mine's local (or GNSS-projected) coordinate frame,
/// meters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point2 {
    pub x: f32,
    pub y: f32,
}

/// One straight run of haul road.
#[derive(Debug, Clone, Copy)]
pub struct RoadSegment {
    pub start: Point2,
    pub end: Point2,
    pub width_m: f32,
}

impl RoadSegment {
    fn length_m(&self) -> f32 {
        let dx = self.end.x - self.start.x;
        let dy = self.end.y - self.start.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Projects `position` onto this segment, returning
    /// `(distance_along_m, lateral_offset_m)`: `distance_along_m` is
    /// clamped to `[0, length]` (closest point on the segment, not the
    /// infinite line through it), and `lateral_offset_m` is signed
    /// (positive to the segment's left, facing from start to end).
    fn project(&self, position: Point2) -> (f32, f32) {
        let dx = self.end.x - self.start.x;
        let dy = self.end.y - self.start.y;
        let len = self.length_m();
        if len <= f32::EPSILON {
            return (
                0.0,
                ((position.x - self.start.x).powi(2) + (position.y - self.start.y).powi(2)).sqrt(),
            );
        }
        let ux = dx / len;
        let uy = dy / len;
        let rel_x = position.x - self.start.x;
        let rel_y = position.y - self.start.y;
        let along = (rel_x * ux + rel_y * uy).clamp(0.0, len);
        // Lateral offset via the 2D cross product of the direction and
        // the vector from the segment start to the position.
        let lateral = ux * rel_y - uy * rel_x;
        (along, lateral)
    }

    /// Whether `position` sits within this segment's safe corridor: its
    /// lateral offset magnitude is within `width_m / 2` minus
    /// `edge_margin_m`.
    pub fn is_within_safe_corridor(&self, position: Point2, edge_margin_m: f32) -> bool {
        let (_, lateral) = self.project(position);
        lateral.abs() <= (self.width_m / 2.0 - edge_margin_m).max(0.0)
    }
}

/// A multi-segment haul road.
#[derive(Debug, Clone)]
pub struct HaulRoad {
    pub segments: Vec<RoadSegment>,
}

/// Which segment a position is nearest to, and the position's offset
/// relative to that segment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NearestSegment {
    pub segment_index: usize,
    pub distance_along_m: f32,
    pub lateral_offset_m: f32,
}

impl HaulRoad {
    /// Finds the segment whose closest point to `position` is nearest
    /// overall — the segment a truck is currently traveling along, even
    /// through a bend where two segments meet.
    pub fn nearest_segment(&self, position: Point2) -> Option<NearestSegment> {
        self.segments
            .iter()
            .enumerate()
            .map(|(index, segment)| {
                let (along, lateral) = segment.project(position);
                let closest = Point2 {
                    x: segment.start.x
                        + (segment.end.x - segment.start.x)
                            * (along / segment.length_m().max(1e-6)),
                    y: segment.start.y
                        + (segment.end.y - segment.start.y)
                            * (along / segment.length_m().max(1e-6)),
                };
                let dist_to_closest =
                    ((position.x - closest.x).powi(2) + (position.y - closest.y).powi(2)).sqrt();
                (dist_to_closest, index, along, lateral)
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
            .map(
                |(_, segment_index, distance_along_m, lateral_offset_m)| NearestSegment {
                    segment_index,
                    distance_along_m,
                    lateral_offset_m,
                },
            )
    }

    /// Whether `position` is within the safe corridor of its nearest
    /// segment. Returns `false` for a road with no segments at all —
    /// there's no safe corridor to be within.
    pub fn is_within_safe_corridor(&self, position: Point2, edge_margin_m: f32) -> bool {
        match self.nearest_segment(position) {
            Some(nearest) => self.segments[nearest.segment_index]
                .is_within_safe_corridor(position, edge_margin_m),
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_segment() -> RoadSegment {
        RoadSegment {
            start: Point2 { x: 0.0, y: 0.0 },
            end: Point2 { x: 100.0, y: 0.0 },
            width_m: 10.0,
        }
    }

    #[test]
    fn centerline_position_is_well_within_corridor() {
        let segment = straight_segment();
        assert!(segment.is_within_safe_corridor(Point2 { x: 50.0, y: 0.0 }, 1.0));
    }

    #[test]
    fn position_past_the_margin_is_outside_the_corridor() {
        let segment = straight_segment();
        // Half-width is 5m; with a 1m margin the safe half-width is 4m.
        assert!(segment.is_within_safe_corridor(Point2 { x: 50.0, y: 3.9 }, 1.0));
        assert!(!segment.is_within_safe_corridor(Point2 { x: 50.0, y: 4.1 }, 1.0));
    }

    #[test]
    fn lateral_offset_sign_reflects_left_vs_right() {
        let segment = straight_segment();
        let (_, left) = segment.project(Point2 { x: 50.0, y: 2.0 });
        let (_, right) = segment.project(Point2 { x: 50.0, y: -2.0 });
        assert!(left > 0.0);
        assert!(right < 0.0);
    }

    #[test]
    fn distance_along_clamps_to_the_segment_ends() {
        let segment = straight_segment();
        let (along_before, _) = segment.project(Point2 { x: -50.0, y: 0.0 });
        let (along_after, _) = segment.project(Point2 { x: 150.0, y: 0.0 });
        assert_eq!(along_before, 0.0);
        assert_eq!(along_after, 100.0);
    }

    #[test]
    fn multi_segment_road_picks_the_nearest_segment() {
        let road = HaulRoad {
            segments: vec![
                RoadSegment {
                    start: Point2 { x: 0.0, y: 0.0 },
                    end: Point2 { x: 100.0, y: 0.0 },
                    width_m: 10.0,
                },
                RoadSegment {
                    start: Point2 { x: 100.0, y: 0.0 },
                    end: Point2 { x: 100.0, y: 100.0 },
                    width_m: 10.0,
                },
            ],
        };
        let nearest = road.nearest_segment(Point2 { x: 100.0, y: 50.0 }).unwrap();
        assert_eq!(nearest.segment_index, 1);
        let nearest = road.nearest_segment(Point2 { x: 50.0, y: 0.0 }).unwrap();
        assert_eq!(nearest.segment_index, 0);
    }

    #[test]
    fn empty_road_has_no_safe_corridor() {
        let road = HaulRoad { segments: vec![] };
        assert!(!road.is_within_safe_corridor(Point2 { x: 0.0, y: 0.0 }, 1.0));
    }
}
