// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Proximity-detection zone classification for the "proximity detection
//! zones and object classifications" view (spec.txt §8). Pure logic, no
//! GUI dependency: given a distance and the zone radii
//! `tpt-t-construction-safety` will eventually compute for real (Phase 7),
//! decide which zone an actor falls in and what color to draw it.
//!
//! Mirrors the deterministic response spec.txt §4.4 describes: a warning
//! zone (audible/visual alert only), a slowdown zone (50% speed cut), and
//! a stop zone (full stop).

/// Which proximity zone an actor is currently in, ordered from least to
/// most severe so `Ord`/`>` comparisons match escalation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ProximityZone {
    Clear,
    Warning,
    Slowdown,
    Stop,
}

/// The radii (meters) defining each zone boundary, measured from the
/// machine. Each radius must be strictly smaller than the last
/// (`stop_radius_m < slowdown_radius_m < warning_radius_m`) — the geometry
/// is concentric rings tightening toward the machine.
#[derive(Debug, Clone, Copy)]
pub struct ZoneRadii {
    pub warning_radius_m: f32,
    pub slowdown_radius_m: f32,
    pub stop_radius_m: f32,
}

impl ZoneRadii {
    pub fn classify(&self, distance_m: f32) -> ProximityZone {
        if distance_m <= self.stop_radius_m {
            ProximityZone::Stop
        } else if distance_m <= self.slowdown_radius_m {
            ProximityZone::Slowdown
        } else if distance_m <= self.warning_radius_m {
            ProximityZone::Warning
        } else {
            ProximityZone::Clear
        }
    }
}

impl ProximityZone {
    /// Display color, `[r, g, b]`: green (clear) through amber (warning),
    /// orange (slowdown), to red (stop) — the conventional traffic-light
    /// escalation an operator reads at a glance.
    pub fn color(self) -> [u8; 3] {
        match self {
            ProximityZone::Clear => [0x2e, 0xa0, 0x4a],
            ProximityZone::Warning => [0xe6, 0xb8, 0x00],
            ProximityZone::Slowdown => [0xe6, 0x7e, 0x00],
            ProximityZone::Stop => [0xcc, 0x2a, 0x2a],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn radii() -> ZoneRadii {
        ZoneRadii {
            warning_radius_m: 20.0,
            slowdown_radius_m: 10.0,
            stop_radius_m: 3.0,
        }
    }

    #[test]
    fn far_away_is_clear() {
        assert_eq!(radii().classify(100.0), ProximityZone::Clear);
    }

    #[test]
    fn boundaries_are_inclusive_toward_the_machine() {
        let r = radii();
        assert_eq!(r.classify(20.0), ProximityZone::Warning);
        assert_eq!(r.classify(10.0), ProximityZone::Slowdown);
        assert_eq!(r.classify(3.0), ProximityZone::Stop);
    }

    #[test]
    fn zones_escalate_as_distance_shrinks() {
        let r = radii();
        assert!(r.classify(15.0) < r.classify(5.0));
        assert!(r.classify(5.0) < r.classify(0.0));
    }

    #[test]
    fn every_zone_has_a_distinct_color() {
        let colors: Vec<_> = [
            ProximityZone::Clear,
            ProximityZone::Warning,
            ProximityZone::Slowdown,
            ProximityZone::Stop,
        ]
        .into_iter()
        .map(ProximityZone::color)
        .collect();
        for i in 0..colors.len() {
            for j in (i + 1)..colors.len() {
                assert_ne!(colors[i], colors[j]);
            }
        }
    }
}
