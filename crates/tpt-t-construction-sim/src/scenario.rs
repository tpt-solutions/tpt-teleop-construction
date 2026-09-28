// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Proximity-detection scenario scripting: timelines of virtual humans and
//! vehicles moving through a scene, for exercising
//! `tpt-t-construction-safety`'s perception pipeline against reproducible,
//! version-controllable test scenarios instead of live sensor capture
//! (spec.txt §8).

use crate::math::Vec3;

/// What kind of actor a [`ScriptedActor`] represents, matching the
/// classification categories `tpt-t-construction-safety` must distinguish
/// between (spec.txt §5.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorKind {
    Human,
    LightVehicle,
    OtherEquipment,
}

/// One point on an actor's scripted path: at `time_s`, the actor is at
/// `position`.
#[derive(Debug, Clone, Copy)]
pub struct Waypoint {
    pub time_s: f32,
    pub position: Vec3,
}

/// A scripted actor: an ordered path through space and time. Position
/// between waypoints is linearly interpolated; before the first waypoint
/// or after the last, the actor holds at that endpoint (it hasn't entered
/// the scene yet / has left it, rather than teleporting).
#[derive(Debug, Clone)]
pub struct ScriptedActor {
    pub id: u32,
    pub kind: ActorKind,
    waypoints: Vec<Waypoint>,
}

impl ScriptedActor {
    /// Builds an actor from waypoints, which must be given in
    /// non-decreasing `time_s` order and must not be empty.
    pub fn new(id: u32, kind: ActorKind, waypoints: Vec<Waypoint>) -> Self {
        assert!(
            !waypoints.is_empty(),
            "a scripted actor needs at least 1 waypoint"
        );
        assert!(
            waypoints.windows(2).all(|w| w[1].time_s >= w[0].time_s),
            "waypoints must be in non-decreasing time order"
        );
        ScriptedActor {
            id,
            kind,
            waypoints,
        }
    }

    pub fn position_at(&self, time_s: f32) -> Vec3 {
        let first = &self.waypoints[0];
        if time_s <= first.time_s {
            return first.position;
        }
        let last = &self.waypoints[self.waypoints.len() - 1];
        if time_s >= last.time_s {
            return last.position;
        }
        let idx = self
            .waypoints
            .windows(2)
            .position(|w| time_s >= w[0].time_s && time_s <= w[1].time_s)
            .expect("time_s is within [first, last] and not before/after, so some segment must contain it");
        let a = &self.waypoints[idx];
        let b = &self.waypoints[idx + 1];
        let span = b.time_s - a.time_s;
        let t = if span > 0.0 {
            (time_s - a.time_s) / span
        } else {
            0.0
        };
        a.position + (b.position - a.position) * t
    }
}

/// A full proximity-detection test scenario: every scripted actor present
/// in the scene over the scenario's duration.
#[derive(Debug, Clone, Default)]
pub struct ProximityScenario {
    pub actors: Vec<ScriptedActor>,
}

impl ProximityScenario {
    pub fn new(actors: Vec<ScriptedActor>) -> Self {
        ProximityScenario { actors }
    }

    /// Every actor's `(id, kind, position)` at `time_s`, for feeding into
    /// a perception pipeline test harness alongside the machine's own
    /// simulated position.
    pub fn actors_at(&self, time_s: f32) -> Vec<(u32, ActorKind, Vec3)> {
        self.actors
            .iter()
            .map(|a| (a.id, a.kind, a.position_at(time_s)))
            .collect()
    }

    /// Distance (m) from `observer_position` to the nearest actor at
    /// `time_s`, if any actors exist — the basic building block a
    /// perception-pipeline test would use to assert "the safety system
    /// warned/slowed/stopped when a human closed to within N meters".
    pub fn nearest_distance_m(&self, observer_position: Vec3, time_s: f32) -> Option<f32> {
        self.actors_at(time_s)
            .into_iter()
            .map(|(_, _, pos)| (pos - observer_position).length())
            .fold(None, |closest, d| {
                Some(closest.map_or(d, |c: f32| c.min(d)))
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_line_actor() -> ScriptedActor {
        ScriptedActor::new(
            1,
            ActorKind::Human,
            vec![
                Waypoint {
                    time_s: 0.0,
                    position: Vec3::new(0.0, 0.0, 0.0),
                },
                Waypoint {
                    time_s: 10.0,
                    position: Vec3::new(10.0, 0.0, 0.0),
                },
            ],
        )
    }

    #[test]
    fn interpolates_linearly_between_waypoints() {
        let actor = straight_line_actor();
        let mid = actor.position_at(5.0);
        assert!((mid.x - 5.0).abs() < 1e-5);
    }

    #[test]
    fn holds_at_endpoints_outside_the_scripted_window() {
        let actor = straight_line_actor();
        assert_eq!(actor.position_at(-5.0), Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(actor.position_at(50.0), Vec3::new(10.0, 0.0, 0.0));
    }

    #[test]
    fn scenario_reports_nearest_actor_distance() {
        let human = straight_line_actor();
        let vehicle = ScriptedActor::new(
            2,
            ActorKind::LightVehicle,
            vec![Waypoint {
                time_s: 0.0,
                position: Vec3::new(100.0, 0.0, 0.0),
            }],
        );
        let scenario = ProximityScenario::new(vec![human, vehicle]);
        let distance = scenario
            .nearest_distance_m(Vec3::new(0.0, 0.0, 0.0), 0.0)
            .unwrap();
        assert!((distance - 0.0).abs() < 1e-5);
    }

    #[test]
    fn empty_scenario_has_no_nearest_distance() {
        let scenario = ProximityScenario::default();
        assert!(scenario.nearest_distance_m(Vec3::ZERO, 0.0).is_none());
    }
}
