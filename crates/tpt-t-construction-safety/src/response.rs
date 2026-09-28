// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Deterministic hazard response (spec.txt §4.4): a detected hazard maps
//! to exactly one of a fixed set of speed commands with no hidden state
//! or timing variance in the mapping itself — the decision function below
//! is pure and O(1), so whatever wraps it in a real control loop is what
//! has to guarantee the <100ms response-time budget, not this logic.
//! Continued human presence in a zone escalates the response over time
//! via [`PresenceEscalation`], separate from the zone/distance decision
//! itself.

/// Which hazard band an object's distance falls into.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum HazardZone {
    Clear,
    Warning,
    Slowdown,
    Stop,
}

/// The radii (m) defining each zone boundary, measured from the machine.
#[derive(Debug, Clone, Copy)]
pub struct ZoneRadii {
    pub warning_m: f32,
    pub slowdown_m: f32,
    pub stop_m: f32,
}

impl ZoneRadii {
    pub fn classify(&self, distance_m: f32) -> HazardZone {
        if distance_m <= self.stop_m {
            HazardZone::Stop
        } else if distance_m <= self.slowdown_m {
            HazardZone::Slowdown
        } else if distance_m <= self.warning_m {
            HazardZone::Warning
        } else {
            HazardZone::Clear
        }
    }
}

/// The speed command fraction for a given hazard zone: `1.0` = full
/// speed, `0.5` = the 50% slowdown spec.txt §4.4 requires, `0.0` = full
/// stop.
pub fn speed_command_fraction(zone: HazardZone) -> f32 {
    match zone {
        HazardZone::Clear | HazardZone::Warning => 1.0,
        HazardZone::Slowdown => 0.5,
        HazardZone::Stop => 0.0,
    }
}

/// Tracks how long a hazard has continuously held at least the
/// `Slowdown` zone, to force a full stop if a human doesn't clear the
/// area — spec.txt §4.4: "If the human remains in the zone, the machine
/// stops completely."
#[derive(Debug, Clone, Copy, Default)]
pub struct PresenceEscalation {
    dwell_s: f32,
}

impl PresenceEscalation {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advances the dwell timer if `zone` is at least `Slowdown`,
    /// otherwise resets it — the hazard has to be continuously present,
    /// not just present at some point.
    pub fn update(&mut self, zone: HazardZone, dt_s: f32) {
        if zone >= HazardZone::Slowdown {
            self.dwell_s += dt_s;
        } else {
            self.dwell_s = 0.0;
        }
    }

    pub fn dwell_seconds(&self) -> f32 {
        self.dwell_s
    }

    pub fn should_escalate_to_stop(&self, escalate_after_s: f32) -> bool {
        self.dwell_s >= escalate_after_s
    }
}

/// Combines a zone's own classification with presence-duration
/// escalation into the effective zone to command a response for: a
/// `Slowdown` that has persisted past `escalate_after_s` becomes a
/// `Stop`; nothing else changes.
pub fn effective_zone(
    zone: HazardZone,
    escalation: &PresenceEscalation,
    escalate_after_s: f32,
) -> HazardZone {
    if zone == HazardZone::Slowdown && escalation.should_escalate_to_stop(escalate_after_s) {
        HazardZone::Stop
    } else {
        zone
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn radii() -> ZoneRadii {
        ZoneRadii {
            warning_m: 20.0,
            slowdown_m: 10.0,
            stop_m: 3.0,
        }
    }

    #[test]
    fn classifies_each_band() {
        let r = radii();
        assert_eq!(r.classify(50.0), HazardZone::Clear);
        assert_eq!(r.classify(15.0), HazardZone::Warning);
        assert_eq!(r.classify(8.0), HazardZone::Slowdown);
        assert_eq!(r.classify(1.0), HazardZone::Stop);
    }

    #[test]
    fn speed_fractions_match_spec_required_50_percent_slowdown() {
        assert_eq!(speed_command_fraction(HazardZone::Clear), 1.0);
        assert_eq!(speed_command_fraction(HazardZone::Warning), 1.0);
        assert_eq!(speed_command_fraction(HazardZone::Slowdown), 0.5);
        assert_eq!(speed_command_fraction(HazardZone::Stop), 0.0);
    }

    #[test]
    fn brief_slowdown_presence_does_not_escalate() {
        let mut escalation = PresenceEscalation::new();
        escalation.update(HazardZone::Slowdown, 1.0);
        assert_eq!(
            effective_zone(HazardZone::Slowdown, &escalation, 5.0),
            HazardZone::Slowdown
        );
    }

    #[test]
    fn sustained_slowdown_presence_escalates_to_stop() {
        let mut escalation = PresenceEscalation::new();
        for _ in 0..6 {
            escalation.update(HazardZone::Slowdown, 1.0);
        }
        assert_eq!(
            effective_zone(HazardZone::Slowdown, &escalation, 5.0),
            HazardZone::Stop
        );
    }

    #[test]
    fn clearing_the_zone_resets_the_dwell_timer() {
        let mut escalation = PresenceEscalation::new();
        for _ in 0..4 {
            escalation.update(HazardZone::Slowdown, 1.0);
        }
        escalation.update(HazardZone::Clear, 1.0);
        assert_eq!(escalation.dwell_seconds(), 0.0);
        escalation.update(HazardZone::Slowdown, 1.0);
        assert_eq!(
            effective_zone(HazardZone::Slowdown, &escalation, 5.0),
            HazardZone::Slowdown
        );
    }

    #[test]
    fn escalation_never_downgrades_an_already_stopped_zone() {
        let escalation = PresenceEscalation::new();
        assert_eq!(
            effective_zone(HazardZone::Stop, &escalation, 5.0),
            HazardZone::Stop
        );
    }

    #[test]
    fn the_decision_path_is_fast_enough_for_a_100ms_response_budget() {
        let r = radii();
        let mut escalation = PresenceEscalation::new();
        let start = Instant::now();
        for _ in 0..10_000 {
            let zone = r.classify(8.0);
            escalation.update(zone, 0.001);
            let _ = speed_command_fraction(effective_zone(zone, &escalation, 5.0));
        }
        let elapsed = start.elapsed();
        assert!(
            elapsed.as_millis() < 100,
            "10k decision cycles took {elapsed:?}, budget is 100ms"
        );
    }
}
