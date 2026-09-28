// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Zero-false-stop validation suite (todo.md Phase 11: "Zero-false-stop
//! validation suite for proximity detection").
//!
//! Unlike the per-module unit tests in `src/response.rs`,
//! `src/perception.rs`, etc., these scenarios exercise the full
//! perception -> classification -> zone -> response pipeline together,
//! the way a real proximity-detection tick actually composes them, and
//! are named for the failure mode each one rules out rather than the
//! function each one calls.
//!
//! A validation suite that only ever asserts "did not stop" would pass
//! trivially for a system that never stops at all, so
//! [`sustained_human_presence_in_the_stop_zone_still_stops`] and
//! [`escalation_still_forces_a_stop_after_the_dwell_threshold`] are
//! included as positive controls: proof the suite would actually catch a
//! regression that silently disabled the response, not just one that
//! made it trigger falsely.

use tpt_t_construction_safety::{
    classify_object, effective_zone, fuse_detections, is_within_dynamic_zone,
    speed_command_fraction, Detection, DynamicZoneParams, HazardZone, ObjectClass, ObjectSignature,
    PresenceEscalation, SensorKind, ZoneRadii,
};

fn radii() -> ZoneRadii {
    ZoneRadii {
        warning_m: 20.0,
        slowdown_m: 10.0,
        stop_m: 3.0,
    }
}

fn full_speed(fraction: f32) {
    assert_eq!(
        fraction, 1.0,
        "expected full speed, got fraction {fraction}"
    );
}

#[test]
fn stationary_equipment_at_a_safe_distance_never_slows_the_machine() {
    // Another machine parked well outside every zone, reported redundantly
    // by all three sensor modalities (a busy, noisy scene shouldn't by
    // itself manufacture a hazard).
    let detections = [
        Detection {
            position: [50.0, 0.0],
            velocity: [0.0, 0.0],
            confidence: 0.9,
            sensor: SensorKind::Lidar,
        },
        Detection {
            position: [50.1, 0.0],
            velocity: [0.0, 0.0],
            confidence: 0.7,
            sensor: SensorKind::Radar,
        },
        Detection {
            position: [49.9, 0.1],
            velocity: [0.0, 0.0],
            confidence: 0.6,
            sensor: SensorKind::Camera,
        },
    ];
    let fused = fuse_detections(&detections, 1.0);
    assert_eq!(fused.len(), 1);

    let distance_m = (fused[0].position[0].powi(2) + fused[0].position[1].powi(2)).sqrt();
    let zone = radii().classify(distance_m);
    assert_eq!(zone, HazardZone::Clear);
    full_speed(speed_command_fraction(zone));

    let sig = ObjectSignature {
        width_m: 4.5,
        height_m: 3.2,
        max_speed_observed_m_s: 0.0,
    };
    assert_ne!(classify_object(&sig), ObjectClass::Human);
}

#[test]
fn a_single_frame_sensor_noise_blip_in_the_slowdown_zone_never_escalates() {
    // Sensor noise producing an isolated one-tick detection that
    // immediately clears the next tick should never accumulate enough
    // continuous dwell to force a stop, however many times it flickers
    // over a long window — only *continuous* presence counts.
    let mut escalation = PresenceEscalation::new();
    let dt_s = 0.1;
    let escalate_after_s = 5.0;

    for tick in 0..600 {
        // Blip into Slowdown on every 20th tick (2s), immediately clearing
        // the very next tick.
        let zone = if tick % 20 == 0 {
            HazardZone::Slowdown
        } else {
            HazardZone::Clear
        };
        escalation.update(zone, dt_s);
        let effective = effective_zone(zone, &escalation, escalate_after_s);
        assert_ne!(
            effective,
            HazardZone::Stop,
            "a single-tick blip at tick {tick} must never force a stop"
        );
    }
}

#[test]
fn low_confidence_detection_behind_a_moving_machine_is_outside_the_shrunk_zone() {
    // Something detected behind a forward-moving machine sits in a
    // deliberately smaller zone (see src/warning_zones.rs) — this should
    // not be flagged, even though the same absolute distance ahead of the
    // machine would be.
    let params = DynamicZoneParams {
        base_radius_m: 5.0,
        speed_gain_s: 0.3,
        forward_bias_s: 1.5,
    };
    let machine_position = [0.0, 0.0];
    let machine_speed_m_s = 6.0;
    let machine_heading_rad = 0.0; // heading +x

    let behind_point = [-8.0, 0.0];
    assert!(!is_within_dynamic_zone(
        machine_position,
        machine_speed_m_s,
        machine_heading_rad,
        behind_point,
        &params,
    ));

    // The same 8m distance directly ahead *is* within the elongated zone —
    // confirms this is the forward/behind asymmetry, not a bug that
    // shrank the zone in every direction.
    let ahead_point = [8.0, 0.0];
    assert!(is_within_dynamic_zone(
        machine_position,
        machine_speed_m_s,
        machine_heading_rad,
        ahead_point,
        &params,
    ));
}

#[test]
fn brief_slowdown_zone_entry_only_reduces_speed_never_stops() {
    // A single momentary entry into Slowdown (well under the escalation
    // threshold) should reduce speed to the spec'd 50%, not stop outright
    // — a full stop is reserved for the Stop zone or sustained presence.
    let mut escalation = PresenceEscalation::new();
    escalation.update(HazardZone::Slowdown, 1.0);
    let effective = effective_zone(HazardZone::Slowdown, &escalation, 5.0);
    assert_eq!(speed_command_fraction(effective), 0.5);
    assert_ne!(effective, HazardZone::Stop);
}

#[test]
fn sustained_human_presence_in_the_stop_zone_still_stops() {
    // Positive control: a real, continuously-present hazard must still
    // reach Stop. Without this, a change that made the pipeline never
    // stop at all would pass every "false stop" test above vacuously.
    let sig = ObjectSignature {
        width_m: 0.5,
        height_m: 1.7,
        max_speed_observed_m_s: 1.2,
    };
    assert_eq!(classify_object(&sig), ObjectClass::Human);

    let zone = radii().classify(2.0); // inside stop_m = 3.0
    assert_eq!(zone, HazardZone::Stop);
    assert_eq!(speed_command_fraction(zone), 0.0);
}

#[test]
fn escalation_still_forces_a_stop_after_the_dwell_threshold() {
    // Positive control for the escalation path specifically: continuous
    // presence in Slowdown for longer than the threshold must still
    // escalate to Stop.
    let mut escalation = PresenceEscalation::new();
    let dt_s = 0.5;
    for _ in 0..20 {
        escalation.update(HazardZone::Slowdown, dt_s); // 10s of continuous presence
    }
    let effective = effective_zone(HazardZone::Slowdown, &escalation, 5.0);
    assert_eq!(effective, HazardZone::Stop);
    assert_eq!(speed_command_fraction(effective), 0.0);
}
