// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Dynamic warning zones based on machine speed and direction (spec.txt
//! §4.4): a stationary machine's danger zone is roughly circular, but a
//! moving one needs a bigger margin ahead (where it's actually headed)
//! than to the sides or behind — and needs more margin overall the faster
//! it's going, since stopping distance grows with speed.

/// Tunable parameters for the zone-size model.
#[derive(Debug, Clone, Copy)]
pub struct DynamicZoneParams {
    /// Zone radius (m) at zero speed, in every direction.
    pub base_radius_m: f32,
    /// Additional radius (m) per m/s of machine speed, applied uniformly
    /// in every direction (a faster machine needs more room to react
    /// everywhere, not just ahead).
    pub speed_gain_s: f32,
    /// Additional forward elongation (m per m/s of speed) applied only in
    /// the direction of travel, tapering to zero to the sides and behind.
    pub forward_bias_s: f32,
}

/// The zone radius (m) in the direction of `bearing_to_point_rad` (angle
/// from the machine's position to the point being checked), given the
/// machine's current speed and heading.
pub fn zone_radius_at_bearing_m(
    machine_speed_m_s: f32,
    machine_heading_rad: f32,
    bearing_to_point_rad: f32,
    params: &DynamicZoneParams,
) -> f32 {
    let speed = machine_speed_m_s.max(0.0);
    let base = params.base_radius_m + params.speed_gain_s * speed;
    // 1.0 directly ahead, 0.0 to the sides, negative behind (clamped to 0
    // so the forward bias never *shrinks* the zone behind the machine).
    let heading_alignment = (bearing_to_point_rad - machine_heading_rad).cos().max(0.0);
    base + params.forward_bias_s * speed * heading_alignment
}

/// Whether `point` falls within the machine's current dynamic zone.
pub fn is_within_dynamic_zone(
    machine_position: [f32; 2],
    machine_speed_m_s: f32,
    machine_heading_rad: f32,
    point: [f32; 2],
    params: &DynamicZoneParams,
) -> bool {
    let dx = point[0] - machine_position[0];
    let dy = point[1] - machine_position[1];
    let distance_m = (dx * dx + dy * dy).sqrt();
    let bearing_rad = dy.atan2(dx);
    distance_m
        <= zone_radius_at_bearing_m(machine_speed_m_s, machine_heading_rad, bearing_rad, params)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn params() -> DynamicZoneParams {
        DynamicZoneParams {
            base_radius_m: 5.0,
            speed_gain_s: 0.5,
            forward_bias_s: 1.0,
        }
    }

    #[test]
    fn stationary_machine_has_a_uniform_circular_zone() {
        let p = params();
        let ahead = zone_radius_at_bearing_m(0.0, 0.0, 0.0, &p);
        let behind = zone_radius_at_bearing_m(0.0, 0.0, PI, &p);
        let side = zone_radius_at_bearing_m(0.0, 0.0, PI / 2.0, &p);
        assert_eq!(ahead, p.base_radius_m);
        assert_eq!(behind, p.base_radius_m);
        assert_eq!(side, p.base_radius_m);
    }

    #[test]
    fn moving_machine_has_a_bigger_zone_ahead_than_behind() {
        let p = params();
        let ahead = zone_radius_at_bearing_m(5.0, 0.0, 0.0, &p);
        let behind = zone_radius_at_bearing_m(5.0, 0.0, PI, &p);
        assert!(ahead > behind);
    }

    #[test]
    fn faster_speed_grows_the_zone_in_every_direction() {
        let p = params();
        let slow = zone_radius_at_bearing_m(1.0, 0.0, PI, &p); // behind, so no forward bias
        let fast = zone_radius_at_bearing_m(10.0, 0.0, PI, &p);
        assert!(fast > slow);
    }

    #[test]
    fn point_within_the_forward_elongated_zone_is_flagged() {
        let p = params();
        // 8m directly ahead of a machine moving at 5 m/s heading +x:
        // zone radius ahead = 5.0 + 0.5*5 + 1.0*5*1.0 = 12.5m, so 8m is inside.
        assert!(is_within_dynamic_zone([0.0, 0.0], 5.0, 0.0, [8.0, 0.0], &p));
    }

    #[test]
    fn point_behind_the_shrunk_zone_is_not_flagged() {
        let p = params();
        // Directly behind: zone radius = 5.0 + 0.5*5 = 7.5m, so 8m behind is outside.
        assert!(!is_within_dynamic_zone(
            [0.0, 0.0],
            5.0,
            0.0,
            [-8.0, 0.0],
            &p
        ));
    }
}
