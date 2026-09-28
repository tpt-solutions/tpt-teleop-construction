// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Proximity detection system (PDS): 360-degree LiDAR/radar/camera fusion,
//! object tracking and classification, dynamic warning zones, and
//! deterministic slowdown/stop response (spec.txt §4.4, §5.6, §7).

mod classification;
mod perception;
mod response;
mod tracking;
mod warning_zones;

pub use classification::{classify_object, ObjectClass, ObjectSignature};
pub use perception::{fuse_detections, Detection, FusedObject, SensorKind};
pub use response::{
    effective_zone, speed_command_fraction, HazardZone, PresenceEscalation, ZoneRadii,
};
pub use tracking::{Track, TrackManager};
pub use warning_zones::{is_within_dynamic_zone, zone_radius_at_bearing_m, DynamicZoneParams};
