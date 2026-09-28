// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Object classification (spec.txt §4.4, §5.6): distinguishing a human on
//! foot from a light vehicle from another piece of heavy equipment, from
//! the size and speed signals available once a track has been observed
//! for a little while (see [`crate::tracking`]) — a heuristic classifier
//! over physical extent and observed speed, standing in for the
//! image/point-cloud classification model a fielded system would run.

/// A classified object category.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectClass {
    Human,
    LightVehicle,
    OtherEquipment,
    Unknown,
}

/// The physical signature a track accumulates: its observed bounding
/// width/height and the fastest speed observed so far.
#[derive(Debug, Clone, Copy)]
pub struct ObjectSignature {
    pub width_m: f32,
    pub height_m: f32,
    pub max_speed_observed_m_s: f32,
}

/// Classifies an object from its size and speed envelope. Thresholds are
/// representative (a walking human tops out well under running speed; a
/// light vehicle is much wider than a person but far narrower than a
/// haul truck or excavator), not a substitute for a trained classifier.
pub fn classify_object(signature: &ObjectSignature) -> ObjectClass {
    const HUMAN_MAX_WIDTH_M: f32 = 0.8;
    const HUMAN_MAX_SPEED_M_S: f32 = 4.0; // a sprint, generously
    const LIGHT_VEHICLE_MAX_WIDTH_M: f32 = 2.2;

    if signature.width_m <= 0.0 || signature.height_m <= 0.0 {
        return ObjectClass::Unknown;
    }
    if signature.width_m <= HUMAN_MAX_WIDTH_M
        && signature.max_speed_observed_m_s <= HUMAN_MAX_SPEED_M_S
    {
        ObjectClass::Human
    } else if signature.width_m <= LIGHT_VEHICLE_MAX_WIDTH_M {
        ObjectClass::LightVehicle
    } else {
        ObjectClass::OtherEquipment
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn narrow_slow_object_classifies_as_human() {
        let sig = ObjectSignature {
            width_m: 0.5,
            height_m: 1.7,
            max_speed_observed_m_s: 1.5,
        };
        assert_eq!(classify_object(&sig), ObjectClass::Human);
    }

    #[test]
    fn narrow_but_too_fast_for_a_human_is_not_classified_as_one() {
        let sig = ObjectSignature {
            width_m: 0.5,
            height_m: 1.0,
            max_speed_observed_m_s: 15.0,
        };
        assert_ne!(classify_object(&sig), ObjectClass::Human);
    }

    #[test]
    fn medium_width_classifies_as_light_vehicle() {
        let sig = ObjectSignature {
            width_m: 1.8,
            height_m: 1.5,
            max_speed_observed_m_s: 10.0,
        };
        assert_eq!(classify_object(&sig), ObjectClass::LightVehicle);
    }

    #[test]
    fn wide_object_classifies_as_other_equipment() {
        let sig = ObjectSignature {
            width_m: 6.0,
            height_m: 4.0,
            max_speed_observed_m_s: 8.0,
        };
        assert_eq!(classify_object(&sig), ObjectClass::OtherEquipment);
    }

    #[test]
    fn zero_size_signature_is_unknown() {
        let sig = ObjectSignature {
            width_m: 0.0,
            height_m: 0.0,
            max_speed_observed_m_s: 0.0,
        };
        assert_eq!(classify_object(&sig), ObjectClass::Unknown);
    }
}
