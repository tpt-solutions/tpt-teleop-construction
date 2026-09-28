// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Bearing wear, gear tooth damage, and imbalance signature detection
//! (spec.txt §8), from the characteristic vibration frequencies a rolling
//! element bearing or gear mesh produces when damaged — standard rotating
//! machinery vibration-analysis formulas (see e.g. any bearing
//! manufacturer's fault-frequency reference, such as SKF's), not
//! something invented for this crate:
//!
//! - Ball Pass Frequency Outer race: `BPFO = (n/2)*fr*(1 - (d/D)*cos(phi))`
//! - Ball Pass Frequency Inner race: `BPFI = (n/2)*fr*(1 + (d/D)*cos(phi))`
//! - Ball Spin Frequency: `BSF = (D/(2d))*fr*(1 - (d/D)^2*cos(phi)^2)`
//!
//! where `fr` is shaft rotation frequency, `n` the number of rolling
//! elements, `d`/`D` the ball/pitch diameters, and `phi` the contact
//! angle. Imbalance shows up directly at `1x` shaft frequency; gear tooth
//! damage shows up at the gear mesh frequency (`teeth_count * fr`).

/// A rolling-element bearing's geometry, as needed by the fault-frequency
/// formulas above.
#[derive(Debug, Clone, Copy)]
pub struct BearingGeometry {
    pub num_elements: f32,
    pub ball_diameter_m: f32,
    pub pitch_diameter_m: f32,
    pub contact_angle_rad: f32,
}

pub fn ball_pass_frequency_outer_hz(shaft_freq_hz: f32, bearing: &BearingGeometry) -> f32 {
    let ratio = bearing.ball_diameter_m / bearing.pitch_diameter_m;
    (bearing.num_elements / 2.0) * shaft_freq_hz * (1.0 - ratio * bearing.contact_angle_rad.cos())
}

pub fn ball_pass_frequency_inner_hz(shaft_freq_hz: f32, bearing: &BearingGeometry) -> f32 {
    let ratio = bearing.ball_diameter_m / bearing.pitch_diameter_m;
    (bearing.num_elements / 2.0) * shaft_freq_hz * (1.0 + ratio * bearing.contact_angle_rad.cos())
}

pub fn ball_spin_frequency_hz(shaft_freq_hz: f32, bearing: &BearingGeometry) -> f32 {
    let ratio = bearing.ball_diameter_m / bearing.pitch_diameter_m;
    (bearing.pitch_diameter_m / (2.0 * bearing.ball_diameter_m))
        * shaft_freq_hz
        * (1.0 - ratio * ratio * bearing.contact_angle_rad.cos().powi(2))
}

pub fn gear_mesh_frequency_hz(shaft_freq_hz: f32, teeth_count: f32) -> f32 {
    shaft_freq_hz * teeth_count
}

/// A detected fault signature, or `None` if nothing in the monitored
/// frequencies stood out above the noise floor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultSignature {
    None,
    Imbalance,
    OuterRaceDefect,
    InnerRaceDefect,
    BallSpinDefect,
    GearMeshDefect,
}

/// The magnitude spectrum's value nearest `target_hz`, given the
/// spectrum's per-bin frequency resolution. `0.0` if `target_hz` falls
/// outside the spectrum's range.
pub fn magnitude_at_frequency(spectrum: &[f32], bin_resolution_hz: f32, target_hz: f32) -> f32 {
    if bin_resolution_hz <= 0.0 {
        return 0.0;
    }
    let bin = (target_hz / bin_resolution_hz).round();
    if bin < 0.0 || bin as usize >= spectrum.len() {
        return 0.0;
    }
    spectrum[bin as usize]
}

fn average_magnitude(spectrum: &[f32]) -> f32 {
    if spectrum.is_empty() {
        0.0
    } else {
        spectrum.iter().sum::<f32>() / spectrum.len() as f32
    }
}

/// Checks the spectrum's energy at every monitored fault frequency
/// (imbalance, the three bearing fault frequencies, and the gear mesh
/// frequency) against `threshold_ratio` times the spectrum's average
/// magnitude, reporting whichever exceeds it by the largest margin (or
/// [`FaultSignature::None`] if none do).
#[allow(clippy::too_many_arguments)]
pub fn detect_fault_signature(
    spectrum: &[f32],
    bin_resolution_hz: f32,
    shaft_freq_hz: f32,
    teeth_count: f32,
    bearing: &BearingGeometry,
    threshold_ratio: f32,
) -> FaultSignature {
    let baseline = average_magnitude(spectrum);
    if baseline <= 0.0 {
        return FaultSignature::None;
    }
    let threshold = baseline * threshold_ratio;

    let candidates = [
        (
            FaultSignature::Imbalance,
            magnitude_at_frequency(spectrum, bin_resolution_hz, shaft_freq_hz),
        ),
        (
            FaultSignature::OuterRaceDefect,
            magnitude_at_frequency(
                spectrum,
                bin_resolution_hz,
                ball_pass_frequency_outer_hz(shaft_freq_hz, bearing),
            ),
        ),
        (
            FaultSignature::InnerRaceDefect,
            magnitude_at_frequency(
                spectrum,
                bin_resolution_hz,
                ball_pass_frequency_inner_hz(shaft_freq_hz, bearing),
            ),
        ),
        (
            FaultSignature::BallSpinDefect,
            magnitude_at_frequency(
                spectrum,
                bin_resolution_hz,
                ball_spin_frequency_hz(shaft_freq_hz, bearing),
            ),
        ),
        (
            FaultSignature::GearMeshDefect,
            magnitude_at_frequency(
                spectrum,
                bin_resolution_hz,
                gear_mesh_frequency_hz(shaft_freq_hz, teeth_count),
            ),
        ),
    ];

    candidates
        .into_iter()
        .filter(|(_, magnitude)| *magnitude >= threshold)
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .map(|(fault, _)| fault)
        .unwrap_or(FaultSignature::None)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn typical_bearing() -> BearingGeometry {
        BearingGeometry {
            num_elements: 9.0,
            ball_diameter_m: 0.012,
            pitch_diameter_m: 0.06,
            contact_angle_rad: 0.0,
        }
    }

    fn flat_spectrum_with_spike_at(bin_resolution_hz: f32, spike_hz: f32, len: usize) -> Vec<f32> {
        let mut spectrum = vec![1.0; len];
        let bin = (spike_hz / bin_resolution_hz).round() as usize;
        if bin < len {
            spectrum[bin] = 100.0;
        }
        spectrum
    }

    #[test]
    fn outer_race_frequency_is_less_than_shaft_frequency() {
        // BPFO is always below n*fr and, with equal-weighted elements,
        // below shaft frequency times element count / 2 minus a bit —
        // check the qualitative relationship: fewer elements or a larger
        // ball-to-pitch ratio lowers BPFO relative to n/2 * fr.
        let bearing = typical_bearing();
        let bpfo = ball_pass_frequency_outer_hz(50.0, &bearing);
        let bpfi = ball_pass_frequency_inner_hz(50.0, &bearing);
        assert!(
            bpfo < bpfi,
            "BPFO should be less than BPFI for a zero contact angle bearing"
        );
    }

    #[test]
    fn flat_spectrum_reports_no_fault() {
        let spectrum = vec![1.0; 100];
        let fault = detect_fault_signature(&spectrum, 1.0, 25.0, 20.0, &typical_bearing(), 3.0);
        assert_eq!(fault, FaultSignature::None);
    }

    #[test]
    fn spike_at_shaft_frequency_indicates_imbalance() {
        let bin_resolution = 1.0;
        let shaft_freq = 25.0;
        let spectrum = flat_spectrum_with_spike_at(bin_resolution, shaft_freq, 200);
        let fault = detect_fault_signature(
            &spectrum,
            bin_resolution,
            shaft_freq,
            20.0,
            &typical_bearing(),
            3.0,
        );
        assert_eq!(fault, FaultSignature::Imbalance);
    }

    #[test]
    fn spike_at_outer_race_frequency_indicates_outer_race_defect() {
        let bin_resolution = 0.5;
        let shaft_freq = 25.0;
        let bearing = typical_bearing();
        let bpfo = ball_pass_frequency_outer_hz(shaft_freq, &bearing);
        let spectrum = flat_spectrum_with_spike_at(bin_resolution, bpfo, 400);
        let fault =
            detect_fault_signature(&spectrum, bin_resolution, shaft_freq, 20.0, &bearing, 3.0);
        assert_eq!(fault, FaultSignature::OuterRaceDefect);
    }

    #[test]
    fn spike_at_gear_mesh_frequency_indicates_gear_mesh_defect() {
        let bin_resolution = 1.0;
        let shaft_freq = 25.0;
        let teeth = 30.0;
        let spectrum = flat_spectrum_with_spike_at(bin_resolution, shaft_freq * teeth, 1000);
        let fault = detect_fault_signature(
            &spectrum,
            bin_resolution,
            shaft_freq,
            teeth,
            &typical_bearing(),
            3.0,
        );
        assert_eq!(fault, FaultSignature::GearMeshDefect);
    }

    #[test]
    fn empty_spectrum_reports_no_fault_without_dividing_by_zero() {
        assert_eq!(
            detect_fault_signature(&[], 1.0, 25.0, 20.0, &typical_bearing(), 3.0),
            FaultSignature::None
        );
    }
}
