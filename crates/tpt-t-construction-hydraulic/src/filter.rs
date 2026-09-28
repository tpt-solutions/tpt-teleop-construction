// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Vibration filtering: a biquad band-stop (notch) filter for rejecting
//! the 50-200Hz diesel engine/hydraulic pump noise spec.txt §4.1 calls
//! out, so the control loop sees the true boom/arm/bucket position signal
//! rather than that vibration riding on top of it.
//!
//! Coefficients follow Robert Bristow-Johnson's widely used "Audio EQ
//! Cookbook" band-stop formulas — a standard, well-analyzed biquad
//! design, not something bespoke to get subtly wrong. State is four
//! `f32`s (`x1`, `x2`, `y1`, `y2`); no allocation, no lookup tables.

use std::f32::consts::PI;

/// A single biquad (2nd-order IIR) filter section in Direct Form I.
#[derive(Debug, Clone, Copy)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    /// A band-stop filter centered on `center_freq_hz` with quality factor
    /// `q` (higher `q` = narrower stop-band), sampling at `sample_rate_hz`.
    pub fn notch(center_freq_hz: f32, sample_rate_hz: f32, q: f32) -> Self {
        let w0 = 2.0 * PI * center_freq_hz / sample_rate_hz;
        let cos_w0 = w0.cos();
        let alpha = w0.sin() / (2.0 * q);
        let a0 = 1.0 + alpha;
        Biquad {
            b0: 1.0 / a0,
            b1: -2.0 * cos_w0 / a0,
            b2: 1.0 / a0,
            a1: -2.0 * cos_w0 / a0,
            a2: (1.0 - alpha) / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// A band-stop filter covering `[low_hz, high_hz]`: center frequency
    /// is the geometric mean of the band edges, and `Q` is derived from
    /// the band's fractional bandwidth so the whole named range is
    /// meaningfully attenuated rather than just its center point.
    /// Convenience for exactly spec.txt §4.1's "reject the 50-200Hz
    /// vibration noise" requirement.
    pub fn reject_band(low_hz: f32, high_hz: f32, sample_rate_hz: f32) -> Self {
        assert!(
            low_hz > 0.0 && high_hz > low_hz,
            "low_hz must be positive and less than high_hz"
        );
        let center_hz = (low_hz * high_hz).sqrt();
        let q = center_hz / (high_hz - low_hz);
        Biquad::notch(center_hz, sample_rate_hz, q)
    }

    /// Filters one sample, returning the filtered output and advancing
    /// the filter's internal state.
    pub fn process(&mut self, input: f32) -> f32 {
        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;
        output
    }

    /// Clears the filter's history (e.g. after a discontinuity like a
    /// sensor reconnect, to avoid the old signal's tail ringing into new
    /// readings).
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// RMS amplitude of a filter's steady-state response to a pure sine
    /// at `freq_hz`, discarding the first half of the run as filter
    /// settling transient.
    fn steady_state_rms(
        filter: &mut Biquad,
        freq_hz: f32,
        sample_rate_hz: f32,
        samples: usize,
    ) -> f32 {
        let mut sum_sq = 0.0_f64;
        let mut counted = 0usize;
        for n in 0..samples {
            let t = n as f32 / sample_rate_hz;
            let input = (2.0 * PI * freq_hz * t).sin();
            let output = filter.process(input);
            if n >= samples / 2 {
                sum_sq += (output as f64) * (output as f64);
                counted += 1;
            }
        }
        (sum_sq / counted as f64).sqrt() as f32
    }

    #[test]
    fn passes_frequencies_well_outside_the_stop_band() {
        let sample_rate = 10_000.0;
        let mut filter = Biquad::reject_band(50.0, 200.0, sample_rate);
        let rms = steady_state_rms(&mut filter, 2000.0, sample_rate, 4000);
        // A pure sine has RMS = amplitude / sqrt(2) ~= 0.707; well outside
        // the stop band this should be nearly unattenuated.
        assert!(
            rms > 0.6,
            "expected near-unity gain far from the stop band, got rms={rms}"
        );
    }

    #[test]
    fn attenuates_frequencies_inside_the_stop_band() {
        let sample_rate = 10_000.0;
        let mut filter = Biquad::reject_band(50.0, 200.0, sample_rate);
        // The filter centers on the geometric mean of the band edges
        // (sqrt(50*200) = 100Hz, not the arithmetic mean) — see
        // `reject_band`'s doc comment.
        let rms = steady_state_rms(&mut filter, 100.0, sample_rate, 4000);
        assert!(
            rms < 0.1,
            "expected strong attenuation at the stop band center, got rms={rms}"
        );
    }

    #[test]
    fn attenuates_across_the_whole_named_band_not_just_its_center() {
        let sample_rate = 10_000.0;
        for freq in [60.0, 100.0, 150.0, 190.0] {
            let mut filter = Biquad::reject_band(50.0, 200.0, sample_rate);
            let rms = steady_state_rms(&mut filter, freq, sample_rate, 4000);
            assert!(
                rms < 0.6,
                "expected some attenuation at {freq}Hz within the named band, got rms={rms}"
            );
        }
    }

    #[test]
    fn reset_clears_filter_state() {
        let mut filter = Biquad::reject_band(50.0, 200.0, 10_000.0);
        for _ in 0..100 {
            filter.process(1.0);
        }
        filter.reset();
        // Immediately after reset, a zero input should produce exactly
        // zero output (no leftover state contributing).
        assert_eq!(filter.process(0.0), 0.0);
    }
}
