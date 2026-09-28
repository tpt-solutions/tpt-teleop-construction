// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A naive O(n^2) discrete Fourier transform, used only to give the
//! "vibration frequency spectra" view (spec.txt §8) something plausible to
//! plot from a synthetic signal. `tpt-t-construction-wear`'s real
//! predictive-maintenance vibration analysis (Phase 8) is a dedicated
//! SIMD FFT over pre-allocated buffers processing live 10kHz accelerometer
//! data — nothing here is a stand-in for that; this module exists only so
//! the visualizer doesn't have to depend on a crate that isn't built yet.

/// Magnitude spectrum (bins `0..len/2`, i.e. up to Nyquist) of `samples`
/// via a direct O(n^2) DFT. Fine for the small (~100-sample) demo buffers
/// this crate plots; not suitable for the 10kHz-class real-time analysis
/// `tpt-t-construction-wear` will need.
pub fn naive_dft_magnitude(samples: &[f32]) -> Vec<f32> {
    let n = samples.len();
    if n == 0 {
        return Vec::new();
    }
    (0..n / 2)
        .map(|k| {
            let mut re = 0.0f32;
            let mut im = 0.0f32;
            for (t, &x) in samples.iter().enumerate() {
                let angle = -2.0 * std::f32::consts::PI * (k as f32) * (t as f32) / (n as f32);
                re += x * angle.cos();
                im += x * angle.sin();
            }
            (re * re + im * im).sqrt()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_yields_empty_spectrum() {
        assert!(naive_dft_magnitude(&[]).is_empty());
    }

    #[test]
    fn pure_tone_peaks_at_its_own_bin() {
        let n = 64;
        let bin = 5;
        let samples: Vec<f32> = (0..n)
            .map(|t| (2.0 * std::f32::consts::PI * bin as f32 * t as f32 / n as f32).sin())
            .collect();
        let spectrum = naive_dft_magnitude(&samples);
        let peak_bin = spectrum
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak_bin, bin);
    }

    #[test]
    fn dc_signal_has_energy_only_in_bin_zero() {
        let samples = vec![1.0f32; 32];
        let spectrum = naive_dft_magnitude(&samples);
        assert!(spectrum[0] > 10.0);
        for &m in &spectrum[1..] {
            assert!(m < 1e-3);
        }
    }
}
