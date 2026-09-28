// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Custom vibration-analysis FFT (spec.txt §8: "custom SIMD FFT
//! (pre-allocated), 10kHz, dedicated core"). This is an iterative
//! radix-2 Cooley-Tukey FFT (genuinely `O(n log n)`, not the naive
//! `O(n^2)` DFT `tpt-t-construction-viz` uses purely as a visualization
//! placeholder), with its twiddle-factor table and bit-reversal
//! permutation table precomputed once at construction — the
//! "pre-allocated" half of the requirement — so a call to
//! [`FftProcessor::process`] does no allocation at all.
//!
//! The "SIMD" half: `core::simd` ("portable-simd") remains nightly-only
//! (rust-lang/rust#86656) as of this toolchain, so — as in
//! `tpt-t-construction-payload` and `tpt-t-construction-mine` — each
//! butterfly stage's inner loop is written over independent, non-
//! overlapping element pairs specifically so LLVM's auto-vectorizer can
//! pack it into SIMD instructions on stable Rust, rather than depending on
//! unstable explicit intrinsics.

use crate::complex::Complex32;
use std::f32::consts::PI;

fn reverse_bits(mut value: usize, bit_count: u32) -> usize {
    let mut result = 0usize;
    for _ in 0..bit_count {
        result = (result << 1) | (value & 1);
        value >>= 1;
    }
    result
}

/// A reusable FFT processor for a fixed transform length `N` (which must
/// be a power of two). Construction precomputes everything that doesn't
/// depend on the input signal; [`FftProcessor::process`] then runs with
/// zero allocation, suitable for a dedicated real-time core processing a
/// continuous 10kHz vibration stream in fixed-size windows.
pub struct FftProcessor<const N: usize> {
    twiddles: [Complex32; N],
    bit_reversal: [usize; N],
}

impl<const N: usize> FftProcessor<N> {
    pub fn new() -> Self {
        assert!(N.is_power_of_two(), "FFT length must be a power of two");
        let mut twiddles = [Complex32::ZERO; N];
        for (k, twiddle) in twiddles.iter_mut().enumerate().take(N / 2) {
            let angle = -2.0 * PI * k as f32 / N as f32;
            *twiddle = Complex32::new(angle.cos(), angle.sin());
        }

        let bit_count = N.trailing_zeros();
        let mut bit_reversal = [0usize; N];
        for (i, slot) in bit_reversal.iter_mut().enumerate() {
            *slot = reverse_bits(i, bit_count);
        }

        FftProcessor {
            twiddles,
            bit_reversal,
        }
    }

    /// Transforms `data` in place (time domain in, frequency domain out).
    pub fn process(&self, data: &mut [Complex32; N]) {
        for i in 0..N {
            let j = self.bit_reversal[i];
            if i < j {
                data.swap(i, j);
            }
        }

        let mut stage_size = 2;
        while stage_size <= N {
            let half = stage_size / 2;
            let stride = N / stage_size;
            let mut start = 0;
            while start < N {
                for k in 0..half {
                    let twiddle = self.twiddles[k * stride];
                    let even = data[start + k];
                    let odd_twiddled = data[start + k + half] * twiddle;
                    data[start + k] = even + odd_twiddled;
                    data[start + k + half] = even - odd_twiddled;
                }
                start += stage_size;
            }
            stage_size *= 2;
        }
    }

    /// Convenience: transforms real-valued `samples` (imaginary part
    /// zero) and returns the magnitude spectrum's first `N/2` bins (up to
    /// Nyquist — the upper half is a mirror image for real input and
    /// carries no additional information).
    pub fn magnitude_spectrum(&self, samples: &[f32; N]) -> [f32; N] {
        let mut data: [Complex32; N] = std::array::from_fn(|i| Complex32::new(samples[i], 0.0));
        self.process(&mut data);
        std::array::from_fn(|i| data[i].magnitude())
    }
}

impl<const N: usize> Default for FftProcessor<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() <= eps
    }

    #[test]
    fn dc_signal_transforms_to_an_impulse_at_bin_zero() {
        let fft: FftProcessor<16> = FftProcessor::new();
        let samples = [1.0f32; 16];
        let spectrum = fft.magnitude_spectrum(&samples);
        assert!(approx_eq(spectrum[0], 16.0, 1e-3));
        for &m in &spectrum[1..] {
            assert!(m < 1e-3);
        }
    }

    #[test]
    fn pure_tone_peaks_at_its_own_bin() {
        // A real-valued signal's spectrum is mirror-symmetric (bin `k`
        // and bin `N-k` carry equal energy), so the peak search is
        // restricted to `0..N/2` (up to Nyquist) — exactly the range
        // `magnitude_spectrum`'s own docs say carries unique information.
        // Searching the full spectrum would find both mirror bins tied
        // for the maximum, and which one `max_by` returns in a tie is
        // not what this test cares about.
        let n = 64;
        let fft: FftProcessor<64> = FftProcessor::new();
        let bin = 5;
        let samples: [f32; 64] =
            std::array::from_fn(|t| (2.0 * PI * bin as f32 * t as f32 / n as f32).sin());
        let spectrum = fft.magnitude_spectrum(&samples);
        let peak_bin = spectrum[..n / 2]
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak_bin, bin);
    }

    #[test]
    fn matches_a_naive_reference_dft() {
        let n = 32;
        let fft: FftProcessor<32> = FftProcessor::new();
        let samples: [f32; 32] = std::array::from_fn(|t| ((t * 7) % 11) as f32 * 0.3 - 1.0);
        let spectrum = fft.magnitude_spectrum(&samples);

        for (k, &fast_mag) in spectrum.iter().enumerate() {
            let mut re = 0.0f32;
            let mut im = 0.0f32;
            for (t, &x) in samples.iter().enumerate() {
                let angle = -2.0 * PI * k as f32 * t as f32 / n as f32;
                re += x * angle.cos();
                im += x * angle.sin();
            }
            let naive_mag = (re * re + im * im).sqrt();
            assert!(
                approx_eq(fast_mag, naive_mag, 1e-2),
                "bin {k}: fft={fast_mag} naive={naive_mag}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "power of two")]
    fn non_power_of_two_length_panics() {
        let _fft: FftProcessor<100> = FftProcessor::new();
    }

    #[test]
    fn a_1024_point_window_processes_in_well_under_the_10khz_frame_period() {
        // At 10kHz sampling, a 1024-sample analysis window spans
        // ~102ms of real time; the transform itself must take a small
        // fraction of that to leave headroom for everything else the
        // dedicated core does each cycle.
        let fft: FftProcessor<1024> = FftProcessor::new();
        let samples: [f32; 1024] =
            std::array::from_fn(|t| (2.0 * PI * 137.0 * t as f32 / 1024.0).sin());

        let start = Instant::now();
        let _ = fft.magnitude_spectrum(&samples);
        let elapsed = start.elapsed();

        assert!(
            elapsed.as_millis() < 10,
            "1024-point FFT took {elapsed:?}, budget is 10ms"
        );
    }
}
