// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Diesel/pump vibration filtering on operator joystick control inputs
//! (todo.md Phase 10: "Diesel-vibration filtering on joystick control
//! inputs"), reusing Phase 3's [`tpt_t_construction_hydraulic::Biquad`]
//! band-stop filter rather than a second implementation — the same
//! 50-200Hz engine/pump noise band that `tpt-t-construction-hydraulic`
//! already rejects on its sensor feedback path also leaks into an
//! operator's hand on a joystick mounted in the cab, so it's filtered out
//! before a [`crate::control_command::DecodedControlCommand`]'s axes ever
//! reach [`crate::valve_mapping`].

use tpt_t_construction_hydraulic::Biquad;

/// One independent [`Biquad`] band-stop filter per control axis. `AXES`
/// is fixed at construction time by the const generic, so this never
/// allocates.
pub struct JoystickFilterBank<const AXES: usize> {
    filters: [Biquad; AXES],
}

impl<const AXES: usize> JoystickFilterBank<AXES> {
    /// Builds a bank rejecting `low_hz..=high_hz` on every axis, each
    /// filter independently initialized (no shared state between axes).
    pub fn new(low_hz: f32, high_hz: f32, sample_rate_hz: f32) -> Self {
        JoystickFilterBank {
            filters: std::array::from_fn(|_| Biquad::reject_band(low_hz, high_hz, sample_rate_hz)),
        }
    }

    /// Filters one sample per axis, advancing each filter's internal state
    /// by exactly one step.
    pub fn filter(&mut self, axes: &[f32; AXES]) -> [f32; AXES] {
        let mut out = [0.0f32; AXES];
        for (i, filter) in self.filters.iter_mut().enumerate() {
            out[i] = filter.process(axes[i]);
        }
        out
    }

    pub fn reset(&mut self) {
        for filter in &mut self.filters {
            filter.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dc_input_passes_through_a_band_stop_filter() {
        // A band-stop filter rejects 50-200Hz but should pass a
        // (near-)constant command input essentially unattenuated once
        // settled, same contract as the single-axis Biquad in
        // `tpt-t-construction-hydraulic`.
        let mut bank = JoystickFilterBank::<4>::new(50.0, 200.0, 1000.0);
        let input = [0.5, -0.3, 1.0, 0.0];
        let mut last = input;
        for _ in 0..500 {
            last = bank.filter(&input);
        }
        for (out, inp) in last.iter().zip(input) {
            assert!((out - inp).abs() < 0.05, "out={out} inp={inp}");
        }
    }

    #[test]
    fn axes_are_filtered_independently() {
        let mut bank = JoystickFilterBank::<2>::new(50.0, 200.0, 1000.0);
        // Drive axis 0 with a steady command and axis 1 with a burst of
        // in-band noise; axis 0's output should be unaffected by axis 1's
        // input.
        let mut baseline = [0.0f32; 2];
        for _ in 0..200 {
            baseline = bank.filter(&[0.5, 0.0]);
        }

        let mut bank2 = JoystickFilterBank::<2>::new(50.0, 200.0, 1000.0);
        let mut noisy = [0.0f32; 2];
        for i in 0..200 {
            let noise = if i % 2 == 0 { 1.0 } else { -1.0 };
            noisy = bank2.filter(&[0.5, noise]);
        }

        assert!((noisy[0] - baseline[0]).abs() < 0.05);
    }

    #[test]
    fn reset_clears_filter_state() {
        let mut bank = JoystickFilterBank::<1>::new(50.0, 200.0, 1000.0);
        for _ in 0..100 {
            bank.filter(&[1.0]);
        }
        bank.reset();
        let first_after_reset = bank.filter(&[0.0]);
        assert_eq!(first_after_reset, [0.0]);
    }
}
