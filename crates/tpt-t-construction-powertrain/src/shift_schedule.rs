// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A production automatic-transmission shift schedule: gear changes
//! decided from vehicle speed *and* throttle position, each gear
//! transition with its own up/downshift speeds and built-in hysteresis.
//!
//! This is deliberately not the same model as
//! `tpt-t-construction-sim::powertrain::Transmission`, which shifts on a
//! single RPM threshold — that's a simplification appropriate for a
//! physics *plant* under test, not how a real controller decides. A real
//! shift schedule holds a lower gear longer under heavy throttle (to keep
//! power available, e.g. climbing out of a dig face) and upshifts earlier
//! at light throttle (for fuel economy and smoothness), and every
//! transition's downshift speed sits below its upshift speed so the
//! transmission doesn't hunt back and forth at a boundary speed.

/// One gear transition's shift speeds, each linearly varying with
/// throttle position between a 0%-throttle and 100%-throttle anchor.
#[derive(Debug, Clone, Copy)]
pub struct ShiftPoint {
    pub upshift_speed_at_zero_throttle_m_s: f32,
    pub upshift_speed_at_full_throttle_m_s: f32,
    pub downshift_speed_at_zero_throttle_m_s: f32,
    pub downshift_speed_at_full_throttle_m_s: f32,
}

impl ShiftPoint {
    fn upshift_speed_m_s(&self, throttle: f32) -> f32 {
        lerp(
            self.upshift_speed_at_zero_throttle_m_s,
            self.upshift_speed_at_full_throttle_m_s,
            throttle,
        )
    }

    fn downshift_speed_m_s(&self, throttle: f32) -> f32 {
        lerp(
            self.downshift_speed_at_zero_throttle_m_s,
            self.downshift_speed_at_full_throttle_m_s,
            throttle,
        )
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    a + (b - a) * t
}

/// A fixed set of `N` gear-transition [`ShiftPoint`]s governing an
/// `N + 1`-speed transmission (transition `i` governs shifting between
/// gear `i` and gear `i + 1`). Stack-allocated: `N` is fixed at
/// construction, no heap storage.
#[derive(Debug, Clone)]
pub struct ShiftSchedule<const N: usize> {
    points: [ShiftPoint; N],
    current_gear: usize,
}

impl<const N: usize> ShiftSchedule<N> {
    pub fn new(points: [ShiftPoint; N]) -> Self {
        ShiftSchedule {
            points,
            current_gear: 0,
        }
    }

    pub fn current_gear(&self) -> usize {
        self.current_gear
    }

    pub fn num_gears(&self) -> usize {
        N + 1
    }

    /// Evaluates the shift schedule against the current vehicle speed and
    /// throttle position, applying at most one gear change (up or down)
    /// per call — a real transmission doesn't skip gears in response to a
    /// single large speed change between control cycles.
    pub fn update(&mut self, speed_m_s: f32, throttle: f32) {
        if self.current_gear < N {
            let point = &self.points[self.current_gear];
            if speed_m_s >= point.upshift_speed_m_s(throttle) {
                self.current_gear += 1;
                return;
            }
        }
        if self.current_gear > 0 {
            let point = &self.points[self.current_gear - 1];
            if speed_m_s <= point.downshift_speed_m_s(throttle) {
                self.current_gear -= 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn two_speed_schedule() -> ShiftSchedule<1> {
        ShiftSchedule::new([ShiftPoint {
            upshift_speed_at_zero_throttle_m_s: 5.0,
            upshift_speed_at_full_throttle_m_s: 10.0,
            downshift_speed_at_zero_throttle_m_s: 3.0,
            downshift_speed_at_full_throttle_m_s: 6.0,
        }])
    }

    #[test]
    fn starts_in_first_gear() {
        let schedule = two_speed_schedule();
        assert_eq!(schedule.current_gear(), 0);
        assert_eq!(schedule.num_gears(), 2);
    }

    #[test]
    fn upshifts_once_speed_crosses_the_threshold_at_light_throttle() {
        let mut schedule = two_speed_schedule();
        schedule.update(4.0, 0.0);
        assert_eq!(schedule.current_gear(), 0);
        schedule.update(5.5, 0.0);
        assert_eq!(schedule.current_gear(), 1);
    }

    #[test]
    fn heavy_throttle_delays_the_upshift_to_a_higher_speed() {
        let mut light = two_speed_schedule();
        let mut heavy = two_speed_schedule();
        light.update(6.0, 0.0);
        heavy.update(6.0, 1.0);
        assert_eq!(
            light.current_gear(),
            1,
            "light throttle should have upshifted by 6 m/s"
        );
        assert_eq!(
            heavy.current_gear(),
            0,
            "full throttle should hold gear past 6 m/s"
        );
        heavy.update(11.0, 1.0);
        assert_eq!(heavy.current_gear(), 1);
    }

    #[test]
    fn downshifts_once_speed_drops_below_the_threshold() {
        let mut schedule = two_speed_schedule();
        schedule.update(6.0, 0.0);
        assert_eq!(schedule.current_gear(), 1);
        schedule.update(2.0, 0.0);
        assert_eq!(schedule.current_gear(), 0);
    }

    #[test]
    fn hysteresis_band_does_not_hunt_at_a_boundary_speed() {
        let mut schedule = two_speed_schedule();
        schedule.update(6.0, 0.0); // upshift at 5.0
        assert_eq!(schedule.current_gear(), 1);
        // 4.0 m/s is below the upshift point but above the downshift
        // point (3.0): a schedule without hysteresis would ping-pong here.
        schedule.update(4.0, 0.0);
        assert_eq!(schedule.current_gear(), 1);
    }

    #[test]
    fn cannot_shift_below_first_or_above_top_gear() {
        let mut schedule = two_speed_schedule();
        schedule.update(0.0, 0.0);
        assert_eq!(schedule.current_gear(), 0);
        schedule.update(100.0, 1.0);
        assert_eq!(schedule.current_gear(), 1);
        schedule.update(100.0, 1.0);
        assert_eq!(schedule.current_gear(), 1);
    }

    #[test]
    fn one_shift_per_update_even_across_a_large_speed_jump() {
        let schedule_points = [
            ShiftPoint {
                upshift_speed_at_zero_throttle_m_s: 2.0,
                upshift_speed_at_full_throttle_m_s: 2.0,
                downshift_speed_at_zero_throttle_m_s: 1.0,
                downshift_speed_at_full_throttle_m_s: 1.0,
            },
            ShiftPoint {
                upshift_speed_at_zero_throttle_m_s: 4.0,
                upshift_speed_at_full_throttle_m_s: 4.0,
                downshift_speed_at_zero_throttle_m_s: 3.0,
                downshift_speed_at_full_throttle_m_s: 3.0,
            },
        ];
        let mut schedule: ShiftSchedule<2> = ShiftSchedule::new(schedule_points);
        // A single huge jump should only move one gear at a time.
        schedule.update(100.0, 0.0);
        assert_eq!(schedule.current_gear(), 1);
        schedule.update(100.0, 0.0);
        assert_eq!(schedule.current_gear(), 2);
    }
}
