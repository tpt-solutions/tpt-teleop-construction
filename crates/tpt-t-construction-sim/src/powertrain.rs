// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Diesel engine torque curves and transmission gear shifting, for the
//! sim's drivetrain model. `tpt-t-construction-powertrain` (Phase 3) is
//! the real controller this feeds a plant model for; this module only
//! simulates the engine/transmission physics themselves.

/// A piecewise-linear engine torque curve: RPM breakpoints (strictly
/// increasing) each paired with the torque (N*m) the engine can deliver at
/// full fueling and that RPM.
#[derive(Debug, Clone)]
pub struct EngineTorqueCurve {
    points: Vec<(f32, f32)>,
}

impl EngineTorqueCurve {
    /// Builds a curve from `(rpm, torque_nm)` breakpoints. Panics (in
    /// debug builds, via the assert) if fewer than two points are given or
    /// RPM values are not strictly increasing — a malformed curve is a
    /// configuration bug, not a runtime condition to recover from.
    pub fn new(points: Vec<(f32, f32)>) -> Self {
        assert!(points.len() >= 2, "torque curve needs at least 2 points");
        assert!(
            points.windows(2).all(|w| w[1].0 > w[0].0),
            "torque curve RPM breakpoints must be strictly increasing"
        );
        EngineTorqueCurve { points }
    }

    /// A representative heavy-equipment diesel curve: torque rises from
    /// idle to a peak in the mid range, then falls off toward the
    /// governed high-RPM limit, per spec.txt §8's "diesel engine torque
    /// curves" requirement.
    pub fn diesel_default() -> Self {
        EngineTorqueCurve::new(vec![
            (600.0, 800.0),   // idle
            (1200.0, 2400.0), // peak torque
            (1800.0, 2200.0), // rated power point
            (2200.0, 1200.0), // governed high-RPM falloff
        ])
    }

    /// Full-fueling torque (N*m) at `rpm`, linearly interpolated between
    /// breakpoints and clamped to the curve's endpoints outside its range.
    pub fn torque_at_nm(&self, rpm: f32) -> f32 {
        if rpm <= self.points[0].0 {
            return self.points[0].1;
        }
        let last = self.points.len() - 1;
        if rpm >= self.points[last].0 {
            return self.points[last].1;
        }
        let idx = self
            .points
            .windows(2)
            .position(|w| rpm >= w[0].0 && rpm <= w[1].0)
            .unwrap();
        let (rpm_lo, torque_lo) = self.points[idx];
        let (rpm_hi, torque_hi) = self.points[idx + 1];
        let t = (rpm - rpm_lo) / (rpm_hi - rpm_lo);
        torque_lo + (torque_hi - torque_lo) * t
    }

    /// Scales full-fueling torque by a throttle/fuel command in `[0, 1]`
    /// (idle torque is still delivered at `throttle = 0`, matching a real
    /// diesel's idle behavior rather than cutting fuel entirely).
    pub fn torque_at_nm_throttled(&self, rpm: f32, throttle: f32) -> f32 {
        let throttle = throttle.clamp(0.0, 1.0);
        let idle = self.points[0].1;
        let full = self.torque_at_nm(rpm);
        idle + (full - idle) * throttle
    }
}

/// A discrete-ratio transmission with RPM-threshold up/downshift logic.
#[derive(Debug, Clone)]
pub struct Transmission {
    pub gear_ratios: Vec<f32>,
    pub final_drive_ratio: f32,
    pub current_gear: usize,
    pub upshift_rpm: f32,
    pub downshift_rpm: f32,
}

impl Transmission {
    pub fn new(
        gear_ratios: Vec<f32>,
        final_drive_ratio: f32,
        upshift_rpm: f32,
        downshift_rpm: f32,
    ) -> Self {
        assert!(
            !gear_ratios.is_empty(),
            "transmission needs at least 1 gear"
        );
        assert!(
            upshift_rpm > downshift_rpm,
            "upshift_rpm must exceed downshift_rpm or the transmission will hunt"
        );
        Transmission {
            gear_ratios,
            final_drive_ratio,
            current_gear: 0,
            upshift_rpm,
            downshift_rpm,
        }
    }

    /// Overall ratio (engine turns per output-shaft turn) in the current gear.
    pub fn overall_ratio(&self) -> f32 {
        self.gear_ratios[self.current_gear] * self.final_drive_ratio
    }

    /// Output (wheel/sprocket) torque (N*m) for a given engine torque, at
    /// the current gear, before drivetrain losses.
    pub fn output_torque_nm(&self, engine_torque_nm: f32) -> f32 {
        engine_torque_nm * self.overall_ratio()
    }

    /// Applies simple threshold-based automatic shift logic: upshift above
    /// `upshift_rpm`, downshift below `downshift_rpm`, one gear at a time
    /// per call (matching a real automatic's one-shift-per-event behavior
    /// and avoiding skipping gears on a single large RPM excursion).
    pub fn update_shift(&mut self, engine_rpm: f32) {
        if engine_rpm > self.upshift_rpm && self.current_gear + 1 < self.gear_ratios.len() {
            self.current_gear += 1;
        } else if engine_rpm < self.downshift_rpm && self.current_gear > 0 {
            self.current_gear -= 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn torque_curve_interpolates_between_breakpoints() {
        let curve = EngineTorqueCurve::new(vec![(1000.0, 100.0), (2000.0, 300.0)]);
        assert_eq!(curve.torque_at_nm(1500.0), 200.0);
    }

    #[test]
    fn torque_curve_clamps_outside_range() {
        let curve = EngineTorqueCurve::new(vec![(1000.0, 100.0), (2000.0, 300.0)]);
        assert_eq!(curve.torque_at_nm(0.0), 100.0);
        assert_eq!(curve.torque_at_nm(5000.0), 300.0);
    }

    #[test]
    fn diesel_default_has_a_torque_peak_below_redline() {
        let curve = EngineTorqueCurve::diesel_default();
        let peak = curve.torque_at_nm(1200.0);
        let redline = curve.torque_at_nm(2200.0);
        assert!(peak > redline);
    }

    #[test]
    fn throttle_scales_between_idle_and_full_torque() {
        let curve = EngineTorqueCurve::diesel_default();
        let idle_torque = curve.torque_at_nm(1200.0);
        assert!(curve.torque_at_nm_throttled(1200.0, 0.0) < idle_torque);
        assert_eq!(
            curve.torque_at_nm_throttled(1200.0, 1.0),
            curve.torque_at_nm(1200.0)
        );
    }

    #[test]
    fn transmission_upshifts_and_downshifts_at_thresholds() {
        let mut tx = Transmission::new(vec![4.0, 2.0, 1.0], 3.5, 2000.0, 1000.0);
        assert_eq!(tx.current_gear, 0);
        tx.update_shift(2100.0);
        assert_eq!(tx.current_gear, 1);
        tx.update_shift(2100.0);
        assert_eq!(tx.current_gear, 2);
        tx.update_shift(2100.0); // already top gear, stays
        assert_eq!(tx.current_gear, 2);
        tx.update_shift(900.0);
        assert_eq!(tx.current_gear, 1);
    }

    #[test]
    fn output_torque_multiplies_by_overall_ratio() {
        let tx = Transmission::new(vec![4.0], 2.0, 2000.0, 1000.0);
        assert_eq!(tx.output_torque_nm(100.0), 800.0);
    }
}
