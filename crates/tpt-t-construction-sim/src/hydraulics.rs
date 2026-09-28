// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Hydraulic system dynamics: proportional valve response, orifice flow,
//! and cylinder pressure/force build-up. This is the *simulated plant*
//! that `tpt-t-construction-hydraulic`'s real 1kHz PID controller
//! (Phase 3) will eventually be tested against in the loop — it does not
//! implement that controller itself.

/// A proportional spool valve modeled as a first-order lag onto its
/// commanded position: spec.txt §1 puts real electro-hydraulic proportional
/// valve response times at ~200ms, which is exactly what `tau_s` is for.
#[derive(Debug, Clone, Copy)]
pub struct SpoolValve {
    /// Time constant of the first-order response, seconds.
    pub tau_s: f32,
    /// Current normalized position, `-1.0` (full flow B) to `1.0`
    /// (full flow A), `0.0` centered/closed.
    pub position: f32,
}

impl SpoolValve {
    pub fn new(tau_s: f32) -> Self {
        SpoolValve {
            tau_s,
            position: 0.0,
        }
    }

    /// Advances the valve toward `commanded` (clamped to `[-1, 1]`) by one
    /// step, returning the new position.
    pub fn step(&mut self, commanded: f32, dt_s: f32) -> f32 {
        let commanded = commanded.clamp(-1.0, 1.0);
        let alpha = (dt_s / self.tau_s).clamp(0.0, 1.0);
        self.position += (commanded - self.position) * alpha;
        self.position
    }
}

/// Volumetric flow (m^3/s) through an orifice of effective area
/// `discharge_coeff * max_area_m2 * valve_opening`, driven by
/// `delta_pressure_pa` across it, per the standard orifice equation
/// `Q = Cd * A * sqrt(2 * |dP| / rho)`, signed to match the pressure drop
/// direction.
pub fn orifice_flow_m3_s(
    valve_opening: f32,
    discharge_coeff: f32,
    max_area_m2: f32,
    delta_pressure_pa: f32,
    fluid_density_kg_m3: f32,
) -> f32 {
    let area = discharge_coeff * max_area_m2 * valve_opening.abs();
    let magnitude = area * (2.0 * delta_pressure_pa.abs() / fluid_density_kg_m3).sqrt();
    magnitude.copysign(delta_pressure_pa)
}

/// A single-rod double-acting hydraulic cylinder: two chambers (`a`, the
/// bore side; `b`, the rod side), each with its own trapped-fluid
/// pressure that builds up from net flow via the fluid's bulk-modulus
/// continuity equation `dP/dt = (beta / V) * (Q_in - A * v_piston)`.
#[derive(Debug, Clone, Copy)]
pub struct Cylinder {
    pub bore_area_m2: f32,
    pub rod_area_m2: f32,
    pub pressure_a_pa: f32,
    pub pressure_b_pa: f32,
    pub volume_a_m3: f32,
    pub volume_b_m3: f32,
    pub bulk_modulus_pa: f32,
}

impl Cylinder {
    /// Net actuation force (N), bore side pushing positive against rod
    /// side pushing negative, before subtracting any external/load force.
    pub fn force_n(&self) -> f32 {
        self.pressure_a_pa * self.bore_area_m2 - self.pressure_b_pa * self.rod_area_m2
    }

    /// Advances chamber pressures by one step given the flow into chamber
    /// `a` and out of chamber `b` (both m^3/s, from [`orifice_flow_m3_s`])
    /// and the piston's current velocity (m/s, positive extending).
    /// Chamber volumes are treated as constant over one step, which is
    /// accurate as long as `dt_s * piston_velocity_m_s` is small relative
    /// to the chamber volumes — true at the simulator's tick rates.
    pub fn step(
        &mut self,
        flow_in_a_m3_s: f32,
        flow_out_b_m3_s: f32,
        piston_velocity_m_s: f32,
        dt_s: f32,
    ) {
        let dpa_dt = (self.bulk_modulus_pa / self.volume_a_m3)
            * (flow_in_a_m3_s - self.bore_area_m2 * piston_velocity_m_s);
        let dpb_dt = (self.bulk_modulus_pa / self.volume_b_m3)
            * (flow_out_b_m3_s + self.rod_area_m2 * piston_velocity_m_s);
        self.pressure_a_pa = (self.pressure_a_pa + dpa_dt * dt_s).max(0.0);
        self.pressure_b_pa = (self.pressure_b_pa + dpb_dt * dt_s).max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valve_reaches_63_percent_after_one_time_constant() {
        let mut valve = SpoolValve::new(0.2);
        // Step at 1kHz for one full time constant (200ms = 200 steps).
        for _ in 0..200 {
            valve.step(1.0, 0.001);
        }
        // First-order step response: 1 - e^-1 ~= 0.632.
        assert!((valve.position - 0.632).abs() < 0.01);
    }

    #[test]
    fn valve_converges_to_commanded_position() {
        let mut valve = SpoolValve::new(0.2);
        for _ in 0..5000 {
            valve.step(0.75, 0.001);
        }
        assert!((valve.position - 0.75).abs() < 1e-3);
    }

    #[test]
    fn valve_position_is_clamped() {
        let mut valve = SpoolValve::new(0.2);
        valve.step(5.0, 1.0);
        assert!(valve.position <= 1.0);
    }

    #[test]
    fn orifice_flow_is_zero_with_valve_closed() {
        assert_eq!(orifice_flow_m3_s(0.0, 0.6, 1e-4, 2e7, 850.0), 0.0);
    }

    #[test]
    fn orifice_flow_direction_matches_pressure_drop() {
        let forward = orifice_flow_m3_s(1.0, 0.6, 1e-4, 2e7, 850.0);
        let reverse = orifice_flow_m3_s(1.0, 0.6, 1e-4, -2e7, 850.0);
        assert!(forward > 0.0);
        assert!(reverse < 0.0);
        assert!((forward + reverse).abs() < 1e-9);
    }

    #[test]
    fn cylinder_pressurizes_under_inflow_with_locked_piston() {
        let mut cyl = Cylinder {
            bore_area_m2: 0.02,
            rod_area_m2: 0.01,
            pressure_a_pa: 1e5,
            pressure_b_pa: 1e5,
            volume_a_m3: 5e-3,
            volume_b_m3: 5e-3,
            bulk_modulus_pa: 1.6e9,
        };
        let initial = cyl.pressure_a_pa;
        for _ in 0..100 {
            cyl.step(2e-4, 0.0, 0.0, 0.001);
        }
        assert!(cyl.pressure_a_pa > initial);
    }

    #[test]
    fn cylinder_force_follows_pressure_differential() {
        let cyl = Cylinder {
            bore_area_m2: 0.02,
            rod_area_m2: 0.01,
            pressure_a_pa: 2e7,
            pressure_b_pa: 1e5,
            volume_a_m3: 5e-3,
            volume_b_m3: 5e-3,
            bulk_modulus_pa: 1.6e9,
        };
        assert!(cyl.force_n() > 0.0);
    }
}
