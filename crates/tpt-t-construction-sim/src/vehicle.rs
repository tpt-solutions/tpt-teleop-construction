// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A minimal longitudinal vehicle model that composes every other physics
//! module in this crate — [`crate::rigid_body`], [`crate::soil`], and
//! [`crate::powertrain`] — into one steppable machine, demonstrating (and
//! integration-testing) how they fit together. It deliberately only
//! models straight-line motion (no steering): a full 6DOF chassis with
//! implement-specific steering/articulation belongs to the individual
//! `tpt-t-construction-{excavator,haul,dozer,...}` crates in later
//! phases, which will drive [`crate::rigid_body::RigidBody6Dof`] directly.

use crate::math::Vec3;
use crate::powertrain::{EngineTorqueCurve, Transmission};
use crate::rigid_body::RigidBody6Dof;
use crate::soil::{slip_ratio, traction_force_n, SoilType};

const GRAVITY_M_S2: f32 = 9.81;
const RAD_PER_S_TO_RPM: f32 = 60.0 / (2.0 * std::f32::consts::PI);

/// A single drive axle/track coupled to an engine and transmission through
/// a rigid driveline (no torque converter/clutch slip modeled — only
/// tire/track-to-ground slip, via [`crate::soil`]).
pub struct SimVehicle {
    pub body: RigidBody6Dof,
    pub engine: EngineTorqueCurve,
    pub transmission: Transmission,
    pub engine_rpm: f32,
    pub engine_idle_rpm: f32,
    pub engine_redline_rpm: f32,
    /// Engine + driveline rotational inertia reflected to the engine
    /// shaft, kg*m^2. Larger values make the engine slower to change
    /// speed under a torque imbalance (i.e. slower to spin up wheel slip
    /// or to lug down under load).
    pub engine_inertia_kg_m2: f32,
    pub wheel_radius_m: f32,
    pub soil: SoilType,
    pub rolling_resistance_n: f32,
}

impl SimVehicle {
    /// Advances the vehicle one step under a throttle command in `[0, 1]`.
    pub fn step(&mut self, throttle: f32, dt_s: f32) {
        self.transmission.update_shift(self.engine_rpm);
        let engine_torque_nm = self
            .engine
            .torque_at_nm_throttled(self.engine_rpm, throttle);

        let forward = self.body.orientation.rotate(Vec3::new(1.0, 0.0, 0.0));
        let ground_speed_m_s = self.body.linear_velocity.dot(forward);

        let overall_ratio = self.transmission.overall_ratio();
        let wheel_angular_speed_rad_s = (self.engine_rpm / RAD_PER_S_TO_RPM) / overall_ratio;
        let wheel_surface_speed_m_s = wheel_angular_speed_rad_s * self.wheel_radius_m;

        let slip = slip_ratio(wheel_surface_speed_m_s, ground_speed_m_s);
        let normal_load_n = self.body.mass_kg * GRAVITY_M_S2;
        let traction_force_n = traction_force_n(normal_load_n, slip, self.soil);

        let rolling_resistance_n = if ground_speed_m_s.abs() > 1e-3 {
            -self.rolling_resistance_n * ground_speed_m_s.signum()
        } else {
            0.0
        };
        let net_force_n = traction_force_n + rolling_resistance_n;
        self.body.step(forward * net_force_n, Vec3::ZERO, dt_s);

        // The traction force actually delivered reflects a load torque
        // back through the driveline to the engine; the difference
        // between what the engine produces and what the wheels demand is
        // what accelerates (or lugs down) the engine itself.
        let wheel_torque_delivered_nm = traction_force_n * self.wheel_radius_m;
        let engine_load_torque_nm = wheel_torque_delivered_nm / overall_ratio;
        let net_engine_torque_nm = engine_torque_nm - engine_load_torque_nm;
        let engine_angular_accel_rad_s2 = net_engine_torque_nm / self.engine_inertia_kg_m2;
        self.engine_rpm = (self.engine_rpm + engine_angular_accel_rad_s2 * RAD_PER_S_TO_RPM * dt_s)
            .clamp(self.engine_idle_rpm, self.engine_redline_rpm);
    }

    pub fn ground_speed_m_s(&self) -> f32 {
        let forward = self.body.orientation.rotate(Vec3::new(1.0, 0.0, 0.0));
        self.body.linear_velocity.dot(forward)
    }

    /// A representative fully-loaded haul truck, parameterized with
    /// plausible (if illustrative) heavy-equipment values. Used by this
    /// module's tests and by [`crate::perf`]'s real-time performance
    /// validation as a stand-in for a real machine model.
    pub fn reference_haul_truck() -> SimVehicle {
        SimVehicle {
            body: RigidBody6Dof::stationary(30_000.0, Vec3::new(20_000.0, 40_000.0, 40_000.0)),
            engine: EngineTorqueCurve::diesel_default(),
            transmission: Transmission::new(vec![6.0, 3.0, 1.5, 1.0], 4.0, 2000.0, 1000.0),
            engine_rpm: 600.0,
            engine_idle_rpm: 600.0,
            engine_redline_rpm: 2400.0,
            engine_inertia_kg_m2: 5.0,
            wheel_radius_m: 0.9,
            soil: SoilType::Dirt,
            rolling_resistance_n: 2000.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_vehicle() -> SimVehicle {
        SimVehicle::reference_haul_truck()
    }

    #[test]
    fn accelerates_forward_from_rest_under_throttle() {
        let mut vehicle = default_vehicle();
        for _ in 0..2000 {
            vehicle.step(1.0, 0.01);
        }
        assert!(vehicle.ground_speed_m_s() > 0.0);
        assert!(vehicle.body.position.x > 0.0);
    }

    #[test]
    fn full_throttle_revs_the_engine_higher_than_idle() {
        // A diesel still delivers idle torque at throttle = 0 (real
        // engines do too, hence "idle creep" in an automatic
        // transmission), and while the wheels are spinning hard enough to
        // saturate the soil's traction limit (see `soil::SLIP_SATURATION`),
        // *ground speed* is soil-limited and barely throttle-sensitive —
        // flooring it doesn't get you moving faster once you're already
        // spinning your wheels, same as in reality. Engine RPM is the
        // throttle-sensitive quantity in that regime: full throttle keeps
        // more net torque available to accelerate the engine itself.
        let mut idling = default_vehicle();
        let mut floored = default_vehicle();
        for _ in 0..300 {
            idling.step(0.0, 0.01);
            floored.step(1.0, 0.01);
        }
        assert_eq!(idling.transmission.current_gear, 0);
        assert_eq!(floored.transmission.current_gear, 0);
        assert!(floored.engine_rpm > idling.engine_rpm);
    }

    #[test]
    fn engine_rpm_stays_within_idle_and_redline() {
        let mut vehicle = default_vehicle();
        for _ in 0..5000 {
            vehicle.step(1.0, 0.01);
            assert!(vehicle.engine_rpm >= vehicle.engine_idle_rpm);
            assert!(vehicle.engine_rpm <= vehicle.engine_redline_rpm);
        }
    }

    #[test]
    fn rock_accelerates_faster_than_mud_from_rest() {
        let mut on_rock = default_vehicle();
        on_rock.soil = SoilType::Rock;
        let mut on_mud = default_vehicle();
        on_mud.soil = SoilType::Mud;

        // Compare only the early transient, both still in first gear: once
        // either vehicle upshifts, the discrete gear-change threshold
        // timing (a bang-bang decision that a slightly different engine
        // load from the soil model can nudge a tick earlier or later)
        // dominates the outcome far more than the underlying traction
        // physics, making a long-run comparison chaotic rather than
        // meaningful.
        for _ in 0..300 {
            on_rock.step(1.0, 0.01);
            on_mud.step(1.0, 0.01);
        }
        assert_eq!(on_rock.transmission.current_gear, 0);
        assert_eq!(on_mud.transmission.current_gear, 0);
        assert!(on_rock.ground_speed_m_s() > on_mud.ground_speed_m_s());
    }
}
