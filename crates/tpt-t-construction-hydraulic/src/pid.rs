// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A zero-allocation PID controller: the core of the 1kHz proportional
//! valve control loop (spec.txt §4.1). No heap, no dynamic dispatch — just
//! `f32` state, safe to call from a `SCHED_FIFO` real-time thread.
//!
//! Derivative acts on the measurement rather than the error (standard
//! "derivative-on-measurement" form), so a step change in setpoint doesn't
//! cause a derivative-kick spike in the output. Anti-windup halts integral
//! accumulation once the output has saturated in the direction the error
//! would keep pushing it, so a long-saturated loop (e.g. commanding more
//! flow than the valve can pass) doesn't wind up an integral term that
//! then overshoots once the error reverses.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct PidController {
    gains: PidGains,
    integral: f32,
    prev_measurement: f32,
    has_prev_measurement: bool,
    output_min: f32,
    output_max: f32,
}

impl PidController {
    pub fn new(gains: PidGains, output_min: f32, output_max: f32) -> Self {
        assert!(
            output_min < output_max,
            "output_min must be less than output_max"
        );
        PidController {
            gains,
            integral: 0.0,
            prev_measurement: 0.0,
            has_prev_measurement: false,
            output_min,
            output_max,
        }
    }

    /// Clears accumulated integral and derivative history. Call when
    /// re-engaging the loop after a mode change (e.g. returning from
    /// teleop to autonomy) to avoid a stale integral term producing a
    /// discontinuous first output.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.has_prev_measurement = false;
    }

    /// Advances the controller by one sample of period `dt_s`, returning
    /// the control output clamped to `[output_min, output_max]`.
    pub fn step(&mut self, setpoint: f32, measurement: f32, dt_s: f32) -> f32 {
        let error = setpoint - measurement;

        let derivative = if self.has_prev_measurement {
            -(measurement - self.prev_measurement) / dt_s
        } else {
            0.0
        };
        self.prev_measurement = measurement;
        self.has_prev_measurement = true;

        let p_term = self.gains.kp * error;
        let d_term = self.gains.kd * derivative;

        // Anti-windup: only accumulate the integral if doing so wouldn't
        // push an already-saturated output further past its limit.
        let candidate_integral = self.integral + error * dt_s;
        let candidate_output = p_term + self.gains.ki * candidate_integral + d_term;
        let would_worsen_high_saturation = candidate_output > self.output_max && error > 0.0;
        let would_worsen_low_saturation = candidate_output < self.output_min && error < 0.0;
        if !would_worsen_high_saturation && !would_worsen_low_saturation {
            self.integral = candidate_integral;
        }

        let i_term = self.gains.ki * self.integral;
        (p_term + i_term + d_term).clamp(self.output_min, self.output_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proportional_only_output_scales_with_error() {
        let gains = PidGains {
            kp: 2.0,
            ki: 0.0,
            kd: 0.0,
        };
        let mut pid = PidController::new(gains, -100.0, 100.0);
        let output = pid.step(10.0, 4.0, 0.01);
        assert!((output - 12.0).abs() < 1e-5);
    }

    #[test]
    fn output_is_clamped_to_bounds() {
        let gains = PidGains {
            kp: 100.0,
            ki: 0.0,
            kd: 0.0,
        };
        let mut pid = PidController::new(gains, -1.0, 1.0);
        assert_eq!(pid.step(100.0, 0.0, 0.01), 1.0);
        assert_eq!(pid.step(-100.0, 0.0, 0.01), -1.0);
    }

    #[test]
    fn integral_accumulates_persistent_error() {
        let gains = PidGains {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
        };
        let mut pid = PidController::new(gains, -100.0, 100.0);
        let first = pid.step(1.0, 0.0, 1.0);
        let second = pid.step(1.0, 0.0, 1.0);
        assert!(
            second > first,
            "integral term should keep growing under sustained error"
        );
    }

    #[test]
    fn closed_loop_converges_on_a_first_order_plant() {
        // A simple first-order plant: measurement chases the commanded
        // output with a bit of lag, standing in for a valve's response.
        // kd is deliberately small: derivative-on-measurement still reacts
        // to the plant's fast first-order response at this dt, and a
        // larger kd here (e.g. 0.1) drives the output into a sustained
        // full-scale limit cycle once it saturates near the setpoint —
        // real deployments filter or slow down the derivative term for
        // exactly this reason. That tuning failure mode is a separate
        // concern from what this test checks (basic P+I+D convergence).
        let gains = PidGains {
            kp: 2.0,
            ki: 4.0,
            kd: 0.01,
        };
        let mut pid = PidController::new(gains, -1.0, 1.0);
        let mut measurement = 0.0_f32;
        let setpoint = 0.5;
        let dt = 0.001;
        for _ in 0..2000 {
            let output = pid.step(setpoint, measurement, dt);
            measurement += (output - measurement) * (dt / 0.05);
        }
        assert!(
            (measurement - setpoint).abs() < 0.02,
            "measurement {measurement} did not converge to setpoint {setpoint}"
        );
    }

    #[test]
    fn anti_windup_prevents_integral_runaway_during_saturation() {
        let gains = PidGains {
            kp: 0.0,
            ki: 10.0,
            kd: 0.0,
        };
        let mut pid = PidController::new(gains, -1.0, 1.0);
        // Drive hard in saturation for a long time.
        for _ in 0..1000 {
            pid.step(1000.0, 0.0, 0.01);
        }
        // Error reverses sign: without anti-windup the wound-up integral
        // would keep the output pinned at max for a long time afterward.
        let output_after_reversal = pid.step(-1000.0, 0.0, 0.01);
        assert!(
            output_after_reversal < 1.0,
            "output should respond promptly once error reverses, not stay pinned at max"
        );
    }

    #[test]
    fn reset_clears_integral_and_derivative_history() {
        let gains = PidGains {
            kp: 0.0,
            ki: 1.0,
            kd: 0.0,
        };
        let mut pid = PidController::new(gains, -100.0, 100.0);
        pid.step(1.0, 0.0, 1.0);
        pid.step(1.0, 0.0, 1.0);
        pid.reset();
        let output = pid.step(1.0, 0.0, 1.0);
        // First step after reset behaves like a fresh controller: only one
        // sample's worth of integral, and no derivative-on-measurement kick.
        assert!((output - 1.0).abs() < 1e-5);
    }
}
