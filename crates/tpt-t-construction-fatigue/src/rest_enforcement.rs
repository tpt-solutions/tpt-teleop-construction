// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Mandatory rest enforcement (spec.txt §7): once an operator has been
//! continuously active for too long, operation locks out until a full
//! rest period has been taken — and a drowsiness signal from
//! [`crate::eye_tracking`] or [`crate::steering_pattern`] can force that
//! rest period to begin early, rather than waiting for the continuous-
//! operating timer to run out on its own.

/// Enforces a maximum continuous operating duration and a minimum rest
/// duration before operation may resume.
#[derive(Debug, Clone, Copy)]
pub struct RestEnforcer {
    continuous_operating_s: f32,
    current_rest_s: f32,
    is_resting: bool,
    max_continuous_s: f32,
    required_rest_s: f32,
}

impl RestEnforcer {
    pub fn new(max_continuous_s: f32, required_rest_s: f32) -> Self {
        RestEnforcer {
            continuous_operating_s: 0.0,
            current_rest_s: 0.0,
            is_resting: false,
            max_continuous_s,
            required_rest_s,
        }
    }

    pub fn is_operation_locked_out(&self) -> bool {
        self.is_resting
    }

    pub fn continuous_operating_s(&self) -> f32 {
        self.continuous_operating_s
    }

    pub fn current_rest_s(&self) -> f32 {
        self.current_rest_s
    }

    /// Advances time while the operator is actively working. A no-op
    /// while a mandatory rest is already in effect — operating time
    /// doesn't quietly accumulate during a lockout.
    pub fn update_operating(&mut self, dt_s: f32) {
        if self.is_resting {
            return;
        }
        self.continuous_operating_s += dt_s;
        if self.continuous_operating_s >= self.max_continuous_s {
            self.is_resting = true;
            self.current_rest_s = 0.0;
        }
    }

    /// Advances time while the operator is resting, clearing the lockout
    /// once `required_rest_s` has accumulated. A no-op if not currently
    /// in a mandatory rest.
    pub fn update_resting(&mut self, dt_s: f32) {
        if !self.is_resting {
            return;
        }
        self.current_rest_s += dt_s;
        if self.current_rest_s >= self.required_rest_s {
            self.is_resting = false;
            self.continuous_operating_s = 0.0;
        }
    }

    /// Immediately begins a mandatory rest period regardless of how much
    /// continuous operating time has accumulated — the hook a drowsiness
    /// or low-steering-activity signal uses to force a break early.
    pub fn force_rest(&mut self) {
        self.is_resting = true;
        self.current_rest_s = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_unlocked() {
        let enforcer = RestEnforcer::new(4.0 * 3600.0, 30.0 * 60.0);
        assert!(!enforcer.is_operation_locked_out());
    }

    #[test]
    fn reaching_max_continuous_operating_time_locks_out() {
        let mut enforcer = RestEnforcer::new(100.0, 50.0);
        enforcer.update_operating(99.0);
        assert!(!enforcer.is_operation_locked_out());
        enforcer.update_operating(1.0);
        assert!(enforcer.is_operation_locked_out());
    }

    #[test]
    fn operating_time_does_not_accumulate_while_locked_out() {
        let mut enforcer = RestEnforcer::new(100.0, 50.0);
        enforcer.update_operating(100.0);
        assert!(enforcer.is_operation_locked_out());
        enforcer.update_operating(10.0);
        assert!((enforcer.continuous_operating_s() - 100.0).abs() < 1e-4);
    }

    #[test]
    fn sufficient_rest_clears_the_lockout_and_resets_operating_time() {
        let mut enforcer = RestEnforcer::new(100.0, 50.0);
        enforcer.update_operating(100.0);
        assert!(enforcer.is_operation_locked_out());
        enforcer.update_resting(49.0);
        assert!(enforcer.is_operation_locked_out());
        enforcer.update_resting(1.0);
        assert!(!enforcer.is_operation_locked_out());
        assert_eq!(enforcer.continuous_operating_s(), 0.0);
    }

    #[test]
    fn force_rest_locks_out_immediately_regardless_of_operating_time() {
        let mut enforcer = RestEnforcer::new(4.0 * 3600.0, 30.0 * 60.0);
        enforcer.update_operating(60.0); // barely any time accumulated
        enforcer.force_rest();
        assert!(enforcer.is_operation_locked_out());
    }

    #[test]
    fn resting_updates_are_ignored_when_not_locked_out() {
        let mut enforcer = RestEnforcer::new(100.0, 50.0);
        enforcer.update_resting(1000.0);
        assert!(!enforcer.is_operation_locked_out());
        assert_eq!(enforcer.current_rest_s(), 0.0);
    }
}
