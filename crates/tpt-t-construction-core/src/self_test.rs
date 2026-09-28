// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Shift-start self-test framework. Every subsystem crate that needs a
//! pre-work diagnostic (hydraulic pressure hold, safety sensor liveness,
//! rollover IMU calibration, ...) implements [`SelfTestCheck`] and registers
//! it with the machine's [`SelfTestSuite`]; [`MachineController`] runs the
//! full suite before allowing the lifecycle FSM to leave [`Idle`].
//!
//! Unlike the real-time hot paths elsewhere in this workspace, this runs
//! once per shift and is not latency- or allocation-sensitive, so it's built
//! on ordinary `Vec`/`Box<dyn Trait>` rather than static arrays.
//!
//! [`Idle`]: crate::state::MachineState::Idle
//! [`MachineController`]: crate::event_loop::MachineController

/// The result of a single diagnostic check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelfTestOutcome {
    Pass,
    /// `reason` should be a short, operator-facing explanation (e.g. "boom
    /// cylinder pressure did not reach hold within 2s").
    Fail {
        reason: &'static str,
    },
}

/// A single, named shift-start diagnostic.
pub trait SelfTestCheck: Send + Sync {
    /// A short, stable identifier shown in the self-test report (e.g.
    /// "hydraulic.pressure_hold").
    fn name(&self) -> &'static str;

    /// Runs the check. Must be safe to call with the machine stationary and
    /// powered but not yet in any work state.
    fn run(&self) -> SelfTestOutcome;
}

/// The aggregate result of running a [`SelfTestSuite`].
#[derive(Debug, Clone)]
pub struct SelfTestReport {
    results: Vec<(&'static str, SelfTestOutcome)>,
}

impl SelfTestReport {
    /// Whether every registered check passed. The lifecycle FSM must not
    /// leave `Idle` unless this is `true`.
    pub fn passed(&self) -> bool {
        self.results
            .iter()
            .all(|(_, outcome)| matches!(outcome, SelfTestOutcome::Pass))
    }

    pub fn results(&self) -> &[(&'static str, SelfTestOutcome)] {
        &self.results
    }

    /// Iterates the checks that failed, for operator-facing diagnostics.
    pub fn failures(&self) -> impl Iterator<Item = (&'static str, &'static str)> + '_ {
        self.results
            .iter()
            .filter_map(|(name, outcome)| match outcome {
                SelfTestOutcome::Fail { reason } => Some((*name, *reason)),
                SelfTestOutcome::Pass => None,
            })
    }
}

/// An ordered collection of shift-start diagnostics.
#[derive(Default)]
pub struct SelfTestSuite {
    checks: Vec<Box<dyn SelfTestCheck>>,
}

impl SelfTestSuite {
    pub fn new() -> Self {
        SelfTestSuite::default()
    }

    /// Registers a check, builder-style.
    pub fn with_check(mut self, check: impl SelfTestCheck + 'static) -> Self {
        self.checks.push(Box::new(check));
        self
    }

    /// Registers a check in place.
    pub fn add_check(&mut self, check: impl SelfTestCheck + 'static) {
        self.checks.push(Box::new(check));
    }

    /// Runs every registered check, in registration order, and returns the
    /// aggregate report. A failing check does not stop later checks from
    /// running: the operator should see every failure at once, not one at a
    /// time across repeated shift-start attempts.
    pub fn run(&self) -> SelfTestReport {
        let results = self
            .checks
            .iter()
            .map(|check| (check.name(), check.run()))
            .collect();
        SelfTestReport { results }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct AlwaysPass;
    impl SelfTestCheck for AlwaysPass {
        fn name(&self) -> &'static str {
            "always.pass"
        }
        fn run(&self) -> SelfTestOutcome {
            SelfTestOutcome::Pass
        }
    }

    struct AlwaysFail;
    impl SelfTestCheck for AlwaysFail {
        fn name(&self) -> &'static str {
            "always.fail"
        }
        fn run(&self) -> SelfTestOutcome {
            SelfTestOutcome::Fail {
                reason: "simulated failure",
            }
        }
    }

    #[test]
    fn empty_suite_passes() {
        let suite = SelfTestSuite::new();
        assert!(suite.run().passed());
    }

    #[test]
    fn all_passing_checks_pass() {
        let suite = SelfTestSuite::new()
            .with_check(AlwaysPass)
            .with_check(AlwaysPass);
        let report = suite.run();
        assert!(report.passed());
        assert_eq!(report.results().len(), 2);
    }

    #[test]
    fn one_failure_fails_the_suite_but_runs_every_check() {
        let suite = SelfTestSuite::new()
            .with_check(AlwaysPass)
            .with_check(AlwaysFail)
            .with_check(AlwaysPass);
        let report = suite.run();
        assert!(!report.passed());
        assert_eq!(report.results().len(), 3);
        let failures: Vec<_> = report.failures().collect();
        assert_eq!(failures, vec![("always.fail", "simulated failure")]);
    }
}
