// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The central event loop: drains [`crate::bus`] events, drives the
//! lifecycle [`MachineFsm`], and enforces that the machine cannot leave
//! [`MachineState::Idle`] until its shift-start [`SelfTestSuite`] has
//! passed.

use crate::bus::BusReceiver;
use crate::messages::{FaultSeverity, MachineEvent};
use crate::self_test::{SelfTestReport, SelfTestSuite};
use crate::state::{MachineFsm, MachineState, TransitionError};

/// A [`MachineController::tick`] could not be applied.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TickError {
    /// The requested lifecycle edge is not legal from the current state.
    Transition(TransitionError),
    /// The machine tried to leave `Idle` before a passing self-test report
    /// was recorded via [`MachineController::run_self_test`].
    SelfTestRequired,
}

/// Owns the lifecycle FSM for one machine, the receive half of its event
/// bus, and its shift-start self-test suite. This is the single consumer of
/// the bus: every subsystem publishes onto `BusSender` clones, and only the
/// controller decides what those events mean for the machine's state.
///
/// [`MachineController::run_self_test`] and [`MachineController::tick`]
/// return the [`MachineEvent`]s they produce (`SelfTestCompleted`,
/// `StateChanged`) rather than re-publishing them onto the same inbound
/// bus: this is the controller's *inbox*, and echoing its own outputs back
/// into it would have the controller dequeue and reprocess its own
/// notifications as if a subsystem had sent them. Forwarding these events
/// to other subsystems (telemetry, safety, the visualizer) is the caller's
/// job, typically over a separate outbound/broadcast channel.
pub struct MachineController {
    fsm: MachineFsm,
    bus_rx: BusReceiver,
    self_test: SelfTestSuite,
    self_test_report: Option<SelfTestReport>,
}

impl MachineController {
    /// Builds a controller in `Idle` with no self-test yet run.
    pub fn new(bus_rx: BusReceiver, self_test: SelfTestSuite) -> Self {
        MachineController {
            fsm: MachineFsm::new(),
            bus_rx,
            self_test,
            self_test_report: None,
        }
    }

    pub fn state(&self) -> MachineState {
        self.fsm.state()
    }

    pub fn self_test_report(&self) -> Option<&SelfTestReport> {
        self.self_test_report.as_ref()
    }

    /// Runs the registered shift-start diagnostics, records the report, and
    /// returns the `SelfTestCompleted` event describing it. Must be called
    /// (and must pass) before the machine can move out of `Idle`.
    pub fn run_self_test(&mut self) -> MachineEvent {
        let report = self.self_test.run();
        let event = MachineEvent::SelfTestCompleted {
            passed: report.passed(),
        };
        self.self_test_report = Some(report);
        event
    }

    /// Drains and applies at most one pending bus event. Returns `Ok(None)`
    /// if the bus was empty, `Ok(Some(event))` describing what happened, or
    /// `Err` if the event could not be legally applied (the bus is drained
    /// either way: a rejected request is not retried automatically).
    pub fn tick(&mut self) -> Result<Option<MachineEvent>, TickError> {
        let Some(event) = self.bus_rx.poll() else {
            return Ok(None);
        };

        match event {
            MachineEvent::RequestTransition { next } => self.apply_transition(next).map(Some),
            MachineEvent::FaultRaised(fault) if fault.severity == FaultSeverity::EmergencyStop => {
                self.apply_transition(MachineState::Idle).map(Some)
            }
            other => Ok(Some(other)),
        }
    }

    fn apply_transition(&mut self, next: MachineState) -> Result<MachineEvent, TickError> {
        if self.fsm.state() == MachineState::Idle && next != MachineState::Idle {
            let passed = self
                .self_test_report
                .as_ref()
                .is_some_and(SelfTestReport::passed);
            if !passed {
                return Err(TickError::SelfTestRequired);
            }
        }
        let from = self.fsm.transition(next).map_err(TickError::Transition)?;
        Ok(MachineEvent::StateChanged { from, to: next })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::new_bus;
    use crate::messages::Fault;
    use crate::self_test::{SelfTestCheck, SelfTestOutcome};

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
            SelfTestOutcome::Fail { reason: "nope" }
        }
    }

    #[test]
    fn cannot_leave_idle_before_self_test() {
        let (tx, rx) = new_bus();
        let mut controller =
            MachineController::new(rx, SelfTestSuite::new().with_check(AlwaysPass));
        tx.publish(MachineEvent::RequestTransition {
            next: MachineState::Moving,
        })
        .unwrap();
        assert_eq!(controller.tick(), Err(TickError::SelfTestRequired));
        assert_eq!(controller.state(), MachineState::Idle);
    }

    #[test]
    fn failing_self_test_still_blocks_transition() {
        let (tx, rx) = new_bus();
        let mut controller =
            MachineController::new(rx, SelfTestSuite::new().with_check(AlwaysFail));
        let outcome = controller.run_self_test();
        assert_eq!(outcome, MachineEvent::SelfTestCompleted { passed: false });
        tx.publish(MachineEvent::RequestTransition {
            next: MachineState::Moving,
        })
        .unwrap();
        assert_eq!(controller.tick(), Err(TickError::SelfTestRequired));
    }

    #[test]
    fn passing_self_test_allows_the_nominal_cycle() {
        let (tx, rx) = new_bus();
        let mut controller =
            MachineController::new(rx, SelfTestSuite::new().with_check(AlwaysPass));
        assert_eq!(
            controller.run_self_test(),
            MachineEvent::SelfTestCompleted { passed: true }
        );

        let mut previous = MachineState::Idle;
        for next in [
            MachineState::Moving,
            MachineState::Working,
            MachineState::Dumping,
            MachineState::Returning,
            MachineState::Idle,
        ] {
            tx.publish(MachineEvent::RequestTransition { next })
                .unwrap();
            let applied = controller.tick().unwrap().unwrap();
            assert_eq!(
                applied,
                MachineEvent::StateChanged {
                    from: previous,
                    to: next
                }
            );
            assert_eq!(controller.state(), next);
            previous = next;
        }
    }

    #[test]
    fn emergency_fault_forces_idle_from_any_state() {
        let (tx, rx) = new_bus();
        let mut controller =
            MachineController::new(rx, SelfTestSuite::new().with_check(AlwaysPass));
        controller.run_self_test();

        for next in [MachineState::Moving, MachineState::Working] {
            tx.publish(MachineEvent::RequestTransition { next })
                .unwrap();
            controller.tick().unwrap();
        }
        assert_eq!(controller.state(), MachineState::Working);

        tx.publish(MachineEvent::FaultRaised(Fault {
            code: 1,
            severity: FaultSeverity::EmergencyStop,
            message: "hydraulic overpressure",
        }))
        .unwrap();
        let applied = controller.tick().unwrap().unwrap();
        assert_eq!(
            applied,
            MachineEvent::StateChanged {
                from: MachineState::Working,
                to: MachineState::Idle
            }
        );
        assert_eq!(controller.state(), MachineState::Idle);
    }

    #[test]
    fn empty_bus_ticks_to_none() {
        let (_tx, rx) = new_bus();
        let mut controller = MachineController::new(rx, SelfTestSuite::new());
        assert_eq!(controller.tick(), Ok(None));
    }
}
