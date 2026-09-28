// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The universal teleoperation handover state machine (bridge spec.txt's
//! `AUTONOMOUS <-> REQUESTING_TELEOP <-> TELEOP_ACTIVE <->
//! RETURNING_TO_AUTONOMY <-> EMERGENCY_STOP`), layered on top of — not
//! replacing — [`tpt_t_construction_core::MachineFsm`]'s work-cycle
//! lifecycle. A machine can be `Working` in the core FSM while
//! `TeleopActive` here: the two state machines track orthogonal concerns
//! (what task phase the machine is in vs. who is driving it).

use std::fmt;

/// A state in the teleoperation handover state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HandoverState {
    /// The machine is under autonomous control; no operator is engaged.
    Autonomous,
    /// Autonomy (or a fault) has asked for an operator; no operator has
    /// taken control yet.
    RequestingTeleop,
    /// An operator holds control via [`crate::control_command`].
    TeleopActive,
    /// The operator has released control; autonomy is resuming (e.g.
    /// re-localizing, re-planning) before it will accept new autonomous
    /// commands.
    ReturningToAutonomy,
    /// Immediate stop, no handover negotiation. Reachable from any state,
    /// and only recoverable back to `Autonomous` via an explicit operator
    /// or maintenance acknowledgement.
    EmergencyStop,
}

impl fmt::Display for HandoverState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            HandoverState::Autonomous => "Autonomous",
            HandoverState::RequestingTeleop => "RequestingTeleop",
            HandoverState::TeleopActive => "TeleopActive",
            HandoverState::ReturningToAutonomy => "ReturningToAutonomy",
            HandoverState::EmergencyStop => "EmergencyStop",
        };
        f.write_str(s)
    }
}

/// A transition was attempted that the handover FSM does not permit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HandoverTransitionError {
    pub from: HandoverState,
    pub to: HandoverState,
}

impl fmt::Display for HandoverTransitionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "illegal handover state transition: {} -> {}",
            self.from, self.to
        )
    }
}

impl std::error::Error for HandoverTransitionError {}

impl HandoverState {
    /// Whether `self -> next` is a legal edge in the handover graph.
    pub fn can_transition_to(self, next: HandoverState) -> bool {
        use HandoverState::*;
        matches!(
            (self, next),
            (Autonomous, RequestingTeleop)
                // Operator proactively engages without a prior request
                // (e.g. a scheduled maintenance drive).
                | (Autonomous, TeleopActive)
                | (RequestingTeleop, TeleopActive)
                // The request is cancelled (fault cleared, operator unavailable).
                | (RequestingTeleop, Autonomous)
                | (TeleopActive, ReturningToAutonomy)
                | (ReturningToAutonomy, Autonomous)
                // Emergency stop preempts everything, from every state.
                | (_, EmergencyStop)
                // Recovery is only ever back to Autonomous, and only from
                // EmergencyStop (never implicit — see `handover_response`).
                | (EmergencyStop, Autonomous)
        )
    }
}

/// A validated instance of the handover state machine. Construct with
/// [`HandoverFsm::new`] (always starts `Autonomous`) and drive it only
/// through [`HandoverFsm::transition`].
#[derive(Debug, Clone, Copy)]
pub struct HandoverFsm {
    state: HandoverState,
}

impl Default for HandoverFsm {
    fn default() -> Self {
        Self::new()
    }
}

impl HandoverFsm {
    pub fn new() -> Self {
        HandoverFsm {
            state: HandoverState::Autonomous,
        }
    }

    pub fn state(&self) -> HandoverState {
        self.state
    }

    /// Attempts to move to `next`. On success, updates `self.state` and
    /// returns the previous state; on failure, `self.state` is left
    /// unchanged and the illegal edge is reported.
    pub fn transition(
        &mut self,
        next: HandoverState,
    ) -> Result<HandoverState, HandoverTransitionError> {
        if !self.state.can_transition_to(next) {
            return Err(HandoverTransitionError {
                from: self.state,
                to: next,
            });
        }
        let previous = self.state;
        self.state = next;
        Ok(previous)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use HandoverState::*;

    #[test]
    fn starts_autonomous() {
        assert_eq!(HandoverFsm::new().state(), Autonomous);
    }

    #[test]
    fn nominal_handover_and_return_cycle() {
        let mut fsm = HandoverFsm::new();
        for next in [
            RequestingTeleop,
            TeleopActive,
            ReturningToAutonomy,
            Autonomous,
        ] {
            fsm.transition(next).unwrap();
            assert_eq!(fsm.state(), next);
        }
    }

    #[test]
    fn operator_can_engage_directly_without_a_request() {
        let mut fsm = HandoverFsm::new();
        fsm.transition(TeleopActive).unwrap();
        assert_eq!(fsm.state(), TeleopActive);
    }

    #[test]
    fn a_pending_request_can_be_cancelled() {
        let mut fsm = HandoverFsm::new();
        fsm.transition(RequestingTeleop).unwrap();
        fsm.transition(Autonomous).unwrap();
        assert_eq!(fsm.state(), Autonomous);
    }

    #[test]
    fn emergency_stop_preempts_every_state() {
        for start in [
            Autonomous,
            RequestingTeleop,
            TeleopActive,
            ReturningToAutonomy,
        ] {
            let mut fsm = HandoverFsm::new();
            match start {
                RequestingTeleop => {
                    fsm.transition(RequestingTeleop).unwrap();
                }
                TeleopActive => {
                    fsm.transition(TeleopActive).unwrap();
                }
                ReturningToAutonomy => {
                    fsm.transition(TeleopActive).unwrap();
                    fsm.transition(ReturningToAutonomy).unwrap();
                }
                Autonomous => {}
                EmergencyStop => unreachable!(),
            }
            assert_eq!(fsm.state(), start);
            fsm.transition(EmergencyStop).unwrap();
            assert_eq!(fsm.state(), EmergencyStop);
        }
    }

    #[test]
    fn emergency_stop_only_recovers_to_autonomous() {
        let mut fsm = HandoverFsm::new();
        fsm.transition(EmergencyStop).unwrap();
        assert!(fsm.transition(TeleopActive).is_err());
        assert!(fsm.transition(RequestingTeleop).is_err());
        fsm.transition(Autonomous).unwrap();
        assert_eq!(fsm.state(), Autonomous);
    }

    #[test]
    fn skipping_from_requesting_to_returning_is_illegal() {
        let mut fsm = HandoverFsm::new();
        fsm.transition(RequestingTeleop).unwrap();
        let err = fsm.transition(ReturningToAutonomy).unwrap_err();
        assert_eq!(
            err,
            HandoverTransitionError {
                from: RequestingTeleop,
                to: ReturningToAutonomy,
            }
        );
        assert_eq!(fsm.state(), RequestingTeleop);
    }
}
