// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Fault-driven handover triggers: bridge spec.txt's
//! `request_teleop_assistance()` is, in this reconciliation, not a method
//! an operator calls but a pure decision function — given a fault's
//! severity and the current handover state, what (if any) handover
//! transition does that fault demand? [`crate::adapter`] is what actually
//! applies the returned transition and, from there, would notify a
//! dispatcher/operator pool that assistance is wanted.

use tpt_t_construction_core::FaultSeverity;

use crate::safety_fsm::HandoverState;

/// Given the machine is currently in `current` and a fault of `severity`
/// was just raised, returns the handover state that fault demands, or
/// `None` if no handover-state change is required (e.g. a `Warning` while
/// already `Autonomous`, or any fault severity that doesn't add a new
/// constraint beyond the state already in force).
///
/// - `EmergencyStop` always forces [`HandoverState::EmergencyStop`],
///   regardless of `current` — this is the one handover edge legal from
///   every state (see [`HandoverState::can_transition_to`]).
/// - `Critical` while `Autonomous` asks for operator assistance by moving
///   to [`HandoverState::RequestingTeleop`]. A `Critical` fault raised
///   while already `RequestingTeleop`/`TeleopActive`/`ReturningToAutonomy`
///   doesn't demand a further transition — the machine is already headed
///   toward or under operator control.
/// - `Warning` never forces a handover transition; it's logged for
///   maintenance review only (see [`FaultSeverity::Warning`]'s docs).
pub fn handover_response(current: HandoverState, severity: FaultSeverity) -> Option<HandoverState> {
    match severity {
        FaultSeverity::EmergencyStop => {
            if current == HandoverState::EmergencyStop {
                None
            } else {
                Some(HandoverState::EmergencyStop)
            }
        }
        FaultSeverity::Critical => {
            if current == HandoverState::Autonomous {
                Some(HandoverState::RequestingTeleop)
            } else {
                None
            }
        }
        FaultSeverity::Warning => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use HandoverState::*;

    #[test]
    fn emergency_fault_forces_emergency_stop_from_any_state() {
        for start in [
            Autonomous,
            RequestingTeleop,
            TeleopActive,
            ReturningToAutonomy,
        ] {
            assert_eq!(
                handover_response(start, FaultSeverity::EmergencyStop),
                Some(EmergencyStop)
            );
        }
    }

    #[test]
    fn emergency_fault_while_already_stopped_is_a_no_op() {
        assert_eq!(
            handover_response(EmergencyStop, FaultSeverity::EmergencyStop),
            None
        );
    }

    #[test]
    fn critical_fault_requests_teleop_only_from_autonomous() {
        assert_eq!(
            handover_response(Autonomous, FaultSeverity::Critical),
            Some(RequestingTeleop)
        );
        for start in [RequestingTeleop, TeleopActive, ReturningToAutonomy] {
            assert_eq!(handover_response(start, FaultSeverity::Critical), None);
        }
    }

    #[test]
    fn warning_never_forces_a_transition() {
        for start in [
            Autonomous,
            RequestingTeleop,
            TeleopActive,
            ReturningToAutonomy,
            EmergencyStop,
        ] {
            assert_eq!(handover_response(start, FaultSeverity::Warning), None);
        }
    }
}
