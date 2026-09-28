// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The core machine lifecycle state machine shared by every autonomous
//! implement: `Idle -> Moving -> Working -> Dumping -> Returning -> Idle`.
//!
//! This is the *lifecycle* FSM (what phase of a work cycle a machine is in),
//! not the teleoperation handover FSM (`AUTONOMOUS <-> TELEOP_ACTIVE <->
//! EMERGENCY_STOP`) defined by `tpt-teleop-domain-bridge`, which
//! `tpt-t-construction-teleop` layers on top in Phase 10.

use std::fmt;

/// A phase in the machine's work cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MachineState {
    /// Powered on, self-tested, no active work cycle.
    Idle,
    /// Traveling to a work location (dig face, dump point, drill collar, ...).
    Moving,
    /// Actively performing its implement-specific task (digging, grading,
    /// drilling, ...).
    Working,
    /// Discharging a payload (truck bed raised, bucket dumping, ...).
    Dumping,
    /// Returning to the next assignment or the yard.
    Returning,
}

impl fmt::Display for MachineState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            MachineState::Idle => "Idle",
            MachineState::Moving => "Moving",
            MachineState::Working => "Working",
            MachineState::Dumping => "Dumping",
            MachineState::Returning => "Returning",
        };
        f.write_str(s)
    }
}

/// A transition was attempted that the lifecycle FSM does not permit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionError {
    pub from: MachineState,
    pub to: MachineState,
}

impl fmt::Display for TransitionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "illegal machine state transition: {} -> {}",
            self.from, self.to
        )
    }
}

impl std::error::Error for TransitionError {}

impl MachineState {
    /// Whether `self -> next` is a legal edge in the lifecycle graph.
    ///
    /// The nominal cycle is `Idle -> Moving -> Working -> Dumping ->
    /// Returning -> Idle`. Every state may also return directly to `Idle`
    /// (an aborted or single-phase cycle, e.g. a dozer pass that never
    /// dumps a discrete payload), and `Working` may re-enter itself to
    /// represent an implement swapping sub-tasks without leaving the work
    /// phase (e.g. an excavator finishing one swing and starting the next).
    pub fn can_transition_to(self, next: MachineState) -> bool {
        use MachineState::*;
        matches!(
            (self, next),
            (Idle, Moving)
                | (Moving, Working)
                | (Working, Working)
                | (Working, Dumping)
                | (Dumping, Returning)
                | (Returning, Idle)
                | (_, Idle)
        )
    }
}

/// A validated instance of the machine lifecycle state machine. Construct
/// with [`MachineFsm::new`] (always starts `Idle`) and drive it only through
/// [`MachineFsm::transition`], which is the sole way to change
/// [`MachineFsm::state`].
#[derive(Debug, Clone, Copy)]
pub struct MachineFsm {
    state: MachineState,
}

impl Default for MachineFsm {
    fn default() -> Self {
        Self::new()
    }
}

impl MachineFsm {
    pub fn new() -> Self {
        MachineFsm {
            state: MachineState::Idle,
        }
    }

    pub fn state(&self) -> MachineState {
        self.state
    }

    /// Attempts to move to `next`. On success, updates `self.state` and
    /// returns the previous state; on failure, `self.state` is left
    /// unchanged and the illegal edge is reported.
    pub fn transition(&mut self, next: MachineState) -> Result<MachineState, TransitionError> {
        if !self.state.can_transition_to(next) {
            return Err(TransitionError {
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
    use MachineState::*;

    #[test]
    fn nominal_cycle_is_legal() {
        let mut fsm = MachineFsm::new();
        assert_eq!(fsm.state(), Idle);
        for next in [Moving, Working, Dumping, Returning, Idle] {
            fsm.transition(next).unwrap();
            assert_eq!(fsm.state(), next);
        }
    }

    #[test]
    fn working_can_re_enter_itself() {
        let mut fsm = MachineFsm::new();
        fsm.transition(Moving).unwrap();
        fsm.transition(Working).unwrap();
        fsm.transition(Working).unwrap();
        assert_eq!(fsm.state(), Working);
    }

    #[test]
    fn any_state_can_abort_to_idle() {
        for start in [Moving, Working, Dumping, Returning] {
            let mut fsm = MachineFsm::new();
            fsm.transition(Moving).unwrap();
            if start != Moving {
                // Walk to `start` via legal edges where possible, otherwise
                // just drive straight through Working since every state
                // used here is reachable from Moving in the nominal cycle.
                if start == Working || start == Dumping || start == Returning {
                    fsm.transition(Working).unwrap();
                }
                if start == Dumping || start == Returning {
                    fsm.transition(Dumping).unwrap();
                }
                if start == Returning {
                    fsm.transition(Returning).unwrap();
                }
            }
            assert_eq!(fsm.state(), start);
            fsm.transition(Idle).unwrap();
            assert_eq!(fsm.state(), Idle);
        }
    }

    #[test]
    fn skipping_phases_is_illegal() {
        let mut fsm = MachineFsm::new();
        let err = fsm.transition(Working).unwrap_err();
        assert_eq!(
            err,
            TransitionError {
                from: Idle,
                to: Working
            }
        );
        // State is unchanged after a rejected transition.
        assert_eq!(fsm.state(), Idle);
    }

    #[test]
    fn dumping_cannot_go_back_to_moving() {
        let mut fsm = MachineFsm::new();
        fsm.transition(Moving).unwrap();
        fsm.transition(Working).unwrap();
        fsm.transition(Dumping).unwrap();
        assert!(fsm.transition(Moving).is_err());
    }
}
