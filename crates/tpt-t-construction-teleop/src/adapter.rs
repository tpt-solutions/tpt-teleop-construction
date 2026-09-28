// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A concrete [`DomainTeleopInterface`] implementation for an excavator,
//! wiring together every other module in this crate: the handover FSM
//! ([`crate::safety_fsm`]), fault-driven handover triggers
//! ([`crate::fault_handover`]), vibration-filtered joystick input
//! ([`crate::vibration_filter`]), implement switching
//! ([`crate::implement_switch`]), and valve-duty translation
//! ([`crate::valve_mapping`]).

use tpt_t_construction_core::Fault;

use crate::control_command::DecodedControlCommand;
use crate::dti::{DomainState, DomainTelemetry, DomainTeleopInterface, SensorFeed, TeleopError};
use crate::fault_handover::handover_response;
use crate::implement_switch::ImplementSwitch;
use crate::safety_fsm::{HandoverFsm, HandoverState};
use crate::valve_mapping::ValveMap;
use crate::vibration_filter::JoystickFilterBank;

/// Diesel/pump vibration band this crate's joystick filtering rejects,
/// matching `tpt-t-construction-hydraulic`'s own sensor-side notch (see
/// `todo.md` Phase 3).
const VIBRATION_REJECT_LOW_HZ: f32 = 50.0;
const VIBRATION_REJECT_HIGH_HZ: f32 = 200.0;

pub struct ExcavatorTeleopAdapter {
    handover: HandoverFsm,
    filter_bank: JoystickFilterBank<8>,
    implement_switch: ImplementSwitch,
    operator_session_id: Option<u32>,
    last_sequence: u32,
    last_valve_duties: ValveMap,
    active_fault: Option<Fault>,
    last_telemetry: DomainTelemetry,
    last_timestamp_us: u64,
}

impl ExcavatorTeleopAdapter {
    /// `control_sample_rate_hz` is the rate [`Self::on_control_command`]
    /// is expected to be called at, used to design the joystick vibration
    /// filter bank (see [`JoystickFilterBank::new`]).
    pub fn new(control_sample_rate_hz: f32) -> Self {
        ExcavatorTeleopAdapter {
            handover: HandoverFsm::new(),
            filter_bank: JoystickFilterBank::new(
                VIBRATION_REJECT_LOW_HZ,
                VIBRATION_REJECT_HIGH_HZ,
                control_sample_rate_hz,
            ),
            implement_switch: ImplementSwitch::new(),
            operator_session_id: None,
            last_sequence: 0,
            last_valve_duties: ValveMap::default(),
            active_fault: None,
            last_telemetry: DomainTelemetry {
                ground_speed_m_s: 0.0,
                payload_kg: 0.0,
                engine_rpm: 0.0,
            },
            last_timestamp_us: 0,
        }
    }

    /// The most recent valve duties computed from an accepted control
    /// command — what a hardware/sim valve driver should actually apply.
    pub fn last_valve_duties(&self) -> ValveMap {
        self.last_valve_duties
    }

    /// Feeds this tick's machine telemetry, for [`Self::get_sensor_feed`]
    /// to report on the downlink. Independent of control-command handling
    /// since telemetry flows regardless of whether an operator is
    /// currently engaged.
    pub fn update_telemetry(&mut self, timestamp_us: u64, telemetry: DomainTelemetry) {
        self.last_timestamp_us = timestamp_us;
        self.last_telemetry = telemetry;
    }

    /// A subsystem raised `fault`. Records it as the active fault and
    /// applies whatever handover transition it demands (see
    /// [`handover_response`]), if any. Returns the new handover state if
    /// a transition was applied.
    pub fn report_fault(&mut self, fault: Fault) -> Option<HandoverState> {
        self.active_fault = Some(fault);
        let next = handover_response(self.handover.state(), fault.severity)?;
        self.handover.transition(next).ok();
        Some(next)
    }

    /// Clears the active fault record (e.g. once maintenance/the operator
    /// acknowledges it). Does not by itself change the handover state.
    pub fn clear_fault(&mut self) {
        self.active_fault = None;
    }
}

impl DomainTeleopInterface for ExcavatorTeleopAdapter {
    fn on_teleop_engage(&mut self, operator_id: u32) -> Result<(), TeleopError> {
        let from = self.handover.state();
        if from != HandoverState::Autonomous && from != HandoverState::RequestingTeleop {
            return Err(TeleopError::IllegalHandoverTransition {
                from,
                to: HandoverState::TeleopActive,
            });
        }
        self.handover
            .transition(HandoverState::TeleopActive)
            .map_err(|e| TeleopError::IllegalHandoverTransition {
                from: e.from,
                to: e.to,
            })?;
        self.operator_session_id = Some(operator_id);
        self.last_sequence = 0;
        Ok(())
    }

    fn on_teleop_disengage(&mut self) -> Result<(), TeleopError> {
        self.handover
            .transition(HandoverState::ReturningToAutonomy)
            .map_err(|e| TeleopError::IllegalHandoverTransition {
                from: e.from,
                to: e.to,
            })?;
        self.operator_session_id = None;
        Ok(())
    }

    fn on_control_command(&mut self, command: DecodedControlCommand) -> Result<(), TeleopError> {
        if self.handover.state() != HandoverState::TeleopActive
            || self.operator_session_id.is_none()
        {
            return Err(TeleopError::NoActiveSession);
        }
        if command.sequence <= self.last_sequence && self.last_sequence != 0 {
            return Err(TeleopError::StaleSequence {
                last: self.last_sequence,
                received: command.sequence,
            });
        }

        let mut combined_axes = [0.0f32; 8];
        combined_axes[..4].copy_from_slice(&command.primary_axes);
        combined_axes[4..].copy_from_slice(&command.secondary_axes);
        let filtered = self.filter_bank.filter(&combined_axes);
        let filtered_primary: [f32; 4] = filtered[..4].try_into().unwrap();

        self.implement_switch
            .update(command.buttons.implement_select);
        self.last_valve_duties = ValveMap::from_primary_axes(&filtered_primary);
        self.last_sequence = command.sequence;
        Ok(())
    }

    fn get_domain_state(&self) -> DomainState {
        DomainState {
            handover_state: self.handover.state(),
            active_fault: self.active_fault,
            implement_mode: self.implement_switch.mode(),
        }
    }

    fn get_sensor_feed(&self) -> SensorFeed {
        SensorFeed {
            timestamp_us: self.last_timestamp_us,
            telemetry: self.last_telemetry,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::implement_switch::ImplementMode;
    use tpt_t_construction_core::FaultSeverity;

    fn command(sequence: u32) -> DecodedControlCommand {
        DecodedControlCommand {
            timestamp_us: 0,
            primary_axes: [0.1, 0.2, 0.3, 0.4],
            secondary_axes: [0.0; 4],
            buttons: crate::control_command::ButtonState::default(),
            operator_session_id: 7,
            sequence,
            mode_bits: 0,
        }
    }

    #[test]
    fn starts_autonomous_with_no_active_fault() {
        let adapter = ExcavatorTeleopAdapter::new(1000.0);
        let state = adapter.get_domain_state();
        assert_eq!(state.handover_state, HandoverState::Autonomous);
        assert_eq!(state.active_fault, None);
        assert_eq!(state.implement_mode, ImplementMode::Bucket);
    }

    #[test]
    fn control_command_rejected_before_engage() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        let err = adapter.on_control_command(command(1)).unwrap_err();
        assert_eq!(err, TeleopError::NoActiveSession);
    }

    #[test]
    fn engage_then_command_then_disengage_cycle() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        adapter.on_teleop_engage(42).unwrap();
        assert_eq!(
            adapter.get_domain_state().handover_state,
            HandoverState::TeleopActive
        );

        adapter.on_control_command(command(1)).unwrap();
        adapter.on_teleop_disengage().unwrap();
        assert_eq!(
            adapter.get_domain_state().handover_state,
            HandoverState::ReturningToAutonomy
        );
    }

    #[test]
    fn stale_sequence_is_rejected() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        adapter.on_teleop_engage(1).unwrap();
        adapter.on_control_command(command(5)).unwrap();
        let err = adapter.on_control_command(command(5)).unwrap_err();
        assert_eq!(
            err,
            TeleopError::StaleSequence {
                last: 5,
                received: 5
            }
        );
        let err = adapter.on_control_command(command(3)).unwrap_err();
        assert_eq!(
            err,
            TeleopError::StaleSequence {
                last: 5,
                received: 3
            }
        );
    }

    #[test]
    fn critical_fault_requests_teleop_while_autonomous() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        let fault = Fault {
            code: 100,
            severity: FaultSeverity::Critical,
            message: "test critical fault",
        };
        let next = adapter.report_fault(fault);
        assert_eq!(next, Some(HandoverState::RequestingTeleop));
        assert_eq!(
            adapter.get_domain_state().handover_state,
            HandoverState::RequestingTeleop
        );
        assert_eq!(adapter.get_domain_state().active_fault, Some(fault));
    }

    #[test]
    fn emergency_fault_preempts_active_teleop_session() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        adapter.on_teleop_engage(1).unwrap();
        let fault = Fault {
            code: 911,
            severity: FaultSeverity::EmergencyStop,
            message: "test emergency fault",
        };
        adapter.report_fault(fault);
        assert_eq!(
            adapter.get_domain_state().handover_state,
            HandoverState::EmergencyStop
        );
    }

    #[test]
    fn implement_mode_toggles_on_button_edge() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        adapter.on_teleop_engage(1).unwrap();
        let mut cmd = command(1);
        cmd.buttons.implement_select = true;
        adapter.on_control_command(cmd).unwrap();
        assert_eq!(
            adapter.get_domain_state().implement_mode,
            ImplementMode::Hammer
        );
    }

    #[test]
    fn telemetry_feed_reports_last_update() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        let telemetry = DomainTelemetry {
            ground_speed_m_s: 3.5,
            payload_kg: 12000.0,
            engine_rpm: 1800.0,
        };
        adapter.update_telemetry(999, telemetry);
        let feed = adapter.get_sensor_feed();
        assert_eq!(feed.timestamp_us, 999);
        assert_eq!(feed.telemetry, telemetry);
    }

    #[test]
    fn disengage_without_active_session_is_illegal() {
        let mut adapter = ExcavatorTeleopAdapter::new(1000.0);
        let err = adapter.on_teleop_disengage().unwrap_err();
        assert_eq!(
            err,
            TeleopError::IllegalHandoverTransition {
                from: HandoverState::Autonomous,
                to: HandoverState::ReturningToAutonomy,
            }
        );
    }
}
