// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The Domain Teleoperation Interface (DTI): bridge spec.txt's trait
//! contract between `tpt-teleop-domain-bridge` and each domain's adapter
//! crate, defined *locally* here rather than imported, since
//! `tpt-teleop-domain-bridge` lives in the sister `tpt-teleop` repo and
//! isn't a dependency this workspace can pull in (see
//! [`crate::control_command`]'s module docs for the same constraint on
//! the wire format). `operator_id` is `u32` here, not `Uuid`, for the same
//! reason and consistency with [`crate::control_command::RawControlCommand::operator_session_id`].
//!
//! [`SensorFeed`] deliberately omits video/point-cloud/haptic/audio
//! streams that bridge spec.txt's sensor feed also carries — those are
//! Phase 12 (hardware bring-up) concerns tied to real device drivers this
//! simulated/sandboxed workspace doesn't have, not something this trait
//! can meaningfully stub out today.

use tpt_t_construction_core::Fault;

use crate::implement_switch::ImplementMode;
use crate::safety_fsm::HandoverState;

/// An error returned by a [`DomainTeleopInterface`] method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TeleopError {
    /// The requested transition is illegal from the adapter's current
    /// [`HandoverState`] (see [`HandoverState::can_transition_to`]).
    IllegalHandoverTransition {
        from: HandoverState,
        to: HandoverState,
    },
    /// A control command arrived while no operator session is engaged
    /// (i.e. the adapter isn't `TeleopActive`).
    NoActiveSession,
    /// A control command's `sequence` was not greater than the last
    /// accepted sequence for this session — a dropped/reordered/replayed
    /// packet, rejected rather than applied.
    StaleSequence { last: u32, received: u32 },
}

/// The current teleoperation-relevant state of the domain adapter,
/// returned by [`DomainTeleopInterface::get_domain_state`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DomainState {
    pub handover_state: HandoverState,
    pub active_fault: Option<Fault>,
    pub implement_mode: ImplementMode,
}

/// A machine's operationally-relevant telemetry, the subset of
/// [`tpt_t_construction_core::TelemetrySample`] a teleop operator needs
/// on the downlink to drive safely.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DomainTelemetry {
    pub ground_speed_m_s: f32,
    pub payload_kg: f32,
    pub engine_rpm: f32,
}

/// A timestamped sensor/telemetry feed for the downlink to the operator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorFeed {
    pub timestamp_us: u64,
    pub telemetry: DomainTelemetry,
}

/// The Domain Teleoperation Interface: the contract
/// `tpt-teleop-domain-bridge` drives against, implemented per-machine
/// (see [`crate::adapter::ExcavatorTeleopAdapter`]).
pub trait DomainTeleopInterface {
    /// An operator is requesting control. Moves the handover FSM toward
    /// [`HandoverState::TeleopActive`] (via [`HandoverState::RequestingTeleop`]
    /// if not already there) and records `operator_id` as the active
    /// session.
    fn on_teleop_engage(&mut self, operator_id: u32) -> Result<(), TeleopError>;

    /// The operator is releasing control. Moves the handover FSM to
    /// [`HandoverState::ReturningToAutonomy`].
    fn on_teleop_disengage(&mut self) -> Result<(), TeleopError>;

    /// A control command arrived from the active operator session.
    /// Rejects commands with a stale/reordered `sequence` or that arrive
    /// while no session is engaged.
    fn on_control_command(
        &mut self,
        command: crate::control_command::DecodedControlCommand,
    ) -> Result<(), TeleopError>;

    /// The adapter's current teleoperation-relevant state.
    fn get_domain_state(&self) -> DomainState;

    /// The current downlink sensor feed for the operator.
    fn get_sensor_feed(&self) -> SensorFeed;
}
