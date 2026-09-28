// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Message types carried on the in-process event bus, and the zero-copy
//! wire/on-disk telemetry record shared by every implement crate.
//!
//! [`MachineEvent`] is the bus payload: plain Rust enums moved directly
//! through [`crate::bus`]'s ring buffer, no serialization involved. It never
//! leaves the process.
//!
//! [`TelemetrySample`] is different: it is the `rkyv`-archived record that
//! `tpt-t-construction-telemetry` logs to NVMe at 100Hz (Phase 8) and that
//! `tpt-t-construction-teleop` may eventually stream off-machine. `rkyv`
//! gives zero-copy reads back off of a log file or network buffer — no
//! deserialization pass, no allocation, just a reinterpret of the bytes.

use crate::state::MachineState;
use rkyv::{Archive, Deserialize, Serialize};

/// Severity of a raised [`Fault`], used to decide whether a fault merely
/// logs, degrades autonomy, or forces an immediate stop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultSeverity {
    /// Logged for maintenance review; no operational impact.
    Warning,
    /// Autonomy should request teleop assistance
    /// (`tpt-t-construction-teleop`'s `request_teleop_assistance()`, Phase 10).
    Critical,
    /// Immediate stop required, no handover negotiation.
    EmergencyStop,
}

/// A fault raised by any subsystem (hydraulic, safety, powertrain, ...).
/// `message` is `&'static str` deliberately: fault descriptions are compiled
/// in, not formatted at fault time, so raising one never allocates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fault {
    pub code: u16,
    pub severity: FaultSeverity,
    pub message: &'static str,
}

/// An event moved across [`crate::bus`] between subsystems and
/// [`crate::event_loop::MachineController`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MachineEvent {
    /// A subsystem is requesting the lifecycle FSM move to `next`
    /// (e.g. the mission planner requesting `Moving` once a route is
    /// computed).
    RequestTransition { next: MachineState },
    /// The lifecycle FSM completed a transition.
    StateChanged {
        from: MachineState,
        to: MachineState,
    },
    /// A subsystem raised a fault.
    FaultRaised(Fault),
    /// The shift-start self-test suite finished; `passed` reflects the
    /// aggregate result (see [`crate::self_test::SelfTestReport::passed`]).
    SelfTestCompleted { passed: bool },
}

/// Zero-copy, `rkyv`-archived high-frequency telemetry record. One sample
/// per subsystem per tick; `values` holds subsystem-specific channels
/// (pressures, positions, currents, ...) whose meaning is defined by the
/// producing crate.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
pub struct TelemetrySample {
    pub timestamp_us: u64,
    /// [`MachineState`] encoded as its discriminant; kept as a plain `u8`
    /// here rather than deriving `Archive` on `MachineState` itself, so the
    /// lifecycle FSM's enum layout is free to evolve independently of the
    /// archived wire format.
    pub machine_state: u8,
    pub values: [f32; 8],
}

impl TelemetrySample {
    pub fn new(timestamp_us: u64, machine_state: MachineState, values: [f32; 8]) -> Self {
        TelemetrySample {
            timestamp_us,
            machine_state: machine_state as u8,
            values,
        }
    }

    /// Serializes to an `rkyv`-archived byte buffer suitable for appending
    /// to a telemetry log or sending over the wire.
    pub fn to_bytes(&self) -> rkyv::AlignedVec {
        rkyv::to_bytes::<_, 128>(self).expect("TelemetrySample serialization is infallible")
    }

    /// Reinterprets a byte buffer produced by [`TelemetrySample::to_bytes`]
    /// as an archived sample with no copy and no allocation.
    ///
    /// # Safety
    ///
    /// `bytes` must be a value previously produced by
    /// [`TelemetrySample::to_bytes`] (or otherwise guaranteed to be a valid,
    /// correctly-aligned `rkyv` archive of `TelemetrySample`); this build
    /// does not enable `rkyv`'s `bytecheck` validation, so malformed input
    /// is undefined behavior rather than a caught error.
    pub unsafe fn from_bytes_unchecked(bytes: &[u8]) -> &ArchivedTelemetrySample {
        rkyv::archived_root::<TelemetrySample>(bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_sample_roundtrips_zero_copy() {
        let sample = TelemetrySample::new(
            42,
            MachineState::Working,
            [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
        let bytes = sample.to_bytes();
        let archived = unsafe { TelemetrySample::from_bytes_unchecked(&bytes) };
        assert_eq!(archived.timestamp_us, 42);
        assert_eq!(archived.machine_state, MachineState::Working as u8);
        assert_eq!(archived.values[1], 2.0);
    }

    #[test]
    fn machine_event_is_plain_data_no_serialization() {
        let event = MachineEvent::StateChanged {
            from: MachineState::Idle,
            to: MachineState::Moving,
        };
        // MachineEvent is Copy: moving it onto the bus is a plain memcpy.
        let copy = event;
        assert_eq!(event, copy);
    }
}
