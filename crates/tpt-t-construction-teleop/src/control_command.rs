// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Reconciling the aspirational `ControlCommand` against the actual wire
//! format (bridge spec.txt §4.1's open decision, not yet resolved
//! upstream): `tpt-teleop-domain-bridge`'s documented struct carries a
//! `Uuid` operator id, two full 8-axis `InputState`s (primary + secondary,
//! each with an optional 6DOF pose and 6 haptic force channels), a
//! `ButtonState`, and a `HapticCmd` — there is no way that fits in a flat
//! 56-byte POD, and bridge spec.txt says the type actually implemented in
//! `tpt-teleop-core/src/ser/cmd.rs` *is* a flat 56-byte POD with none of
//! those fields. `tpt-teleop-domain-bridge` isn't a dependency this
//! workspace can pull in (it lives in the sister `tpt-teleop` repo, not
//! here), so this module makes the concrete engineering call bridge
//! spec.txt leaves open: define the 56-byte wire struct this crate
//! actually decodes, and be explicit about what got cut to fit.
//!
//! What got cut, and why it's a reasonable cut:
//! - **`Uuid` operator_id (16 bytes) -> `u32` session id.** A hot-path
//!   per-tick command struct shouldn't carry a full UUID; a `u32` handle
//!   into a session lookup table (resolved once at `on_teleop_engage`,
//!   not every tick) covers the same need in 4 bytes. This is the
//!   specific fix this module recommends upstream.
//! - **Two independent 8-axis `InputState`s -> one shared 8-axis array,
//!   split in half.** `axes[0..4]` are the primary implement axes (boom,
//!   arm, bucket, swing); `axes[4..8]` are secondary/implement-specific
//!   axes (see [`crate::implement_switch`]). A VR/AR 6DOF pose and
//!   per-axis haptic force feedback don't fit this budget at all — those
//!   need their own, larger, lower-rate message type, not the 100Hz
//!   control-command hot path.
//! - **`HapticCmd` -> dropped entirely.** Force feedback to the operator
//!   is a downlink concern (sensor feed / haptic device driver), not
//!   something that needs to ride on the uplink control command.

/// The actual wire-format control command: exactly 56 bytes, matching
/// bridge spec.txt's description of `tpt-teleop-core/src/ser/cmd.rs`.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawControlCommand {
    pub timestamp_us: u64,
    pub axes: [f32; 8],
    /// Bitmask; see [`ButtonState::from_bits`] for the assigned bits.
    pub buttons: u32,
    /// A session handle, not a `Uuid` — see the module docs.
    pub operator_session_id: u32,
    /// Monotonically increasing per session, for drop/reorder detection.
    pub sequence: u32,
    /// Encodes [`crate::implement_switch::ImplementMode`] (and reserved
    /// for future modes), as a fixed-width field rather than relying on
    /// an enum's platform-dependent representation.
    pub mode: u32,
}

impl RawControlCommand {
    pub const SIZE: usize = std::mem::size_of::<RawControlCommand>();
}

/// Individual button/switch states, decoded from
/// [`RawControlCommand::buttons`]'s bitmask.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ButtonState {
    pub emergency_stop: bool,
    pub mode_switch: bool,
    pub implement_select: bool,
}

impl ButtonState {
    const EMERGENCY_STOP_BIT: u32 = 0x1;
    const MODE_SWITCH_BIT: u32 = 0x2;
    const IMPLEMENT_SELECT_BIT: u32 = 0x4;

    pub fn from_bits(bits: u32) -> Self {
        ButtonState {
            emergency_stop: bits & Self::EMERGENCY_STOP_BIT != 0,
            mode_switch: bits & Self::MODE_SWITCH_BIT != 0,
            implement_select: bits & Self::IMPLEMENT_SELECT_BIT != 0,
        }
    }
}

/// The decoded form of a [`RawControlCommand`] this crate's control logic
/// actually operates on — the reconciled stand-in for bridge spec.txt's
/// aspirational `ControlCommand`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodedControlCommand {
    pub timestamp_us: u64,
    /// Boom, arm, bucket, swing — see [`crate::valve_mapping`].
    pub primary_axes: [f32; 4],
    /// Implement-specific (e.g. hammer trigger, thumb wheel) — see
    /// [`crate::implement_switch`].
    pub secondary_axes: [f32; 4],
    pub buttons: ButtonState,
    pub operator_session_id: u32,
    pub sequence: u32,
    pub mode_bits: u32,
}

impl From<RawControlCommand> for DecodedControlCommand {
    fn from(raw: RawControlCommand) -> Self {
        DecodedControlCommand {
            timestamp_us: raw.timestamp_us,
            primary_axes: [raw.axes[0], raw.axes[1], raw.axes[2], raw.axes[3]],
            secondary_axes: [raw.axes[4], raw.axes[5], raw.axes[6], raw.axes[7]],
            buttons: ButtonState::from_bits(raw.buttons),
            operator_session_id: raw.operator_session_id,
            sequence: raw.sequence,
            mode_bits: raw.mode,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_control_command_is_exactly_56_bytes() {
        assert_eq!(RawControlCommand::SIZE, 56);
    }

    #[test]
    fn button_bits_decode_independently() {
        let state = ButtonState::from_bits(0x1 | 0x4);
        assert!(state.emergency_stop);
        assert!(!state.mode_switch);
        assert!(state.implement_select);
    }

    #[test]
    fn no_bits_set_decodes_to_all_false() {
        assert_eq!(ButtonState::from_bits(0), ButtonState::default());
    }

    #[test]
    fn decoding_splits_axes_into_primary_and_secondary() {
        let raw = RawControlCommand {
            timestamp_us: 1000,
            axes: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
            buttons: 0,
            operator_session_id: 42,
            sequence: 7,
            mode: 0,
        };
        let decoded: DecodedControlCommand = raw.into();
        assert_eq!(decoded.primary_axes, [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(decoded.secondary_axes, [5.0, 6.0, 7.0, 8.0]);
        assert_eq!(decoded.operator_session_id, 42);
        assert_eq!(decoded.sequence, 7);
    }
}
