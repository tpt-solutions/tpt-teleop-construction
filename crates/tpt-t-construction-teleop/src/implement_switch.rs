// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Operator implement-switching (todo.md Phase 10: "Implement operator
//! implement-switching (e.g. bucket <-> hammer)"), driven by
//! [`crate::control_command::ButtonState::implement_select`].

/// Which attachment [`crate::control_command::DecodedControlCommand::secondary_axes`]
/// currently drives. Extend this enum (and [`ImplementMode::toggled`])
/// as more attachments are supported — the cycle order is deliberately
/// explicit rather than derived, so adding a variant is a one-line,
/// review-visible change to the cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ImplementMode {
    #[default]
    Bucket,
    Hammer,
}

impl ImplementMode {
    /// The next mode in the operator-facing cycle.
    pub fn toggled(self) -> Self {
        match self {
            ImplementMode::Bucket => ImplementMode::Hammer,
            ImplementMode::Hammer => ImplementMode::Bucket,
        }
    }
}

/// Tracks the current [`ImplementMode`] and advances it on a
/// button-press edge (not level) of `implement_select`, so holding the
/// button down doesn't rapidly cycle modes every tick it's sampled.
#[derive(Debug, Clone, Copy, Default)]
pub struct ImplementSwitch {
    mode: ImplementMode,
    button_was_pressed: bool,
}

impl ImplementSwitch {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn mode(&self) -> ImplementMode {
        self.mode
    }

    /// Feeds one tick's `implement_select` button state. Returns the
    /// (possibly unchanged) current mode.
    pub fn update(&mut self, implement_select_pressed: bool) -> ImplementMode {
        if implement_select_pressed && !self.button_was_pressed {
            self.mode = self.mode.toggled();
        }
        self.button_was_pressed = implement_select_pressed;
        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_in_bucket_mode() {
        assert_eq!(ImplementSwitch::new().mode(), ImplementMode::Bucket);
    }

    #[test]
    fn toggle_cycles_bucket_and_hammer() {
        assert_eq!(ImplementMode::Bucket.toggled(), ImplementMode::Hammer);
        assert_eq!(ImplementMode::Hammer.toggled(), ImplementMode::Bucket);
    }

    #[test]
    fn a_single_press_switches_mode_once() {
        let mut switch = ImplementSwitch::new();
        assert_eq!(switch.update(true), ImplementMode::Hammer);
        assert_eq!(switch.mode(), ImplementMode::Hammer);
    }

    #[test]
    fn holding_the_button_does_not_repeatedly_cycle() {
        let mut switch = ImplementSwitch::new();
        for _ in 0..10 {
            switch.update(true);
        }
        assert_eq!(switch.mode(), ImplementMode::Hammer);
    }

    #[test]
    fn release_and_press_again_switches_back() {
        let mut switch = ImplementSwitch::new();
        switch.update(true);
        switch.update(false);
        switch.update(true);
        assert_eq!(switch.mode(), ImplementMode::Bucket);
    }
}
