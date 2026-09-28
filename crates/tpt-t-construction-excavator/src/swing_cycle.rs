// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Truck loading with minimal swing cycle time (spec.txt §5.1): the
//! repeating dig -> swing-to-truck -> dump -> swing-to-face cycle, and an
//! estimate of how long one full cycle takes so the coordination logic in
//! `tpt-t-construction-core` (Phase 9's excavator/haul-truck queue
//! management) can predict when this excavator will next be ready to load.
//!
//! This is a specialization of the swing/dig/dump portion of the
//! lifecycle that sits *inside* `tpt-t-construction-core::MachineState::
//! Working` — the excavator doesn't leave `Working` between passes, it
//! just cycles through these four phases while it stays there.

use std::time::Duration;

/// One phase of the repeating dig/swing/dump/swing cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwingPhase {
    Digging,
    SwingToTruck,
    Dumping,
    SwingToFace,
}

impl SwingPhase {
    fn next(self) -> SwingPhase {
        match self {
            SwingPhase::Digging => SwingPhase::SwingToTruck,
            SwingPhase::SwingToTruck => SwingPhase::Dumping,
            SwingPhase::Dumping => SwingPhase::SwingToFace,
            SwingPhase::SwingToFace => SwingPhase::Digging,
        }
    }
}

/// Tracks which phase of the cycle the excavator is currently in and how
/// many full cycles (loads delivered) it has completed.
#[derive(Debug, Clone, Copy)]
pub struct SwingCycle {
    phase: SwingPhase,
    completed_cycles: u32,
}

impl Default for SwingCycle {
    fn default() -> Self {
        SwingCycle {
            phase: SwingPhase::Digging,
            completed_cycles: 0,
        }
    }
}

impl SwingCycle {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn phase(&self) -> SwingPhase {
        self.phase
    }

    pub fn completed_cycles(&self) -> u32 {
        self.completed_cycles
    }

    /// Advances to the next phase, incrementing the completed-cycle count
    /// exactly when a `SwingToFace` finishes (i.e. one full load has been
    /// delivered and the arm is back at the dig face ready to start over).
    pub fn advance(&mut self) {
        if self.phase == SwingPhase::SwingToFace {
            self.completed_cycles += 1;
        }
        self.phase = self.phase.next();
    }
}

/// Estimates the duration of one full dig/swing/dump/swing cycle from its
/// component timings — the quantity spec.txt §4.6's truck-arrival
/// prediction and dispatch queueing needs to know "when will this
/// excavator next be ready to load a truck".
///
/// `swing_angle_rad` is the angle actually swung through to reach the
/// truck (and swung back), so a truck spotted closer to on-axis with the
/// dig face costs less swing time than one parked at a wide angle — this
/// is exactly the "minimal swing cycle time" lever spec.txt calls out.
pub fn estimate_cycle_time(
    dig_time: Duration,
    dump_time: Duration,
    swing_angle_rad: f32,
    swing_speed_rad_s: f32,
) -> Duration {
    let swing_seconds = 2.0 * swing_angle_rad.abs() / swing_speed_rad_s;
    dig_time + dump_time + Duration::from_secs_f32(swing_seconds.max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_digging() {
        let cycle = SwingCycle::new();
        assert_eq!(cycle.phase(), SwingPhase::Digging);
        assert_eq!(cycle.completed_cycles(), 0);
    }

    #[test]
    fn advances_through_every_phase_in_order() {
        let mut cycle = SwingCycle::new();
        cycle.advance();
        assert_eq!(cycle.phase(), SwingPhase::SwingToTruck);
        cycle.advance();
        assert_eq!(cycle.phase(), SwingPhase::Dumping);
        cycle.advance();
        assert_eq!(cycle.phase(), SwingPhase::SwingToFace);
        cycle.advance();
        assert_eq!(cycle.phase(), SwingPhase::Digging);
    }

    #[test]
    fn completed_cycles_increments_once_per_full_loop() {
        let mut cycle = SwingCycle::new();
        for _ in 0..4 {
            cycle.advance();
        }
        assert_eq!(cycle.completed_cycles(), 1);
        for _ in 0..8 {
            cycle.advance();
        }
        assert_eq!(cycle.completed_cycles(), 3);
    }

    #[test]
    fn wider_swing_angle_takes_longer() {
        let narrow = estimate_cycle_time(Duration::from_secs(4), Duration::from_secs(2), 0.5, 1.0);
        let wide = estimate_cycle_time(Duration::from_secs(4), Duration::from_secs(2), 1.5, 1.0);
        assert!(wide > narrow);
    }

    #[test]
    fn cycle_time_includes_dig_and_dump_time_at_zero_swing() {
        let cycle_time =
            estimate_cycle_time(Duration::from_secs(5), Duration::from_secs(3), 0.0, 1.0);
        assert_eq!(cycle_time, Duration::from_secs(8));
    }
}
