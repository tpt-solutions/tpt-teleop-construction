// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! The wheel loader's repeating dig/lift/carry/dump cycle (spec.txt
//! §5.4): analogous to `tpt-t-construction-excavator`'s swing cycle, but
//! a loader drives to the truck rather than swinging a turret, so its
//! phases are named for that: dig the bucket into the pile, lift it
//! clear, drive/carry to the truck, dump, then return.

/// One phase of the repeating loading cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadPhase {
    Digging,
    Lifting,
    Carrying,
    Dumping,
    Returning,
}

impl LoadPhase {
    fn next(self) -> LoadPhase {
        match self {
            LoadPhase::Digging => LoadPhase::Lifting,
            LoadPhase::Lifting => LoadPhase::Carrying,
            LoadPhase::Carrying => LoadPhase::Dumping,
            LoadPhase::Dumping => LoadPhase::Returning,
            LoadPhase::Returning => LoadPhase::Digging,
        }
    }
}

/// Tracks which phase of the cycle the loader is in and how many full
/// bucket passes it has completed.
#[derive(Debug, Clone, Copy)]
pub struct LoadCycle {
    phase: LoadPhase,
    completed_passes: u32,
}

impl Default for LoadCycle {
    fn default() -> Self {
        LoadCycle {
            phase: LoadPhase::Digging,
            completed_passes: 0,
        }
    }
}

impl LoadCycle {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn phase(&self) -> LoadPhase {
        self.phase
    }

    pub fn completed_passes(&self) -> u32 {
        self.completed_passes
    }

    /// Advances to the next phase, incrementing the completed-pass count
    /// exactly when `Returning` finishes.
    pub fn advance(&mut self) {
        if self.phase == LoadPhase::Returning {
            self.completed_passes += 1;
        }
        self.phase = self.phase.next();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_digging() {
        let cycle = LoadCycle::new();
        assert_eq!(cycle.phase(), LoadPhase::Digging);
        assert_eq!(cycle.completed_passes(), 0);
    }

    #[test]
    fn advances_through_every_phase_in_order() {
        let mut cycle = LoadCycle::new();
        let expected = [
            LoadPhase::Lifting,
            LoadPhase::Carrying,
            LoadPhase::Dumping,
            LoadPhase::Returning,
            LoadPhase::Digging,
        ];
        for phase in expected {
            cycle.advance();
            assert_eq!(cycle.phase(), phase);
        }
    }

    #[test]
    fn completed_passes_increments_once_per_full_loop() {
        let mut cycle = LoadCycle::new();
        for _ in 0..5 {
            cycle.advance();
        }
        assert_eq!(cycle.completed_passes(), 1);
        for _ in 0..10 {
            cycle.advance();
        }
        assert_eq!(cycle.completed_passes(), 3);
    }
}
