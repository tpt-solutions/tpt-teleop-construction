// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Triple modular redundancy (TMR) voting (todo.md Phase 11: "Redundant
//! safety monitoring / TMR voter implementation").
//!
//! Safety-critical inputs this workspace depends on for a stop/slowdown
//! decision — a proximity classification, a tilt estimate, a hydraulic
//! pressure reading — are worth cross-checking against multiple redundant
//! sources (three physically independent sensors of the same quantity,
//! or three independently-computed estimates of the same value) before
//! acting on them: a single faulted sensor should never be trusted
//! silently. [`vote`] implements the standard TMR decision: take the
//! value two or more of three channels agree on (within a caller-supplied
//! tolerance for floating-point channels), and flag disagreement so a
//! caller can raise a maintenance fault even when a majority still let it
//! reach a safe decision.

/// The outcome of voting three redundant channels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VoteResult<T> {
    /// All three channels agreed.
    Unanimous(T),
    /// Exactly two channels agreed; `dissenting_channel` identifies the
    /// outlier (`0`, `1`, or `2`) so it can be flagged for maintenance.
    Majority { value: T, dissenting_channel: usize },
    /// No two channels agreed — the voter cannot produce a trustworthy
    /// value at all. A caller must treat this as a fault condition (e.g.
    /// force [`crate`]'s hazard response to its most conservative state)
    /// rather than falling back to any single channel.
    NoQuorum,
}

impl<T> VoteResult<T> {
    /// The voted value, if a quorum (unanimous or majority) was reached.
    pub fn value(self) -> Option<T> {
        match self {
            VoteResult::Unanimous(v) => Some(v),
            VoteResult::Majority { value, .. } => Some(value),
            VoteResult::NoQuorum => None,
        }
    }

    pub fn is_quorum(&self) -> bool {
        !matches!(self, VoteResult::NoQuorum)
    }
}

/// Votes three exact-equality-comparable channel readings (e.g. a
/// discrete classification or FSM state read redundantly from three
/// independent subsystems).
pub fn vote_exact<T: PartialEq + Copy>(channels: [T; 3]) -> VoteResult<T> {
    let [a, b, c] = channels;
    if a == b && b == c {
        VoteResult::Unanimous(a)
    } else if a == b {
        VoteResult::Majority {
            value: a,
            dissenting_channel: 2,
        }
    } else if a == c {
        VoteResult::Majority {
            value: a,
            dissenting_channel: 1,
        }
    } else if b == c {
        VoteResult::Majority {
            value: b,
            dissenting_channel: 0,
        }
    } else {
        VoteResult::NoQuorum
    }
}

/// Votes three floating-point channel readings that agree within
/// `tolerance` of each other (e.g. three redundant pressure transducers
/// or tilt estimators reading the same physical quantity). The voted
/// value, when a quorum exists, is the mean of the agreeing channels —
/// not an arbitrary pick among them.
pub fn vote_f32(channels: [f32; 3], tolerance: f32) -> VoteResult<f32> {
    let [a, b, c] = channels;
    let ab = (a - b).abs() <= tolerance;
    let ac = (a - c).abs() <= tolerance;
    let bc = (b - c).abs() <= tolerance;

    if ab && ac && bc {
        VoteResult::Unanimous((a + b + c) / 3.0)
    } else if ab {
        VoteResult::Majority {
            value: (a + b) / 2.0,
            dissenting_channel: 2,
        }
    } else if ac {
        VoteResult::Majority {
            value: (a + c) / 2.0,
            dissenting_channel: 1,
        }
    } else if bc {
        VoteResult::Majority {
            value: (b + c) / 2.0,
            dissenting_channel: 0,
        }
    } else {
        VoteResult::NoQuorum
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_unanimous_agreement() {
        let result = vote_exact([7, 7, 7]);
        assert_eq!(result, VoteResult::Unanimous(7));
        assert_eq!(result.value(), Some(7));
        assert!(result.is_quorum());
    }

    #[test]
    fn exact_majority_flags_the_correct_dissenter() {
        assert_eq!(
            vote_exact([1, 1, 2]),
            VoteResult::Majority {
                value: 1,
                dissenting_channel: 2
            }
        );
        assert_eq!(
            vote_exact([1, 2, 1]),
            VoteResult::Majority {
                value: 1,
                dissenting_channel: 1
            }
        );
        assert_eq!(
            vote_exact([2, 1, 1]),
            VoteResult::Majority {
                value: 1,
                dissenting_channel: 0
            }
        );
    }

    #[test]
    fn exact_no_quorum_when_all_three_disagree() {
        let result = vote_exact([1, 2, 3]);
        assert_eq!(result, VoteResult::NoQuorum);
        assert_eq!(result.value(), None);
        assert!(!result.is_quorum());
    }

    #[test]
    fn f32_unanimous_within_tolerance_averages_all_three() {
        let result = vote_f32([10.0, 10.05, 9.98], 0.1);
        match result {
            VoteResult::Unanimous(v) => assert!((v - 10.01).abs() < 1e-3),
            other => panic!("expected Unanimous, got {other:?}"),
        }
    }

    #[test]
    fn f32_majority_excludes_the_outlier_from_the_average() {
        // channel 2 (index 2) is a wild outlier; 0 and 1 agree.
        let result = vote_f32([10.0, 10.02, 500.0], 0.1);
        assert_eq!(
            result,
            VoteResult::Majority {
                value: 10.01,
                dissenting_channel: 2
            }
        );
    }

    #[test]
    fn f32_no_quorum_when_every_pair_exceeds_tolerance() {
        let result = vote_f32([0.0, 10.0, 20.0], 0.1);
        assert_eq!(result, VoteResult::NoQuorum);
    }

    #[test]
    fn f32_boundary_exactly_at_tolerance_counts_as_agreement() {
        let result = vote_f32([10.0, 10.1, 10.1], 0.1);
        assert!(result.is_quorum());
    }
}
