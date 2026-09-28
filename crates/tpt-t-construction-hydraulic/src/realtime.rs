// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Real-time scheduling for the 1kHz control loop (spec.txt §4.1: "a
//! dedicated, pinned CPU core with `SCHED_FIFO` priority") and a
//! zero-allocation timing helper for verifying the loop's <100us budget.
//!
//! Pinning a thread and elevating it to `SCHED_FIFO` requires a privilege
//! (`CAP_SYS_NICE` on Linux) that a developer's machine or a CI container
//! commonly lacks, so [`pin_and_elevate`] returns a [`Result`] rather than
//! panicking on failure — callers should log and continue at the default
//! scheduling policy, not treat this as fatal outside a real deployment.

use std::fmt;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RealtimeError {
    /// A Linux syscall (`sched_setaffinity`/`sched_setscheduler`) failed;
    /// carries its `errno` (e.g. `EPERM` without `CAP_SYS_NICE`).
    Errno(i32),
    /// This platform has no `SCHED_FIFO`/CPU-affinity support wired up.
    Unsupported,
}

impl fmt::Display for RealtimeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RealtimeError::Errno(errno) => {
                write!(f, "real-time scheduling syscall failed (errno {errno})")
            }
            RealtimeError::Unsupported => {
                write!(f, "real-time scheduling is not supported on this platform")
            }
        }
    }
}

impl std::error::Error for RealtimeError {}

/// Pins the calling thread to logical CPU `core_id` and elevates it to
/// `SCHED_FIFO` at `priority` (1-99, higher preempts lower). Intended to be
/// called once, from the thread that will run the 1kHz loop, before
/// entering it.
#[cfg(target_os = "linux")]
pub fn pin_and_elevate(core_id: usize, priority: i32) -> Result<(), RealtimeError> {
    // SAFETY: `set` is a plain `cpu_set_t` we own and zero-initialize
    // before use; `sched_setaffinity`/`sched_setscheduler` are called with
    // pid 0 (the calling thread) and valid, correctly-sized pointers to
    // stack-local data whose lifetime covers the call.
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut set);
        libc::CPU_SET(core_id, &mut set);
        if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
            return Err(RealtimeError::Errno(*libc::__errno_location()));
        }

        let param = libc::sched_param {
            sched_priority: priority,
        };
        if libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) != 0 {
            return Err(RealtimeError::Errno(*libc::__errno_location()));
        }
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
pub fn pin_and_elevate(_core_id: usize, _priority: i32) -> Result<(), RealtimeError> {
    Err(RealtimeError::Unsupported)
}

/// Tracks whether each iteration of a real-time loop stayed within its
/// timing budget (spec.txt §4.1's "<100 microseconds"), with no heap
/// allocation on the hot path: just two counters and an `Instant`.
#[derive(Debug, Clone, Copy)]
pub struct LoopBudget {
    budget: Duration,
    started_at: Option<Instant>,
    iterations: u64,
    overruns: u64,
}

impl LoopBudget {
    pub fn new(budget: Duration) -> Self {
        LoopBudget {
            budget,
            started_at: None,
            iterations: 0,
            overruns: 0,
        }
    }

    /// Marks the start of one loop iteration.
    pub fn begin(&mut self) {
        self.started_at = Some(Instant::now());
    }

    /// Marks the end of the iteration started by the most recent
    /// [`LoopBudget::begin`] call, recording an overrun if it took longer
    /// than the configured budget, and returns the measured duration.
    ///
    /// # Panics
    ///
    /// Panics if called without a preceding `begin()` — that pairing is a
    /// programming error in the caller's loop, not a runtime condition to
    /// recover from.
    pub fn end(&mut self) -> Duration {
        let elapsed = self
            .started_at
            .take()
            .expect("LoopBudget::end called without a matching begin()")
            .elapsed();
        self.iterations += 1;
        if elapsed > self.budget {
            self.overruns += 1;
        }
        elapsed
    }

    pub fn iterations(&self) -> u64 {
        self.iterations
    }

    pub fn overruns(&self) -> u64 {
        self.overruns
    }

    /// Fraction of recorded iterations that exceeded budget, `0.0` if none
    /// have been recorded yet.
    pub fn overrun_fraction(&self) -> f64 {
        if self.iterations == 0 {
            0.0
        } else {
            self.overruns as f64 / self.iterations as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pin_and_elevate_does_not_panic_regardless_of_privilege() {
        // This may succeed (running as root/CAP_SYS_NICE) or fail
        // (a normal user, a restricted container) depending on the
        // environment; either is a valid outcome the caller must handle,
        // so this test only asserts it never panics.
        let _ = pin_and_elevate(0, 1);
    }

    #[test]
    fn fast_iteration_is_not_an_overrun() {
        let mut budget = LoopBudget::new(Duration::from_millis(50));
        budget.begin();
        let _ = budget.end();
        assert_eq!(budget.iterations(), 1);
        assert_eq!(budget.overruns(), 0);
    }

    #[test]
    fn slow_iteration_is_recorded_as_an_overrun() {
        let mut budget = LoopBudget::new(Duration::from_millis(5));
        budget.begin();
        std::thread::sleep(Duration::from_millis(20));
        let elapsed = budget.end();
        assert!(elapsed >= Duration::from_millis(20));
        assert_eq!(budget.overruns(), 1);
        assert!((budget.overrun_fraction() - 1.0).abs() < 1e-9);
    }

    #[test]
    #[should_panic(expected = "without a matching begin()")]
    fn end_without_begin_panics() {
        let mut budget = LoopBudget::new(Duration::from_millis(1));
        budget.end();
    }
}
