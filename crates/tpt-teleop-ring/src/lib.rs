// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Vendored lock-free, bounded, zero-allocation-after-construction ring
//! buffers used across the `tpt` ecosystem for IPC and telemetry between
//! real-time threads.
//!
//! Two flavors are provided:
//!
//! - [`spsc`]: single-producer single-consumer. The fastest option, and the
//!   right choice whenever a hot loop (e.g. the 1kHz hydraulic PID loop)
//!   hands data to exactly one downstream reader.
//! - [`mpmc`]: multi-producer multi-consumer (Vyukov's bounded queue). Used
//!   as an MPSC channel wherever several subsystems independently publish
//!   events onto `tpt-t-construction-core`'s single event-loop consumer.
//!
//! Neither type allocates on the push/pop path: the backing storage is
//! sized once at construction and never resized. Both are `no_std`-friendly
//! modulo `alloc` (the one allocation is the boxed backing buffer); this
//! crate currently depends on `std` for `Arc`/threading primitives only.

mod mpmc;
mod spsc;
mod util;

pub use mpmc::{mpmc, Full as MpmcFull, Receiver, Sender};
pub use spsc::{spsc, Consumer, Full as SpscFull, Producer};
