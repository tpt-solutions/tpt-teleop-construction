// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Predictive maintenance for heavy equipment: a custom pre-allocated FFT
//! for vibration analysis, bearing/gear-mesh/imbalance fault signature
//! detection, and hydraulic pressure / engine hours / undercarriage wear
//! monitoring (spec.txt §8).

mod complex;
mod fft;
mod signature_detection;
mod wear_monitoring;

pub use complex::Complex32;
pub use fft::FftProcessor;
pub use signature_detection::{
    ball_pass_frequency_inner_hz, ball_pass_frequency_outer_hz, ball_spin_frequency_hz,
    detect_fault_signature, gear_mesh_frequency_hz, magnitude_at_frequency, BearingGeometry,
    FaultSignature,
};
pub use wear_monitoring::{EngineHoursMeter, PressureTrendMonitor, UndercarriageWear};
