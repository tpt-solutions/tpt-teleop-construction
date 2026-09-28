// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! UWB (Ultra-Wideband) beacon integration for absolute positioning
//! underground (spec.txt §6: "<0.5m accuracy"), via least-squares
//! multilateration from range measurements to three or more beacons at
//! known, surveyed positions.
//!
//! Uses the standard range-difference-of-squares linearization (subtract
//! one reference beacon's equation from every other beacon's equation to
//! cancel the quadratic position terms, leaving a linear least-squares
//! system) rather than an iterative nonlinear solve — exact for
//! noise-free ranges, and a good linear approximation for the small
//! range errors real UWB hardware produces.

/// A UWB beacon's fixed, surveyed position.
#[derive(Debug, Clone, Copy)]
pub struct Beacon {
    pub position: [f32; 2],
}

/// One range measurement to a beacon, identified by its index into the
/// beacon list passed to [`multilateration_2d`].
#[derive(Debug, Clone, Copy)]
pub struct RangeMeasurement {
    pub beacon_index: usize,
    pub range_m: f32,
}

/// Estimates a 2D position from range measurements to three or more
/// beacons, via linearized least squares. Returns `None` if fewer than
/// three measurements are given (underdetermined) or the resulting
/// linear system is singular (e.g. all beacons collinear).
pub fn multilateration_2d(
    beacons: &[Beacon],
    measurements: &[RangeMeasurement],
) -> Option<[f32; 2]> {
    if measurements.len() < 3 {
        return None;
    }

    let reference = measurements[0];
    let (x0, y0) = (
        beacons[reference.beacon_index].position[0],
        beacons[reference.beacon_index].position[1],
    );
    let r0 = reference.range_m;

    // Normal equations for the least-squares linear system.
    let mut ata = [[0.0f32; 2]; 2];
    let mut atb = [0.0f32; 2];
    for m in &measurements[1..] {
        let (xi, yi) = (
            beacons[m.beacon_index].position[0],
            beacons[m.beacon_index].position[1],
        );
        let a0 = 2.0 * (xi - x0);
        let a1 = 2.0 * (yi - y0);
        let b = (xi * xi + yi * yi - m.range_m * m.range_m) - (x0 * x0 + y0 * y0 - r0 * r0);

        ata[0][0] += a0 * a0;
        ata[0][1] += a0 * a1;
        ata[1][0] += a1 * a0;
        ata[1][1] += a1 * a1;
        atb[0] += a0 * b;
        atb[1] += a1 * b;
    }

    let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
    if det.abs() < 1e-9 {
        return None;
    }
    let x = (atb[0] * ata[1][1] - atb[1] * ata[0][1]) / det;
    let y = (ata[0][0] * atb[1] - ata[1][0] * atb[0]) / det;
    Some([x, y])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn range_to(beacon: &Beacon, position: [f32; 2]) -> f32 {
        let dx = beacon.position[0] - position[0];
        let dy = beacon.position[1] - position[1];
        (dx * dx + dy * dy).sqrt()
    }

    fn square_beacons() -> Vec<Beacon> {
        vec![
            Beacon {
                position: [0.0, 0.0],
            },
            Beacon {
                position: [50.0, 0.0],
            },
            Beacon {
                position: [0.0, 50.0],
            },
            Beacon {
                position: [50.0, 50.0],
            },
        ]
    }

    #[test]
    fn fewer_than_three_measurements_is_underdetermined() {
        let beacons = square_beacons();
        let measurements = vec![
            RangeMeasurement {
                beacon_index: 0,
                range_m: 10.0,
            },
            RangeMeasurement {
                beacon_index: 1,
                range_m: 10.0,
            },
        ];
        assert_eq!(multilateration_2d(&beacons, &measurements), None);
    }

    #[test]
    fn exact_ranges_recover_the_true_position() {
        let beacons = square_beacons();
        let true_position = [22.0, 31.0];
        let measurements: Vec<RangeMeasurement> = beacons
            .iter()
            .enumerate()
            .map(|(i, b)| RangeMeasurement {
                beacon_index: i,
                range_m: range_to(b, true_position),
            })
            .collect();

        let estimated = multilateration_2d(&beacons, &measurements).unwrap();
        assert!((estimated[0] - true_position[0]).abs() < 1e-2);
        assert!((estimated[1] - true_position[1]).abs() < 1e-2);
    }

    #[test]
    fn small_range_errors_still_meet_the_half_meter_accuracy_target() {
        let beacons = square_beacons();
        let true_position = [15.0, 40.0];
        // +/- 5cm range error per beacon, representative UWB ranging noise.
        let errors = [0.05, -0.04, 0.03, -0.02];
        let measurements: Vec<RangeMeasurement> = beacons
            .iter()
            .enumerate()
            .map(|(i, b)| RangeMeasurement {
                beacon_index: i,
                range_m: range_to(b, true_position) + errors[i],
            })
            .collect();

        let estimated = multilateration_2d(&beacons, &measurements).unwrap();
        let error_m = ((estimated[0] - true_position[0]).powi(2)
            + (estimated[1] - true_position[1]).powi(2))
        .sqrt();
        assert!(
            error_m < 0.5,
            "position error {error_m}m exceeds the 0.5m target"
        );
    }

    #[test]
    fn collinear_beacons_yield_a_singular_system() {
        let beacons = vec![
            Beacon {
                position: [0.0, 0.0],
            },
            Beacon {
                position: [10.0, 0.0],
            },
            Beacon {
                position: [20.0, 0.0],
            },
        ];
        let true_position = [5.0, 5.0];
        let measurements: Vec<RangeMeasurement> = beacons
            .iter()
            .enumerate()
            .map(|(i, b)| RangeMeasurement {
                beacon_index: i,
                range_m: range_to(b, true_position),
            })
            .collect();
        assert_eq!(multilateration_2d(&beacons, &measurements), None);
    }
}
