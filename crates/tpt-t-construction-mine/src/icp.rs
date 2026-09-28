// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! 2D point-to-point ICP (Iterative Closest Point) scan matching (spec.txt
//! §6): aligning a new LiDAR scan against the previous one (or a map) to
//! estimate how far the machine has moved — the measurement
//! [`crate::ekf::Ekf2D::update_pose`] corrects against. Tunnels are
//! navigated in a horizontal plane, so this works in 2D rather than 3D.
//!
//! Rotation/translation extraction uses the closed-form 2D case of Horn's
//! method (Horn, B.K.P. (1987), "Closed-form solution of absolute
//! orientation using unit quaternions"): for 2D, the optimal rotation
//! angle has a direct `atan2` form from the point sets' cross-covariance,
//! with no iterative solve or SVD needed. `core::simd` ("portable-simd")
//! remains nightly-only (rust-lang/rust#86656) as of this toolchain, so —
//! as in `tpt-t-construction-payload::cut_fill_volumes` — the
//! correspondence and cross-covariance loops below are written so LLVM's
//! auto-vectorizer can pack them into SIMD instructions on stable Rust,
//! rather than using unstable explicit intrinsics.

/// A rigid 2D transform: rotate by `rotation_rad` then translate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidTransform2D {
    pub rotation_rad: f32,
    pub translation: [f32; 2],
}

impl RigidTransform2D {
    pub const IDENTITY: RigidTransform2D = RigidTransform2D {
        rotation_rad: 0.0,
        translation: [0.0, 0.0],
    };

    /// Applies this transform to a point.
    pub fn apply(&self, point: [f32; 2]) -> [f32; 2] {
        let (sin_r, cos_r) = self.rotation_rad.sin_cos();
        [
            point[0] * cos_r - point[1] * sin_r + self.translation[0],
            point[0] * sin_r + point[1] * cos_r + self.translation[1],
        ]
    }

    /// Composes `self` applied after `first` (i.e. `self.apply(first.apply(p))`).
    pub fn then(&self, first: &RigidTransform2D) -> RigidTransform2D {
        RigidTransform2D {
            rotation_rad: self.rotation_rad + first.rotation_rad,
            translation: self.apply(first.translation),
        }
    }
}

/// For each point in `source`, the index of its nearest neighbor in
/// `target` (brute-force; tunnel-scan point counts are small enough that
/// this outperforms the bookkeeping overhead of a spatial index).
pub fn nearest_neighbor_correspondences(source: &[[f32; 2]], target: &[[f32; 2]]) -> Vec<usize> {
    source
        .iter()
        .map(|&s| {
            target
                .iter()
                .enumerate()
                .map(|(i, &t)| (i, (s[0] - t[0]).powi(2) + (s[1] - t[1]).powi(2)))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .map(|(i, _)| i)
                .unwrap_or(0)
        })
        .collect()
}

fn centroid(points: &[[f32; 2]]) -> [f32; 2] {
    let n = points.len().max(1) as f32;
    let sum = points
        .iter()
        .fold([0.0, 0.0], |acc, p| [acc[0] + p[0], acc[1] + p[1]]);
    [sum[0] / n, sum[1] / n]
}

/// The optimal rigid transform mapping `source` onto `matched_target`
/// (already paired index-for-index, e.g. via
/// [`nearest_neighbor_correspondences`]), via the closed-form 2D case of
/// Horn's method.
pub fn compute_rigid_transform_2d(
    source: &[[f32; 2]],
    matched_target: &[[f32; 2]],
) -> RigidTransform2D {
    let src_centroid = centroid(source);
    let tgt_centroid = centroid(matched_target);

    // Cross-covariance sums, accumulated across four independent
    // running totals so the loop auto-vectorizes (see module docs).
    let (mut sxx, mut sxy, mut syx, mut syy) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for (s, t) in source.iter().zip(matched_target) {
        let sx = s[0] - src_centroid[0];
        let sy = s[1] - src_centroid[1];
        let tx = t[0] - tgt_centroid[0];
        let ty = t[1] - tgt_centroid[1];
        sxx += sx * tx;
        sxy += sx * ty;
        syx += sy * tx;
        syy += sy * ty;
    }

    let rotation_rad = (sxy - syx).atan2(sxx + syy);
    let (sin_r, cos_r) = rotation_rad.sin_cos();
    let rotated_src_centroid = [
        src_centroid[0] * cos_r - src_centroid[1] * sin_r,
        src_centroid[0] * sin_r + src_centroid[1] * cos_r,
    ];
    let translation = [
        tgt_centroid[0] - rotated_src_centroid[0],
        tgt_centroid[1] - rotated_src_centroid[1],
    ];

    RigidTransform2D {
        rotation_rad,
        translation,
    }
}

/// Runs ICP to alignment: repeatedly finds nearest-neighbor
/// correspondences, solves for the rigid transform they imply, and
/// applies it, until an iteration's incremental step is smaller than
/// both `tolerance_rad` (rotation) and `tolerance_m` (translation), or
/// `max_iterations` is reached. Both must be checked — a pure
/// translation's incremental *rotation* is exactly zero from the first
/// iteration, so rotation alone would report convergence before the
/// translation has actually settled. Returns the total accumulated
/// transform from the original `source` to `target`.
pub fn icp_align(
    source: &[[f32; 2]],
    target: &[[f32; 2]],
    max_iterations: usize,
    tolerance_rad: f32,
    tolerance_m: f32,
) -> RigidTransform2D {
    let mut accumulated = RigidTransform2D::IDENTITY;
    let mut working: Vec<[f32; 2]> = source.to_vec();

    for _ in 0..max_iterations {
        let correspondences = nearest_neighbor_correspondences(&working, target);
        let matched: Vec<[f32; 2]> = correspondences.iter().map(|&i| target[i]).collect();
        let step = compute_rigid_transform_2d(&working, &matched);

        for point in &mut working {
            *point = step.apply(*point);
        }
        accumulated = step.then(&accumulated);

        let step_translation_m = (step.translation[0].powi(2) + step.translation[1].powi(2)).sqrt();
        if step.rotation_rad.abs() < tolerance_rad && step_translation_m < tolerance_m {
            break;
        }
    }
    accumulated
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn sample_ring(n: usize, radius: f32) -> Vec<[f32; 2]> {
        (0..n)
            .map(|i| {
                let angle = 2.0 * std::f32::consts::PI * i as f32 / n as f32;
                [radius * angle.cos(), radius * angle.sin()]
            })
            .collect()
    }

    #[test]
    fn then_composes_transforms_in_apply_order() {
        let first = RigidTransform2D {
            rotation_rad: 0.3,
            translation: [1.0, 2.0],
        };
        let second = RigidTransform2D {
            rotation_rad: -0.5,
            translation: [-3.0, 0.5],
        };
        let composed = second.then(&first);
        let point = [4.0, -1.0];
        let direct = second.apply(first.apply(point));
        let via_composed = composed.apply(point);
        assert!((direct[0] - via_composed[0]).abs() < 1e-4);
        assert!((direct[1] - via_composed[1]).abs() < 1e-4);
    }

    #[test]
    fn identity_transform_leaves_points_unchanged() {
        let p = [3.0, -2.0];
        assert_eq!(RigidTransform2D::IDENTITY.apply(p), p);
    }

    #[test]
    fn correspondences_pick_the_truly_nearest_point() {
        let target = vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]];
        let source = vec![[9.0, 1.0]];
        assert_eq!(nearest_neighbor_correspondences(&source, &target), vec![1]);
    }

    #[test]
    fn recovers_a_pure_translation() {
        // A dense ring (small spacing between points) with a translation
        // well under half that spacing, so nearest-neighbor
        // correspondence can't alias onto the wrong ring point — a
        // sparser ring or a larger translation leaves this a classic
        // ICP local-minimum trap for a rotationally-symmetric shape.
        let source = sample_ring(60, 5.0);
        let translation = [0.3, -0.2];
        let target: Vec<[f32; 2]> = source
            .iter()
            .map(|p| [p[0] + translation[0], p[1] + translation[1]])
            .collect();

        let transform = icp_align(&source, &target, 20, 1e-6, 1e-5);
        assert!((transform.translation[0] - translation[0]).abs() < 1e-2);
        assert!((transform.translation[1] - translation[1]).abs() < 1e-2);
        assert!(transform.rotation_rad.abs() < 1e-2);
    }

    #[test]
    fn recovers_a_pure_rotation() {
        // A ring is rotation-symmetric under nearest-neighbor matching
        // only if the rotation is a multiple of the point spacing, so use
        // an asymmetric shape (points at varying radii) instead.
        let source: Vec<[f32; 2]> = (0..12)
            .map(|i| {
                let angle = 2.0 * std::f32::consts::PI * i as f32 / 12.0;
                let radius = 3.0 + (i as f32 * 0.3);
                [radius * angle.cos(), radius * angle.sin()]
            })
            .collect();
        let true_rotation = 0.15_f32;
        let rotate = RigidTransform2D {
            rotation_rad: true_rotation,
            translation: [0.0, 0.0],
        };
        let target: Vec<[f32; 2]> = source.iter().map(|&p| rotate.apply(p)).collect();

        let transform = icp_align(&source, &target, 30, 1e-6, 1e-5);
        assert!((transform.rotation_rad - true_rotation).abs() < 0.02);
    }

    #[test]
    fn a_representative_scan_aligns_in_well_under_one_hundred_milliseconds() {
        // ~360 points is representative of a single 2D LiDAR sweep at
        // 1-degree resolution in a tunnel — the scan size a 10Hz update
        // rate (spec.txt §6) actually needs to process.
        let source = sample_ring(360, 4.0);
        let translation = [0.3, 0.1];
        let target: Vec<[f32; 2]> = source
            .iter()
            .map(|p| [p[0] + translation[0], p[1] + translation[1]])
            .collect();

        let start = Instant::now();
        let _ = icp_align(&source, &target, 10, 1e-4, 1e-3);
        let elapsed = start.elapsed();

        assert!(
            elapsed.as_millis() < 100,
            "ICP alignment over 360 points took {elapsed:?}, budget is 100ms for 10Hz"
        );
    }
}
