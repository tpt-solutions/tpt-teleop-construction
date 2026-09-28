// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Minimal hand-rolled 3x3 matrix/3-vector math for [`crate::ekf`]'s
//! covariance propagation. A fixed 3x3 size doesn't justify pulling in a
//! general linear algebra crate (matching this workspace's no-bloat
//! policy, spec.txt §7) — these are the handful of operations an EKF over
//! a 3-state `(x, y, theta)` pose actually needs.

pub type Mat3 = [[f32; 3]; 3];
pub type Vec3f = [f32; 3];

pub fn identity() -> Mat3 {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

pub fn add(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] + b[i][j]))
}

pub fn sub(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] - b[i][j]))
}

pub fn mul(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

pub fn transpose(a: &Mat3) -> Mat3 {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

pub fn vec_mul(a: &Mat3, v: &Vec3f) -> Vec3f {
    std::array::from_fn(|i| (0..3).map(|k| a[i][k] * v[k]).sum())
}

/// The matrix inverse via the cofactor/adjugate method, or `None` if `m`
/// is singular (determinant magnitude below a small epsilon).
pub fn inverse(m: &Mat3) -> Option<Mat3> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_times_anything_is_that_thing() {
        let m: Mat3 = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]];
        assert_eq!(mul(&identity(), &m), m);
    }

    #[test]
    fn inverse_of_identity_is_identity() {
        assert_eq!(inverse(&identity()), Some(identity()));
    }

    #[test]
    fn matrix_times_its_inverse_is_identity() {
        let m: Mat3 = [[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 2.0]];
        let inv = inverse(&m).unwrap();
        let product = mul(&m, &inv);
        for (i, row) in product.iter().enumerate() {
            for (j, &value) in row.iter().enumerate() {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((value - expected).abs() < 1e-4);
            }
        }
    }

    #[test]
    fn singular_matrix_has_no_inverse() {
        let m: Mat3 = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [1.0, 1.0, 1.0]];
        assert_eq!(inverse(&m), None);
    }

    #[test]
    fn vec_mul_applies_each_row_as_a_dot_product() {
        let m: Mat3 = [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]];
        assert_eq!(vec_mul(&m, &[1.0, 1.0, 1.0]), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn transpose_swaps_rows_and_columns() {
        let m: Mat3 = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        assert_eq!(transpose(&m)[0], [1.0, 4.0, 7.0]);
    }
}
