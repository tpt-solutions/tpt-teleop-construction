// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! A minimal `f32` complex number, hand-rolled rather than pulling in
//! `num-complex`: [`crate::fft`] needs exactly add/subtract/multiply and
//! magnitude, and a single-purpose type keeps this crate's only
//! dependency budget spent on nothing at all.

use std::ops::{Add, Mul, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Complex32 {
    pub re: f32,
    pub im: f32,
}

impl Complex32 {
    pub const ZERO: Complex32 = Complex32 { re: 0.0, im: 0.0 };

    pub fn new(re: f32, im: f32) -> Self {
        Complex32 { re, im }
    }

    pub fn magnitude(self) -> f32 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
}

impl Add for Complex32 {
    type Output = Complex32;
    fn add(self, rhs: Complex32) -> Complex32 {
        Complex32::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl Sub for Complex32 {
    type Output = Complex32;
    fn sub(self, rhs: Complex32) -> Complex32 {
        Complex32::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl Mul for Complex32 {
    type Output = Complex32;
    fn mul(self, rhs: Complex32) -> Complex32 {
        Complex32::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn magnitude_of_a_3_4_5_triangle() {
        assert_eq!(Complex32::new(3.0, 4.0).magnitude(), 5.0);
    }

    #[test]
    fn multiplication_matches_the_standard_formula() {
        let a = Complex32::new(1.0, 2.0);
        let b = Complex32::new(3.0, 4.0);
        // (1+2i)(3+4i) = 3+4i+6i+8i^2 = 3+10i-8 = -5+10i
        let product = a * b;
        assert_eq!(product, Complex32::new(-5.0, 10.0));
    }

    #[test]
    fn add_and_sub_are_componentwise() {
        let a = Complex32::new(1.0, 2.0);
        let b = Complex32::new(3.0, 4.0);
        assert_eq!(a + b, Complex32::new(4.0, 6.0));
        assert_eq!(a - b, Complex32::new(-2.0, -2.0));
    }
}
