// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Cut/fill heatmap color mapping: pure `[u8; 3]` RGB, independent of any
//! GUI toolkit, so it's unit-testable without pulling `egui` into the test
//! build and reusable if the rendering backend ever changes.
//!
//! A diverging blue (fill needed, too low) - white (on grade) - red (cut
//! needed, too high) scale, the conventional choice for a signed quantity
//! centered on a meaningful zero (spec.txt §8's "3D terrain with cut/fill
//! heatmaps").

/// Maps a cut/fill delta (design surface minus actual surface, meters) to
/// an RGB color. `delta_m > 0` means material needs to be cut (actual
/// surface too high); `delta_m < 0` means fill is needed (too low).
/// `max_delta_m` (must be positive) sets the saturation point of the
/// scale: deltas at or beyond `+-max_delta_m` render at full color.
pub fn cut_fill_color(delta_m: f32, max_delta_m: f32) -> [u8; 3] {
    debug_assert!(max_delta_m > 0.0, "max_delta_m must be positive");
    let t = (delta_m / max_delta_m).clamp(-1.0, 1.0);

    // White at t=0, blending toward blue (fill, t<0) or red (cut, t>0).
    let blue = [0x21, 0x6a, 0xcc];
    let white = [0xf2, 0xf2, 0xf2];
    let red = [0xcc, 0x33, 0x2a];

    let (from, to, frac) = if t < 0.0 {
        (white, blue, -t)
    } else {
        (white, red, t)
    };
    lerp_rgb(from, to, frac)
}

fn lerp_rgb(from: [u8; 3], to: [u8; 3], t: f32) -> [u8; 3] {
    let t = t.clamp(0.0, 1.0);
    std::array::from_fn(|i| {
        let a = from[i] as f32;
        let b = to[i] as f32;
        (a + (b - a) * t).round() as u8
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn on_grade_is_white() {
        assert_eq!(cut_fill_color(0.0, 1.0), [0xf2, 0xf2, 0xf2]);
    }

    #[test]
    fn saturates_at_and_beyond_max_delta() {
        let at_max = cut_fill_color(1.0, 1.0);
        let beyond_max = cut_fill_color(5.0, 1.0);
        assert_eq!(at_max, beyond_max);
    }

    #[test]
    fn cut_and_fill_are_different_hues() {
        let cut = cut_fill_color(0.5, 1.0);
        let fill = cut_fill_color(-0.5, 1.0);
        assert_ne!(cut, fill);
    }

    #[test]
    fn magnitude_increases_color_saturation() {
        let small_cut = cut_fill_color(0.1, 1.0);
        let big_cut = cut_fill_color(0.9, 1.0);
        // Bigger cut should be further from white (red channel dominates,
        // green/blue channels drop further from white's 0xf2).
        assert!(big_cut[1] < small_cut[1]);
    }
}
