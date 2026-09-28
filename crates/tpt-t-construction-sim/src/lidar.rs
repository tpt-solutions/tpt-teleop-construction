// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Synthetic LiDAR/radar returns from a terrain heightmap and spherical
//! obstacles, per spec.txt §8's "LiDAR and radar returns from synthetic
//! terrain and obstacles". Rays are found by marching (simple, and exact
//! enough at the sim's sub-decimeter step size) against the heightmap,
//! combined with an exact analytic ray/sphere test against obstacles so
//! the two never disagree about which is closer.

use crate::math::Vec3;

/// A regular-grid terrain heightmap, `width x depth` cells of `resolution_m`
/// meters each, centered on the origin in the XY plane.
#[derive(Debug, Clone)]
pub struct Heightmap {
    width: usize,
    depth: usize,
    resolution_m: f32,
    heights_m: Vec<f32>,
}

impl Heightmap {
    pub fn flat(width: usize, depth: usize, resolution_m: f32, height_m: f32) -> Self {
        Heightmap {
            width,
            depth,
            resolution_m,
            heights_m: vec![height_m; width * depth],
        }
    }

    pub fn set_height(&mut self, col: usize, row: usize, height_m: f32) {
        self.heights_m[row * self.width + col] = height_m;
    }

    /// World-space extent, in meters, this heightmap covers along each axis.
    fn half_extent(&self) -> (f32, f32) {
        (
            self.width as f32 * self.resolution_m * 0.5,
            self.depth as f32 * self.resolution_m * 0.5,
        )
    }

    /// Terrain height (m) at world-space `(x, y)`, bilinearly interpolated
    /// between grid cells and clamped to the nearest edge cell outside the
    /// map's extent (an unbounded flat plane at the map's edge height is a
    /// more useful default for ray marching than a hard boundary).
    pub fn height_at(&self, x: f32, y: f32) -> f32 {
        let (half_w, half_d) = self.half_extent();
        let gx = ((x + half_w) / self.resolution_m).clamp(0.0, (self.width - 1) as f32);
        let gy = ((y + half_d) / self.resolution_m).clamp(0.0, (self.depth - 1) as f32);

        let x0 = gx.floor() as usize;
        let y0 = gy.floor() as usize;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.depth - 1);
        let tx = gx - x0 as f32;
        let ty = gy - y0 as f32;

        let h00 = self.heights_m[y0 * self.width + x0];
        let h10 = self.heights_m[y0 * self.width + x1];
        let h01 = self.heights_m[y1 * self.width + x0];
        let h11 = self.heights_m[y1 * self.width + x1];

        let h0 = h00 + (h10 - h00) * tx;
        let h1 = h01 + (h11 - h01) * tx;
        h0 + (h1 - h0) * ty
    }
}

/// A spherical stand-in for a discrete obstacle (a boulder, another
/// machine's rough bounding volume, a virtual human's torso for proximity
/// scenarios).
#[derive(Debug, Clone, Copy)]
pub struct Obstacle {
    pub center: Vec3,
    pub radius_m: f32,
}

impl Obstacle {
    /// Exact analytic ray/sphere intersection distance along `direction`
    /// (assumed normalized) from `origin`, if any, within `max_range_m`.
    fn ray_hit_distance(&self, origin: Vec3, direction: Vec3, max_range_m: f32) -> Option<f32> {
        let oc = origin - self.center;
        let b = oc.dot(direction);
        let c = oc.dot(oc) - self.radius_m * self.radius_m;
        let discriminant = b * b - c;
        if discriminant < 0.0 {
            return None;
        }
        let sqrt_disc = discriminant.sqrt();
        let t0 = -b - sqrt_disc;
        let t1 = -b + sqrt_disc;
        let t = if t0 >= 0.0 { t0 } else { t1 };
        if t >= 0.0 && t <= max_range_m {
            Some(t)
        } else {
            None
        }
    }
}

/// A single LiDAR/radar unit at a fixed world position.
#[derive(Debug, Clone, Copy)]
pub struct Scanner {
    pub origin: Vec3,
    pub max_range_m: f32,
    /// Marching step size along each ray, meters. Smaller is more accurate
    /// against the heightmap at the cost of more steps per ray.
    pub step_m: f32,
}

impl Scanner {
    /// Casts one ray in `direction` (need not be pre-normalized) against
    /// both the heightmap and every obstacle, returning the distance (m)
    /// to whichever is hit first, or `None` if nothing is within range.
    pub fn cast_ray(
        &self,
        heightmap: &Heightmap,
        obstacles: &[Obstacle],
        direction: Vec3,
    ) -> Option<f32> {
        let direction = direction.normalized();

        let obstacle_hit = obstacles
            .iter()
            .filter_map(|o| o.ray_hit_distance(self.origin, direction, self.max_range_m))
            .fold(f32::INFINITY, f32::min);

        let mut t = 0.0_f32;
        let mut ground_hit = f32::INFINITY;
        while t <= self.max_range_m && t < obstacle_hit {
            let p = self.origin + direction * t;
            let ground_z = heightmap.height_at(p.x, p.y);
            if p.z <= ground_z {
                ground_hit = t;
                break;
            }
            t += self.step_m;
        }

        let closest = obstacle_hit.min(ground_hit);
        if closest.is_finite() {
            Some(closest)
        } else {
            None
        }
    }

    /// A horizontal 360-degree (or partial) sweep of `num_rays` evenly
    /// spaced beams starting at `start_rad`, returning each ray's range
    /// (`None` where nothing was in range) in sweep order.
    pub fn scan_horizontal_sweep(
        &self,
        heightmap: &Heightmap,
        obstacles: &[Obstacle],
        num_rays: usize,
        start_rad: f32,
        sweep_rad: f32,
    ) -> Vec<Option<f32>> {
        (0..num_rays)
            .map(|i| {
                let frac = if num_rays > 1 {
                    i as f32 / (num_rays - 1) as f32
                } else {
                    0.0
                };
                let angle = start_rad + sweep_rad * frac;
                let direction = Vec3::new(angle.cos(), angle.sin(), 0.0);
                self.cast_ray(heightmap, obstacles, direction)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    #[test]
    fn flat_ground_hit_matches_height_difference() {
        let heightmap = Heightmap::flat(10, 10, 1.0, 0.0);
        let scanner = Scanner {
            origin: Vec3::new(0.0, 0.0, 5.0),
            max_range_m: 20.0,
            step_m: 0.01,
        };
        let hit = scanner
            .cast_ray(&heightmap, &[], Vec3::new(0.0, 0.0, -1.0))
            .unwrap();
        assert!((hit - 5.0).abs() < 0.05);
    }

    #[test]
    fn out_of_range_ray_returns_none() {
        let heightmap = Heightmap::flat(10, 10, 1.0, -1000.0); // ground far below
        let scanner = Scanner {
            origin: Vec3::new(0.0, 0.0, 0.0),
            max_range_m: 5.0,
            step_m: 0.1,
        };
        assert!(scanner
            .cast_ray(&heightmap, &[], Vec3::new(1.0, 0.0, 0.0))
            .is_none());
    }

    #[test]
    fn obstacle_closer_than_ground_wins() {
        let heightmap = Heightmap::flat(10, 10, 1.0, -1000.0);
        let scanner = Scanner {
            origin: Vec3::ZERO,
            max_range_m: 50.0,
            step_m: 0.1,
        };
        let obstacles = [Obstacle {
            center: Vec3::new(10.0, 0.0, 0.0),
            radius_m: 1.0,
        }];
        let hit = scanner
            .cast_ray(&heightmap, &obstacles, Vec3::new(1.0, 0.0, 0.0))
            .unwrap();
        assert!((hit - 9.0).abs() < 1e-3);
    }

    #[test]
    fn ray_pointed_away_from_obstacle_misses_it() {
        let heightmap = Heightmap::flat(10, 10, 1.0, -1000.0);
        let scanner = Scanner {
            origin: Vec3::ZERO,
            max_range_m: 50.0,
            step_m: 0.1,
        };
        let obstacles = [Obstacle {
            center: Vec3::new(10.0, 0.0, 0.0),
            radius_m: 1.0,
        }];
        assert!(scanner
            .cast_ray(&heightmap, &obstacles, Vec3::new(-1.0, 0.0, 0.0))
            .is_none());
    }

    #[test]
    fn full_sweep_returns_one_reading_per_ray() {
        let heightmap = Heightmap::flat(10, 10, 1.0, -1000.0);
        let scanner = Scanner {
            origin: Vec3::ZERO,
            max_range_m: 50.0,
            step_m: 0.5,
        };
        let obstacles = [Obstacle {
            center: Vec3::new(5.0, 0.0, 0.0),
            radius_m: 0.5,
        }];
        let sweep = scanner.scan_horizontal_sweep(&heightmap, &obstacles, 36, 0.0, TAU);
        assert_eq!(sweep.len(), 36);
        assert!(sweep.iter().any(|r| r.is_some()));
    }

    #[test]
    fn bilinear_height_interpolates_between_grid_points() {
        let mut heightmap = Heightmap::flat(4, 4, 1.0, 0.0);
        heightmap.set_height(2, 2, 2.0);
        let corner = heightmap.height_at(-2.0 + 2.0, -2.0 + 2.0);
        assert!((corner - 2.0).abs() < 1e-4);
    }
}
