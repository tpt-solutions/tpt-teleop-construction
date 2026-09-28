// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! egui-based visualizer binary: 3D terrain cut/fill heatmaps, machine
//! trajectories and payload weights, proximity detection zones and object
//! classifications, hydraulic pressures and valve positions, vibration
//! frequency spectra (spec.txt §8).
//!
//! This binary drives a live [`tpt_t_construction_sim::SimVehicle`] and a
//! synthetic terrain/proximity/hydraulic/vibration scene so every view has
//! real (if illustrative) data to render, end to end from the physics in
//! `tpt-t-construction-sim` through to the pixels here — rather than
//! hard-coded placeholder numbers.

mod heatmap_color;
mod proximity_view;
mod spectrum;
mod trajectory;

use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints, Points};
use tpt_t_construction_sim::hydraulics::{Cylinder, SpoolValve};
use tpt_t_construction_sim::lidar::Heightmap;
use tpt_t_construction_sim::scenario::{ActorKind, ProximityScenario, ScriptedActor, Waypoint};
use tpt_t_construction_sim::vehicle::SimVehicle;
use tpt_t_construction_sim::Vec3;

use heatmap_color::cut_fill_color;
use proximity_view::ZoneRadii;
use spectrum::naive_dft_magnitude;
use trajectory::{TrajectoryBuffer, TrajectorySample};

const SIM_DT_S: f32 = 0.02;
const TERRAIN_GRID: usize = 24;
const TERRAIN_CELL_M: f32 = 1.5;
const VIBRATION_WINDOW: usize = 128;
/// A relief valve's set point, ~350 bar (spec.txt's "exceed 300 bar"): the
/// demo cylinder below has no piston motion or return-to-tank path to
/// naturally bleed off pressure, so without a relief cap it would climb
/// without bound over a long-running session instead of settling into the
/// realistic range a real circuit's relief valve would enforce.
const RELIEF_PRESSURE_PA: f32 = 3.5e7;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Terrain,
    Trajectory,
    Proximity,
    Hydraulic,
    Vibration,
}

struct VizApp {
    tab: Tab,
    sim_time_s: f32,

    vehicle: SimVehicle,
    trajectory: TrajectoryBuffer,

    heightmap: Heightmap,
    design_height_m: f32,

    scenario: ProximityScenario,
    zone_radii: ZoneRadii,

    valve: SpoolValve,
    cylinder: Cylinder,

    vibration_signal: Vec<f32>,
}

impl Default for VizApp {
    fn default() -> Self {
        let mut heightmap = Heightmap::flat(TERRAIN_GRID, TERRAIN_GRID, TERRAIN_CELL_M, 0.0);
        // A shallow berm and a dug trench, so the cut/fill view has both
        // colors to show instead of a uniformly flat/on-grade field.
        for row in 0..TERRAIN_GRID {
            for col in 0..TERRAIN_GRID {
                let bump = if (6..10).contains(&col) { 0.6 } else { 0.0 };
                let trench = if (14..18).contains(&row) { -0.8 } else { 0.0 };
                heightmap.set_height(col, row, bump + trench);
            }
        }

        let scenario = ProximityScenario::new(vec![
            ScriptedActor::new(
                1,
                ActorKind::Human,
                vec![
                    Waypoint {
                        time_s: 0.0,
                        position: Vec3::new(-20.0, 6.0, 0.0),
                    },
                    Waypoint {
                        time_s: 30.0,
                        position: Vec3::new(20.0, -4.0, 0.0),
                    },
                ],
            ),
            ScriptedActor::new(
                2,
                ActorKind::LightVehicle,
                vec![Waypoint {
                    time_s: 0.0,
                    position: Vec3::new(15.0, 15.0, 0.0),
                }],
            ),
        ]);

        VizApp {
            tab: Tab::Terrain,
            sim_time_s: 0.0,
            vehicle: SimVehicle::reference_haul_truck(),
            trajectory: TrajectoryBuffer::new(2000),
            heightmap,
            design_height_m: 0.0,
            scenario,
            zone_radii: ZoneRadii {
                warning_radius_m: 20.0,
                slowdown_radius_m: 10.0,
                stop_radius_m: 4.0,
            },
            valve: SpoolValve::new(0.2),
            cylinder: Cylinder {
                bore_area_m2: 0.02,
                rod_area_m2: 0.01,
                pressure_a_pa: 1.0e5,
                pressure_b_pa: 1.0e5,
                volume_a_m3: 5e-3,
                volume_b_m3: 5e-3,
                bulk_modulus_pa: 1.6e9,
            },
            vibration_signal: vec![0.0; VIBRATION_WINDOW],
        }
    }
}

/// Cumulative straight-line distance between consecutive trajectory
/// samples, i.e. an approximation of distance actually traveled over the
/// recorded window.
fn path_length_m(trajectory: &TrajectoryBuffer) -> f32 {
    trajectory
        .iter()
        .zip(trajectory.iter().skip(1))
        .map(|(a, b)| ((b.x_m - a.x_m).powi(2) + (b.y_m - a.y_m).powi(2)).sqrt())
        .sum()
}

impl VizApp {
    fn step_simulation(&mut self) {
        self.sim_time_s += SIM_DT_S;

        // Gentle throttle oscillation so the trajectory view has a path to
        // draw instead of a straight line off to infinity.
        let throttle = 0.5 + 0.5 * (self.sim_time_s * 0.15).sin();
        self.vehicle.step(throttle.clamp(0.0, 1.0), SIM_DT_S);
        self.trajectory.push(TrajectorySample {
            x_m: self.vehicle.body.position.x,
            y_m: self.vehicle.body.position.y,
            payload_kg: 40_000.0,
        });

        // A slowly alternating valve command, standing in for an
        // operator's joystick input, feeding the cylinder's flow.
        let commanded = (self.sim_time_s * 0.5).sin();
        let valve_position = self.valve.step(commanded, SIM_DT_S);
        let flow_a = valve_position.max(0.0) * 4e-4;
        let flow_b = (-valve_position).max(0.0) * 4e-4;
        self.cylinder.step(flow_a, flow_b, 0.0, SIM_DT_S);
        self.cylinder.pressure_a_pa = self.cylinder.pressure_a_pa.min(RELIEF_PRESSURE_PA);
        self.cylinder.pressure_b_pa = self.cylinder.pressure_b_pa.min(RELIEF_PRESSURE_PA);

        // Two superposed tones (diesel/pump-noise-like) plus the valve's
        // own motion, as a synthetic stand-in for an accelerometer trace.
        let sample = (self.sim_time_s * 2.0 * std::f32::consts::PI * 6.0).sin() * 0.6
            + (self.sim_time_s * 2.0 * std::f32::consts::PI * 17.0).sin() * 0.3
            + valve_position * 0.2;
        self.vibration_signal.remove(0);
        self.vibration_signal.push(sample);
    }

    fn draw_terrain(&self, ui: &mut egui::Ui) {
        ui.label("Cut/fill heatmap: red = cut needed, blue = fill needed, white = on grade.");
        let available = ui.available_size();
        let cell_px = (available.x / TERRAIN_GRID as f32).min(available.y / TERRAIN_GRID as f32);
        let (response, painter) = ui.allocate_painter(
            egui::vec2(cell_px * TERRAIN_GRID as f32, cell_px * TERRAIN_GRID as f32),
            egui::Sense::hover(),
        );
        let origin = response.rect.min;
        for row in 0..TERRAIN_GRID {
            for col in 0..TERRAIN_GRID {
                let half = TERRAIN_GRID as f32 * TERRAIN_CELL_M * 0.5;
                let x = col as f32 * TERRAIN_CELL_M - half + TERRAIN_CELL_M * 0.5;
                let y = row as f32 * TERRAIN_CELL_M - half + TERRAIN_CELL_M * 0.5;
                let height = self.heightmap.height_at(x, y);
                let delta = height - self.design_height_m;
                let [r, g, b] = cut_fill_color(delta, 1.0);
                let rect = egui::Rect::from_min_size(
                    origin + egui::vec2(col as f32 * cell_px, row as f32 * cell_px),
                    egui::vec2(cell_px, cell_px),
                );
                painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(r, g, b));
            }
        }
    }

    fn draw_trajectory(&self, ui: &mut egui::Ui) {
        if self.trajectory.is_empty() {
            ui.label("No trajectory samples yet.");
            return;
        }
        if let Some(latest) = self.trajectory.latest() {
            ui.label(format!(
                "Position: ({:.1}, {:.1}) m   Speed: {:.1} m/s   Payload: {:.0} kg   Samples: {}   Path length: {:.1} m",
                latest.x_m,
                latest.y_m,
                self.vehicle.ground_speed_m_s(),
                latest.payload_kg,
                self.trajectory.len(),
                path_length_m(&self.trajectory),
            ));
        }
        Plot::new("trajectory_plot")
            .view_aspect(1.6)
            .show(ui, |plot_ui| {
                plot_ui.line(Line::new(PlotPoints::from(self.trajectory.path_points())));
                if let Some(latest) = self.trajectory.latest() {
                    plot_ui.points(
                        Points::new(PlotPoints::from(vec![[
                            latest.x_m as f64,
                            latest.y_m as f64,
                        ]]))
                        .radius(5.0),
                    );
                }
            });
    }

    fn draw_proximity(&self, ui: &mut egui::Ui) {
        let observer = Vec3::new(
            self.vehicle.body.position.x,
            self.vehicle.body.position.y,
            0.0,
        );
        for (id, kind, position) in self.scenario.actors_at(self.sim_time_s) {
            let distance = (position - observer).length();
            let zone = self.zone_radii.classify(distance);
            let [r, g, b] = zone.color();
            ui.horizontal(|ui| {
                let (rect, _) =
                    ui.allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
                ui.painter()
                    .rect_filled(rect, 2.0, egui::Color32::from_rgb(r, g, b));
                ui.label(format!(
                    "actor #{id} ({kind:?}): {distance:.1} m -> {zone:?}"
                ));
            });
        }
    }

    fn draw_hydraulic(&self, ui: &mut egui::Ui) {
        ui.label(format!(
            "Valve position: {:+.2} (-1 = full B, +1 = full A)",
            self.valve.position
        ));
        ui.add(egui::ProgressBar::new((self.valve.position + 1.0) / 2.0).text("valve"));

        ui.label(format!(
            "Chamber A: {:.1} bar",
            self.cylinder.pressure_a_pa / 1e5
        ));
        ui.add(egui::ProgressBar::new(
            (self.cylinder.pressure_a_pa / RELIEF_PRESSURE_PA).clamp(0.0, 1.0),
        ));
        ui.label(format!(
            "Chamber B: {:.1} bar",
            self.cylinder.pressure_b_pa / 1e5
        ));
        ui.add(egui::ProgressBar::new(
            (self.cylinder.pressure_b_pa / RELIEF_PRESSURE_PA).clamp(0.0, 1.0),
        ));
        ui.label(format!("Cylinder force: {:.0} N", self.cylinder.force_n()));
    }

    fn draw_vibration(&self, ui: &mut egui::Ui) {
        ui.label("Synthetic accelerometer trace (top) and its magnitude spectrum (bottom).");
        Plot::new("vibration_time_domain")
            .height(150.0)
            .show(ui, |plot_ui| {
                let points: PlotPoints = self
                    .vibration_signal
                    .iter()
                    .enumerate()
                    .map(|(i, &v)| [i as f64, v as f64])
                    .collect();
                plot_ui.line(Line::new(points));
            });

        let spectrum = naive_dft_magnitude(&self.vibration_signal);
        Plot::new("vibration_spectrum")
            .height(150.0)
            .show(ui, |plot_ui| {
                let points: PlotPoints = spectrum
                    .iter()
                    .enumerate()
                    .map(|(bin, &mag)| [bin as f64, mag as f64])
                    .collect();
                plot_ui.line(Line::new(points));
            });
    }
}

impl eframe::App for VizApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.step_simulation();
        ctx.request_repaint();

        egui::SidePanel::left("nav").show(ctx, |ui| {
            ui.heading("tpt-t-construction-viz");
            ui.separator();
            ui.selectable_value(&mut self.tab, Tab::Terrain, "Terrain");
            ui.selectable_value(&mut self.tab, Tab::Trajectory, "Trajectory & payload");
            ui.selectable_value(&mut self.tab, Tab::Proximity, "Proximity zones");
            ui.selectable_value(&mut self.tab, Tab::Hydraulic, "Hydraulic pressures");
            ui.selectable_value(&mut self.tab, Tab::Vibration, "Vibration spectra");
        });

        egui::CentralPanel::default().show(ctx, |ui| match self.tab {
            Tab::Terrain => self.draw_terrain(ui),
            Tab::Trajectory => self.draw_trajectory(ui),
            Tab::Proximity => self.draw_proximity(ui),
            Tab::Hydraulic => self.draw_hydraulic(ui),
            Tab::Vibration => self.draw_vibration(ui),
        });
    }
}

fn main() -> eframe::Result<()> {
    eframe::run_native(
        "tpt-t-construction-viz",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(VizApp::default()))),
    )
}
