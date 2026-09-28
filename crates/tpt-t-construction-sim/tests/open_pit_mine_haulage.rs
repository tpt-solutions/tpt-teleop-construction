// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! End-to-end "Open-Pit Mine Haulage" scenario test (spec.txt §6, §9):
//! walks the full lifecycle spec.txt's data-flow narrative describes —
//! shift start, self-test, dispatch, the haul out (with a detected
//! hazard forcing a slowdown), loading, the return haul, dumping, a
//! maintenance vibration alert, and shift end — driving the actual crates
//! built for each phase rather than re-describing the flow in prose.
//! This lives in `tpt-t-construction-sim`'s integration tests (all of the
//! other crates are dev-dependencies only) because a full-shift scenario
//! is exactly the kind of test spec.txt §8 says the simulator exists to
//! make practical to run at all.

use std::path::Path;
use std::time::Duration;

use tpt_t_construction_core::{
    new_bus, CoordinationLoop, Fault, FaultSeverity, MachineController, MachineEvent, MachineState,
    SelfTestSuite,
};
use tpt_t_construction_excavator::{estimate_cycle_time, SwingCycle, SwingPhase};
use tpt_t_construction_haul::{
    combined_speed_limit_m_s, downgrade_speed_limit_m_s, estimated_wait_s, DumpQueue, HaulRoad,
    Point2, RoadSegment, ShiftTonnage, SpeedLimitParams,
};
use tpt_t_construction_payload::{LoadCell, PayloadScale, SettlingDetector};
use tpt_t_construction_powertrain::arbitrate_torque_nm;
use tpt_t_construction_rollover::should_apply_brakes;
use tpt_t_construction_safety::{
    classify_object, effective_zone, fuse_detections, speed_command_fraction, Detection,
    HazardZone, ObjectClass, ObjectSignature, PresenceEscalation, SensorKind, ZoneRadii,
};
use tpt_t_construction_telemetry::{
    export_csv, read_all_records, summarize_shift, TelemetryLogger, TelemetryRecord,
};
use tpt_t_construction_wear::{
    ball_pass_frequency_outer_hz, detect_fault_signature, BearingGeometry, FaultSignature,
    FftProcessor,
};

const CYCLES_PER_SHIFT: u32 = 3; // stands in for spec.txt's 15; enough to prove the loop, fast enough to run every time.

#[test]
fn full_shift_simulation() {
    let mut telemetry_path = std::env::temp_dir();
    telemetry_path.push(format!("tpt-t-construction-e2e-{}.bin", std::process::id()));
    let summary = run_shift(&telemetry_path);
    std::fs::remove_file(&telemetry_path).ok();

    assert_eq!(summary.cycles_completed, CYCLES_PER_SHIFT);
    // Two 100-tonne excavator passes per cycle = 200 tonnes/cycle,
    // matching spec.txt §6's "Total payload: 200 tons" example.
    assert!((summary.tonnage.total_tonnes() - 200.0 * CYCLES_PER_SHIFT as f64).abs() < 1.0);
    assert_eq!(summary.maintenance_fault, FaultSignature::OuterRaceDefect);
    assert!(
        !summary.rollover_braked,
        "a flat haul road should never trigger rollover braking"
    );
    assert!(
        summary.min_speed_during_hazard < summary.base_haul_speed_m_s,
        "detecting a hazard should slow the truck"
    );
    assert_eq!(
        summary.telemetry_summary.sample_count,
        summary.telemetry_samples_logged
    );
}

struct ShiftResult {
    cycles_completed: u32,
    tonnage: ShiftTonnage,
    maintenance_fault: FaultSignature,
    rollover_braked: bool,
    base_haul_speed_m_s: f32,
    min_speed_during_hazard: f32,
    telemetry_samples_logged: usize,
    telemetry_summary: tpt_t_construction_telemetry::ShiftSummary,
}

fn run_shift(telemetry_path: &Path) -> ShiftResult {
    let mut telemetry = TelemetryLogger::create(telemetry_path).expect("create telemetry log");
    let mut timestamp_us: u64 = 0;
    let mut samples_logged = 0usize;

    // --- Shift start: self-test must pass before the truck can leave Idle. ---
    let (bus_tx, bus_rx) = new_bus();
    let mut controller = MachineController::new(bus_rx, SelfTestSuite::new());
    let self_test_event = controller.run_self_test();
    assert_eq!(
        self_test_event,
        MachineEvent::SelfTestCompleted { passed: true }
    );
    bus_tx
        .publish(MachineEvent::RequestTransition {
            next: MachineState::Moving,
        })
        .unwrap();
    let applied = controller.tick().unwrap().unwrap();
    assert_eq!(
        applied,
        MachineEvent::StateChanged {
            from: MachineState::Idle,
            to: MachineState::Moving
        }
    );

    // --- Dispatch: the excavator's coordination loop plans around this truck's ETA. ---
    let mut coordination: CoordinationLoop<4> = CoordinationLoop::new();
    let dig_time = Duration::from_secs(8);
    let dump_time = Duration::from_secs(3);
    let cycle_time = estimate_cycle_time(dig_time, dump_time, 1.2, 0.8);
    coordination.report_truck_position(1, 900.0, 11.0, 1.0);
    let (next_truck, adjustment) = coordination.tick(cycle_time.as_secs_f32()).unwrap();
    assert_eq!(next_truck.truck_id, 1);
    assert!(adjustment.recommended_dwell_s >= 0.0);

    // --- The haul out: stay on the road, and slow down for a detected hazard. ---
    let road = HaulRoad {
        segments: vec![RoadSegment {
            start: Point2 { x: 0.0, y: 0.0 },
            end: Point2 { x: 900.0, y: 0.0 },
            width_m: 12.0,
        }],
    };
    let truck_position = Point2 { x: 450.0, y: 1.0 };
    assert!(road.is_within_safe_corridor(truck_position, 1.0));

    let speed_params = SpeedLimitParams {
        max_lateral_accel_m_s2: 2.5,
        reaction_time_s: 1.5,
        max_decel_m_s2: 3.0,
    };
    let base_haul_speed_m_s = combined_speed_limit_m_s(11.0, 0.0, None, None, &speed_params);

    // A light vehicle detected 200m ahead (matches spec.txt §6's example).
    let detections = [
        Detection {
            position: [200.0, 0.5],
            velocity: [-2.0, 0.0],
            confidence: 0.9,
            sensor: SensorKind::Lidar,
        },
        Detection {
            position: [200.2, 0.4],
            velocity: [-2.1, 0.0],
            confidence: 0.7,
            sensor: SensorKind::Radar,
        },
    ];
    let fused = fuse_detections(&detections, 1.0);
    assert_eq!(fused.len(), 1);
    let hazard_distance_m = fused[0].position[0];
    let classification = classify_object(&ObjectSignature {
        width_m: 1.8,
        height_m: 1.6,
        max_speed_observed_m_s: 15.0,
    });
    assert_eq!(classification, ObjectClass::LightVehicle);

    // Radii chosen so a light vehicle ~200m out (spec.txt §6's example)
    // falls in the slowdown band, matching that example's 40 -> 20 km/h halving.
    let zone_radii = ZoneRadii {
        warning_m: 400.0,
        slowdown_m: 250.0,
        stop_m: 10.0,
    };
    let zone = zone_radii.classify(hazard_distance_m);
    assert_eq!(zone, HazardZone::Slowdown);
    let mut escalation = PresenceEscalation::new();
    escalation.update(zone, 1.0);
    let effective = effective_zone(zone, &escalation, 5.0);
    let min_speed_during_hazard = base_haul_speed_m_s * speed_command_fraction(effective);

    timestamp_us += 1_000_000;
    telemetry
        .log(&TelemetryRecord {
            timestamp_us,
            hydraulic_pressure_pa: 1.5e7,
            engine_rpm: 1600.0,
            ground_speed_m_s: min_speed_during_hazard,
            payload_kg: 0.0,
        })
        .unwrap();
    samples_logged += 1;

    // --- Loading, dumping, and the return haul, repeated for the shift. ---
    let mut tonnage = ShiftTonnage::new();
    let mut rollover_braked = false;
    let scale = PayloadScale::new([
        LoadCell {
            tare_kg: 0.0,
            scale_factor_kg_per_count: 50.0,
        },
        LoadCell {
            tare_kg: 0.0,
            scale_factor_kg_per_count: 50.0,
        },
    ]);

    for _cycle in 0..CYCLES_PER_SHIFT {
        // Arrival at the dig face: the truck queues for its turn.
        let wait_s = estimated_wait_s(0, dump_time.as_secs_f32());
        assert_eq!(wait_s, 0.0);

        // Loading: the excavator swings two passes into the truck bed,
        // each weighed by the on-board scale once its reading settles.
        let mut swing_cycle = SwingCycle::new();
        let mut settler: SettlingDetector<4> = SettlingDetector::new();
        for _pass in 0..2 {
            assert_eq!(swing_cycle.phase(), SwingPhase::Digging);
            swing_cycle.advance(); // -> SwingToTruck
            swing_cycle.advance(); // -> Dumping (the bucket drops its load)

            let raw_counts = [1000i32, 1000i32]; // 2 cells * 1000 counts * 50 kg/count = 100,000 kg
            for _ in 0..4 {
                settler.push(scale.total_weight_kg(&raw_counts));
            }
            assert!(settler.is_settled(1.0));
            tonnage.record_load(settler.settled_weight_kg());

            swing_cycle.advance(); // -> SwingToFace
            swing_cycle.advance(); // -> Digging
        }
        assert_eq!(swing_cycle.completed_cycles(), 2);

        // The return haul: torque management on the downgrade, with the
        // rollover monitor confirming the machine stays well within its
        // stability margin the whole way down.
        let descent_speed_limit = downgrade_speed_limit_m_s(base_haul_speed_m_s, -10.0);
        assert!(descent_speed_limit < base_haul_speed_m_s);
        let commanded_torque = arbitrate_torque_nm(1500.0, 2000.0, 1800.0, 1.0);
        assert!(commanded_torque > 0.0);
        if should_apply_brakes(0.02, 3.0, 1.6, 0.05) {
            rollover_braked = true;
        }

        // Dumping at the crusher: the queue coordinates arrival order.
        let mut dump_queue = DumpQueue::new();
        dump_queue.request_slot(1, 0);
        assert_eq!(dump_queue.next_to_dump(), Some(1));

        timestamp_us += 600_000_000; // ~10 minutes per cycle
        telemetry
            .log(&TelemetryRecord {
                timestamp_us,
                hydraulic_pressure_pa: 2.0e7,
                engine_rpm: 1400.0,
                ground_speed_m_s: descent_speed_limit,
                payload_kg: 200_000.0,
            })
            .unwrap();
        samples_logged += 1;
    }
    coordination.complete_load(1);

    // --- Maintenance alert: a vibration signature indicates early bearing wear. ---
    let bearing = BearingGeometry {
        num_elements: 9.0,
        ball_diameter_m: 0.012,
        pitch_diameter_m: 0.06,
        contact_angle_rad: 0.0,
    };
    let shaft_freq_hz = 25.0;
    let sample_rate_hz = 2000.0;
    let bpfo_hz = ball_pass_frequency_outer_hz(shaft_freq_hz, &bearing);
    const N: usize = 64;
    let samples: [f32; N] = std::array::from_fn(|t| {
        let time_s = t as f32 / sample_rate_hz;
        5.0 * (2.0 * std::f32::consts::PI * bpfo_hz * time_s).sin()
            + 0.3 * (2.0 * std::f32::consts::PI * shaft_freq_hz * time_s).sin()
    });
    let fft: FftProcessor<N> = FftProcessor::new();
    let spectrum = fft.magnitude_spectrum(&samples);
    let bin_resolution_hz = sample_rate_hz / N as f32;
    let maintenance_fault = detect_fault_signature(
        &spectrum,
        bin_resolution_hz,
        shaft_freq_hz,
        20.0,
        &bearing,
        3.0,
    );

    if maintenance_fault != FaultSignature::None {
        bus_tx
            .publish(MachineEvent::FaultRaised(Fault {
                code: 42,
                severity: FaultSeverity::Warning,
                message: "right rear wheel motor bearing: early outer-race wear signature detected",
            }))
            .unwrap();
        // A Warning-severity fault doesn't force a state change (only
        // EmergencyStop does), but it's still sitting in the controller's
        // inbox as a pass-through event; drain it now so it doesn't
        // displace one of the shift-end transition requests below (the
        // bus is FIFO and `tick()` only processes one event per call).
        controller.tick().unwrap();
    }

    // --- Shift end: return to Idle, flush telemetry, and summarize. ---
    bus_tx
        .publish(MachineEvent::RequestTransition {
            next: MachineState::Working,
        })
        .unwrap();
    controller.tick().unwrap();
    bus_tx
        .publish(MachineEvent::RequestTransition {
            next: MachineState::Dumping,
        })
        .unwrap();
    controller.tick().unwrap();
    bus_tx
        .publish(MachineEvent::RequestTransition {
            next: MachineState::Returning,
        })
        .unwrap();
    controller.tick().unwrap();
    bus_tx
        .publish(MachineEvent::RequestTransition {
            next: MachineState::Idle,
        })
        .unwrap();
    controller.tick().unwrap();
    assert_eq!(controller.state(), MachineState::Idle);

    telemetry.flush().unwrap();
    let records = read_all_records(telemetry_path).unwrap();
    let telemetry_summary = summarize_shift(&records).unwrap();
    let csv = export_csv(&records);
    assert!(csv.lines().count() > 1);

    ShiftResult {
        cycles_completed: CYCLES_PER_SHIFT,
        tonnage,
        maintenance_fault,
        rollover_braked,
        base_haul_speed_m_s,
        min_speed_during_hazard,
        telemetry_samples_logged: samples_logged,
        telemetry_summary,
    }
}
