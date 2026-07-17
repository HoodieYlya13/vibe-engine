//! Tests for the vehicle impact-damage model (PRD §7.1 "cosmetic-plus"):
//! chassis health drops on hard contacts and scales with impact speed, clean
//! driving costs nothing, zero health kills the drivetrain (with the
//! explosion pop), and a reset repairs the wreck.

use sim::SimEngine;
use sim::constants::{BTN_FORWARD, BTN_RIGHT, CAR_HEALTH_OFFSET};

const HZ: usize = 60;

struct Harness {
    engine: SimEngine,
    buf: Vec<f32>,
}

impl Harness {
    /// Open-field engine (arena walls only), already seated in the car.
    fn new() -> Self {
        let mut engine = SimEngine::new_open_field(0);
        engine.set_vehicle_class(0);
        let len = engine.state_len();
        let mut h = Harness {
            engine,
            buf: vec![0.0; len],
        };
        h.step_seconds(1.0);
        h.engine.toggle_enter();
        h
    }

    fn step_seconds(&mut self, seconds: f32) {
        for _ in 0..((seconds * HZ as f32) as usize) {
            self.engine.step();
        }
    }

    fn hold(&mut self, buttons: u32, seconds: f32) {
        self.engine.set_input_buttons(buttons, 0.0);
        self.step_seconds(seconds);
    }

    fn pos(&mut self) -> (f32, f32, f32) {
        self.engine.write_state(&mut self.buf);
        (self.buf[0], self.buf[1], self.buf[2])
    }

    /// Health as the client sees it — read from the snapshot, so these tests
    /// also pin the `CAR_HEALTH_OFFSET` wiring.
    fn health(&mut self) -> f32 {
        self.engine.write_state(&mut self.buf);
        self.buf[CAR_HEALTH_OFFSET as usize]
    }
}

/// Full-throttle run into the +Z arena wall (inner face at z = 60) with
/// `run_up` meters of acceleration; returns the health after the dust settles.
fn wall_crash_health(run_up: f32) -> f32 {
    let mut h = Harness::new();
    h.engine.teleport_car(0.0, 1.5, 58.0 - run_up);
    h.hold(BTN_FORWARD, 6.0);
    h.health()
}

#[test]
fn harder_crashes_hurt_more() {
    let gentle = wall_crash_health(10.0);
    let hard = wall_crash_health(50.0);
    assert!(
        gentle < 0.995,
        "a wall bump left the car pristine: health {gentle}"
    );
    assert!(
        gentle > 0.5,
        "a slow bump nearly wrecked the car: health {gentle}"
    );
    assert!(
        hard < gentle - 0.15,
        "damage should scale with impact speed: gentle {gentle} vs hard {hard}"
    );
    assert!(
        hard > 0.0,
        "a single crash should never total the car outright: health {hard}"
    );
}

#[test]
fn clean_driving_keeps_the_car_pristine() {
    // Hard acceleration, a full-lock turn, and engine braking to a stop —
    // none of it touches the chassis, so none of it should cost health.
    // Start against the -Z wall; keep the run short enough that the wide
    // lateral-g-capped arc at speed stays inside the arena walls.
    let mut h = Harness::new();
    h.engine.teleport_car(0.0, 1.5, -50.0);
    h.hold(BTN_FORWARD, 1.5);
    h.hold(BTN_FORWARD | BTN_RIGHT, 1.5);
    h.hold(0, 1.5);
    let health = h.health();
    assert!(
        health > 0.999,
        "clean driving damaged the car: health {health}"
    );
}

#[test]
fn wrecked_car_loses_its_drivetrain() {
    let mut h = Harness::new();
    let start_y = h.pos().1;
    h.engine.damage_car(1.0);
    assert_eq!(h.health(), 0.0);

    // The destruction pop should visibly toss the wreck upward.
    let mut max_y = start_y;
    for _ in 0..HZ {
        h.engine.step();
        max_y = max_y.max(h.pos().1);
    }
    assert!(
        max_y > start_y + 0.4,
        "no explosion pop: start y {start_y}, max y {max_y}"
    );

    // With the drivetrain dead, throttle must not move the wreck.
    h.step_seconds(2.0);
    let before = h.pos();
    h.hold(BTN_FORWARD, 2.0);
    let after = h.pos();
    let moved = ((after.0 - before.0).powi(2) + (after.2 - before.2).powi(2)).sqrt();
    assert!(
        moved < 1.0,
        "destroyed car still drives: moved {moved} m under throttle"
    );
}

#[test]
fn reset_repairs_the_wreck() {
    let mut h = Harness::new();
    h.engine.damage_car(0.4);
    let partial = h.health();
    assert!(
        (partial - 0.6).abs() < 1e-6,
        "partial damage should just subtract: health {partial}"
    );

    h.engine.damage_car(1.0);
    h.step_seconds(2.0);
    h.engine.reset_car();
    assert!(
        (h.health() - 1.0).abs() < 1e-6,
        "reset should repair the car"
    );

    // And the repaired car drives again.
    h.engine.toggle_enter();
    let before = h.pos();
    h.hold(BTN_FORWARD, 2.0);
    let after = h.pos();
    let moved = ((after.0 - before.0).powi(2) + (after.2 - before.2).powi(2)).sqrt();
    assert!(
        moved > 5.0,
        "repaired car does not drive: moved {moved} m under throttle"
    );
}
