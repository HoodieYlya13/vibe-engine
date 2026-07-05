//! Tests for the arcade handling assists (PRD Phase 2): counter-phase rear
//! steering, roll-only anti-flip righting, reverse yaw damping, and airborne
//! throttle/steer control. Each test compares an assist against its disabled
//! (zeroed) variant so it proves the assist itself, not just stable defaults.

use sim::SimEngine;
use sim::constants::{
    BTN_BACK, BTN_FORWARD, BTN_RIGHT, VEHICLE_AIR_CONTROL, VEHICLE_ANTI_ROLL,
    VEHICLE_DIRECTION_SWITCH_SPEED, VEHICLE_ENGINE_BRAKE_IMPULSE, VEHICLE_FOOTBRAKE_IMPULSE,
    VEHICLE_HANDBRAKE_IMPULSE, VEHICLE_MAX_ENGINE_FORCE, VEHICLE_MAX_REVERSE_FORCE,
    VEHICLE_MAX_STEER, VEHICLE_REAR_STEER, VEHICLE_REVERSE_STABILITY,
};

const HZ: usize = 60;

struct Harness {
    engine: SimEngine,
    buf: Vec<f32>,
}

/// Overrides for the assist-related tuning values; everything else stays stock.
struct Assists {
    engine_force: f32,
    rear_steer: f32,
    anti_roll: f32,
    reverse_stability: f32,
    air_control: f32,
}

impl Default for Assists {
    fn default() -> Self {
        Assists {
            engine_force: VEHICLE_MAX_ENGINE_FORCE,
            rear_steer: VEHICLE_REAR_STEER,
            anti_roll: VEHICLE_ANTI_ROLL,
            reverse_stability: VEHICLE_REVERSE_STABILITY,
            air_control: VEHICLE_AIR_CONTROL,
        }
    }
}

impl Harness {
    fn new(assists: &Assists) -> Self {
        let mut engine = SimEngine::new_open_field(0);
        engine.set_vehicle_tuning(
            assists.engine_force,
            VEHICLE_MAX_REVERSE_FORCE,
            VEHICLE_FOOTBRAKE_IMPULSE,
            VEHICLE_ENGINE_BRAKE_IMPULSE,
            VEHICLE_HANDBRAKE_IMPULSE,
            VEHICLE_DIRECTION_SWITCH_SPEED,
            VEHICLE_MAX_STEER,
            assists.rear_steer,
            assists.anti_roll,
            assists.reverse_stability,
            assists.air_control,
        );
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

    fn quat(&mut self) -> (f32, f32, f32, f32) {
        self.engine.write_state(&mut self.buf);
        (self.buf[3], self.buf[4], self.buf[5], self.buf[6])
    }

    /// Y component of the chassis up axis: 1 = level, 0 = on its side.
    fn up_y(&mut self) -> f32 {
        let (x, _, z, _) = self.quat();
        1.0 - 2.0 * (x * x + z * z)
    }

    /// Y component of the chassis forward axis: negative = nose down.
    fn forward_y(&mut self) -> f32 {
        let (x, y, z, w) = self.quat();
        2.0 * (y * z - w * x)
    }

    fn yaw(&mut self) -> f32 {
        let (x, y, z, w) = self.quat();
        let fx = 2.0 * (x * z + w * y);
        let fz = 1.0 - 2.0 * (x * x + y * y);
        fx.atan2(fz)
    }

    /// Accumulated (wrap-safe) heading change while holding `buttons`.
    fn turned_while_holding(&mut self, buttons: u32, seconds: f32) -> f32 {
        self.engine.set_input_buttons(buttons, 0.0);
        let mut total = 0.0;
        let mut last = self.yaw();
        for _ in 0..((seconds * HZ as f32) as usize) {
            self.engine.step();
            let yaw = self.yaw();
            let mut delta = yaw - last;
            if delta > std::f32::consts::PI {
                delta -= std::f32::consts::TAU;
            } else if delta < -std::f32::consts::PI {
                delta += std::f32::consts::TAU;
            }
            total += delta;
            last = yaw;
        }
        total
    }
}

#[test]
fn rear_steer_tightens_the_turn() {
    let turn = |rear_steer: f32| {
        let mut h = Harness::new(&Assists {
            rear_steer,
            ..Assists::default()
        });
        h.hold(BTN_FORWARD, 1.0);
        h.turned_while_holding(BTN_FORWARD | BTN_RIGHT, 1.5).abs()
    };

    let front_only = turn(0.0);
    let with_rear = turn(0.8);
    assert!(
        with_rear > front_only + 0.15,
        "counter-phase rear steer should turn the car faster: \
         front-only {front_only} rad vs rear-steer {with_rear} rad"
    );
}

#[test]
fn reverse_stability_calms_reverse_steering() {
    // Unassisted reverse steering yaws ~1.5 rad/s — far twitchier than
    // forward driving. The yaw damping should cut a full-lock reverse turn
    // down substantially without touching forward handling.
    let reverse_turn = |reverse_stability: f32| {
        let mut h = Harness::new(&Assists {
            reverse_stability,
            ..Assists::default()
        });
        h.hold(BTN_BACK, 2.0);
        h.turned_while_holding(BTN_BACK | BTN_RIGHT, 1.0).abs()
    };

    let twitchy = reverse_turn(0.0);
    let damped = reverse_turn(VEHICLE_REVERSE_STABILITY);
    assert!(
        damped < twitchy * 0.75,
        "reverse yaw damping should calm reverse steering: \
         undamped {twitchy} rad vs damped {damped} rad"
    );
    assert!(
        damped > 0.2,
        "reverse steering authority damped away entirely: {damped} rad"
    );
}

#[test]
fn hard_cornering_stays_on_four_wheels() {
    // Overpowered engine + full rear steer + a hard flick is the flip recipe;
    // the lowered center of mass alone must keep this nearly flat.
    let mut h = Harness::new(&Assists {
        engine_force: 14000.0,
        rear_steer: 1.0,
        anti_roll: 0.0,
        ..Assists::default()
    });
    h.hold(BTN_FORWARD, 2.5);
    h.engine.set_input_buttons(BTN_FORWARD | BTN_RIGHT, 0.0);
    let mut min = h.up_y();
    for _ in 0..(2 * HZ) {
        h.engine.step();
        min = min.min(h.up_y());
    }
    assert!(
        min > 0.9,
        "car leaned past ~25 degrees in a flat-out flick: min up_y = {min}"
    );
}

#[test]
fn anti_roll_rights_a_flipped_car() {
    let up_after_flip = |anti_roll: f32| {
        let mut h = Harness::new(&Assists {
            anti_roll,
            ..Assists::default()
        });
        h.engine.flip_car();
        h.step_seconds(4.0);
        h.up_y()
    };

    let assisted = up_after_flip(VEHICLE_ANTI_ROLL);
    let unassisted = up_after_flip(0.0);
    assert!(
        assisted > 0.9,
        "anti-roll failed to self-right the flipped car: up_y = {assisted}"
    );
    assert!(
        unassisted < 0.3,
        "control run invalid — car righted itself without the assist: up_y = {unassisted}"
    );
}

#[test]
fn air_control_pitches_the_airborne_car() {
    // Drop the car from high up and hold throttle: with air control the nose
    // should dip; with it off the car keeps falling flat.
    let pitch_after_fall = |air_control: f32| {
        let mut h = Harness::new(&Assists {
            air_control,
            ..Assists::default()
        });
        h.engine.teleport_car(0.0, 14.0, 0.0);
        h.hold(BTN_FORWARD, 1.0);
        h.forward_y()
    };

    let with_control = pitch_after_fall(VEHICLE_AIR_CONTROL);
    let without = pitch_after_fall(0.0);
    assert!(
        with_control < -0.1,
        "throttle in the air should pitch the nose down: forward_y = {with_control}"
    );
    assert!(
        without.abs() < 0.05,
        "car pitched with air control disabled: forward_y = {without}"
    );
}
