//! Tests for the arcade handling assists (PRD Phase 2): counter-phase rear
//! steering, roll-only anti-flip righting, reverse steer scaling, airborne
//! throttle/steer control, and the handbrake rear-grip cut. Each test compares
//! an assist against its disabled variant so it proves the assist itself, not
//! just stable defaults.

use sim::SimEngine;
use sim::constants::{BTN_BACK, BTN_FORWARD, BTN_HANDBRAKE, BTN_RIGHT};

const HZ: usize = 60;

/// Indices into the `vehicle_tuning()` vector (= `set_vehicle_tuning` order).
const ENGINE_FORCE: usize = 0;
const REAR_STEER: usize = 7;
const ANTI_ROLL: usize = 8;
const REVERSE_STABILITY: usize = 9;
const AIR_CONTROL: usize = 10;
const HANDBRAKE_GRIP: usize = 11;

struct Harness {
    engine: SimEngine,
    buf: Vec<f32>,
}

impl Harness {
    /// Open-field engine with the default (sedan) tuning, after letting
    /// `tweak` override individual values, already seated in the car.
    fn new(tweak: impl FnOnce(&mut [f32])) -> Self {
        let mut engine = SimEngine::new_open_field(0);
        let mut v = engine.vehicle_tuning();
        tweak(&mut v);
        engine.set_vehicle_tuning(
            v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11],
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

    fn pos_xz(&mut self) -> (f32, f32) {
        self.engine.write_state(&mut self.buf);
        (self.buf[0], self.buf[2])
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

/// The sedan's default value for one tuning slot (for restoring after zeroing).
fn default_tuning(index: usize) -> f32 {
    SimEngine::new_open_field(0).vehicle_tuning()[index]
}

#[test]
fn rear_steer_tightens_the_turn() {
    let turn = |rear_steer: f32| {
        let mut h = Harness::new(|v| v[REAR_STEER] = rear_steer);
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
    // forward driving. The reverse steer scaling should cut a full-lock
    // reverse turn down substantially without touching forward handling.
    let reverse_turn = |reverse_stability: f32| {
        let mut h = Harness::new(|v| v[REVERSE_STABILITY] = reverse_stability);
        h.hold(BTN_BACK, 2.0);
        h.turned_while_holding(BTN_BACK | BTN_RIGHT, 1.0).abs()
    };

    let twitchy = reverse_turn(0.0);
    let damped = reverse_turn(default_tuning(REVERSE_STABILITY));
    assert!(
        damped < twitchy * 0.75,
        "reverse steer scaling should calm reverse steering: \
         unassisted {twitchy} rad vs assisted {damped} rad"
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
    let mut h = Harness::new(|v| {
        v[ENGINE_FORCE] = 14000.0;
        v[REAR_STEER] = 1.0;
        v[ANTI_ROLL] = 0.0;
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
        let mut h = Harness::new(|v| v[ANTI_ROLL] = anti_roll);
        h.engine.flip_car();
        h.step_seconds(4.0);
        h.up_y()
    };

    let assisted = up_after_flip(default_tuning(ANTI_ROLL));
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
fn handbrake_grip_cut_swings_the_tail() {
    // Build speed, then yank the handbrake while steering. Heading change is
    // dominated by the front wheels either way; the drift shows up as slip —
    // the nose pointing into the turn (positive, tail-out) while the velocity
    // keeps going wide. With full grip the handbrake only plows (slip ≤ 0).
    let max_tail_out = |handbrake_grip: f32| {
        let mut h = Harness::new(|v| v[HANDBRAKE_GRIP] = handbrake_grip);
        h.hold(BTN_FORWARD, 2.0);
        h.engine
            .set_input_buttons(BTN_FORWARD | BTN_RIGHT | BTN_HANDBRAKE, 0.0);
        let mut prev = h.pos_xz();
        let mut max_slip = 0.0f32;
        for _ in 0..((1.2 * HZ as f32) as usize) {
            h.engine.step();
            let pos = h.pos_xz();
            let (dx, dz) = (pos.0 - prev.0, pos.1 - prev.1);
            prev = pos;
            if dx * dx + dz * dz < 1e-6 {
                continue;
            }
            let mut slip = h.yaw() - dx.atan2(dz);
            if slip > std::f32::consts::PI {
                slip -= std::f32::consts::TAU;
            } else if slip < -std::f32::consts::PI {
                slip += std::f32::consts::TAU;
            }
            max_slip = max_slip.max(slip);
        }
        max_slip
    };

    let gripped = max_tail_out(1.0);
    let loose = max_tail_out(default_tuning(HANDBRAKE_GRIP));
    assert!(
        loose > 0.25,
        "drift grip failed to swing the tail out: max slip {loose} rad"
    );
    assert!(
        gripped < 0.15,
        "control run invalid — tail swung out even at full grip: {gripped} rad"
    );
}

#[test]
fn air_control_pitches_the_airborne_car() {
    // Drop the car from high up and hold throttle: with air control the nose
    // should dip; with it off the car keeps falling flat.
    let pitch_after_fall = |air_control: f32| {
        let mut h = Harness::new(|v| v[AIR_CONTROL] = air_control);
        h.engine.teleport_car(0.0, 14.0, 0.0);
        h.hold(BTN_FORWARD, 1.0);
        h.forward_y()
    };

    let with_control = pitch_after_fall(default_tuning(AIR_CONTROL));
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
