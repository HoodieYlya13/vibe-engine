//! Tests for how the car answers the steering: calm on a key tap, bounded by
//! tire grip at speed, straight when released, and quick to swing around only
//! in a handbrake slide.

use sim::SimEngine;
use sim::constants::{BTN_FORWARD, BTN_HANDBRAKE, BTN_RIGHT};

const HZ: f32 = 60.0;
const CLASS_COUNT: u32 = 3;
const SPORTS: u32 = 1;
const GRAVITY: f32 = 9.81;
const ROLLING_SPEED: f32 = 1.0;

struct Motion {
    /// Heading change over the run (rad, positive to the right).
    turned: f32,
    /// Speed at the end of the run (m/s).
    speed: f32,
    /// Turn rate at the end of the run (rad/s).
    turn_rate: f32,
    /// Highest sideways acceleration the turn demanded (m/s²).
    peak_cornering: f32,
    /// Smallest radius the car turned in while it was rolling (m).
    tightest_radius: f32,
}

struct Drive {
    engine: SimEngine,
    state: Vec<f32>,
}

impl Drive {
    /// Seated in `class` on flat open ground, after `seconds` of full throttle.
    fn launched(class: u32, seconds: f32) -> Self {
        let mut engine = SimEngine::new_city(0);
        engine.set_vehicle_class(class);
        let state = vec![0.0; engine.state_len()];
        let mut drive = Drive { engine, state };
        drive.hold(0, 1.0);
        drive.engine.toggle_enter();
        drive.hold(BTN_FORWARD, seconds);
        drive
    }

    fn heading(&mut self) -> f32 {
        self.engine.write_state(&mut self.state);
        let (x, y, z, w) = (self.state[3], self.state[4], self.state[5], self.state[6]);
        (2.0 * (x * z + w * y)).atan2(1.0 - 2.0 * (x * x + y * y))
    }

    fn position(&mut self) -> (f32, f32) {
        self.engine.write_state(&mut self.state);
        (self.state[0], self.state[2])
    }

    fn hold(&mut self, buttons: u32, seconds: f32) -> Motion {
        self.engine.set_input_buttons(buttons, 0.0);
        let mut motion = Motion {
            turned: 0.0,
            speed: 0.0,
            turn_rate: 0.0,
            peak_cornering: 0.0,
            tightest_radius: f32::INFINITY,
        };
        let mut heading = self.heading();
        let mut position = self.position();
        for _ in 0..((seconds * HZ) as usize) {
            self.engine.step();
            let turn = wrapped(self.heading() - heading);
            let moved = self.position();
            motion.speed = (moved.0 - position.0).hypot(moved.1 - position.1) * HZ;
            motion.turn_rate = turn * HZ;
            motion.turned += turn;
            motion.peak_cornering = motion
                .peak_cornering
                .max(motion.turn_rate.abs() * motion.speed);
            if motion.speed > ROLLING_SPEED && motion.turn_rate.abs() > 1e-3 {
                motion.tightest_radius = motion
                    .tightest_radius
                    .min(motion.speed / motion.turn_rate.abs());
            }
            heading += turn;
            position = moved;
        }
        motion
    }
}

fn wrapped(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

#[test]
fn a_tap_on_the_steering_only_nudges_the_car() {
    for class in 0..CLASS_COUNT {
        let mut drive = Drive::launched(class, 2.0);
        let tap = drive.hold(BTN_FORWARD | BTN_RIGHT, 0.1);
        let settle = drive.hold(BTN_FORWARD, 0.4);
        let turned = (tap.turned + settle.turned).to_degrees();
        assert!(
            turned > 0.05 && turned < 2.5,
            "class {class}: a 0.1 s tap at {:.0} m/s turned the car {turned:.1} degrees",
            tap.speed
        );
    }
}

#[test]
fn cornering_never_asks_more_than_the_tires_hold() {
    for class in 0..CLASS_COUNT {
        for launch in [0.5, 1.0, 2.0, 4.0, 7.0] {
            let mut drive = Drive::launched(class, launch);
            let corner = drive.hold(BTN_FORWARD | BTN_RIGHT, 1.5);
            assert!(
                corner.peak_cornering < 1.6 * GRAVITY,
                "class {class}: full lock after {launch} s of throttle pulled {:.1} g",
                corner.peak_cornering / GRAVITY
            );
        }
    }
}

#[test]
fn steering_calms_down_as_speed_builds() {
    for class in 0..CLASS_COUNT {
        let in_town = Drive::launched(class, 0.5).hold(BTN_FORWARD | BTN_RIGHT, 0.5);
        let flat_out = Drive::launched(class, 15.0).hold(BTN_FORWARD | BTN_RIGHT, 0.5);
        assert!(
            flat_out.turn_rate < 0.45 && flat_out.turn_rate < in_town.turn_rate * 0.6,
            "class {class}: full lock turns {:.2} rad/s at {:.0} m/s against {:.2} rad/s at {:.0} m/s",
            flat_out.turn_rate,
            flat_out.speed,
            in_town.turn_rate,
            in_town.speed
        );
    }
}

#[test]
fn full_lock_keeps_a_real_turning_circle() {
    for class in 0..CLASS_COUNT {
        let pulling_away = Drive::launched(class, 0.0).hold(BTN_FORWARD | BTN_RIGHT, 1.5);
        assert!(
            pulling_away.tightest_radius > 5.0 && pulling_away.tightest_radius < 12.0,
            "class {class}: full lock from rest turns in a {:.1} m radius",
            pulling_away.tightest_radius
        );
    }
}

#[test]
fn releasing_the_steering_straightens_the_car() {
    for class in 0..CLASS_COUNT {
        let mut drive = Drive::launched(class, 2.0);
        drive.hold(BTN_FORWARD | BTN_RIGHT, 1.0);
        let released = drive.hold(BTN_FORWARD, 0.3);
        assert!(
            released.turn_rate.abs() < 0.03,
            "class {class}: still turning {:.2} rad/s 0.3 s after letting go",
            released.turn_rate
        );
    }
}

#[test]
fn a_handbrake_slide_swings_the_car_around() {
    let steered = Drive::launched(SPORTS, 4.0).hold(BTN_RIGHT, 1.0);
    let slid = Drive::launched(SPORTS, 4.0).hold(BTN_RIGHT | BTN_HANDBRAKE, 1.0);
    assert!(
        slid.turned > 1.0 && slid.turned > steered.turned * 2.0,
        "the handbrake turned the car {:.2} rad in a second against {:.2} rad without it",
        slid.turned,
        steered.turned
    );
    assert!(
        slid.turned < 2.4 && slid.speed > 6.0,
        "the slide spun out: {:.2} rad in a second, {:.1} m/s left",
        slid.turned,
        slid.speed
    );
}

#[test]
fn the_handbrake_alone_does_not_turn_the_car() {
    let slid = Drive::launched(SPORTS, 4.0).hold(BTN_HANDBRAKE, 1.0);
    assert!(
        slid.turned.abs() < 0.05,
        "the car turned {:.2} rad under the handbrake with the steering centered",
        slid.turned
    );
}
