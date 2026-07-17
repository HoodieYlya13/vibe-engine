//! High-speed stability regressions (playtest 2026-07-18): in the city world
//! there are no arena walls, so the car reaches true top speed (~70 m/s) —
//! where full-lock cornering used to creep into a wheelie, and a sudden
//! direction flick nearly rolled the car (46° in the browser repro). The
//! guards here are the whole stability stack working together: speed-capped
//! slewed steering, the yaw/grip assist, anti-wheelie traction control, and
//! the righting springs.

use sim::SimEngine;
use sim::constants::{BTN_FORWARD, BTN_LEFT, BTN_RIGHT};

const HZ: usize = 60;

fn quat(buf: &[f32]) -> (f32, f32, f32, f32) {
    (buf[3], buf[4], buf[5], buf[6])
}

/// Y of the chassis forward axis: positive = nose up.
fn fwd_y(buf: &[f32]) -> f32 {
    let (x, y, z, w) = quat(buf);
    2.0 * (y * z - w * x)
}

/// Y of the chassis up axis: 1 = level, 0 = on its side.
fn up_y(buf: &[f32]) -> f32 {
    let (x, _, z, _) = quat(buf);
    1.0 - 2.0 * (x * x + z * z)
}

/// City-world engine (flat ground, no walls), seated, at rest at the origin.
fn setup(class: u32) -> (SimEngine, Vec<f32>) {
    let mut engine = SimEngine::new_city(0);
    engine.set_vehicle_class(class);
    let buf = vec![0.0; engine.state_len()];
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    (engine, buf)
}

/// Worst attitude over `seconds` while holding `buttons`.
fn worst_attitude(
    engine: &mut SimEngine,
    buf: &mut [f32],
    buttons: u32,
    seconds: f32,
) -> (f32, f32) {
    engine.set_input_buttons(buttons, 0.0);
    let (mut max_lift, mut min_up) = (0.0f32, 1.0f32);
    for _ in 0..((seconds * HZ as f32) as usize) {
        engine.step();
        engine.write_state(buf);
        max_lift = max_lift.max(fwd_y(buf));
        min_up = min_up.min(up_y(buf));
    }
    (max_lift, min_up)
}

#[test]
fn top_speed_cornering_stays_level() {
    for class in 0..3u32 {
        let (mut engine, mut buf) = setup(class);
        // 6 s flat out reaches every class's terminal speed.
        engine.set_input_buttons(BTN_FORWARD, 0.0);
        for _ in 0..(6 * HZ) {
            engine.step();
        }
        let (max_lift, min_up) =
            worst_attitude(&mut engine, &mut buf, BTN_FORWARD | BTN_RIGHT, 3.0);
        assert!(
            max_lift < 0.25,
            "class {class} wheelied cornering at top speed: fwd_y {max_lift}"
        );
        assert!(
            min_up > 0.85,
            "class {class} leaned past ~30° cornering at top speed: up_y {min_up}"
        );
    }
}

#[test]
fn top_speed_flick_reversal_stays_level() {
    for class in 0..3u32 {
        let (mut engine, mut buf) = setup(class);
        engine.set_input_buttons(BTN_FORWARD, 0.0);
        for _ in 0..(5 * HZ) {
            engine.step();
        }
        engine.set_input_buttons(BTN_FORWARD | BTN_RIGHT, 0.0);
        for _ in 0..(3 * HZ / 2) {
            engine.step();
        }
        let (max_lift, min_up) = worst_attitude(&mut engine, &mut buf, BTN_FORWARD | BTN_LEFT, 2.0);
        assert!(
            max_lift < 0.25,
            "class {class} wheelied on a top-speed flick reversal: fwd_y {max_lift}"
        );
        assert!(
            min_up > 0.85,
            "class {class} nearly rolled on a top-speed flick reversal: up_y {min_up}"
        );
    }
}
