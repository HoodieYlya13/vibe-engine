//! Tests for the sim-owned day/night clock (Phase 3): advances with sim
//! steps, wraps, can be pinned, and halts while paused.

use sim::SimEngine;
use sim::constants::{DAY_CYCLE_SECONDS, DAY_START_FRACTION, ENV_STATE_OFFSET};

const HZ: usize = 60;
const DT: f32 = 1.0 / 60.0;

fn time_of_day(engine: &mut SimEngine) -> f32 {
    let mut buf = vec![0.0; engine.state_len()];
    engine.write_state(&mut buf);
    buf[ENV_STATE_OFFSET as usize]
}

#[test]
fn clock_starts_at_the_boot_hour_and_advances() {
    let mut engine = SimEngine::new_open_field(0);
    assert_eq!(time_of_day(&mut engine), DAY_START_FRACTION);

    for _ in 0..(10 * HZ) {
        engine.step();
    }
    let expected = DAY_START_FRACTION + 10.0 * DT * HZ as f32 / DAY_CYCLE_SECONDS;
    let got = time_of_day(&mut engine);
    assert!(
        (got - expected).abs() < 1e-4,
        "10 s should advance the clock by 10/{DAY_CYCLE_SECONDS} of a day: {got} vs {expected}"
    );
}

#[test]
fn clock_wraps_past_midnight() {
    let mut engine = SimEngine::new_open_field(0);
    engine.set_time_of_day(0.999);
    for _ in 0..(5 * HZ) {
        engine.step();
    }
    let got = time_of_day(&mut engine);
    assert!(
        (0.0..0.01).contains(&got),
        "clock should wrap 0.999 → ~0.002, got {got}"
    );
}

#[test]
fn set_time_pins_and_normalizes() {
    let mut engine = SimEngine::new_open_field(0);
    engine.set_time_of_day(0.75);
    assert_eq!(time_of_day(&mut engine), 0.75);
    // Out-of-range values normalize into 0..1 instead of breaking the sky.
    engine.set_time_of_day(2.25);
    assert_eq!(time_of_day(&mut engine), 0.25);
    engine.set_time_of_day(-0.25);
    assert_eq!(time_of_day(&mut engine), 0.75);
}

#[test]
fn pause_halts_the_clock() {
    let mut engine = SimEngine::new_open_field(0);
    engine.toggle_pause();
    let before = time_of_day(&mut engine);
    for _ in 0..(5 * HZ) {
        engine.step();
    }
    assert_eq!(
        time_of_day(&mut engine),
        before,
        "a paused world must not advance the day"
    );
}
