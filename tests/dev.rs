//! Tests for the dev/test-mode surface: live tuning, teleports, wireframes.

use sim::SimEngine;
use sim::constants::{
    BTN_FORWARD, PLAYER_STATE_OFFSET, VEHICLE_DIRECTION_SWITCH_SPEED, VEHICLE_ENGINE_BRAKE_IMPULSE,
    VEHICLE_FOOTBRAKE_IMPULSE, VEHICLE_HANDBRAKE_IMPULSE, VEHICLE_MAX_REVERSE_FORCE,
    VEHICLE_MAX_STEER,
};

const HZ: usize = 60;

fn drive_forward_distance(engine_force: Option<f32>) -> f32 {
    let mut engine = SimEngine::new_open_field(0);
    let mut buf = vec![0.0; engine.state_len()];

    if let Some(force) = engine_force {
        engine.set_vehicle_tuning(
            force,
            VEHICLE_MAX_REVERSE_FORCE,
            VEHICLE_FOOTBRAKE_IMPULSE,
            VEHICLE_ENGINE_BRAKE_IMPULSE,
            VEHICLE_HANDBRAKE_IMPULSE,
            VEHICLE_DIRECTION_SWITCH_SPEED,
            VEHICLE_MAX_STEER,
        );
    }

    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    engine.set_input_buttons(BTN_FORWARD, 0.0);
    for _ in 0..(2 * HZ) {
        engine.step();
    }
    engine.write_state(&mut buf);
    buf[2]
}

#[test]
fn live_tuning_changes_acceleration() {
    let stock = drive_forward_distance(None);
    let weak = drive_forward_distance(Some(1000.0));
    assert!(
        stock - weak > 5.0,
        "1000N engine should be much slower than stock: stock {stock} m vs weak {weak} m"
    );
}

#[test]
fn teleport_player_moves_player() {
    let mut engine = SimEngine::new_open_field(0);
    let mut buf = vec![0.0; engine.state_len()];
    for _ in 0..HZ {
        engine.step();
    }

    engine.teleport_player(-20.0, 1.0, 30.0);
    for _ in 0..HZ {
        engine.step();
    }

    engine.write_state(&mut buf);
    let off = PLAYER_STATE_OFFSET as usize;
    let (x, y, z) = (buf[off], buf[off + 1], buf[off + 2]);
    assert!(
        (x - -20.0).abs() < 0.5 && (z - 30.0).abs() < 0.5,
        "player not at teleport target: ({x}, {y}, {z})"
    );
    assert!(
        (0.5..1.2).contains(&y),
        "player should settle on the ground after teleport: y = {y}"
    );
}

#[test]
fn teleport_car_moves_car() {
    let mut engine = SimEngine::new_open_field(0);
    let mut buf = vec![0.0; engine.state_len()];
    for _ in 0..HZ {
        engine.step();
    }

    engine.teleport_car(25.0, 1.5, -25.0);
    for _ in 0..HZ {
        engine.step();
    }

    engine.write_state(&mut buf);
    let (x, y, z) = (buf[0], buf[1], buf[2]);
    assert!(
        (x - 25.0).abs() < 0.5 && (z - -25.0).abs() < 0.5,
        "car not at teleport target: ({x}, {y}, {z})"
    );
    assert!(
        (0.0..3.0).contains(&y),
        "car should settle on its wheels after teleport: y = {y}"
    );
}

#[test]
fn debug_render_emits_line_list() {
    let mut engine = SimEngine::new_with_peds(0);
    for _ in 0..HZ {
        engine.step();
    }

    let lines = engine.debug_render_lines();
    assert!(
        !lines.is_empty(),
        "debug render produced no lines for the greybox world"
    );
    assert_eq!(
        lines.len() % 6,
        0,
        "line list must be flat [x0,y0,z0,x1,y1,z1] pairs: len = {}",
        lines.len()
    );
    assert!(
        lines.iter().all(|v| v.is_finite()),
        "debug render contains non-finite vertices"
    );
}
