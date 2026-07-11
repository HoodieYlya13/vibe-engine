//! Tests for chunk collider streaming (Phase 3): loading/unloading is
//! balanced, streamed colliders are solid, unloading actually frees the
//! space, and streaming stays deterministic.

use sim::SimEngine;
use sim::constants::{BTN_FORWARD, PLAYER_STATE_OFFSET};

const HZ: usize = 60;

/// A wall across the road ahead of the car spawn: chunk-local box rows of
/// (x, y, z, hx, hy, hz) for chunk (0, 0), so world coords match local.
const WALL_AHEAD: &[f32] = &[0.0, 3.0, 20.0, 10.0, 3.0, 1.0];

fn step_seconds(engine: &mut SimEngine, seconds: f32) {
    for _ in 0..((seconds * HZ as f32) as usize) {
        engine.step();
    }
}

fn car_z(engine: &mut SimEngine, buf: &mut Vec<f32>) -> f32 {
    buf.resize(engine.state_len(), 0.0);
    engine.write_state(buf);
    buf[2]
}

#[test]
fn load_and_unload_balance() {
    let mut engine = SimEngine::new_city(0);
    assert_eq!(engine.loaded_chunk_count(), 0);

    engine.load_chunk(0, 0, WALL_AHEAD);
    engine.load_chunk(1, 0, &[0.0, 1.0, 0.0, 5.0, 1.0, 5.0]);
    assert_eq!(engine.loaded_chunk_count(), 2);

    // Re-loading the same chunk replaces it, not duplicates it.
    engine.load_chunk(0, 0, WALL_AHEAD);
    assert_eq!(engine.loaded_chunk_count(), 2);

    engine.unload_chunk(0, 0);
    engine.unload_chunk(1, 0);
    // Unloading something never loaded is a no-op.
    engine.unload_chunk(9, 9);
    assert_eq!(engine.loaded_chunk_count(), 0);

    // The world still steps normally afterwards.
    step_seconds(&mut engine, 1.0);
}

#[test]
fn streamed_wall_blocks_the_car() {
    let mut engine = SimEngine::new_city(0);
    engine.load_chunk(0, 0, WALL_AHEAD);
    engine.toggle_enter();
    engine.set_input_buttons(BTN_FORWARD, 0.0);
    step_seconds(&mut engine, 4.0);

    let mut buf = Vec::new();
    let z = car_z(&mut engine, &mut buf);
    assert!(z > 5.0, "car never moved: z {z}");
    assert!(
        z < 20.0,
        "car drove through the streamed wall at z 20: z {z}"
    );
}

#[test]
fn unloading_frees_the_path() {
    let mut engine = SimEngine::new_city(0);
    engine.load_chunk(0, 0, WALL_AHEAD);
    engine.unload_chunk(0, 0);
    engine.toggle_enter();
    engine.set_input_buttons(BTN_FORWARD, 0.0);
    step_seconds(&mut engine, 4.0);

    let mut buf = Vec::new();
    let z = car_z(&mut engine, &mut buf);
    assert!(z > 25.0, "unloaded wall still blocks the road: car z {z}");
}

#[test]
fn player_walks_into_a_streamed_box() {
    let mut engine = SimEngine::new_city(0);
    // A cube right in the player's +Z walking path (player spawns at x 2).
    engine.load_chunk(0, 0, &[2.0, 2.0, 6.0, 2.0, 2.0, 2.0]);
    engine.set_input_buttons(BTN_FORWARD, 0.0);
    step_seconds(&mut engine, 3.0);

    let mut buf = Vec::new();
    buf.resize(engine.state_len(), 0.0);
    engine.write_state(&mut buf);
    let pz = buf[PLAYER_STATE_OFFSET as usize + 2];
    assert!(pz > 1.0, "player never moved: z {pz}");
    assert!(
        pz < 4.0,
        "player walked through the streamed box at z 4..8: z {pz}"
    );
}

#[test]
fn streaming_is_deterministic() {
    let run = || {
        let mut engine = SimEngine::new_city(0);
        engine.load_chunk(0, 0, WALL_AHEAD);
        engine.load_chunk(0, 1, &[0.0, 1.0, -30.0, 3.0, 1.0, 3.0]);
        engine.toggle_enter();
        engine.set_input_buttons(BTN_FORWARD, 0.0);
        step_seconds(&mut engine, 2.0);
        engine.unload_chunk(0, 1);
        step_seconds(&mut engine, 2.0);
        let mut buf = vec![0.0; engine.state_len()];
        engine.write_state(&mut buf);
        (buf[0].to_bits(), buf[1].to_bits(), buf[2].to_bits())
    };
    assert_eq!(run(), run(), "same loads + inputs must replay bitwise");
}
