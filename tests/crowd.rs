//! Crowd v2 (Phase 4): city peds converge onto the sidewalks derived from
//! the streamed lanes, walk them, flee from the speeding player car, and
//! respawn into the density bubble when the player leaves them behind. The
//! arena keeps the bounded wander. All through the public wasm API, so these
//! also pin the snapshot ped block (matrix translation).

use sim::SimEngine;
use sim::constants::{BTN_FORWARD, PED_STATE_OFFSET, PED_STATE_STRIDE, WORLD_HALF};

const HZ: usize = 60;

/// Ped positions as (x, y, z) from the snapshot matrices.
fn ped_positions(engine: &SimEngine, buf: &mut Vec<f32>) -> Vec<(f32, f32, f32)> {
    buf.resize(engine.state_len(), 0.0);
    engine.write_state(buf);
    (0..engine.ped_count() as usize)
        .map(|i| {
            let c = PED_STATE_OFFSET as usize + i * PED_STATE_STRIDE as usize;
            (buf[c + 12], buf[c + 13], buf[c + 14])
        })
        .collect()
}

/// Load a two-way street through chunk (cx, cz), running along Z at
/// world x = cx·100 ± 2.6, split into two segments with a 14 m
/// intersection-style gap at the chunk center (like the kit exporter's
/// 7 m lane insets). Derived sidewalks land at cx·100 ± 6.75.
fn load_twin_street(engine: &mut SimEngine, cx: i32, cz: i32) {
    engine.load_chunk(cx, cz, &[]);
    let mut points = Vec::new();
    let mut counts = Vec::new();
    for (x, z0, z1) in [
        (2.6f32, -43.0f32, -7.0f32),
        (2.6, 7.0, 43.0),
        (-2.6, -7.0, -43.0),
        (-2.6, 43.0, 7.0),
    ] {
        points.extend_from_slice(&[x, 0.25, z0, x, 0.25, z1]);
        counts.push(2);
    }
    engine.load_chunk_lanes(cx, cz, &points, &counts);
}

/// Count of peds standing on one of the two sidewalk lines |x - center| =
/// 6.75 (± the per-ped lateral jitter and catch-up slack).
fn on_sidewalk(positions: &[(f32, f32, f32)], center_x: f32) -> usize {
    positions
        .iter()
        .filter(|(x, _, _)| ((x - center_x).abs() - 6.75).abs() < 1.0)
        .count()
}

#[test]
fn city_peds_converge_onto_the_sidewalks() {
    let mut engine = SimEngine::new_city(60);
    for cz in -1..=1 {
        load_twin_street(&mut engine, 0, cz);
    }
    let mut buf = Vec::new();

    let before = on_sidewalk(&ped_positions(&engine, &mut buf), 0.0);
    for _ in 0..(60 * HZ) {
        engine.step();
    }
    let positions = ped_positions(&engine, &mut buf);
    let after = on_sidewalk(&positions, 0.0);
    assert!(
        after >= 15 && after > before + 5,
        "peds did not converge onto the sidewalk lines: {before} -> {after} of 60"
    );

    // The sidewalk crowd is walking the street, not standing on it.
    let walkers: Vec<usize> = (0..positions.len())
        .filter(|&i| ((positions[i].0.abs()) - 6.75).abs() < 1.0)
        .collect();
    for _ in 0..(2 * HZ) {
        engine.step();
    }
    let later = ped_positions(&engine, &mut buf);
    let moved = walkers
        .iter()
        .filter(|&&i| {
            let (x0, _, z0) = positions[i];
            let (x1, _, z1) = later[i];
            ((x1 - x0).powi(2) + (z1 - z0).powi(2)).sqrt() > 1.2
        })
        .count();
    assert!(
        moved * 2 >= walkers.len(),
        "sidewalk peds are mostly standing still: {moved} of {} moved",
        walkers.len()
    );
}

#[test]
fn peds_flee_from_the_speeding_car() {
    let mut engine = SimEngine::new_city(400);
    let mut buf = Vec::new();
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    for _ in 0..HZ {
        engine.step();
    }

    // Peds in the corridor the car is about to plow through (it drives +Z
    // from the origin).
    let before = ped_positions(&engine, &mut buf);
    let corridor: Vec<usize> = (0..before.len())
        .filter(|&i| {
            let (x, _, z) = before[i];
            x.abs() < 8.0 && (10.0..140.0).contains(&z)
        })
        .collect();
    assert!(
        corridor.len() >= 3,
        "test needs peds in the car's path, got {}",
        corridor.len()
    );

    engine.set_input_buttons(BTN_FORWARD, 0.0);
    for _ in 0..(8 * HZ) {
        engine.step();
    }
    let after = ped_positions(&engine, &mut buf);

    let mean = |set: &[(f32, f32, f32)], idx: &[usize]| {
        idx.iter().map(|&i| set[i].0.abs()).sum::<f32>() / idx.len() as f32
    };
    let before_lat = mean(&before, &corridor);
    let after_lat = mean(&after, &corridor);
    assert!(
        after_lat > before_lat + 2.0,
        "peds in the corridor did not scatter: mean |x| {before_lat} -> {after_lat}"
    );
}

#[test]
fn far_peds_respawn_into_the_bubble() {
    let mut engine = SimEngine::new_city(30);
    for cz in -1..=1 {
        load_twin_street(&mut engine, 5, cz);
    }
    let mut buf = Vec::new();
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    engine.teleport_car(500.0, 1.0, 0.0);
    for _ in 0..(20 * HZ) {
        engine.step();
    }

    let positions = ped_positions(&engine, &mut buf);
    for (i, (x, _, z)) in positions.iter().enumerate() {
        let dist = ((x - 500.0).powi(2) + z.powi(2)).sqrt();
        assert!(
            dist < 200.0,
            "ped {i} was left outside the bubble: {dist} m from the focus"
        );
    }
    let relocated = on_sidewalk(&positions, 500.0);
    assert!(
        relocated >= 15,
        "respawned peds should land on the sidewalks: {relocated} of 30"
    );
}

#[test]
fn arena_peds_stay_inside_the_walls() {
    let mut engine = SimEngine::new_with_peds(200);
    let mut buf = Vec::new();
    for _ in 0..(30 * HZ) {
        engine.step();
    }
    for (i, (x, y, z)) in ped_positions(&engine, &mut buf).iter().enumerate() {
        assert!(
            x.abs() <= WORLD_HALF && z.abs() <= WORLD_HALF,
            "arena ped {i} escaped the walls: ({x}, {z})"
        );
        assert!(
            (0.7..4.5).contains(y),
            "arena ped {i} at a weird height: {y}"
        );
    }
}

#[test]
fn crowd_is_deterministic() {
    let run = || {
        let mut engine = SimEngine::new_city(50);
        for cz in -1..=1 {
            load_twin_street(&mut engine, 0, cz);
        }
        for _ in 0..HZ {
            engine.step();
        }
        engine.toggle_enter();
        engine.set_input_buttons(BTN_FORWARD, 0.0);
        for _ in 0..(6 * HZ) {
            engine.step();
        }
        let mut buf = vec![0.0; engine.state_len()];
        engine.write_state(&mut buf);
        buf
    };
    let a = run();
    let b = run();
    assert_eq!(a.len(), b.len());
    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        assert!(
            x.to_bits() == y.to_bits(),
            "state diverged at float {i}: {x} vs {y}"
        );
    }
}

#[test]
fn city_peds_stand_on_raised_sidewalks() {
    // A 15 cm sidewalk slab under the +X sidewalk line of a twin street:
    // peds walking it must ride at slab height, peds elsewhere at road level.
    const CURB: f32 = 0.15;
    let mut engine = SimEngine::new_city(60);
    for cz in -1..=1 {
        load_twin_street(&mut engine, 0, cz);
    }
    let slab = [6.75f32, CURB / 2.0, 0.0, 0.75, CURB / 2.0, 150.0];
    engine.load_chunk(
        1,
        0,
        &[slab[0] - 100.0, slab[1], slab[2], slab[3], slab[4], slab[5]],
    );

    let mut buf = Vec::new();
    for _ in 0..(40 * HZ) {
        engine.step();
    }
    let positions = ped_positions(&engine, &mut buf);
    let road_level = positions.iter().map(|p| p.1).fold(f32::MAX, f32::min);
    let on_slab: Vec<f32> = positions
        .iter()
        .filter(|(x, _, z)| (x - 6.75).abs() < 0.5 && z.abs() < 140.0)
        .map(|p| p.1)
        .collect();
    assert!(!on_slab.is_empty(), "no ped walked the raised sidewalk");
    for y in &on_slab {
        assert!(
            (y - road_level - CURB).abs() < 0.02,
            "ped on the slab stands at {y}, expected {}",
            road_level + CURB
        );
    }
    let off_slab = positions
        .iter()
        .filter(|(x, _, _)| (x + 6.75).abs() < 0.5)
        .count();
    assert!(off_slab > 0, "no ped on the road-level sidewalk to compare");
}
