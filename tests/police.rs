//! Wanted / heat system (Phase 4, PRD §7.3 ★1–3): crimes raise heat, police
//! cruisers spawn on the streamed lanes and chase the player along the road
//! grid, escaping out of sight decays the heat away, and ★3 drops roadblocks
//! ahead of a fast player. All through the public wasm API; these also pin
//! the snapshot WANTED/POLICE blocks.

use sim::SimEngine;
use sim::constants::{
    BTN_FORWARD, CHUNK_SIZE, MAX_POLICE, POLICE_STATE_OFFSET, POLICE_STATE_STRIDE,
    WANTED_STATE_OFFSET,
};

const HZ: usize = 60;

/// Active police cruisers as (x, y, z) from the snapshot block.
fn police_positions(buf: &[f32]) -> Vec<(f32, f32, f32)> {
    (0..MAX_POLICE as usize)
        .filter_map(|i| {
            let c = POLICE_STATE_OFFSET as usize + i * POLICE_STATE_STRIDE as usize;
            (buf[c] > 0.5).then(|| (buf[c + 1], buf[c + 2], buf[c + 3]))
        })
        .collect()
}

/// Twin one-way streets (x = ±2.6, opposite directions) through chunk
/// (cx, cz), segmented like the kit exporter insets them at intersections —
/// same street layout the crowd tests use.
fn load_twin_street(engine: &mut SimEngine, cx: i32, cz: i32) {
    let up = [2.6f32, 0.25, -43.0, 2.6, 0.25, 43.0];
    let down = [-2.6f32, 0.25, 43.0, -2.6, 0.25, -43.0];
    let points: Vec<f32> = up.iter().chain(down.iter()).copied().collect();
    engine.load_chunk(cx, cz, &[]);
    engine.load_chunk_lanes(cx, cz, &points, &[2, 2]);
}

/// City engine with streets streamed along the z axis around the origin.
fn setup_city(peds: u32) -> (SimEngine, Vec<f32>) {
    let mut engine = SimEngine::new_city(peds);
    for cz in -2..=2 {
        load_twin_street(&mut engine, 0, cz);
    }
    let buf = vec![0.0; engine.state_len()];
    (engine, buf)
}

#[test]
fn running_over_peds_raises_heat_and_spawns_police() {
    let (mut engine, mut buf) = setup_city(800);
    engine.set_vehicle_class(0);
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    for _ in 0..HZ {
        engine.step();
    }
    assert_eq!(engine.wanted_stars(), 0, "wanted before any crime");

    // Plow through the ped scatter, off to the side of the streets so the
    // queueing ambient traffic stays out of the experiment.
    engine.teleport_car(30.0, 1.0, -160.0);
    engine.set_input_buttons(BTN_FORWARD, 0.0);
    for _ in 0..(9 * HZ) {
        engine.step();
    }
    engine.set_input_buttons(0, 0.0);

    assert!(
        engine.wanted_stars() >= 1,
        "driving through the crowd raised no heat"
    );

    // The pursuit spawns onto the streamed lanes within a few seconds.
    for _ in 0..(6 * HZ) {
        engine.step();
    }
    assert!(
        engine.police_count() >= 1,
        "wanted level {} but no cruiser spawned",
        engine.wanted_stars()
    );
    engine.write_state(&mut buf);
    assert!(
        buf[WANTED_STATE_OFFSET as usize] >= 1.0,
        "snapshot wanted block disagrees with the API"
    );
}

#[test]
fn police_pursue_toward_the_player() {
    let (mut engine, mut buf) = setup_city(0);
    engine.set_vehicle_class(0);
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    for _ in 0..HZ {
        engine.step();
    }
    engine.teleport_car(8.0, 1.0, 0.0);
    engine.set_wanted(2);

    let mut ever_spawned = false;
    let mut closest = f32::MAX;
    for _ in 0..(20 * HZ) {
        engine.step();
        engine.write_state(&mut buf);
        for (x, _, z) in police_positions(&buf) {
            ever_spawned = true;
            closest = closest.min((x - 8.0).hypot(z));
        }
    }
    assert!(ever_spawned, "no cruiser ever spawned at wanted level 2");
    assert!(
        closest < 20.0,
        "cruisers never closed in on the parked player: closest {closest}"
    );
}

#[test]
fn escaping_out_of_sight_clears_the_wanted_level() {
    let (mut engine, mut buf) = setup_city(0);
    engine.set_vehicle_class(0);
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    for _ in 0..HZ {
        engine.step();
    }
    engine.set_wanted(1);
    for _ in 0..(4 * HZ) {
        engine.step();
    }
    assert!(engine.police_count() >= 1, "no cruiser to escape from");

    // Vanish beyond every cop's sight range and wait out grace + decay.
    engine.teleport_car(600.0, 1.0, 0.0);
    for _ in 0..(15 * HZ) {
        engine.step();
    }
    assert_eq!(engine.wanted_stars(), 0, "heat never decayed while unseen");
    assert_eq!(
        engine.police_count(),
        0,
        "pursuit outlived the wanted level"
    );
    engine.write_state(&mut buf);
    assert_eq!(
        buf[WANTED_STATE_OFFSET as usize], 0.0,
        "snapshot still shows stars"
    );
}

#[test]
fn a_roadblock_forms_ahead_at_three_stars() {
    let (mut engine, mut buf) = setup_city(0);
    // One long straight lane to race down (regular streets on the side).
    engine.load_chunk(3, 0, &[]);
    engine.load_chunk_lanes(
        3,
        0,
        &[
            30.0 - 3.0 * CHUNK_SIZE,
            0.25,
            -400.0,
            30.0 - 3.0 * CHUNK_SIZE,
            0.25,
            400.0,
        ],
        &[2],
    );
    engine.set_vehicle_class(0);
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    for _ in 0..HZ {
        engine.step();
    }
    engine.teleport_car(30.0, 1.0, -350.0);
    engine.set_wanted(3);
    engine.set_input_buttons(BTN_FORWARD, 0.0);

    // A pair of cruisers parked nose-to-tail across the road ahead of the
    // player's travel is the roadblock signature.
    let mut roadblock_seen = false;
    for _ in 0..(20 * HZ) {
        engine.step();
        engine.write_state(&mut buf);
        let car_z = buf[2];
        let cops = police_positions(&buf);
        for (i, a) in cops.iter().enumerate() {
            for b in cops.iter().skip(i + 1) {
                let pair_dist = (a.0 - b.0).hypot(a.2 - b.2);
                let ahead = a.2 > car_z + 20.0 && b.2 > car_z + 20.0;
                if pair_dist < 5.5 && ahead {
                    roadblock_seen = true;
                }
            }
        }
    }
    engine.set_input_buttons(0, 0.0);
    assert!(
        roadblock_seen,
        "no roadblock pair ever formed ahead at 3 stars"
    );
}

#[test]
fn police_is_deterministic() {
    let run = || {
        let (mut engine, mut buf) = setup_city(50);
        engine.set_vehicle_class(0);
        for _ in 0..HZ {
            engine.step();
        }
        engine.toggle_enter();
        engine.set_wanted(2);
        engine.set_input_buttons(BTN_FORWARD, 0.15);
        for _ in 0..(8 * HZ) {
            engine.step();
        }
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
