//! Ambient rail traffic (Phase 4): cars spawn on streamed lane polylines,
//! follow them at street speed, queue up behind the player's parked car
//! without touching it, convert to dynamic bodies when the player rams them,
//! and vanish with their chunk. All through the public wasm API, so these
//! also pin the snapshot traffic block.

use sim::SimEngine;
use sim::constants::{
    BTN_FORWARD, CAR_HEALTH_OFFSET, MAX_TRAFFIC, TRAFFIC_STATE_OFFSET, TRAFFIC_STATE_STRIDE,
};

const HZ: usize = 60;

/// Active traffic cars as (x, y, z) from the snapshot block.
fn traffic_positions(buf: &[f32]) -> Vec<(f32, f32, f32)> {
    (0..MAX_TRAFFIC as usize)
        .filter_map(|i| {
            let c = TRAFFIC_STATE_OFFSET as usize + i * TRAFFIC_STATE_STRIDE as usize;
            (buf[c] > 0.5).then(|| (buf[c + 1], buf[c + 2], buf[c + 3]))
        })
        .collect()
}

/// City engine with no peds and one straight lane along +Z at x = 0,
/// running world z 60 → 140 (chunk (0, 1), chunk-local coordinates).
fn setup_with_lane() -> (SimEngine, Vec<f32>) {
    let mut engine = SimEngine::new_city(0);
    engine.load_chunk(0, 1, &[]);
    engine.load_chunk_lanes(0, 1, &[0.0, 0.25, -40.0, 0.0, 0.25, 40.0], &[2]);
    let buf = vec![0.0; engine.state_len()];
    (engine, buf)
}

#[test]
fn traffic_spawns_and_follows_the_lane() {
    let (mut engine, mut buf) = setup_with_lane();
    for _ in 0..(2 * HZ) {
        engine.step();
    }
    assert!(
        engine.traffic_count() >= 1,
        "no traffic spawned on the lane"
    );

    engine.write_state(&mut buf);
    let before = traffic_positions(&buf);
    for _ in 0..HZ {
        engine.step();
    }
    engine.write_state(&mut buf);
    let after = traffic_positions(&buf);

    let (x0, y0, z0) = before[0];
    let (x1, _, z1) = after[0];
    assert!(
        z1 > z0 + 4.0,
        "traffic car did not advance along +Z: {z0} -> {z1}"
    );
    assert!(
        x0.abs() < 0.5 && x1.abs() < 0.5,
        "traffic car left the lane line: x {x0} -> {x1}"
    );
    assert!(
        (0.5..1.2).contains(&y0),
        "traffic car rides at the wrong height: y {y0}"
    );
}

#[test]
fn traffic_queues_behind_the_player_car_without_touching() {
    let (mut engine, mut buf) = setup_with_lane();
    // Park the player car across the lane, 40 m past the lane start.
    engine.teleport_car(0.0, 1.0, 100.0);
    for _ in 0..(12 * HZ) {
        engine.step();
    }

    engine.write_state(&mut buf);
    let cars = traffic_positions(&buf);
    assert!(!cars.is_empty(), "no traffic spawned");
    for (x, _, z) in &cars {
        assert!(
            *z < 96.5,
            "traffic ran into / past the parked player car: z {z} (x {x})"
        );
    }

    // The queue is stationary, and stopping cost the player no health.
    let lead_z = cars.iter().map(|c| c.2).fold(f32::MIN, f32::max);
    for _ in 0..HZ {
        engine.step();
    }
    engine.write_state(&mut buf);
    let lead_z_after = traffic_positions(&buf)
        .iter()
        .map(|c| c.2)
        .fold(f32::MIN, f32::max);
    assert!(
        (lead_z_after - lead_z).abs() < 0.2,
        "lead traffic car is still creeping: {lead_z} -> {lead_z_after}"
    );
    let health = buf[CAR_HEALTH_OFFSET as usize];
    assert!(
        health > 0.999,
        "queued traffic damaged the parked player car: health {health}"
    );
}

#[test]
fn ramming_traffic_converts_it_to_a_dynamic_body() {
    let (mut engine, mut buf) = setup_with_lane();
    engine.set_vehicle_class(0);
    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    // Let a traffic car spawn and get rolling, then floor it after it.
    for _ in 0..(2 * HZ) {
        engine.step();
    }
    assert!(engine.traffic_count() >= 1, "no traffic to ram");

    engine.set_input_buttons(BTN_FORWARD, 0.0);
    let mut max_step = 0.0f32;
    let mut prev = Vec::new();
    let dt = engine.dt();
    for _ in 0..(6 * HZ) {
        engine.step();
        engine.write_state(&mut buf);
        let now = traffic_positions(&buf);
        for (i, p) in now.iter().enumerate() {
            if let Some(q) = prev.get(i) {
                let (qx, _, qz): (f32, f32, f32) = *q;
                let d = ((p.0 - qx).powi(2) + (p.2 - qz).powi(2)).sqrt();
                max_step = max_step.max(d / dt);
            }
        }
        prev = now;
    }

    // Rail speed caps at 9 m/s; only a rammed (now dynamic) car can be
    // shoved faster than that.
    assert!(
        max_step > 12.0,
        "no traffic car was shoved dynamically: max speed {max_step}"
    );
    let health = buf[CAR_HEALTH_OFFSET as usize];
    assert!(
        health < 0.999,
        "ramming a 1.3 t car should cost some health: {health}"
    );
    assert!(
        health > 0.2,
        "hitting converted traffic should be a shove, not a wall: health {health}"
    );
}

#[test]
fn traffic_passes_a_car_parked_beside_the_lane() {
    // Adjacent-lane geometry (kit lanes sit 2.6 m off a parked car's line):
    // passing traffic must NOT convert to dynamic against the parked player
    // car — that used to grind its health to zero while it sat at spawn.
    let (mut engine, mut buf) = setup_with_lane();
    engine.teleport_car(2.6, 1.0, 100.0);
    let mut passed = false;
    for _ in 0..(20 * HZ) {
        engine.step();
        engine.write_state(&mut buf);
        if traffic_positions(&buf).iter().any(|c| c.2 > 110.0) {
            passed = true;
        }
    }
    assert!(
        passed,
        "traffic never passed the car parked beside the lane"
    );
    let health = buf[CAR_HEALTH_OFFSET as usize];
    assert!(
        health > 0.999,
        "passing traffic damaged the parked player car: health {health}"
    );
}

#[test]
fn unloading_the_chunk_despawns_its_traffic() {
    let (mut engine, _) = setup_with_lane();
    for _ in 0..(3 * HZ) {
        engine.step();
    }
    assert!(engine.traffic_count() >= 1, "no traffic spawned");
    engine.unload_chunk(0, 1);
    assert_eq!(
        engine.traffic_count(),
        0,
        "traffic outlived its unloaded chunk"
    );
    // And the sim keeps stepping happily without the lane.
    for _ in 0..HZ {
        engine.step();
    }
}

#[test]
fn traffic_is_deterministic() {
    let run = || {
        let (mut engine, mut buf) = setup_with_lane();
        for _ in 0..(4 * HZ) {
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

/// Four-way crossroads at the origin with the kit city's lane layout: every
/// street has a twin pair of one-way lanes 2.6 m off its centerline, inset
/// 7 m from the intersection center.
fn load_crossroads(engine: &mut SimEngine) {
    let lanes: [[f32; 6]; 8] = [
        [2.6, 0.25, -77.0, 2.6, 0.25, -7.0],
        [-2.6, 0.25, -7.0, -2.6, 0.25, -77.0],
        [-7.0, 0.25, -2.6, -77.0, 0.25, -2.6],
        [-77.0, 0.25, 2.6, -7.0, 0.25, 2.6],
        [2.6, 0.25, 7.0, 2.6, 0.25, 77.0],
        [-2.6, 0.25, 77.0, -2.6, 0.25, 7.0],
        [77.0, 0.25, -2.6, 7.0, 0.25, -2.6],
        [7.0, 0.25, 2.6, 77.0, 0.25, 2.6],
    ];
    let points: Vec<f32> = lanes.iter().flatten().copied().collect();
    engine.load_chunk(0, 0, &[]);
    engine.load_chunk_lanes(0, 0, &points, &[2; 8]);
}

#[test]
fn turning_traffic_never_rams_a_car_parked_in_the_intersection() {
    // The default spawn: player car parked dead-center on a crossroads, the
    // player standing next to it. Traffic turning across the intersection
    // used to bulldoze the parked car as an unstoppable kinematic body.
    let mut engine = SimEngine::new_city(0);
    load_crossroads(&mut engine);
    let mut buf = vec![0.0; engine.state_len()];
    engine.write_state(&mut buf);
    let (x0, z0) = (buf[0], buf[2]);

    let mut crossed = 0usize;
    for _ in 0..(120 * HZ) {
        engine.step();
        engine.write_state(&mut buf);
        crossed = crossed.max(
            traffic_positions(&buf)
                .iter()
                .filter(|c| c.0.abs() < 7.0 && c.2.abs() < 7.0)
                .count(),
        );
    }

    assert!(crossed >= 1, "no traffic ever entered the intersection");
    let moved = ((buf[0] - x0).powi(2) + (buf[2] - z0).powi(2)).sqrt();
    assert!(
        moved < 0.3,
        "traffic shoved the parked player car {moved} m"
    );
    let health = buf[CAR_HEALTH_OFFSET as usize];
    assert!(
        health > 0.999,
        "traffic damaged the parked player car: health {health}"
    );
    assert_eq!(
        engine.wanted_stars(),
        0,
        "a driverless parked car earned a wanted level"
    );
}
