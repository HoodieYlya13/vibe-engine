//! Tests for enter/exit (PRD defect #10): entering walks the player to the
//! door instead of teleporting and needs realistic proximity; exiting picks
//! a free spot around the car instead of always ejecting +X — even into a
//! wall.

use sim::SimEngine;
use sim::constants::{BTN_FORWARD, PLAYER_RUN_SPEED, PLAYER_STATE_OFFSET, WORLD_HALF};

const HZ: usize = 60;
const DT: f32 = 1.0 / 60.0;

struct Harness {
    engine: SimEngine,
    buf: Vec<f32>,
}

impl Harness {
    /// Open-field engine (arena walls only), player on foot at spawn.
    fn new() -> Self {
        let mut engine = SimEngine::new_open_field(0);
        engine.set_vehicle_class(0);
        let len = engine.state_len();
        let mut h = Harness {
            engine,
            buf: vec![0.0; len],
        };
        h.step_seconds(1.0);
        h
    }

    fn step_seconds(&mut self, seconds: f32) {
        for _ in 0..((seconds * HZ as f32) as usize) {
            self.engine.step();
        }
    }

    /// Player (x, y, z, in_car) as the client sees it.
    fn player(&mut self) -> (f32, f32, f32, bool) {
        self.engine.write_state(&mut self.buf);
        let o = PLAYER_STATE_OFFSET as usize;
        (
            self.buf[o],
            self.buf[o + 1],
            self.buf[o + 2],
            self.buf[o + 4] > 0.5,
        )
    }

    fn car(&mut self) -> (f32, f32, f32) {
        self.engine.write_state(&mut self.buf);
        (self.buf[0], self.buf[1], self.buf[2])
    }
}

/// The compatibility contract every other suite relies on: at spawn the
/// player stands at the door, so entering is immediate — no walk needed.
#[test]
fn enter_at_the_door_is_immediate() {
    let mut h = Harness::new();
    h.engine.toggle_enter();
    let (.., in_car) = h.player();
    assert!(in_car, "player at the door should enter instantly");
}

#[test]
fn enter_from_afar_walks_to_the_door() {
    let mut h = Harness::new();
    h.engine.teleport_player(5.0, 1.0, 2.0);
    h.engine.step();
    h.engine.toggle_enter();

    let (.., in_car) = h.player();
    assert!(!in_car, "entering from 5 m away must not teleport");

    // The approach must be a continuous jog: never faster than run speed.
    let mut prev = h.player();
    let mut entered_after = None;
    for frame in 0..(3 * HZ) {
        h.engine.step();
        let cur = h.player();
        if cur.3 {
            entered_after = Some(frame as f32 * DT);
            break;
        }
        let step = ((cur.0 - prev.0).powi(2) + (cur.2 - prev.2).powi(2)).sqrt();
        assert!(
            step <= PLAYER_RUN_SPEED * DT * 1.6 + 0.01,
            "teleport-sized hop while entering: {step} m in one frame"
        );
        prev = cur;
    }
    let entered_after = entered_after.expect("player never reached the door");
    assert!(
        entered_after < 1.5,
        "the ~3 m walk to the door took {entered_after} s"
    );
}

#[test]
fn enter_beyond_radius_is_ignored() {
    let mut h = Harness::new();
    h.engine.teleport_player(10.0, 1.0, 0.0);
    h.engine.step();
    let before = h.player();
    h.engine.toggle_enter();
    h.step_seconds(1.0);
    let after = h.player();
    assert!(!after.3, "entered from 10 m away");
    let moved = ((after.0 - before.0).powi(2) + (after.2 - before.2).powi(2)).sqrt();
    assert!(
        moved < 0.2,
        "an out-of-range enter press started a walk: moved {moved} m"
    );
}

#[test]
fn movement_input_cancels_the_walk() {
    let mut h = Harness::new();
    h.engine.teleport_player(5.0, 1.0, 0.0);
    h.engine.step();
    h.engine.toggle_enter();
    // Grabbing the stick takes control back; camera yaw 0 walks +Z, away
    // from the door.
    h.engine.set_input_buttons(BTN_FORWARD, 0.0);
    h.step_seconds(2.0);
    let (_, _, pz, in_car) = h.player();
    assert!(!in_car, "movement input should cancel the walk-to-door");
    assert!(pz > 2.0, "player should be walking +Z instead: z {pz}");
}

/// Parks flank-to-wall at `car_x` and exits; returns (player_x, car_x).
fn wall_exit_x(car_x: f32) -> (f32, f32) {
    let mut h = Harness::new();
    h.engine.toggle_enter();
    h.engine.teleport_car(car_x, 1.0, 0.0);
    h.step_seconds(0.5);
    let (cx, ..) = h.car();
    h.engine.toggle_enter();
    let (px, _, _, in_car) = h.player();
    assert!(!in_car, "exit failed entirely at car x {car_x}");
    (px, cx)
}

/// Both flanks against both X walls (inner faces at ±WORLD_HALF): whichever
/// door faces the wall, the player must come out on the interior side. One
/// of the two cases necessarily has the first-choice door blocked, so this
/// exercises the fallback, not just the happy path.
#[test]
fn exit_avoids_the_wall() {
    let (px, cx) = wall_exit_x(WORLD_HALF - 2.2);
    assert!(
        px < cx - 1.0 && px < WORLD_HALF - 0.35,
        "+X wall: ejected into/toward the wall (player x {px}, car x {cx})"
    );

    let (px, cx) = wall_exit_x(-(WORLD_HALF - 2.2));
    assert!(
        px > cx + 1.0 && px > -(WORLD_HALF - 0.35),
        "-X wall: ejected into/toward the wall (player x {px}, car x {cx})"
    );
}

#[test]
fn exit_lands_on_the_ground_nearby() {
    let mut h = Harness::new();
    h.engine.toggle_enter();
    h.engine.teleport_car(20.0, 1.0, 20.0);
    h.step_seconds(0.5);
    h.engine.toggle_enter();
    h.step_seconds(1.0);
    let (px, py, pz, in_car) = h.player();
    let (cx, _, cz) = h.car();
    assert!(!in_car);
    let dist = ((px - cx).powi(2) + (pz - cz).powi(2)).sqrt();
    assert!(
        (1.5..=4.5).contains(&dist),
        "player should stand beside the car: {dist} m away"
    );
    assert!(
        (py - 1.0).abs() < 0.3,
        "player should have settled on the ground: y {py}"
    );
}
