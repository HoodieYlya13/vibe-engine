//! On-foot replay tests for the Rapier character controller. The player must
//! collide with the same colliders the car drives on (blocks, ramp, walls,
//! chassis) — world geometry exists exactly once.

use sim::SimEngine;
use sim::constants::{BTN_FORWARD, BTN_RUN, CAR_HALF_WIDTH, PLAYER_STATE_OFFSET, WORLD_HALF};

const HZ: usize = 60;

struct Harness {
    engine: SimEngine,
    buf: Vec<f32>,
}

impl Harness {
    fn open_field() -> Self {
        Self::wrap(SimEngine::new_open_field(0))
    }

    /// The greybox world: stair blocks, the ramp, and the car parked at origin.
    fn greybox() -> Self {
        Self::wrap(SimEngine::new_with_peds(0))
    }

    fn wrap(engine: SimEngine) -> Self {
        let len = engine.state_len();
        Harness {
            engine,
            buf: vec![0.0; len],
        }
    }

    fn step_seconds(&mut self, seconds: f32) {
        for _ in 0..((seconds * HZ as f32) as usize) {
            self.engine.step();
        }
    }

    /// Holds buttons while looking along `yaw` (on foot, W walks toward yaw).
    fn hold_look(&mut self, buttons: u32, yaw: f32, seconds: f32) {
        self.engine.set_input_buttons(buttons, yaw);
        self.step_seconds(seconds);
    }

    fn player_pos(&mut self) -> (f32, f32, f32) {
        self.engine.write_state(&mut self.buf);
        let off = PLAYER_STATE_OFFSET as usize;
        (self.buf[off], self.buf[off + 1], self.buf[off + 2])
    }
}

#[test]
fn walking_moves_player_forward() {
    let mut h = Harness::open_field();
    h.step_seconds(0.5);
    let (x0, _, z0) = h.player_pos();

    // yaw 0 looks toward +Z.
    h.hold_look(BTN_FORWARD, 0.0, 2.0);

    let (x, y, z) = h.player_pos();
    assert!(
        (6.5..9.0).contains(&(z - z0)),
        "2s of walking should cover ~8m: moved {} m",
        z - z0
    );
    assert!((x - x0).abs() < 0.2, "player veered sideways: x = {x}");
    assert!(
        (0.5..1.2).contains(&y),
        "player should stay on the ground while walking: y = {y}"
    );
}

#[test]
fn running_is_faster_than_walking() {
    let mut h = Harness::open_field();
    h.step_seconds(0.5);
    let (_, _, z0) = h.player_pos();
    h.hold_look(BTN_FORWARD | BTN_RUN, 0.0, 2.0);
    let (_, _, z) = h.player_pos();
    assert!(
        z - z0 > 12.0,
        "2s of running should cover ~14m: moved {} m",
        z - z0
    );
}

#[test]
fn wall_stops_player() {
    let mut h = Harness::open_field();
    h.step_seconds(0.5);

    // 20s at walk speed would be 80m; the arena wall is at z = 60.
    h.hold_look(BTN_FORWARD, 0.0, 20.0);

    let (_, y, z) = h.player_pos();
    assert!(
        z < WORLD_HALF,
        "player tunneled through the arena wall: z = {z}"
    );
    assert!(z > 58.0, "player never reached the wall: z = {z}");
    assert!(
        y < 2.0,
        "player should not climb the wall (autostep too high): y = {y}"
    );
}

#[test]
fn jump_rises_and_lands() {
    let mut h = Harness::open_field();
    h.step_seconds(0.5);
    let (_, y_rest, _) = h.player_pos();

    h.engine.jump();
    let mut apex = y_rest;
    for _ in 0..(2 * HZ) {
        h.engine.step();
        let (_, y, _) = h.player_pos();
        apex = apex.max(y);
    }
    let (_, y_end, _) = h.player_pos();

    assert!(
        apex - y_rest > 1.2,
        "jump apex too low: rose only {} m",
        apex - y_rest
    );
    assert!(
        (y_end - y_rest).abs() < 0.1,
        "player did not land back at rest height: {y_rest} -> {y_end}"
    );

    // A second jump must work after landing (grounded state recovered).
    h.engine.jump();
    h.step_seconds(0.2);
    let (_, y_again, _) = h.player_pos();
    assert!(
        y_again - y_rest > 0.4,
        "second jump failed: y = {y_again} (rest {y_rest})"
    );
}

#[test]
fn ramp_is_walkable_up_its_z_axis() {
    // The ramp runs from z=-8 (ground) to z=-24 (3m high) centered on x=0 —
    // the same orientation as its Rapier collider and its visual. The old
    // hand-rolled player math treated it as running along X (defect #5).
    let mut h = Harness::greybox();
    h.step_seconds(0.5);

    // Walk from spawn (2, _, 0) toward (0, _, -20): onto and up the ramp.
    let yaw = (0.0_f32 - 2.0).atan2(-20.0);
    h.hold_look(BTN_FORWARD, yaw, 5.0);

    let (x, y, z) = h.player_pos();
    assert!(
        z < -15.0,
        "player never made it onto the ramp: z = {z} (x = {x})"
    );
    assert!(
        y > 2.0,
        "player gained no height walking up the ramp: y = {y} at z = {z}"
    );
}

#[test]
fn parked_car_blocks_player() {
    // The old code let the player walk straight through the chassis (only a
    // soft push-out existed). The character controller collides with it.
    let mut h = Harness::greybox();
    h.step_seconds(0.5);

    // Walk from spawn (2, _, 0) toward -X, straight into the parked car.
    h.hold_look(BTN_FORWARD, -std::f32::consts::FRAC_PI_2, 2.0);

    let (x, _, _) = h.player_pos();
    // Blocked at the chassis face: half-width + most of the capsule radius.
    assert!(
        x > CAR_HALF_WIDTH + 0.25,
        "player walked through the parked car chassis: x = {x}"
    );
}

#[test]
fn on_foot_sim_is_deterministic() {
    let run = || {
        let mut h = Harness::greybox();
        h.step_seconds(0.5);
        h.hold_look(BTN_FORWARD, 1.1, 1.5);
        h.engine.jump();
        h.hold_look(BTN_FORWARD | BTN_RUN, -2.0, 1.5);
        h.hold_look(BTN_FORWARD, 3.0, 1.0);
        h.engine.write_state(&mut h.buf);
        h.buf.clone()
    };

    let a = run();
    let b = run();
    assert_eq!(
        a.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
        b.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
        "identical on-foot input scripts diverged — sim is not deterministic"
    );
}
