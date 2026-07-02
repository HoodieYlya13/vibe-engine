use sim::SimEngine;

const HZ: usize = 60;

struct Harness {
    engine: SimEngine,
    buf: Vec<f32>,
}

impl Harness {
    fn new() -> Self {
        let engine = SimEngine::new_open_field(0);
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

    fn hold(&mut self, buttons: u32, seconds: f32) {
        self.engine.set_input_buttons(buttons, 0.0);
        self.step_seconds(seconds);
    }

    fn car_pos(&mut self) -> (f32, f32, f32) {
        self.engine.write_state(&mut self.buf);
        (self.buf[0], self.buf[1], self.buf[2])
    }

    fn assert_sane(&mut self) {
        self.engine.write_state(&mut self.buf);
        for (i, v) in self.buf.iter().enumerate() {
            assert!(v.is_finite(), "state[{i}] is not finite: {v}");
        }
        let (_, y, _) = self.car_pos();
        assert!(
            (-1.0..20.0).contains(&y),
            "car left plausible height range: y = {y}"
        );
    }

    fn enter_car(&mut self) {
        self.engine.toggle_enter();
        self.engine.write_state(&mut self.buf);
        let in_car = self.buf[sim::player_state_offset() as usize + 4] > 0.5;
        assert!(in_car, "player failed to enter the car");
    }
}

#[test]
fn w_drives_forward_even_after_sleep() {
    let mut h = Harness::new();

    h.step_seconds(5.0);
    assert!(
        h.engine.car_sleeping(),
        "precondition failed: car should be asleep after 5s idle"
    );

    h.enter_car();
    let (_, y0, z0) = h.car_pos();

    h.hold(sim::btn_forward(), 2.0);

    let (x, y, z) = h.car_pos();
    assert!(
        z - z0 > 15.0,
        "W must drive the car forward (+Z): moved {} m",
        z - z0
    );
    assert!(x.abs() < 2.0, "car drifted sideways with no steering: x = {x}");
    assert!(
        (y - y0).abs() < 1.5,
        "car jumped on wake-up (loaded-spring bug): y {y0} -> {y}"
    );
    h.assert_sane();
}

#[test]
fn s_brakes_before_reversing() {
    let mut h = Harness::new();
    h.step_seconds(1.0);
    h.enter_car();

    h.hold(sim::btn_forward(), 1.5);
    let (_, _, z_fast) = h.car_pos();

    h.hold(sim::btn_back(), 0.5);
    let (_, _, z_braking) = h.car_pos();
    assert!(
        z_braking > z_fast - 1.0,
        "car snapped backwards under braking: {z_fast} -> {z_braking}"
    );

    h.hold(sim::btn_back(), 3.5);
    let (_, _, z_a) = h.car_pos();
    h.hold(sim::btn_back(), 1.0);
    let (_, _, z_b) = h.car_pos();
    assert!(
        z_b < z_a - 0.5,
        "car should be reversing by now: {z_a} -> {z_b}"
    );
    h.assert_sane();
}

#[test]
fn handbrake_stops_the_car() {
    let mut h = Harness::new();
    h.step_seconds(1.0);
    h.enter_car();

    h.hold(sim::btn_forward(), 2.0);
    h.hold(sim::btn_handbrake(), 3.0);
    let (_, _, z_a) = h.car_pos();
    h.hold(sim::btn_handbrake(), 1.0);
    let (_, _, z_b) = h.car_pos();
    assert!(
        (z_b - z_a).abs() < 0.5,
        "handbrake failed to hold the car: moved {} m in 1s",
        z_b - z_a
    );
    h.assert_sane();
}

#[test]
fn steering_turns_the_car() {
    let mut h = Harness::new();
    h.step_seconds(1.0);
    h.enter_car();

    h.hold(sim::btn_forward(), 1.0);
    h.hold(sim::btn_forward() | sim::btn_right(), 2.0);
    let (x, _, _) = h.car_pos();
    assert!(x > 1.0, "D while driving should curve toward +X: x = {x}");
    h.assert_sane();
}

#[test]
fn sim_is_deterministic() {
    let run = || {
        let mut h = Harness::new();
        h.step_seconds(1.0);
        h.enter_car();
        h.hold(sim::btn_forward(), 2.0);
        h.hold(sim::btn_forward() | sim::btn_left(), 1.5);
        h.hold(sim::btn_handbrake() | sim::btn_left(), 1.0);
        h.hold(sim::btn_back(), 2.0);
        h.engine.write_state(&mut h.buf);
        h.buf.clone()
    };

    let a = run();
    let b = run();
    assert_eq!(
        a.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
        b.iter().map(|v| v.to_bits()).collect::<Vec<_>>(),
        "identical input scripts diverged — sim is not deterministic"
    );
}

#[test]
fn parked_car_stays_parked() {
    let mut h = Harness::new();
    h.step_seconds(1.0);
    let (x0, y0, z0) = h.car_pos();
    h.step_seconds(30.0);
    let (x1, y1, z1) = h.car_pos();
    let drift =
        ((x1 - x0).powi(2) + (y1 - y0).powi(2) + (z1 - z0).powi(2)).sqrt();
    assert!(drift < 0.05, "parked car drifted {drift} m over 30s");
    assert!(h.engine.car_sleeping(), "parked car should be asleep");
}
