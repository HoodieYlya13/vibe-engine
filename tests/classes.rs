//! Tests for the vehicle class data file (`data/vehicles.ron`) and runtime
//! class switching (PRD Phase 2).

use sim::SimEngine;
use sim::constants::BTN_FORWARD;

const HZ: usize = 60;

fn car_pos(engine: &SimEngine, buf: &mut [f32]) -> (f32, f32, f32) {
    engine.write_state(buf);
    (buf[0], buf[1], buf[2])
}

#[test]
fn data_file_defines_the_three_classes() {
    let engine = SimEngine::new_open_field(0);
    assert_eq!(
        engine.vehicle_class_names(),
        vec!["sedan", "sports", "truck"],
        "vehicles.ron class list changed — update the tests and dev console expectations"
    );
    assert_eq!(engine.vehicle_class_index(), 1, "sports must be the default");
}

#[test]
fn tuning_getter_matches_setter_order() {
    let mut engine = SimEngine::new_open_field(0);
    let mut v = engine.vehicle_tuning();
    assert_eq!(v.len(), 12);
    v[0] = 1234.0;
    v[11] = 0.42;
    engine.set_vehicle_tuning(
        v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11],
    );
    assert_eq!(
        engine.vehicle_tuning(),
        v,
        "vehicle_tuning() must round-trip set_vehicle_tuning in the same order"
    );
}

#[test]
fn classes_have_distinct_acceleration() {
    let sprint = |class_index: u32| {
        let mut engine = SimEngine::new_open_field(0);
        engine.set_vehicle_class(class_index);
        let mut buf = vec![0.0; engine.state_len()];
        for _ in 0..HZ {
            engine.step();
        }
        engine.toggle_enter();
        engine.set_input_buttons(BTN_FORWARD, 0.0);
        for _ in 0..(2 * HZ) {
            engine.step();
        }
        car_pos(&engine, &mut buf).2
    };

    let sedan = sprint(0);
    let sports = sprint(1);
    let truck = sprint(2);
    assert!(
        sports > sedan + 3.0,
        "sports should out-accelerate the sedan: sports {sports} m vs sedan {sedan} m"
    );
    assert!(
        sedan > truck + 1.0,
        "the truck should be the slowest off the line: truck {truck} m vs sedan {sedan} m"
    );
}

#[test]
fn class_switch_preserves_pose_and_keeps_simulating() {
    let mut engine = SimEngine::new_open_field(0);
    let mut buf = vec![0.0; engine.state_len()];

    for _ in 0..HZ {
        engine.step();
    }
    engine.toggle_enter();
    engine.set_input_buttons(BTN_FORWARD, 0.0);
    for _ in 0..HZ {
        engine.step();
    }
    let (x0, _, z0) = car_pos(&engine, &mut buf);

    engine.set_vehicle_class(2);
    let (x1, y1, z1) = car_pos(&engine, &mut buf);
    assert!(
        (x1 - x0).abs() < 0.01 && (z1 - z0).abs() < 0.01,
        "class switch teleported the car: ({x0}, {z0}) -> ({x1}, {z1})"
    );

    // The heavier truck must keep driving (and stay upright) after the swap.
    for _ in 0..(2 * HZ) {
        engine.step();
    }
    let (x2, y2, z2) = car_pos(&engine, &mut buf);
    assert!(
        z2 > z1 + 5.0,
        "car stopped driving after the class switch: z {z1} -> {z2}"
    );
    assert!(
        buf.iter().all(|v| v.is_finite()),
        "state went non-finite after class switch"
    );
    assert!(
        (0.0..3.0).contains(&y1) && (0.0..3.0).contains(&y2),
        "car left plausible height range after switch: y {y1} -> {y2}"
    );
    let _ = x2;
}

#[test]
fn reapplying_the_class_resets_live_tuning() {
    let mut engine = SimEngine::new_open_field(0);
    let defaults = engine.vehicle_tuning();

    let mut v = defaults.clone();
    v[0] = 15000.0;
    engine.set_vehicle_tuning(
        v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11],
    );
    assert_ne!(engine.vehicle_tuning(), defaults);

    engine.set_vehicle_class(engine.vehicle_class_index());
    assert_eq!(
        engine.vehicle_tuning(),
        defaults,
        "re-applying the active class should restore its data-file tuning"
    );
}
