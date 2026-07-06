//! Vehicle class definitions, deserialized from `sim/data/vehicles.ron`
//! (embedded at compile time). The data file is the source of truth for
//! everything about how a class drives — editing it needs a rebuild but no
//! code change; frame-to-frame experimentation happens through the dev
//! console sliders instead (`SimEngine::set_vehicle_tuning`).

use serde::Deserialize;

use super::VehicleTuning;

#[derive(Clone, Copy, Deserialize)]
pub(crate) struct ChassisSpec {
    pub(crate) mass: f32,
    /// How far below the chassis center the mass sits — the anti-flip lever.
    pub(crate) com_drop: f32,
    pub(crate) linear_damping: f32,
    pub(crate) angular_damping: f32,
}

#[derive(Clone, Copy, Deserialize)]
pub(crate) struct WheelSpec {
    pub(crate) radius: f32,
    pub(crate) suspension_rest: f32,
    /// Mass-relative: Rapier multiplies the spring force by chassis mass,
    /// so the same number means the same softness at any weight.
    pub(crate) suspension_stiffness: f32,
    pub(crate) suspension_compression: f32,
    pub(crate) suspension_damping: f32,
    pub(crate) max_suspension_travel: f32,
    pub(crate) side_friction_stiffness: f32,
    pub(crate) friction_slip: f32,
    /// Hard cap in newtons (NOT mass-relative) — heavier classes need more.
    pub(crate) max_suspension_force: f32,
}

#[derive(Clone, Deserialize)]
pub(crate) struct VehicleSpec {
    pub(crate) name: String,
    pub(crate) chassis: ChassisSpec,
    pub(crate) wheels: WheelSpec,
    pub(crate) drive: VehicleTuning,
}

#[derive(Deserialize)]
struct VehicleDataFile {
    classes: Vec<VehicleSpec>,
}

/// Parses the embedded data file. Panics on a malformed file — that is a
/// build-content bug and `tests/classes.rs` catches it natively.
pub(crate) fn load_vehicle_classes() -> Vec<VehicleSpec> {
    let file: VehicleDataFile = ron::from_str(include_str!("../../data/vehicles.ron"))
        .expect("sim/data/vehicles.ron is malformed");
    assert!(
        !file.classes.is_empty(),
        "sim/data/vehicles.ron defines no vehicle classes"
    );
    file.classes
}
