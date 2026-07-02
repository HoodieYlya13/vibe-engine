//! Emits the shared SAB layout and input constants as TypeScript, so the
//! client and the sim can never drift (PRD §8.2). Run from `sim/`:
//!
//! ```sh
//! cargo run --bin gen_layout            # writes ../client/src/generated/sim-layout.ts
//! cargo run --bin gen_layout -- <path>  # custom output path
//! ```

use std::fmt::Write as _;
use std::path::PathBuf;

use sim::constants::*;

fn flat<const N: usize>(rows: &[[f32; N]]) -> String {
    rows.iter()
        .flat_map(|r| r.iter())
        .map(|v| format!("{v}"))
        .collect::<Vec<_>>()
        .join(", ")
}

fn main() {
    let blocks: Vec<[f32; 6]> = BLOCKS
        .iter()
        .map(|b| [b.0, b.1, b.2, b.3, b.4, b.5])
        .collect();
    let ramps: Vec<[f32; 5]> = RAMPS.iter().map(|r| [r.0, r.1, r.2, r.3, r.4]).collect();

    let mut out = String::new();
    let mut c = |name: &str, value: u32| {
        writeln!(out, "export const {name} = {value};").unwrap();
    };

    c("DEFAULT_PED_COUNT", DEFAULT_PED_COUNT);

    c("BTN_FORWARD", BTN_FORWARD);
    c("BTN_BACK", BTN_BACK);
    c("BTN_LEFT", BTN_LEFT);
    c("BTN_RIGHT", BTN_RIGHT);
    c("BTN_RUN", BTN_RUN);
    c("BTN_HANDBRAKE", BTN_HANDBRAKE);

    c("ACTION_JUMP", ACTION_JUMP);
    c("ACTION_ENTER", ACTION_ENTER);
    c("ACTION_RESET", ACTION_RESET);
    c("ACTION_PAUSE", ACTION_PAUSE);

    c("HUD_HINT_NONE", HUD_HINT_NONE);
    c("HUD_HINT_ENTER", HUD_HINT_ENTER);
    c("HUD_HINT_EXIT", HUD_HINT_EXIT);

    c("CAR_STATE_FLOATS", CAR_STATE_FLOATS);
    c("PLAYER_STATE_FLOATS", PLAYER_STATE_FLOATS);
    c("PLAYER_STATE_OFFSET", PLAYER_STATE_OFFSET);
    c("HUD_STATE_FLOATS", HUD_STATE_FLOATS);
    c("HUD_STATE_OFFSET", HUD_STATE_OFFSET);
    c("CAM_STATE_FLOATS", CAM_STATE_FLOATS);
    c("CAM_STATE_OFFSET", CAM_STATE_OFFSET);
    c("PED_STATE_OFFSET", PED_STATE_OFFSET);
    c("PED_STATE_STRIDE", PED_STATE_STRIDE);
    c("STATE_HEADER_INTS", STATE_HEADER_INTS);

    c("INPUT_HEADER_INTS", INPUT_HEADER_INTS);
    c("INPUT_CAPACITY", INPUT_CAPACITY);
    c("INPUT_STRIDE", INPUT_STRIDE);

    c("BLOCK_STATE_STRIDE", BLOCK_STATE_STRIDE);
    c("RAMP_STATE_STRIDE", RAMP_STATE_STRIDE);

    // Vehicle tuning defaults — the dev console seeds its sliders from these
    // and writes changes back via SimEngine::set_vehicle_tuning.
    let mut cf = |name: &str, value: f32| {
        writeln!(out, "export const {name} = {value};").unwrap();
    };
    cf("VEHICLE_MAX_ENGINE_FORCE", VEHICLE_MAX_ENGINE_FORCE);
    cf("VEHICLE_MAX_REVERSE_FORCE", VEHICLE_MAX_REVERSE_FORCE);
    cf("VEHICLE_FOOTBRAKE_IMPULSE", VEHICLE_FOOTBRAKE_IMPULSE);
    cf("VEHICLE_ENGINE_BRAKE_IMPULSE", VEHICLE_ENGINE_BRAKE_IMPULSE);
    cf("VEHICLE_HANDBRAKE_IMPULSE", VEHICLE_HANDBRAKE_IMPULSE);
    cf(
        "VEHICLE_DIRECTION_SWITCH_SPEED",
        VEHICLE_DIRECTION_SWITCH_SPEED,
    );
    cf("VEHICLE_MAX_STEER", VEHICLE_MAX_STEER);

    let body = out;
    let file = format!(
        "// AUTO-GENERATED from /sim constants by `cargo run --bin gen_layout`.\n\
         // DO NOT EDIT — change sim/src/constants.rs and regenerate.\n\n\
         {body}\n\
         // Static world geometry, flattened rows of (x, y, z, hx, hy, hz).\n\
         export const BLOCKS: readonly number[] = [{blocks}];\n\n\
         // Static ramps, flattened rows of (z_start, z_end, x_center, half_width, height).\n\
         export const RAMPS: readonly number[] = [{ramps}];\n",
        blocks = flat(&blocks),
        ramps = flat(&ramps),
    );

    let default_path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../client/src/generated/sim-layout.ts");
    let path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or(default_path);

    std::fs::create_dir_all(path.parent().expect("output path has a parent"))
        .expect("create generated dir");
    std::fs::write(&path, file).expect("write generated layout");
    println!("wrote {}", path.display());
}
