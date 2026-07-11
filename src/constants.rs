//! Single source of truth for input bits, SAB layout, and world constants.
//! The TypeScript mirror (`client/src/generated/sim-layout.ts`) is emitted by
//! `cargo run --bin gen_layout` — regenerate it after changing anything here.

pub const DEFAULT_PED_COUNT: u32 = 1500;
pub const BTN_FORWARD: u32 = 1 << 0;
pub const BTN_BACK: u32 = 1 << 1;
pub const BTN_LEFT: u32 = 1 << 2;
pub const BTN_RIGHT: u32 = 1 << 3;
pub const BTN_RUN: u32 = 1 << 4;
pub const BTN_HANDBRAKE: u32 = 1 << 5;
pub const ACTION_JUMP: u32 = 1 << 0;
pub const ACTION_ENTER: u32 = 1 << 1;
pub const ACTION_RESET: u32 = 1 << 2;
pub const ACTION_PAUSE: u32 = 1 << 3;
pub const HUD_HINT_NONE: u32 = 0;
pub const HUD_HINT_ENTER: u32 = 1;
pub const HUD_HINT_EXIT: u32 = 2;

// Vehicle driving parameters live in `data/vehicles.ron` (per-class), not
// here — the drive-tuning source of truth is the data file (PRD Phase 2).

pub const WORLD_HALF: f32 = 60.0;
pub const WALL_THICKNESS: f32 = 1.5;
pub const WALL_HEIGHT: f32 = 6.0;

/// City streaming (Phase 3, PRD §6): the world is a square grid of chunks of
/// this size (m); chunk (cx, cz) is centered at (cx·CHUNK_SIZE, cz·CHUNK_SIZE).
/// Collider sidecars and visuals stream in/out per chunk.
pub const CHUNK_SIZE: f32 = 100.0;
/// Half-extent of the city's flat ground slab (placeholder until terrain).
pub const CITY_GROUND_HALF: f32 = 2000.0;

/// Spawn/exit height for the player capsule center (the character controller
/// snaps it onto the ground from here).
pub const PLAYER_STAND_HEIGHT: f32 = 1.0;
pub const PLAYER_FOOT_RADIUS: f32 = 0.35;
/// Capsule cylinder half-height; total capsule height is
/// `2 * (PLAYER_CAPSULE_HALF_HEIGHT + PLAYER_FOOT_RADIUS)` = 1.7, matching the visual.
pub const PLAYER_CAPSULE_HALF_HEIGHT: f32 = 0.5;
/// Max autostep height and snap-to-ground distance for the character controller.
pub const PLAYER_STEP_DOWN: f32 = 0.35;
pub const PLAYER_JUMP_SPEED: f32 = 6.2;
pub const PLAYER_WALK_SPEED: f32 = 4.0;
pub const PLAYER_RUN_SPEED: f32 = 7.0;

pub const CAR_HALF_WIDTH: f32 = 1.8;
pub const CAR_HALF_HEIGHT: f32 = 0.8;
pub const CAR_HALF_LENGTH: f32 = 3.2;
/// Max horizontal distance (car center → player) at which pressing enter
/// starts the walk-to-door. Also drives the HUD "enter" hint.
pub const ENTER_RADIUS: f32 = 6.0;

pub const BLOCK_STATE_STRIDE: u32 = 6;
/// (x, y, z, half_x, half_y, half_z)
pub type BlockDef = (f32, f32, f32, f32, f32, f32);
pub const BLOCKS: &[BlockDef] = &[
    (6.0, 0.5, 6.0, 1.2, 0.5, 1.2),
    (9.0, 1.0, 6.0, 1.2, 1.0, 1.2),
    (12.0, 1.6, 6.0, 1.2, 1.6, 1.2),
    (15.0, 2.3, 6.0, 1.2, 2.3, 1.2),
    (-8.0, 0.75, -6.0, 1.5, 0.75, 1.5),
    (-12.0, 1.5, -6.0, 1.5, 1.5, 1.5),
    (0.0, 0.9, 10.0, 2.2, 0.9, 2.2),
];

pub const RAMP_STATE_STRIDE: u32 = 5;
/// (z_start, z_end, x_center, half_width, height)
pub type RampDef = (f32, f32, f32, f32, f32);
pub const RAMPS: &[RampDef] = &[(-8.0, -24.0, 0.0, 2.6, 3.0)];

pub const CAR_STATE_FLOATS: u32 = 8;
/// Index of the car health float (1.0 pristine → 0.0 destroyed) in the car block.
pub const CAR_HEALTH_OFFSET: u32 = 7;
pub const PLAYER_STATE_FLOATS: u32 = 5;
pub const HUD_STATE_FLOATS: u32 = 1;
pub const CAM_STATE_FLOATS: u32 = 4;
/// Environment block: [time_of_day 0..1) — 0 = midnight, 0.5 = noon.
pub const ENV_STATE_FLOATS: u32 = 1;
pub const PED_STATE_STRIDE: u32 = 16;
pub const PLAYER_STATE_OFFSET: u32 = CAR_STATE_FLOATS;
pub const HUD_STATE_OFFSET: u32 = PLAYER_STATE_OFFSET + PLAYER_STATE_FLOATS;
pub const CAM_STATE_OFFSET: u32 = HUD_STATE_OFFSET + HUD_STATE_FLOATS;
pub const ENV_STATE_OFFSET: u32 = CAM_STATE_OFFSET + CAM_STATE_FLOATS;
pub const PED_STATE_OFFSET: u32 = ENV_STATE_OFFSET + ENV_STATE_FLOATS;

/// One full day/night cycle in real seconds (24 game-min, PRD §6).
pub const DAY_CYCLE_SECONDS: f32 = 1440.0;
/// Boot time of day: 10:00 — bright morning light.
pub const DAY_START_FRACTION: f32 = 10.0 / 24.0;
pub const STATE_HEADER_INTS: u32 = 3;
pub const INPUT_HEADER_INTS: u32 = 2;
pub const INPUT_CAPACITY: u32 = 128;
pub const INPUT_STRIDE: u32 = 3;
