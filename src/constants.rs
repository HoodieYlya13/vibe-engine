use wasm_bindgen::prelude::*;

pub(crate) const DEFAULT_PED_COUNT: u32 = 1500;
pub(crate) const BTN_FORWARD: u32 = 1 << 0;
pub(crate) const BTN_BACK: u32 = 1 << 1;
pub(crate) const BTN_LEFT: u32 = 1 << 2;
pub(crate) const BTN_RIGHT: u32 = 1 << 3;
pub(crate) const BTN_RUN: u32 = 1 << 4;
pub(crate) const BTN_HANDBRAKE: u32 = 1 << 5;
pub(crate) const ACTION_JUMP: u32 = 1 << 0;
pub(crate) const ACTION_ENTER: u32 = 1 << 1;
pub(crate) const ACTION_RESET: u32 = 1 << 2;
pub(crate) const ACTION_PAUSE: u32 = 1 << 3;
pub(crate) const HUD_HINT_NONE: u32 = 0;
pub(crate) const HUD_HINT_ENTER: u32 = 1;
pub(crate) const HUD_HINT_EXIT: u32 = 2;

pub(crate) const VEHICLE_MAX_ENGINE_FORCE: f32 = 5200.0;
pub(crate) const VEHICLE_MAX_BRAKE_FORCE: f32 = 45.0;
pub(crate) const VEHICLE_MAX_STEER: f32 = 0.6;

pub(crate) const WORLD_HALF: f32 = 60.0;
pub(crate) const WALL_THICKNESS: f32 = 1.5;
pub(crate) const WALL_HEIGHT: f32 = 6.0;

pub(crate) const PLAYER_RADIUS: f32 = 0.6;
pub(crate) const PLAYER_STAND_HEIGHT: f32 = 1.0;
pub(crate) const PLAYER_FOOT_RADIUS: f32 = 0.35;
pub(crate) const PLAYER_STEP_DOWN: f32 = 0.35;
pub(crate) const PLAYER_JUMP_SPEED: f32 = 6.2;

pub(crate) const CAR_HALF_WIDTH: f32 = 1.8;
pub(crate) const CAR_HALF_HEIGHT: f32 = 0.8;
pub(crate) const CAR_HALF_LENGTH: f32 = 3.2;

pub(crate) const BLOCK_STATE_STRIDE: u32 = 6;
pub(crate) type BlockDef = (f32, f32, f32, f32, f32, f32);
pub(crate) const BLOCKS: &[BlockDef] = &[
    (6.0, 0.5, 6.0, 1.2, 0.5, 1.2),
    (9.0, 1.0, 6.0, 1.2, 1.0, 1.2),
    (12.0, 1.6, 6.0, 1.2, 1.6, 1.2),
    (15.0, 2.3, 6.0, 1.2, 2.3, 1.2),
    (-8.0, 0.75, -6.0, 1.5, 0.75, 1.5),
    (-12.0, 1.5, -6.0, 1.5, 1.5, 1.5),
    (0.0, 0.9, 10.0, 2.2, 0.9, 2.2),
];

pub(crate) const RAMP_STATE_STRIDE: u32 = 5;
pub(crate) type RampDef = (f32, f32, f32, f32, f32);
// (x_start, x_end, z_center, half_width, height)
pub(crate) const RAMPS: &[RampDef] = &[(14.0, 30.0, -16.0, 2.6, 3.0)];

pub(crate) const CAR_STATE_FLOATS: u32 = 7;
pub(crate) const PLAYER_STATE_FLOATS: u32 = 5;
pub(crate) const HUD_STATE_FLOATS: u32 = 1;
pub(crate) const CAM_STATE_FLOATS: u32 = 4;
pub(crate) const PED_STATE_STRIDE: u32 = 16;
pub(crate) const PLAYER_STATE_OFFSET: u32 = CAR_STATE_FLOATS;
pub(crate) const HUD_STATE_OFFSET: u32 = PLAYER_STATE_OFFSET + PLAYER_STATE_FLOATS;
pub(crate) const CAM_STATE_OFFSET: u32 = HUD_STATE_OFFSET + HUD_STATE_FLOATS;
pub(crate) const PED_STATE_OFFSET: u32 = CAM_STATE_OFFSET + CAM_STATE_FLOATS;
pub(crate) const STATE_HEADER_INTS: u32 = 3;
pub(crate) const INPUT_HEADER_INTS: u32 = 2;
pub(crate) const INPUT_CAPACITY: u32 = 128;
pub(crate) const INPUT_STRIDE: u32 = 3;

#[wasm_bindgen]
pub fn default_ped_count() -> u32 {
    DEFAULT_PED_COUNT
}

#[wasm_bindgen]
pub fn btn_forward() -> u32 {
    BTN_FORWARD
}

#[wasm_bindgen]
pub fn btn_back() -> u32 {
    BTN_BACK
}

#[wasm_bindgen]
pub fn btn_left() -> u32 {
    BTN_LEFT
}

#[wasm_bindgen]
pub fn btn_right() -> u32 {
    BTN_RIGHT
}

#[wasm_bindgen]
pub fn btn_run() -> u32 {
    BTN_RUN
}

#[wasm_bindgen]
pub fn btn_handbrake() -> u32 {
    BTN_HANDBRAKE
}

#[wasm_bindgen]
pub fn action_jump() -> u32 {
    ACTION_JUMP
}

#[wasm_bindgen]
pub fn action_enter() -> u32 {
    ACTION_ENTER
}

#[wasm_bindgen]
pub fn action_reset() -> u32 {
    ACTION_RESET
}

#[wasm_bindgen]
pub fn action_pause() -> u32 {
    ACTION_PAUSE
}

#[wasm_bindgen]
pub fn hud_hint_none() -> u32 {
    HUD_HINT_NONE
}

#[wasm_bindgen]
pub fn hud_hint_enter() -> u32 {
    HUD_HINT_ENTER
}

#[wasm_bindgen]
pub fn hud_hint_exit() -> u32 {
    HUD_HINT_EXIT
}

#[wasm_bindgen]
pub fn car_state_floats() -> u32 {
    CAR_STATE_FLOATS
}

#[wasm_bindgen]
pub fn player_state_floats() -> u32 {
    PLAYER_STATE_FLOATS
}

#[wasm_bindgen]
pub fn player_state_offset() -> u32 {
    PLAYER_STATE_OFFSET
}

#[wasm_bindgen]
pub fn hud_state_floats() -> u32 {
    HUD_STATE_FLOATS
}

#[wasm_bindgen]
pub fn hud_state_offset() -> u32 {
    HUD_STATE_OFFSET
}

#[wasm_bindgen]
pub fn cam_state_floats() -> u32 {
    CAM_STATE_FLOATS
}

#[wasm_bindgen]
pub fn cam_state_offset() -> u32 {
    CAM_STATE_OFFSET
}

#[wasm_bindgen]
pub fn ped_state_offset() -> u32 {
    PED_STATE_OFFSET
}

#[wasm_bindgen]
pub fn ped_state_stride() -> u32 {
    PED_STATE_STRIDE
}

#[wasm_bindgen]
pub fn state_header_ints() -> u32 {
    STATE_HEADER_INTS
}

#[wasm_bindgen]
pub fn input_header_ints() -> u32 {
    INPUT_HEADER_INTS
}

#[wasm_bindgen]
pub fn input_capacity() -> u32 {
    INPUT_CAPACITY
}

#[wasm_bindgen]
pub fn input_stride() -> u32 {
    INPUT_STRIDE
}

#[wasm_bindgen]
pub fn block_count() -> u32 {
    BLOCKS.len() as u32
}

#[wasm_bindgen]
pub fn block_state_stride() -> u32 {
    BLOCK_STATE_STRIDE
}

#[wasm_bindgen]
pub fn write_blocks(out: &mut [f32]) {
    let needed = BLOCKS.len() * BLOCK_STATE_STRIDE as usize;
    if out.len() < needed {
        return;
    }
    let mut cursor = 0usize;
    for block in BLOCKS {
        out[cursor] = block.0;
        out[cursor + 1] = block.1;
        out[cursor + 2] = block.2;
        out[cursor + 3] = block.3;
        out[cursor + 4] = block.4;
        out[cursor + 5] = block.5;
        cursor += BLOCK_STATE_STRIDE as usize;
    }
}

#[wasm_bindgen]
pub fn ramp_count() -> u32 {
    RAMPS.len() as u32
}

#[wasm_bindgen]
pub fn ramp_state_stride() -> u32 {
    RAMP_STATE_STRIDE
}

#[wasm_bindgen]
pub fn write_ramps(out: &mut [f32]) {
    let needed = RAMPS.len() * RAMP_STATE_STRIDE as usize;
    if out.len() < needed {
        return;
    }
    let mut cursor = 0usize;
    for ramp in RAMPS {
        out[cursor] = ramp.0;
        out[cursor + 1] = ramp.1;
        out[cursor + 2] = ramp.2;
        out[cursor + 3] = ramp.3;
        out[cursor + 4] = ramp.4;
        cursor += RAMP_STATE_STRIDE as usize;
    }
}
