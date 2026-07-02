use wasm_bindgen::prelude::*;

use crate::constants::{BTN_BACK, BTN_FORWARD, BTN_HANDBRAKE, BTN_LEFT, BTN_RIGHT, BTN_RUN};
use crate::engine::SimEngine;

/// Decoded input state, updated once per worker tick from the input ring.
#[derive(Clone, Copy, Default)]
pub(crate) struct InputState {
    pub(crate) forward: f32,
    pub(crate) right: f32,
    pub(crate) run: f32,
    pub(crate) handbrake: f32,
    pub(crate) camera_yaw: f32,
}

#[wasm_bindgen]
impl SimEngine {
    pub fn set_input_buttons(&mut self, buttons: u32, camera_yaw: f32) {
        let forward: f32 = if (buttons & BTN_FORWARD) != 0 {
            1.0
        } else {
            0.0
        };
        let back: f32 = if (buttons & BTN_BACK) != 0 { 1.0 } else { 0.0 };
        let left: f32 = if (buttons & BTN_LEFT) != 0 { 1.0 } else { 0.0 };
        let right: f32 = if (buttons & BTN_RIGHT) != 0 { 1.0 } else { 0.0 };

        self.input.forward = (forward - back).clamp(-1.0, 1.0);
        self.input.right = (right - left).clamp(-1.0, 1.0);
        self.input.handbrake = if (buttons & BTN_HANDBRAKE) != 0 {
            1.0
        } else {
            0.0
        };
        self.input.run = if (buttons & BTN_RUN) != 0 { 1.0 } else { 0.0 };
        self.input.camera_yaw = camera_yaw;
    }
}
