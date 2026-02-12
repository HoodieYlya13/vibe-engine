use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::{BTN_BACK, BTN_FORWARD, BTN_HANDBRAKE, BTN_LEFT, BTN_RIGHT, BTN_RUN};
use crate::state::SimEngine;

#[wasm_bindgen]
impl SimEngine {
    pub fn reset_car(&mut self) {
        if let Some(body) = self.rigid_body_set.get_mut(self.vehicle.body_handle) {
            body.set_translation(Vector::new(0.0, 1.0, 0.0), true);
            body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
            body.set_angvel(Vector::new(0.0, 0.0, 0.0), true);
            body.wake_up(true);
        }

        self.player.pos = Vector::new(2.0, 1.0, 0.0);
        self.player.yaw = 0.0;
        self.player.vel_y = 0.0;
        self.player.in_car = false;

        for wheel in self.vehicle.controller.wheels_mut() {
            wheel.engine_force = 0.0;
            wheel.brake = 0.0;
            wheel.steering = 0.0;
            wheel.rotation = 0.0;
        }
    }

    pub fn toggle_pause(&mut self) {
        if self.runtime.allow_pause {
            self.runtime.paused = !self.runtime.paused;
        }
    }

    pub fn is_paused(&self) -> bool {
        self.runtime.paused
    }

    pub fn set_allow_pause(&mut self, allow: bool) {
        self.runtime.allow_pause = allow;
        if !allow {
            self.runtime.paused = false;
        }
    }

    pub fn set_input_buttons(&mut self, buttons: u32, camera_yaw: f32) {
        let forward = if (buttons & BTN_FORWARD) != 0 {
            1.0
        } else {
            0.0
        };
        let back = if (buttons & BTN_BACK) != 0 { 1.0 } else { 0.0 };
        let left = if (buttons & BTN_LEFT) != 0 { 1.0 } else { 0.0 };
        let right = if (buttons & BTN_RIGHT) != 0 { 1.0 } else { 0.0 };

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
