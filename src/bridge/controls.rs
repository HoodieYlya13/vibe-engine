use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::engine::SimEngine;

#[wasm_bindgen]
impl SimEngine {
    pub fn reset_car(&mut self) {
        if let Some(body) = self.rigid_body_set.get_mut(self.vehicle.body_handle) {
            body.set_translation(Vector::new(0.0, 1.0, 0.0), true);
            body.set_rotation(self.vehicle.spawn_rotation, true);
            body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
            body.set_angvel(Vector::new(0.0, 0.0, 0.0), true);
            body.wake_up(true);
        }

        self.player.pos = Vector::new(2.0, 1.0, 0.0);
        self.player.yaw = 0.0;
        self.player.vel_y = 0.0;
        self.player.in_car = false;
        self.player.grounded = true;
        if let Some(collider) = self.collider_set.get_mut(self.player.collider_handle) {
            collider.set_enabled(true);
        }
        self.teleport_player_body();

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

    pub fn car_sleeping(&self) -> bool {
        self.rigid_body_set[self.vehicle.body_handle].is_sleeping()
    }

    pub fn set_allow_pause(&mut self, allow: bool) {
        self.runtime.allow_pause = allow;
        if !allow {
            self.runtime.paused = false;
        }
    }
}
