use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::{
    BLOCKS, CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, PLAYER_FOOT_RADIUS,
    PLAYER_JUMP_SPEED, PLAYER_RADIUS, PLAYER_STAND_HEIGHT, PLAYER_STEP_DOWN, RAMPS, WORLD_HALF,
};
use crate::state::SimEngine;

impl SimEngine {
    pub(crate) fn update_player(&mut self, dt: f32) {
        if self.player.in_car {
            let car_pos = self.rigid_body_set[self.vehicle.body_handle].translation();
            self.player.pos = Vector::new(car_pos.x, car_pos.y + 0.9, car_pos.z);
            self.player.vel_y = 0.0;
            return;
        }

        let speed = if self.input.run > 0.5 { 7.0 } else { 4.0 };
        let yaw = self.input.camera_yaw;
        let forward = Vector::new(yaw.sin(), 0.0, yaw.cos());
        let right = Vector::new(forward.z, 0.0, -forward.x);
        let mut move_dir = forward * self.input.forward + right * self.input.right;
        let len = move_dir.length();
        if len > 1.0 {
            move_dir /= len;
        }

        if len > 0.001 {
            self.player.yaw = move_dir.x.atan2(move_dir.z);
        }

        self.player.pos += move_dir * speed * dt;
        self.resolve_block_horizontal_collisions();
        self.resolve_ramp_horizontal_collisions();

        let prev_y = self.player.pos.y;
        self.player.vel_y += -9.81 * dt;
        self.player.pos.y += self.player.vel_y * dt;

        let mut grounded_at = PLAYER_STAND_HEIGHT;
        if let Some(block_ground) = self.block_ground_height_at(
            self.player.pos.x,
            self.player.pos.z,
            prev_y,
            self.player.pos.y,
        ) && block_ground > grounded_at
        {
            grounded_at = block_ground;
        }
        if let Some(car_ground) = self.car_ground_height_at(
            self.player.pos.x,
            self.player.pos.z,
            prev_y,
            self.player.pos.y,
        ) && car_ground > grounded_at
        {
            grounded_at = car_ground;
        }
        if let Some(ramp_ground) = self.ramp_ground_height_at(
            self.player.pos.x,
            self.player.pos.z,
            prev_y,
            self.player.pos.y,
        ) && ramp_ground > grounded_at
        {
            grounded_at = ramp_ground;
        }

        if self.player.pos.y <= grounded_at {
            self.player.pos.y = grounded_at;
            self.player.vel_y = 0.0;
        }

        let max_bound = WORLD_HALF - PLAYER_RADIUS;
        self.player.pos.x = self.player.pos.x.clamp(-max_bound, max_bound);
        self.player.pos.z = self.player.pos.z.clamp(-max_bound, max_bound);
    }

    fn resolve_block_horizontal_collisions(&mut self) {
        for block in BLOCKS {
            let min_y = block.1 - block.4;
            let top = block.1 + block.4;
            let max_y = top + PLAYER_STAND_HEIGHT - 0.05;
            if self.player.pos.y < min_y || self.player.pos.y > max_y {
                continue;
            }

            let dx = self.player.pos.x - block.0;
            let dz = self.player.pos.z - block.2;
            let overlap_x = block.3 + PLAYER_FOOT_RADIUS - dx.abs();
            let overlap_z = block.5 + PLAYER_FOOT_RADIUS - dz.abs();
            if overlap_x <= 0.0 || overlap_z <= 0.0 {
                continue;
            }

            if overlap_x < overlap_z {
                self.player.pos.x += if dx >= 0.0 { overlap_x } else { -overlap_x };
            } else {
                self.player.pos.z += if dz >= 0.0 { overlap_z } else { -overlap_z };
            }
        }
    }

    fn resolve_ramp_horizontal_collisions(&mut self) {
        let radius = PLAYER_FOOT_RADIUS;
        for ramp in RAMPS {
            let x0 = ramp.0;
            let x1 = ramp.1;
            let z0 = ramp.2 - ramp.3;
            let z1 = ramp.2 + ramp.3;
            let h = ramp.4;

            let run = (x1 - x0).max(0.001);
            let t = ((self.player.pos.x - x0) / run).clamp(0.0, 1.0);
            let stand_at_x = t * h + PLAYER_STAND_HEIGHT;
            if self.player.pos.y > stand_at_x + 0.2 {
                continue;
            }

            // Ramp side walls.
            if self.player.pos.x >= x0 - radius && self.player.pos.x <= x1 + radius {
                if self.player.pos.z > z1 && self.player.pos.z < z1 + radius {
                    self.player.pos.z = z1 + radius;
                } else if self.player.pos.z < z0 && self.player.pos.z > z0 - radius {
                    self.player.pos.z = z0 - radius;
                }
            }

            // Ramp back wall at the high edge.
            if self.player.pos.z >= z0 - radius
                && self.player.pos.z <= z1 + radius
                && self.player.pos.x > x1
                && self.player.pos.x < x1 + radius
                && self.player.pos.y <= h + PLAYER_STAND_HEIGHT + 0.2
            {
                self.player.pos.x = x1 + radius;
            }
        }
    }

    fn block_ground_height_at(
        &self,
        x: f32,
        z: f32,
        prev_center_y: f32,
        current_center_y: f32,
    ) -> Option<f32> {
        let mut best: Option<f32> = None;
        for block in BLOCKS {
            let top = block.1 + block.4;
            let stand = top + PLAYER_STAND_HEIGHT;
            if x < block.0 - block.3 - PLAYER_FOOT_RADIUS
                || x > block.0 + block.3 + PLAYER_FOOT_RADIUS
                || z < block.2 - block.5 - PLAYER_FOOT_RADIUS
                || z > block.2 + block.5 + PLAYER_FOOT_RADIUS
            {
                continue;
            }
            if prev_center_y < stand - PLAYER_STEP_DOWN
                || current_center_y > stand + 0.05
                || self.player.vel_y > 0.0
            {
                continue;
            }
            best = Some(best.map_or(stand, |v| v.max(stand)));
        }
        best
    }

    fn car_ground_height_at(
        &self,
        x: f32,
        z: f32,
        prev_center_y: f32,
        current_center_y: f32,
    ) -> Option<f32> {
        let body = &self.rigid_body_set[self.vehicle.body_handle];
        let rot = body.rotation();
        let up = rot * Vector::new(0.0, 1.0, 0.0);
        if up.y <= 0.35 {
            return None;
        }

        let center = body.translation();
        let center_v = Vector::new(center.x, center.y, center.z);
        let right = rot * Vector::new(1.0, 0.0, 0.0);
        let forward = rot * Vector::new(0.0, 0.0, 1.0);

        let top_center = center_v + up * CAR_HALF_HEIGHT;
        let denom = up.y.max(0.0001);
        let top_y = top_center.y - (up.x * (x - top_center.x) + up.z * (z - top_center.z)) / denom;
        let stand = top_y + PLAYER_STAND_HEIGHT;
        if prev_center_y < stand - PLAYER_STEP_DOWN
            || current_center_y > stand + 0.05
            || self.player.vel_y > 0.0
        {
            return None;
        }

        let delta = Vector::new(x - center_v.x, top_y - center_v.y, z - center_v.z);
        let local_x = delta.dot(right);
        let local_z = delta.dot(forward);
        if local_x.abs() > CAR_HALF_WIDTH + PLAYER_FOOT_RADIUS
            || local_z.abs() > CAR_HALF_LENGTH + PLAYER_FOOT_RADIUS
        {
            return None;
        }

        Some(stand)
    }

    fn ramp_ground_height_at(
        &self,
        x: f32,
        z: f32,
        prev_center_y: f32,
        current_center_y: f32,
    ) -> Option<f32> {
        let mut best: Option<f32> = None;
        for ramp in RAMPS {
            let x0 = ramp.0;
            let x1 = ramp.1;
            let z0 = ramp.2 - ramp.3;
            let z1 = ramp.2 + ramp.3;
            let h = ramp.4;

            if x < x0 || x > x1 || z < z0 || z > z1 {
                continue;
            }

            let run = (x1 - x0).max(0.001);
            let t = ((x - x0) / run).clamp(0.0, 1.0);
            let stand = t * h + PLAYER_STAND_HEIGHT;

            if prev_center_y < stand - PLAYER_STEP_DOWN
                || current_center_y > stand + 0.05
                || self.player.vel_y > 0.0
            {
                continue;
            }

            best = Some(best.map_or(stand, |v| v.max(stand)));
        }

        best
    }
}

#[wasm_bindgen]
impl SimEngine {
    pub fn jump(&mut self) {
        if self.player.in_car || self.player.vel_y.abs() > 0.001 {
            return;
        }

        if self.player.pos.y <= PLAYER_STAND_HEIGHT + 0.05 {
            self.player.vel_y = PLAYER_JUMP_SPEED;
            return;
        }

        if let Some(block_ground) = self.block_ground_height_at(
            self.player.pos.x,
            self.player.pos.z,
            self.player.pos.y,
            self.player.pos.y,
        ) && (self.player.pos.y - block_ground).abs() <= 0.08
        {
            self.player.vel_y = PLAYER_JUMP_SPEED;
            return;
        }

        if let Some(car_ground) = self.car_ground_height_at(
            self.player.pos.x,
            self.player.pos.z,
            self.player.pos.y,
            self.player.pos.y,
        ) && (self.player.pos.y - car_ground).abs() <= 0.08
        {
            self.player.vel_y = PLAYER_JUMP_SPEED;
            return;
        }

        if let Some(ramp_ground) = self.ramp_ground_height_at(
            self.player.pos.x,
            self.player.pos.z,
            self.player.pos.y,
            self.player.pos.y,
        ) && (self.player.pos.y - ramp_ground).abs() <= 0.08
        {
            self.player.vel_y = PLAYER_JUMP_SPEED;
        }
    }

    pub fn toggle_enter(&mut self) {
        let car_pos = self.rigid_body_set[self.vehicle.body_handle].translation();
        if self.player.in_car {
            self.player.pos = Vector::new(
                car_pos.x + CAR_HALF_WIDTH + 1.0,
                PLAYER_STAND_HEIGHT,
                car_pos.z,
            );
            self.player.in_car = false;
            return;
        }

        let dx = self.player.pos.x - car_pos.x;
        let dz = self.player.pos.z - car_pos.z;
        let enter_radius = 6.0;
        if dx * dx + dz * dz <= enter_radius * enter_radius {
            self.player.in_car = true;
            self.player.pos = Vector::new(car_pos.x, car_pos.y, car_pos.z);
        }
    }
}
