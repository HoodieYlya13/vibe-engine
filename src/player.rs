use rapier3d::control::{CharacterAutostep, CharacterLength, KinematicCharacterController};
use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::{
    CAR_HALF_WIDTH, PLAYER_JUMP_SPEED, PLAYER_RUN_SPEED, PLAYER_STAND_HEIGHT, PLAYER_STEP_DOWN,
    PLAYER_WALK_SPEED,
};
use crate::state::SimEngine;

/// Controller configuration for every on-foot move. The player collides with
/// the same colliders the car drives on — blocks, ramp, walls, car chassis —
/// so world geometry exists exactly once (was defect #5).
fn character_controller() -> KinematicCharacterController {
    KinematicCharacterController {
        autostep: Some(CharacterAutostep {
            max_height: CharacterLength::Absolute(PLAYER_STEP_DOWN),
            min_width: CharacterLength::Absolute(0.1),
            include_dynamic_bodies: false,
        }),
        snap_to_ground: Some(CharacterLength::Absolute(PLAYER_STEP_DOWN)),
        ..Default::default()
    }
}

impl SimEngine {
    pub(crate) fn update_player(&mut self, dt: f32) {
        if self.player.in_car {
            let car_pos = self.rigid_body_set[self.vehicle.body_handle].translation();
            self.player.pos = Vector::new(car_pos.x, car_pos.y + 0.9, car_pos.z);
            self.player.vel_y = 0.0;
            self.sync_player_body();
            return;
        }

        let speed = if self.input.run > 0.5 {
            PLAYER_RUN_SPEED
        } else {
            PLAYER_WALK_SPEED
        };
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

        self.player.vel_y -= 9.81 * dt;
        let desired = move_dir * speed * dt + Vector::new(0.0, self.player.vel_y * dt, 0.0);

        let movement = {
            let filter = QueryFilter::new()
                .exclude_rigid_body(self.player.body_handle)
                .exclude_sensors();
            let queries = self.broad_phase.as_query_pipeline(
                self.narrow_phase.query_dispatcher(),
                &self.rigid_body_set,
                &self.collider_set,
                filter,
            );
            let shape = self.collider_set[self.player.collider_handle].shape();
            character_controller().move_shape(
                dt,
                &queries,
                shape,
                &Pose::from_translation(self.player.pos),
                desired,
                |_| {},
            )
        };

        self.player.pos += movement.translation;
        self.player.grounded = movement.grounded;
        if movement.grounded {
            self.player.vel_y = 0.0;
        }
        self.sync_player_body();
    }

    fn sync_player_body(&mut self) {
        let pos = self.player.pos;
        if let Some(body) = self.rigid_body_set.get_mut(self.player.body_handle) {
            body.set_next_kinematic_translation(pos);
        }
    }

    fn set_player_collider_enabled(&mut self, enabled: bool) {
        if let Some(collider) = self.collider_set.get_mut(self.player.collider_handle) {
            collider.set_enabled(enabled);
        }
    }
}

#[wasm_bindgen]
impl SimEngine {
    pub fn jump(&mut self) {
        if self.player.in_car || !self.player.grounded {
            return;
        }
        self.player.vel_y = PLAYER_JUMP_SPEED;
        self.player.grounded = false;
    }

    pub fn toggle_enter(&mut self) {
        let car_pos = self.rigid_body_set[self.vehicle.body_handle].translation();
        let car_pos = Vector::new(car_pos.x, car_pos.y, car_pos.z);

        if self.player.in_car {
            self.player.pos = Vector::new(
                car_pos.x + CAR_HALF_WIDTH + 1.0,
                PLAYER_STAND_HEIGHT,
                car_pos.z,
            );
            self.player.in_car = false;
            self.player.vel_y = 0.0;
            self.player.grounded = true;
            self.set_player_collider_enabled(true);
            self.teleport_player_body();
            return;
        }

        let dx = self.player.pos.x - car_pos.x;
        let dz = self.player.pos.z - car_pos.z;
        let enter_radius = 6.0;
        if dx * dx + dz * dz <= enter_radius * enter_radius {
            self.player.in_car = true;
            self.player.pos = car_pos;
            self.set_player_collider_enabled(false);
            self.teleport_player_body();
        }
    }
}

impl SimEngine {
    pub(crate) fn teleport_player_body(&mut self) {
        let pos = self.player.pos;
        if let Some(body) = self.rigid_body_set.get_mut(self.player.body_handle) {
            body.set_translation(pos, true);
        }
    }
}
