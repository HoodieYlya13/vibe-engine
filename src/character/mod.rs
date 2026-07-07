use rapier3d::control::{CharacterAutostep, CharacterLength, KinematicCharacterController};
use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::{
    CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, ENTER_RADIUS, PLAYER_CAPSULE_HALF_HEIGHT,
    PLAYER_FOOT_RADIUS, PLAYER_JUMP_SPEED, PLAYER_RUN_SPEED, PLAYER_STAND_HEIGHT, PLAYER_STEP_DOWN,
    PLAYER_WALK_SPEED,
};
use crate::engine::SimEngine;

/// Player capsule center rests this far outside the car's flank — the "door"
/// (also the side-exit distance).
const DOOR_OFFSET: f32 = CAR_HALF_WIDTH + 0.6;
/// Within this horizontal distance of the door, entering is immediate.
const DOOR_REACH: f32 = 0.6;
/// The enter auto-walk gives up after this long (blocked path).
const ENTER_TIMEOUT: f32 = 2.5;

#[derive(Clone, Copy)]
pub(crate) struct PlayerState {
    pub(crate) pos: Vector,
    pub(crate) yaw: f32,
    pub(crate) vel_y: f32,
    pub(crate) in_car: bool,
    /// Walk-to-door in progress (defect #10: enter used to teleport).
    pub(crate) entering: bool,
    pub(crate) enter_timer: f32,
    pub(crate) grounded: bool,
    /// Kinematic body carrying the capsule collider; positions are driven by
    /// the character controller, never by the solver.
    pub(crate) body_handle: RigidBodyHandle,
    pub(crate) collider_handle: ColliderHandle,
}

/// The player: a kinematic body + sensor capsule moved by the character
/// controller. Sensor, so the solver never generates contacts against it
/// (the car should not crash into an immovable pole).
pub(crate) fn spawn_player(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
) -> PlayerState {
    let spawn = Vector::new(2.0, PLAYER_STAND_HEIGHT, 0.0);
    let body = RigidBodyBuilder::kinematic_position_based()
        .translation(spawn)
        .build();
    let body_handle = rigid_body_set.insert(body);
    let collider = ColliderBuilder::capsule_y(PLAYER_CAPSULE_HALF_HEIGHT, PLAYER_FOOT_RADIUS)
        .sensor(true)
        .build();
    let collider_handle = collider_set.insert_with_parent(collider, body_handle, rigid_body_set);

    PlayerState {
        pos: spawn,
        yaw: 0.0,
        vel_y: 0.0,
        in_car: false,
        entering: false,
        enter_timer: 0.0,
        grounded: true,
        body_handle,
        collider_handle,
    }
}

/// Horizontal unit vector of a rotated local axis; falls back to the local
/// axis itself when the car points straight up/down (flipped mid-air).
fn horizontal_axis(rotation: Rotation, local: Vector) -> Vector {
    let v = rotation * local;
    let flat = Vector::new(v.x, 0.0, v.z);
    let len = flat.length();
    if len < 1e-3 { local } else { flat / len }
}

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

        if self.player.entering {
            self.update_enter_walk(dt);
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
        self.move_capsule(desired, dt);
    }

    /// Walk-to-door (defect #10): the enter action jogs the player to the
    /// nearest door through the normal character controller — same collisions,
    /// no teleport — and seats them on arrival. Movement input takes back
    /// control; a timeout or the car driving off cancels.
    fn update_enter_walk(&mut self, dt: f32) {
        if self.input.forward.abs() > 0.1 || self.input.right.abs() > 0.1 {
            self.player.entering = false;
            return;
        }
        self.player.enter_timer -= dt;
        let car_pos = self.rigid_body_set[self.vehicle.body_handle].translation();
        let dx = self.player.pos.x - car_pos.x;
        let dz = self.player.pos.z - car_pos.z;
        let give_up = ENTER_RADIUS + 2.0;
        if self.player.enter_timer <= 0.0 || dx * dx + dz * dz > give_up * give_up {
            self.player.entering = false;
            return;
        }

        let door = self.nearest_door();
        let to_door = Vector::new(door.x - self.player.pos.x, 0.0, door.z - self.player.pos.z);
        let dist = to_door.length();
        if dist <= DOOR_REACH {
            self.seat_player();
            return;
        }

        let dir = to_door / dist;
        self.player.yaw = dir.x.atan2(dir.z);
        self.player.vel_y -= 9.81 * dt;
        // Jog, but never overshoot the door in one step.
        let step = (PLAYER_RUN_SPEED * dt).min(dist);
        let desired = dir * step + Vector::new(0.0, self.player.vel_y * dt, 0.0);
        self.move_capsule(desired, dt);
    }

    /// One character-controller move against the shared world colliders.
    fn move_capsule(&mut self, desired: Vector, dt: f32) {
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

    /// The door point nearest the player: the car's flank on the player's
    /// side, in the car's actual (horizontal) orientation.
    fn nearest_door(&self) -> Vector {
        let body = &self.rigid_body_set[self.vehicle.body_handle];
        let car_pos = body.translation();
        let right = horizontal_axis(*body.rotation(), Vector::X);
        let to_player = self.player.pos - car_pos;
        let side = if to_player.dot(right) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        car_pos + right * (side * DOOR_OFFSET)
    }

    fn seat_player(&mut self) {
        let car_pos = self.rigid_body_set[self.vehicle.body_handle].translation();
        self.player.entering = false;
        self.player.in_car = true;
        self.player.pos = car_pos;
        self.player.vel_y = 0.0;
        self.set_player_collider_enabled(false);
        self.teleport_player_body();
    }

    /// True when the player capsule, centered at `center`, overlaps nothing.
    fn capsule_fits(&self, center: Vector) -> bool {
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
        queries
            .intersect_shape(Pose::from_translation(center), shape)
            .next()
            .is_none()
    }

    /// Exit toward the first free spot — driver door, passenger door, behind
    /// the car, then the roof (defect #10: exit used to always eject +X, even
    /// into a wall). All candidates follow the car's actual pose. If
    /// everything is blocked, stay seated.
    fn try_exit_car(&mut self) {
        let body = &self.rigid_body_set[self.vehicle.body_handle];
        let car_pos = body.translation();
        let rotation = *body.rotation();
        let right = horizontal_axis(rotation, Vector::X);
        let forward = horizontal_axis(rotation, Vector::Z);
        let candidates = [
            car_pos - right * DOOR_OFFSET,
            car_pos + right * DOOR_OFFSET,
            car_pos - forward * (CAR_HALF_LENGTH + 1.0),
            car_pos + Vector::Y * (CAR_HALF_HEIGHT + 1.0),
        ];
        // Slightly above the chassis center line so the capsule clears the
        // ground on flat poses; the controller settles it next step.
        let Some(spot) = candidates
            .into_iter()
            .map(|c| c + Vector::Y * 0.1)
            .find(|c| self.capsule_fits(*c))
        else {
            return;
        };

        self.player.pos = spot;
        self.player.in_car = false;
        self.player.vel_y = 0.0;
        self.player.grounded = false;
        let out = Vector::new(spot.x - car_pos.x, 0.0, spot.z - car_pos.z);
        if out.length() > 1e-3 {
            self.player.yaw = out.x.atan2(out.z);
        }
        self.set_player_collider_enabled(true);
        self.teleport_player_body();
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
        if self.player.in_car {
            self.try_exit_car();
            return;
        }
        if self.player.entering {
            // E again cancels the walk-to-door.
            self.player.entering = false;
            return;
        }

        let car_pos = self.rigid_body_set[self.vehicle.body_handle].translation();
        let dx = self.player.pos.x - car_pos.x;
        let dz = self.player.pos.z - car_pos.z;
        if dx * dx + dz * dz > ENTER_RADIUS * ENTER_RADIUS {
            return;
        }

        let door = self.nearest_door();
        let ddx = door.x - self.player.pos.x;
        let ddz = door.z - self.player.pos.z;
        if ddx * ddx + ddz * ddz <= DOOR_REACH * DOOR_REACH {
            // Already at the door: get straight in.
            self.seat_player();
        } else {
            self.player.entering = true;
            self.player.enter_timer = ENTER_TIMEOUT;
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
