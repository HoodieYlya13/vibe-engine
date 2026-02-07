use wasm_bindgen::prelude::*;
use rapier3d::prelude::*;
use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};

const DEFAULT_PED_COUNT: u32 = 1500;
const BTN_FORWARD: u32 = 1 << 0;
const BTN_BACK: u32 = 1 << 1;
const BTN_LEFT: u32 = 1 << 2;
const BTN_RIGHT: u32 = 1 << 3;
const BTN_RUN: u32 = 1 << 4;
const BTN_HANDBRAKE: u32 = 1 << 5;
const ACTION_JUMP: u32 = 1 << 0;
const ACTION_ENTER: u32 = 1 << 1;
const ACTION_RESET: u32 = 1 << 2;
const ACTION_PAUSE: u32 = 1 << 3;
const HUD_HINT_NONE: u32 = 0;
const HUD_HINT_ENTER: u32 = 1;
const HUD_HINT_EXIT: u32 = 2;
const VEHICLE_MAX_ENGINE_FORCE: f32 = 2200.0;
const VEHICLE_MAX_BRAKE_FORCE: f32 = 45.0;
const VEHICLE_MAX_STEER: f32 = 0.6;
const WORLD_HALF: f32 = 60.0;
const WALL_THICKNESS: f32 = 1.5;
const WALL_HEIGHT: f32 = 6.0;
const PLAYER_RADIUS: f32 = 0.6;
const CAR_STATE_FLOATS: u32 = 7;
const PLAYER_STATE_FLOATS: u32 = 5;
const HUD_STATE_FLOATS: u32 = 1;
const CAM_STATE_FLOATS: u32 = 4;
const PED_STATE_STRIDE: u32 = 16;
const PLAYER_STATE_OFFSET: u32 = CAR_STATE_FLOATS;
const HUD_STATE_OFFSET: u32 = PLAYER_STATE_OFFSET + PLAYER_STATE_FLOATS;
const CAM_STATE_OFFSET: u32 = HUD_STATE_OFFSET + HUD_STATE_FLOATS;
const PED_STATE_OFFSET: u32 = CAM_STATE_OFFSET + CAM_STATE_FLOATS;
const STATE_HEADER_INTS: u32 = 3;
const INPUT_HEADER_INTS: u32 = 2;
const INPUT_CAPACITY: u32 = 128;
const INPUT_STRIDE: u32 = 3;

#[wasm_bindgen]
pub struct SimEngine {
    pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    gravity: Vector,
    integration_parameters: IntegrationParameters,
    car_handle: RigidBodyHandle,
    vehicle: DynamicRayCastVehicleController,
    input_handbrake: f32,
    input_forward: f32,
    input_right: f32,
    input_run: f32,
    input_camera_yaw: f32,
    player_pos: Vector,
    player_yaw: f32,
    player_vel_y: f32,
    in_car: bool,
    paused: bool,
    allow_pause: bool,
    sim_time: f32,
    ped_positions: Vec<Vector>,
}

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
impl SimEngine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> SimEngine {
        SimEngine::new_with_peds(DEFAULT_PED_COUNT)
    }

    pub fn new_with_peds(count: u32) -> SimEngine {
        console_error_panic_hook::set_once();

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        let ground_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(0.0, -0.5, 0.0))
            .build();
        let ground_collider =
            ColliderBuilder::cuboid(WORLD_HALF, 0.5, WORLD_HALF).build();
        let ground_handle = rigid_body_set.insert(ground_body);
        collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

        let wall_body = RigidBodyBuilder::fixed().build();
        let wall_handle = rigid_body_set.insert(wall_body);
        let wall_x = WORLD_HALF + WALL_THICKNESS;
        let wall_z = WORLD_HALF + WALL_THICKNESS;
        let wall_depth = WORLD_HALF + WALL_THICKNESS;
        let wall_height = WALL_HEIGHT;
        let wall_thickness = WALL_THICKNESS;
        let wall_x_pos = ColliderBuilder::cuboid(wall_thickness, wall_height, wall_depth)
            .translation(Vector::new(wall_x, wall_height, 0.0))
            .build();
        let wall_x_neg = ColliderBuilder::cuboid(wall_thickness, wall_height, wall_depth)
            .translation(Vector::new(-wall_x, wall_height, 0.0))
            .build();
        let wall_z_pos = ColliderBuilder::cuboid(wall_depth, wall_height, wall_thickness)
            .translation(Vector::new(0.0, wall_height, wall_z))
            .build();
        let wall_z_neg = ColliderBuilder::cuboid(wall_depth, wall_height, wall_thickness)
            .translation(Vector::new(0.0, wall_height, -wall_z))
            .build();
        collider_set.insert_with_parent(wall_x_pos, wall_handle, &mut rigid_body_set);
        collider_set.insert_with_parent(wall_x_neg, wall_handle, &mut rigid_body_set);
        collider_set.insert_with_parent(wall_z_pos, wall_handle, &mut rigid_body_set);
        collider_set.insert_with_parent(wall_z_neg, wall_handle, &mut rigid_body_set);

        let car_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 1.0, 0.0))
            .additional_mass(1200.0)
            .linear_damping(0.4)
            .angular_damping(1.2)
            .build();
        let car_collider = ColliderBuilder::cuboid(0.9, 0.4, 1.6)
            .restitution(0.1)
            .friction(1.2)
            .build();
        let car_handle = rigid_body_set.insert(car_body);
        collider_set.insert_with_parent(car_collider, car_handle, &mut rigid_body_set);

        let mut vehicle = DynamicRayCastVehicleController::new(car_handle);
        vehicle.index_up_axis = 1;
        vehicle.index_forward_axis = 2;
        let wheel_radius = 0.38;
        let suspension_rest = 0.45;
        let tuning = WheelTuning {
            suspension_stiffness: 12.0,
            suspension_compression: 3.0,
            suspension_damping: 2.0,
            max_suspension_travel: 0.35,
            side_friction_stiffness: 1.2,
            friction_slip: 8.0,
            max_suspension_force: 12000.0,
        };
        let half_width = 0.9;
        let half_length = 1.3;
        let connection_height = -0.25;
        let direction = Vector::new(0.0, -1.0, 0.0);
        let axle = Vector::new(1.0, 0.0, 0.0);
        vehicle.add_wheel(
            Vector::new(-half_width, connection_height, half_length),
            direction,
            axle,
            suspension_rest,
            wheel_radius,
            &tuning,
        );
        vehicle.add_wheel(
            Vector::new(half_width, connection_height, half_length),
            direction,
            axle,
            suspension_rest,
            wheel_radius,
            &tuning,
        );
        vehicle.add_wheel(
            Vector::new(-half_width, connection_height, -half_length),
            direction,
            axle,
            suspension_rest,
            wheel_radius,
            &tuning,
        );
        vehicle.add_wheel(
            Vector::new(half_width, connection_height, -half_length),
            direction,
            axle,
            suspension_rest,
            wheel_radius,
            &tuning,
        );

        let ped_count = count as usize;
        let mut ped_positions = Vec::with_capacity(ped_count);
        let cols = (ped_count as f32).sqrt().ceil() as usize;
        let spacing = 2.5;
        let half = cols as f32 * 0.5;
        for i in 0..ped_count {
            let x = (i % cols) as f32;
            let z = (i / cols) as f32;
            let pos = Vector::new(
                (x - half) * spacing,
                0.8,
                (z - half) * spacing,
            );
            ped_positions.push(pos);
        }

        SimEngine {
            pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set,
            collider_set,
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            gravity: Vector::new(0.0, -9.81, 0.0),
            integration_parameters: IntegrationParameters::default(),
            car_handle,
            vehicle,
            input_handbrake: 0.0,
            input_forward: 0.0,
            input_right: 0.0,
            input_run: 0.0,
            input_camera_yaw: 0.0,
            player_pos: Vector::new(2.0, 1.0, 0.0),
            player_yaw: 0.0,
            player_vel_y: 0.0,
            in_car: false,
            paused: false,
            allow_pause: true,
            sim_time: 0.0,
            ped_positions,
        }
    }

    pub fn step(&mut self) {
        if self.paused {
            return;
        }
        let dt = self.integration_parameters.dt;
        self.sim_time += dt;

        {
            let throttle = if self.in_car { self.input_forward } else { 0.0 };
            let steer = if self.in_car { -self.input_right } else { 0.0 };
            let handbrake = if self.in_car { self.input_handbrake } else { 0.0 };

            let engine_force = throttle * VEHICLE_MAX_ENGINE_FORCE;
            let brake_force = if handbrake > 0.5 {
                VEHICLE_MAX_BRAKE_FORCE
            } else {
                0.0
            };
            let steer_angle = steer * VEHICLE_MAX_STEER;

            for (index, wheel) in self.vehicle.wheels_mut().iter_mut().enumerate() {
                wheel.engine_force = engine_force;
                wheel.brake = brake_force;
                wheel.steering = if index < 2 { steer_angle } else { 0.0 };
            }

            let queries = self.broad_phase.as_query_pipeline_mut(
                self.narrow_phase.query_dispatcher(),
                &mut self.rigid_body_set,
                &mut self.collider_set,
                QueryFilter::default().exclude_rigid_body(self.car_handle),
            );
            self.vehicle.update_vehicle(dt, queries);
        }

        if !self.in_car {
            let speed = if self.input_run > 0.5 { 7.0 } else { 4.0 };
            let yaw = self.input_camera_yaw;
            let forward = Vector::new(yaw.sin(), 0.0, yaw.cos());
            let right = Vector::new(forward.z, 0.0, -forward.x);
            let mut move_dir = forward * self.input_forward + right * self.input_right;
            let len = move_dir.length();
            if len > 1.0 {
                move_dir /= len;
            }

            if len > 0.001 {
                self.player_yaw = move_dir.x.atan2(move_dir.z);
            }

            self.player_pos += move_dir * speed * dt;
            self.player_vel_y += -9.81 * dt;
            self.player_pos.y += self.player_vel_y * dt;
            if self.player_pos.y <= 1.0 {
                self.player_pos.y = 1.0;
                self.player_vel_y = 0.0;
            }
            let max_bound = WORLD_HALF - PLAYER_RADIUS;
            if self.player_pos.x > max_bound {
                self.player_pos.x = max_bound;
            } else if self.player_pos.x < -max_bound {
                self.player_pos.x = -max_bound;
            }
            if self.player_pos.z > max_bound {
                self.player_pos.z = max_bound;
            } else if self.player_pos.z < -max_bound {
                self.player_pos.z = -max_bound;
            }
        } else {
            let car_pos = self.rigid_body_set[self.car_handle].translation();
            self.player_pos = Vector::new(car_pos.x, car_pos.y + 0.9, car_pos.z);
            self.player_vel_y = 0.0;
        }

        self.update_pedestrians(dt);

        self.pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );
    }

    pub fn write_state(&self, out: &mut [f32]) {
        let needed =
            PED_STATE_OFFSET as usize + self.ped_positions.len() * PED_STATE_STRIDE as usize;
        if out.len() < needed {
            return;
        }

        let body = &self.rigid_body_set[self.car_handle];
        let translation = body.translation();
        let rotation = body.rotation();

        out[0] = translation.x;
        out[1] = translation.y;
        out[2] = translation.z;
        out[3] = rotation.x;
        out[4] = rotation.y;
        out[5] = rotation.z;
        out[6] = rotation.w;

        let player_offset = PLAYER_STATE_OFFSET as usize;
        out[player_offset] = self.player_pos.x;
        out[player_offset + 1] = self.player_pos.y;
        out[player_offset + 2] = self.player_pos.z;
        out[player_offset + 3] = self.player_yaw;
        out[player_offset + 4] = if self.in_car { 1.0 } else { 0.0 };

        let car_pos = body.translation();
        let dx = self.player_pos.x - car_pos.x;
        let dz = self.player_pos.z - car_pos.z;
        let enter_radius = 4.0;
        let can_enter = !self.in_car && (dx * dx + dz * dz <= enter_radius * enter_radius);
        let hud_hint = if self.in_car {
            HUD_HINT_EXIT
        } else if can_enter {
            HUD_HINT_ENTER
        } else {
            HUD_HINT_NONE
        };

        let hud_offset = HUD_STATE_OFFSET as usize;
        out[hud_offset] = hud_hint as f32;

        let cam_offset = CAM_STATE_OFFSET as usize;
        let target = if self.in_car { car_pos } else { self.player_pos };
        let target_y = if self.in_car { 0.6 } else { 0.9 };
        out[cam_offset] = target.x;
        out[cam_offset + 1] = target.y + target_y;
        out[cam_offset + 2] = target.z;
        out[cam_offset + 3] = if self.in_car { 14.0 } else { 8.0 };

        let mut cursor = PED_STATE_OFFSET as usize;
        for pos in &self.ped_positions {
            out[cursor] = 1.0;
            out[cursor + 1] = 0.0;
            out[cursor + 2] = 0.0;
            out[cursor + 3] = 0.0;
            out[cursor + 4] = 0.0;
            out[cursor + 5] = 1.0;
            out[cursor + 6] = 0.0;
            out[cursor + 7] = 0.0;
            out[cursor + 8] = 0.0;
            out[cursor + 9] = 0.0;
            out[cursor + 10] = 1.0;
            out[cursor + 11] = 0.0;
            out[cursor + 12] = pos.x;
            out[cursor + 13] = pos.y;
            out[cursor + 14] = pos.z;
            out[cursor + 15] = 1.0;
            cursor += PED_STATE_STRIDE as usize;
        }
    }

    pub fn state_len(&self) -> usize {
        PED_STATE_OFFSET as usize + self.ped_positions.len() * PED_STATE_STRIDE as usize
    }

    pub fn ped_count(&self) -> u32 {
        self.ped_positions.len() as u32
    }

    pub fn dt(&self) -> f32 {
        self.integration_parameters.dt
    }

    pub fn reset_car(&mut self) {
        if let Some(body) = self.rigid_body_set.get_mut(self.car_handle) {
            body.set_translation(Vector::new(0.0, 1.0, 0.0), true);
            body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
            body.set_angvel(Vector::new(0.0, 0.0, 0.0), true);
            body.wake_up(true);
        }
        self.player_pos = Vector::new(2.0, 1.0, 0.0);
        self.player_yaw = 0.0;
        self.player_vel_y = 0.0;
        self.in_car = false;
        for wheel in self.vehicle.wheels_mut() {
            wheel.engine_force = 0.0;
            wheel.brake = 0.0;
            wheel.steering = 0.0;
            wheel.rotation = 0.0;
        }
    }

    pub fn toggle_pause(&mut self) {
        if self.allow_pause {
            self.paused = !self.paused;
        }
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    pub fn set_allow_pause(&mut self, allow: bool) {
        self.allow_pause = allow;
        if !allow {
            self.paused = false;
        }
    }

    pub fn set_input_buttons(&mut self, buttons: u32, camera_yaw: f32) {
        let forward = if (buttons & BTN_FORWARD) != 0 { 1.0 } else { 0.0 };
        let back = if (buttons & BTN_BACK) != 0 { 1.0 } else { 0.0 };
        let left = if (buttons & BTN_LEFT) != 0 { 1.0 } else { 0.0 };
        let right = if (buttons & BTN_RIGHT) != 0 { 1.0 } else { 0.0 };

        self.input_forward = (forward - back).clamp(-1.0, 1.0);
        self.input_right = (right - left).clamp(-1.0, 1.0);
        self.input_handbrake = if (buttons & BTN_HANDBRAKE) != 0 { 1.0 } else { 0.0 };
        self.input_run = if (buttons & BTN_RUN) != 0 { 1.0 } else { 0.0 };
        self.input_camera_yaw = camera_yaw;
    }

    pub fn jump(&mut self) {
        if self.in_car {
            return;
        }
        if self.player_pos.y <= 1.01 {
            self.player_vel_y = 6.2;
        }
    }

    pub fn toggle_enter(&mut self) {
        if self.in_car {
            let car_pos = self.rigid_body_set[self.car_handle].translation();
            self.player_pos = Vector::new(car_pos.x + 2.0, 1.0, car_pos.z);
            self.in_car = false;
            return;
        }

        let car_pos = self.rigid_body_set[self.car_handle].translation();
        let dx = self.player_pos.x - car_pos.x;
        let dz = self.player_pos.z - car_pos.z;
        let enter_radius = 4.0;
        if dx * dx + dz * dz <= enter_radius * enter_radius {
            self.in_car = true;
            self.player_pos = Vector::new(car_pos.x, car_pos.y, car_pos.z);
        }
    }

    fn update_pedestrians(&mut self, dt: f32) {
        if self.ped_positions.is_empty() {
            return;
        }

        let world_half = WORLD_HALF;
        let car_body = &self.rigid_body_set[self.car_handle];
        let car_pos = car_body.translation();
        let player_pos = self.player_pos;
        let mut player_push = Vector::new(0.0, 0.0, 0.0);
        let ped_radius = 0.4;
        let player_radius = 0.5;
        let car_radius = 2.0;
        let car_speed = car_body.linvel().length();
        let car_push_scale = 1.2 + (car_speed * 0.1).min(2.0);

        for (i, pos) in self.ped_positions.iter_mut().enumerate() {
            let angle = self.sim_time * 0.35 + i as f32 * 0.37;
            let mut dir = Vector::new(angle.cos(), 0.0, angle.sin());

            let dx = pos.x - car_pos.x;
            let dz = pos.z - car_pos.z;
            let dist_sq = dx * dx + dz * dz;
            if dist_sq < 36.0 {
                let avoid = Vector::new(dx, 0.0, dz);
                dir += avoid.normalize_or_zero() * 2.5;
            }

            if !self.in_car {
                let px = pos.x - player_pos.x;
                let pz = pos.z - player_pos.z;
                let pdist_sq = px * px + pz * pz;
                let min_dist = ped_radius + player_radius;
                if pdist_sq < min_dist * min_dist {
                    let dist = pdist_sq.sqrt().max(0.001);
                    let n = Vector::new(px / dist, 0.0, pz / dist);
                    let push = min_dist - dist;
                    *pos += n * push;
                    player_push -= n * push;
                }
            }

            let cdist_sq = dist_sq;
            let car_min = car_radius + ped_radius;
            if cdist_sq < car_min * car_min {
                let dist = cdist_sq.sqrt().max(0.001);
                let n = Vector::new(dx / dist, 0.0, dz / dist);
                let push = (car_min - dist) * car_push_scale;
                *pos += n * push;
            }

            let speed = 0.8 + (i % 7) as f32 * 0.1;
            *pos += dir * speed * dt;

            let limit = world_half - ped_radius;
            if pos.x > limit {
                pos.x = limit;
            } else if pos.x < -limit {
                pos.x = -limit;
            }

            if pos.z > limit {
                pos.z = limit;
            } else if pos.z < -limit {
                pos.z = -limit;
            }
        }

        if !self.in_car {
            let dx = player_pos.x - car_pos.x;
            let dz = player_pos.z - car_pos.z;
            let dist_sq = dx * dx + dz * dz;
            let min_dist = player_radius + car_radius;
            if dist_sq < min_dist * min_dist {
                let dist = dist_sq.sqrt().max(0.001);
                let n = Vector::new(dx / dist, 0.0, dz / dist);
                let push = min_dist - dist;
                player_push += n * push;
            }

            self.player_pos += player_push;
        }
    }
}
