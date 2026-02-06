use wasm_bindgen::prelude::*;
use rapier3d::prelude::*;

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
    input_throttle: f32,
    input_steer: f32,
    input_handbrake: f32,
    input_move_x: f32,
    input_move_z: f32,
    player_pos: Vector,
    player_yaw: f32,
    in_car: bool,
    sim_time: f32,
    ped_positions: Vec<Vector>,
}

#[wasm_bindgen]
impl SimEngine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> SimEngine {
        SimEngine::new_with_peds(0)
    }

    pub fn new_with_peds(count: u32) -> SimEngine {
        console_error_panic_hook::set_once();

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        let ground_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(0.0, -0.5, 0.0))
            .build();
        let ground_collider = ColliderBuilder::cuboid(50.0, 0.5, 50.0).build();
        let ground_handle = rigid_body_set.insert(ground_body);
        collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

        let car_body = RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 1.0, 0.0))
            .linear_damping(0.2)
            .angular_damping(0.9)
            .build();
        let car_collider = ColliderBuilder::cuboid(0.9, 0.4, 1.6)
            .restitution(0.1)
            .friction(1.2)
            .build();
        let car_handle = rigid_body_set.insert(car_body);
        collider_set.insert_with_parent(car_collider, car_handle, &mut rigid_body_set);

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
            input_throttle: 0.0,
            input_steer: 0.0,
            input_handbrake: 0.0,
            input_move_x: 0.0,
            input_move_z: 0.0,
            player_pos: Vector::new(2.0, 1.0, 0.0),
            player_yaw: 0.0,
            in_car: false,
            sim_time: 0.0,
            ped_positions,
        }
    }

    pub fn step(&mut self) {
        let dt = self.integration_parameters.dt;
        self.sim_time += dt;

        {
            let max_speed = 18.0;
            let turn_rate = 1.8;
            let drift_turn = 3.2;

            let throttle = if self.in_car { self.input_throttle } else { 0.0 };
            let steer = if self.in_car { self.input_steer } else { 0.0 };
            let handbrake = if self.in_car { self.input_handbrake } else { 0.0 };

            if let Some(body) = self.rigid_body_set.get_mut(self.car_handle) {
                let rotation = body.rotation();
                let forward = rotation * Vector::new(0.0, 0.0, 1.0);
                let target_speed = throttle * max_speed;
                let current_vel = body.linvel();
                let desired = Vector::new(
                    forward.x * target_speed,
                    current_vel.y,
                    forward.z * target_speed,
                );
                body.set_linvel(desired, true);

                let turn_strength = if handbrake > 0.5 {
                    drift_turn
                } else {
                    turn_rate
                };
                body.set_angvel(Vector::new(0.0, steer * turn_strength, 0.0), true);
            }
        }

        if !self.in_car {
            let speed = 4.0;
            let mut move_dir = Vector::new(self.input_move_x, 0.0, self.input_move_z);
            let len = move_dir.length();
            if len > 1.0 {
                move_dir /= len;
            }

            if len > 0.001 {
                self.player_yaw = move_dir.x.atan2(move_dir.z);
            }

            self.player_pos += move_dir * speed * dt;
            self.player_pos.y = 1.0;
        } else {
            let car_pos = self.rigid_body_set[self.car_handle].translation();
            self.player_pos = Vector::new(car_pos.x, car_pos.y + 0.9, car_pos.z);
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
        let needed = 12 + self.ped_positions.len() * 3;
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

        out[7] = self.player_pos.x;
        out[8] = self.player_pos.y;
        out[9] = self.player_pos.z;
        out[10] = self.player_yaw;
        out[11] = if self.in_car { 1.0 } else { 0.0 };

        let mut cursor = 12;
        for pos in &self.ped_positions {
            out[cursor] = pos.x;
            out[cursor + 1] = pos.y;
            out[cursor + 2] = pos.z;
            cursor += 3;
        }
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
        self.in_car = false;
    }

    pub fn set_input(&mut self, throttle: f32, steer: f32, handbrake: f32, move_x: f32, move_z: f32) {
        self.input_throttle = throttle.clamp(-1.0, 1.0);
        self.input_steer = steer.clamp(-1.0, 1.0);
        self.input_handbrake = handbrake.clamp(0.0, 1.0);
        self.input_move_x = move_x.clamp(-1.0, 1.0);
        self.input_move_z = move_z.clamp(-1.0, 1.0);
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
        if dx * dx + dz * dz < 4.0 {
            self.in_car = true;
        }
    }

    fn update_pedestrians(&mut self, dt: f32) {
        if self.ped_positions.is_empty() {
            return;
        }

        let world_half = 60.0;
        let car_pos = self.rigid_body_set[self.car_handle].translation();

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

            let speed = 0.8 + (i % 7) as f32 * 0.1;
            *pos += dir * speed * dt;

            if pos.x > world_half {
                pos.x = -world_half;
            } else if pos.x < -world_half {
                pos.x = world_half;
            }

            if pos.z > world_half {
                pos.z = -world_half;
            } else if pos.z < -world_half {
                pos.z = world_half;
            }
        }
    }
}
