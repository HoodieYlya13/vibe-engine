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
}

#[wasm_bindgen]
impl SimEngine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> SimEngine {
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
        }
    }

    pub fn step(&mut self) {
        {
            let max_speed = 18.0;
            let turn_rate = 1.8;
            let drift_turn = 3.2;

            if let Some(body) = self.rigid_body_set.get_mut(self.car_handle) {
                let rotation = body.rotation();
                let forward = rotation * Vector::new(0.0, 0.0, 1.0);
                let target_speed = self.input_throttle * max_speed;
                let current_vel = body.linvel();
                let desired = Vector::new(
                    forward.x * target_speed,
                    current_vel.y,
                    forward.z * target_speed,
                );
                body.set_linvel(desired, true);

                let turn_strength = if self.input_handbrake > 0.5 {
                    drift_turn
                } else {
                    turn_rate
                };
                body.set_angvel(Vector::new(0.0, self.input_steer * turn_strength, 0.0), true);
            }
        }

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

    pub fn write_car_state(&self, out: &mut [f32]) {
        if out.len() < 7 {
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
    }

    pub fn reset_car(&mut self) {
        if let Some(body) = self.rigid_body_set.get_mut(self.car_handle) {
            body.set_translation(Vector::new(0.0, 1.0, 0.0), true);
            body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
            body.set_angvel(Vector::new(0.0, 0.0, 0.0), true);
            body.wake_up(true);
        }
    }

    pub fn set_input(&mut self, throttle: f32, steer: f32, handbrake: f32) {
        self.input_throttle = throttle.clamp(-1.0, 1.0);
        self.input_steer = steer.clamp(-1.0, 1.0);
        self.input_handbrake = handbrake.clamp(0.0, 1.0);
    }
}
