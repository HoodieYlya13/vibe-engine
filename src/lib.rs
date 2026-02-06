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
    ball_handles: Vec<RigidBodyHandle>,
    ball_start_positions: Vec<Vector>,
}

#[wasm_bindgen]
impl SimEngine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> SimEngine {
        SimEngine::new_with_balls(1)
    }

    pub fn new_with_balls(count: u32) -> SimEngine {
        console_error_panic_hook::set_once();

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        let ground_body = RigidBodyBuilder::fixed()
            .translation(Vector::new(0.0, -6.0, 0.0))
            .build();
        let ground_collider = ColliderBuilder::cuboid(50.0, 0.5, 50.0).build();
        let ground_handle = rigid_body_set.insert(ground_body);
        collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

        let mut ball_handles = Vec::new();
        let mut ball_start_positions = Vec::new();
        let ball_count = count.max(1) as usize;
        let cols = (ball_count as f32).sqrt().ceil() as usize;
        let spacing = 1.2;
        let half = cols as f32 * 0.5;

        for i in 0..ball_count {
            let x = (i % cols) as f32;
            let z = (i / cols) as f32;
            let position = Vector::new(
                (x - half) * spacing,
                5.0 + (z * 0.1),
                (z - half) * spacing,
            );

            let rigid_body = RigidBodyBuilder::dynamic()
                .translation(position)
                .build();

            let collider = ColliderBuilder::ball(0.5)
                .restitution(0.7)
                .build();

            let handle = rigid_body_set.insert(rigid_body);
            collider_set.insert_with_parent(collider, handle, &mut rigid_body_set);
            ball_handles.push(handle);
            ball_start_positions.push(position);
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
            ball_handles,
            ball_start_positions,
        }
    }

    pub fn step(&mut self) {
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

    pub fn write_positions(&self, out: &mut [f32]) {
        let needed = self.ball_handles.len() * 3;
        if out.len() < needed {
            return;
        }

        for (i, handle) in self.ball_handles.iter().enumerate() {
            let body = &self.rigid_body_set[*handle];
            let translation = body.translation();
            let base = i * 3;
            out[base] = translation.x;
            out[base + 1] = translation.y;
            out[base + 2] = translation.z;
        }
    }

    pub fn reset_balls(&mut self) {
        for (i, handle) in self.ball_handles.iter().enumerate() {
            let position = self.ball_start_positions[i];
            if let Some(body) = self.rigid_body_set.get_mut(*handle) {
                body.set_translation(position, true);
                body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
                body.set_angvel(Vector::new(0.0, 0.0, 0.0), true);
                body.wake_up(true);
            }
        }
    }
}
