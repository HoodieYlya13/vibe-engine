use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::{
    BLOCKS, DEFAULT_PED_COUNT, PLAYER_STAND_HEIGHT, RAMPS, WALL_HEIGHT, WALL_THICKNESS, WORLD_HALF,
};
use crate::state::{CrowdState, InputState, PlayerState, RuntimeState, SimEngine};
use crate::vehicle::create_vehicle;

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
        let ground_collider = ColliderBuilder::cuboid(WORLD_HALF, 0.5, WORLD_HALF).build();
        let ground_handle = rigid_body_set.insert(ground_body);
        collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

        let wall_body = RigidBodyBuilder::fixed().build();
        let wall_handle = rigid_body_set.insert(wall_body);
        let wall_x = WORLD_HALF + WALL_THICKNESS;
        let wall_z = WORLD_HALF + WALL_THICKNESS;
        let wall_depth = WORLD_HALF + WALL_THICKNESS;

        let wall_x_pos = ColliderBuilder::cuboid(WALL_THICKNESS, WALL_HEIGHT, wall_depth)
            .translation(Vector::new(wall_x, WALL_HEIGHT, 0.0))
            .build();
        let wall_x_neg = ColliderBuilder::cuboid(WALL_THICKNESS, WALL_HEIGHT, wall_depth)
            .translation(Vector::new(-wall_x, WALL_HEIGHT, 0.0))
            .build();
        let wall_z_pos = ColliderBuilder::cuboid(wall_depth, WALL_HEIGHT, WALL_THICKNESS)
            .translation(Vector::new(0.0, WALL_HEIGHT, wall_z))
            .build();
        let wall_z_neg = ColliderBuilder::cuboid(wall_depth, WALL_HEIGHT, WALL_THICKNESS)
            .translation(Vector::new(0.0, WALL_HEIGHT, -wall_z))
            .build();
        collider_set.insert_with_parent(wall_x_pos, wall_handle, &mut rigid_body_set);
        collider_set.insert_with_parent(wall_x_neg, wall_handle, &mut rigid_body_set);
        collider_set.insert_with_parent(wall_z_pos, wall_handle, &mut rigid_body_set);
        collider_set.insert_with_parent(wall_z_neg, wall_handle, &mut rigid_body_set);

        let block_body = RigidBodyBuilder::fixed().build();
        let block_handle = rigid_body_set.insert(block_body);
        for block in BLOCKS {
            let collider = ColliderBuilder::cuboid(block.3, block.4, block.5)
                .translation(Vector::new(block.0, block.1, block.2))
                .build();
            collider_set.insert_with_parent(collider, block_handle, &mut rigid_body_set);
        }

        let ramp_body = RigidBodyBuilder::fixed().build();
        let ramp_handle = rigid_body_set.insert(ramp_body);
        for ramp in RAMPS {
            let z0 = ramp.0;
            let z1 = ramp.1;
            let x0 = ramp.2 - ramp.3;
            let x1 = ramp.2 + ramp.3;
            let h = ramp.4;
            let points = [
                Vector::new(x0, 0.0, z0),
                Vector::new(x1, 0.0, z0),
                Vector::new(x0, 0.0, z1),
                Vector::new(x1, 0.0, z1),
                Vector::new(x0, h, z1),
                Vector::new(x1, h, z1),
            ];
            if let Some(collider) = ColliderBuilder::convex_hull(&points) {
                collider_set.insert_with_parent(collider.build(), ramp_handle, &mut rigid_body_set);
            }
        }

        let vehicle = create_vehicle(&mut rigid_body_set, &mut collider_set);

        let ped_count = count as usize;
        let mut ped_positions = Vec::with_capacity(ped_count);
        let cols = (ped_count as f32).sqrt().ceil() as usize;
        let spacing = 2.5;
        let half = cols as f32 * 0.5;
        for i in 0..ped_count {
            let x = (i % cols) as f32;
            let z = (i / cols) as f32;
            ped_positions.push(Vector::new((x - half) * spacing, 0.8, (z - half) * spacing));
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
            vehicle,
            input: InputState {
                forward: 0.0,
                right: 0.0,
                run: 0.0,
                handbrake: 0.0,
                camera_yaw: 0.0,
            },
            player: PlayerState {
                pos: Vector::new(2.0, PLAYER_STAND_HEIGHT, 0.0),
                yaw: 0.0,
                vel_y: 0.0,
                in_car: false,
            },
            runtime: RuntimeState {
                paused: false,
                allow_pause: true,
                sim_time: 0.0,
            },
            crowd: CrowdState { ped_positions },
        }
    }

    pub fn step(&mut self) {
        if self.runtime.paused {
            return;
        }

        let dt = self.integration_parameters.dt;
        self.runtime.sim_time += dt;

        self.step_vehicle(dt);
        self.update_player(dt);
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
}
