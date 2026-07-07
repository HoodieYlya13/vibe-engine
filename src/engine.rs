use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::bridge::input::InputState;
use crate::character::{self, PlayerState};
use crate::constants::DEFAULT_PED_COUNT;
use crate::crowd::{self, CrowdState};
use crate::vehicle::{self, VehicleState};
use crate::world;

pub(crate) struct RuntimeState {
    pub(crate) paused: bool,
    pub(crate) allow_pause: bool,
    pub(crate) sim_time: f32,
}

#[wasm_bindgen]
pub struct SimEngine {
    pub(crate) pipeline: PhysicsPipeline,
    pub(crate) island_manager: IslandManager,
    pub(crate) broad_phase: DefaultBroadPhase,
    pub(crate) narrow_phase: NarrowPhase,
    pub(crate) rigid_body_set: RigidBodySet,
    pub(crate) collider_set: ColliderSet,
    pub(crate) impulse_joint_set: ImpulseJointSet,
    pub(crate) multibody_joint_set: MultibodyJointSet,
    pub(crate) ccd_solver: CCDSolver,
    pub(crate) gravity: Vector,
    pub(crate) integration_parameters: IntegrationParameters,
    pub(crate) vehicle: VehicleState,
    pub(crate) input: InputState,
    pub(crate) player: PlayerState,
    pub(crate) runtime: RuntimeState,
    pub(crate) crowd: CrowdState,
}

impl Default for SimEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen]
impl SimEngine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> SimEngine {
        SimEngine::new_with_peds(DEFAULT_PED_COUNT)
    }

    pub fn new_with_peds(count: u32) -> SimEngine {
        Self::new_internal(count, true)
    }

    pub fn new_open_field(count: u32) -> SimEngine {
        Self::new_internal(count, false)
    }

    fn new_internal(count: u32, with_obstacles: bool) -> SimEngine {
        console_error_panic_hook::set_once();

        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        world::build_static_world(&mut rigid_body_set, &mut collider_set, with_obstacles);

        let mut vehicle = vehicle::create_vehicle(&mut rigid_body_set, &mut collider_set);
        if with_obstacles {
            let rotation = Rotation::from_rotation_y(std::f32::consts::PI);
            vehicle.spawn_rotation = rotation;
            if let Some(body) = rigid_body_set.get_mut(vehicle.body_handle) {
                body.set_rotation(rotation, true);
            }
        }

        let player = character::spawn_player(&mut rigid_body_set, &mut collider_set);

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
            input: InputState::default(),
            player,
            runtime: RuntimeState {
                paused: false,
                allow_pause: true,
                sim_time: 0.0,
            },
            crowd: crowd::spawn_peds(count as usize),
        }
    }

    /// One fixed-dt tick. System order matters: controllers write forces and
    /// kinematic targets first, then the solver integrates.
    pub fn step(&mut self) {
        if self.runtime.paused {
            return;
        }

        let dt = self.integration_parameters.dt;
        self.runtime.sim_time += dt;

        self.step_vehicle(dt);
        self.update_player(dt);
        self.update_pedestrians(dt);

        // Wheel/suspension impulses are already applied above, so the car's
        // velocity change across the solver step isolates chassis contact
        // impulses — the crash-damage signal (see apply_impact_damage).
        let car_vel_before = self.rigid_body_set[self.vehicle.body_handle].linvel();

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

        self.apply_impact_damage(car_vel_before);
    }
}
