use rapier3d::control::DynamicRayCastVehicleController;
use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

#[derive(Clone, Copy)]
pub(crate) struct InputState {
    pub(crate) forward: f32,
    pub(crate) right: f32,
    pub(crate) run: f32,
    pub(crate) handbrake: f32,
    pub(crate) camera_yaw: f32,
}

#[derive(Clone, Copy)]
pub(crate) struct PlayerState {
    pub(crate) pos: Vector,
    pub(crate) yaw: f32,
    pub(crate) vel_y: f32,
    pub(crate) in_car: bool,
}

pub(crate) struct VehicleState {
    pub(crate) body_handle: RigidBodyHandle,
    pub(crate) controller: DynamicRayCastVehicleController,
}

pub(crate) struct RuntimeState {
    pub(crate) paused: bool,
    pub(crate) allow_pause: bool,
    pub(crate) sim_time: f32,
}

pub(crate) struct CrowdState {
    pub(crate) ped_positions: Vec<Vector>,
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
