//! Dev/test-mode surface (PRD §8.6): live vehicle tuning, teleports, and
//! physics wireframes. Only the worker's dev message handler calls these;
//! nothing here runs in the fixed-step loop unless invoked.

use rapier3d::pipeline::{DebugColor, DebugRenderBackend, DebugRenderObject, DebugRenderPipeline};
use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::CAR_HALF_WIDTH;
use crate::engine::SimEngine;

/// Collects debug lines as a flat [x0,y0,z0, x1,y1,z1, ...] vertex list.
struct LineCollector {
    vertices: Vec<f32>,
}

impl DebugRenderBackend for LineCollector {
    fn draw_line(&mut self, _object: DebugRenderObject, a: Vector, b: Vector, _color: DebugColor) {
        self.vertices
            .extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
    }
}

#[wasm_bindgen]
impl SimEngine {
    /// Overwrites the live driving parameters (dev console sliders). Order
    /// matches `VehicleTuning`; defaults come from the active class in
    /// `data/vehicles.ron` (read them back with `vehicle_tuning`).
    #[allow(clippy::too_many_arguments)]
    pub fn set_vehicle_tuning(
        &mut self,
        max_engine_force: f32,
        max_reverse_force: f32,
        footbrake_impulse: f32,
        engine_brake_impulse: f32,
        handbrake_impulse: f32,
        direction_switch_speed: f32,
        max_steer: f32,
        rear_steer: f32,
        anti_roll: f32,
        reverse_stability: f32,
        air_control: f32,
    ) {
        let tuning = &mut self.vehicle.tuning;
        tuning.max_engine_force = max_engine_force;
        tuning.max_reverse_force = max_reverse_force;
        tuning.footbrake_impulse = footbrake_impulse;
        tuning.engine_brake_impulse = engine_brake_impulse;
        tuning.handbrake_impulse = handbrake_impulse;
        tuning.direction_switch_speed = direction_switch_speed;
        tuning.max_steer = max_steer;
        tuning.rear_steer = rear_steer;
        tuning.anti_roll = anti_roll;
        tuning.reverse_stability = reverse_stability;
        tuning.air_control = air_control;
    }

    /// The current live tuning values, in `set_vehicle_tuning` argument order
    /// (seeds the dev console sliders for the active class).
    pub fn vehicle_tuning(&self) -> Vec<f32> {
        let t = &self.vehicle.tuning;
        vec![
            t.max_engine_force,
            t.max_reverse_force,
            t.footbrake_impulse,
            t.engine_brake_impulse,
            t.handbrake_impulse,
            t.direction_switch_speed,
            t.max_steer,
            t.rear_steer,
            t.anti_roll,
            t.reverse_stability,
            t.air_control,
        ]
    }

    pub fn vehicle_class_names(&self) -> Vec<String> {
        self.vehicle
            .classes
            .iter()
            .map(|class| class.name.clone())
            .collect()
    }

    pub fn vehicle_class_index(&self) -> u32 {
        self.vehicle.class_index as u32
    }

    /// Swaps the car to another class from `data/vehicles.ron`: rebuilds the
    /// chassis/wheels in place (same pose) and resets the live tuning to the
    /// class defaults. Re-applying the current class is a tuning reset.
    pub fn set_vehicle_class(&mut self, index: u32) {
        let index = index as usize;
        let Some(class) = self.vehicle.classes.get(index).cloned() else {
            return;
        };

        let body = &self.rigid_body_set[self.vehicle.body_handle];
        let translation = body.translation();
        let rotation = *body.rotation();
        self.rigid_body_set.remove(
            self.vehicle.body_handle,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );

        let (body_handle, controller) = crate::vehicle::build_vehicle_body(
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &class,
        );
        if let Some(body) = self.rigid_body_set.get_mut(body_handle) {
            body.set_translation(translation, true);
            body.set_rotation(rotation, true);
        }
        self.vehicle.body_handle = body_handle;
        self.vehicle.controller = controller;
        self.vehicle.tuning = class.drive;
        self.vehicle.class_index = index;
    }

    pub fn teleport_player(&mut self, x: f32, y: f32, z: f32) {
        if self.player.in_car {
            return;
        }
        self.player.pos = Vector::new(x, y, z);
        self.player.vel_y = 0.0;
        self.player.grounded = false;
        self.teleport_player_body();
    }

    pub fn teleport_car(&mut self, x: f32, y: f32, z: f32) {
        if let Some(body) = self.rigid_body_set.get_mut(self.vehicle.body_handle) {
            body.set_translation(Vector::new(x, y, z), true);
            body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
            body.set_angvel(Vector::new(0.0, 0.0, 0.0), true);
            body.wake_up(true);
        }
        if self.player.in_car {
            self.player.pos = Vector::new(x, y + 0.9, z);
            self.teleport_player_body();
        }
    }

    /// Drops the car onto its side at its current position — dev tool for
    /// exercising the anti-roll self-righting assist.
    pub fn flip_car(&mut self) {
        if let Some(body) = self.rigid_body_set.get_mut(self.vehicle.body_handle) {
            let pos = body.translation();
            body.set_translation(Vector::new(pos.x, CAR_HALF_WIDTH + 0.3, pos.z), true);
            body.set_rotation(Rotation::from_rotation_z(std::f32::consts::FRAC_PI_2), true);
            body.set_linvel(Vector::new(0.0, 0.0, 0.0), true);
            body.set_angvel(Vector::new(0.0, 0.0, 0.0), true);
            body.wake_up(true);
        }
    }

    /// Renders every collider/joint/contact as a wireframe line list:
    /// flat [x0,y0,z0, x1,y1,z1, ...], two vertices per line.
    pub fn debug_render_lines(&self) -> Vec<f32> {
        let mut backend = LineCollector {
            vertices: Vec::new(),
        };
        DebugRenderPipeline::default().render(
            &mut backend,
            &self.rigid_body_set,
            &self.collider_set,
            &self.impulse_joint_set,
            &self.multibody_joint_set,
            &self.narrow_phase,
        );
        backend.vertices
    }
}
