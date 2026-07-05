use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

use crate::constants::{
    CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, VEHICLE_AIR_CONTROL, VEHICLE_ANTI_ROLL,
    VEHICLE_DIRECTION_SWITCH_SPEED, VEHICLE_ENGINE_BRAKE_IMPULSE, VEHICLE_FOOTBRAKE_IMPULSE,
    VEHICLE_HANDBRAKE_IMPULSE, VEHICLE_MAX_ENGINE_FORCE, VEHICLE_MAX_REVERSE_FORCE,
    VEHICLE_MAX_STEER, VEHICLE_REAR_STEER, VEHICLE_REVERSE_STABILITY,
};
use crate::engine::SimEngine;

/// Runtime-mutable driving parameters. Defaults come from `constants.rs`;
/// the dev console can overwrite them live so iterating on car feel never
/// requires a wasm rebuild (PRD §8.3). Phase 2 loads these from data files.
#[derive(Clone, Copy)]
pub(crate) struct VehicleTuning {
    pub(crate) max_engine_force: f32,
    pub(crate) max_reverse_force: f32,
    pub(crate) footbrake_impulse: f32,
    pub(crate) engine_brake_impulse: f32,
    pub(crate) handbrake_impulse: f32,
    pub(crate) direction_switch_speed: f32,
    pub(crate) max_steer: f32,
    pub(crate) rear_steer: f32,
    pub(crate) anti_roll: f32,
    pub(crate) reverse_stability: f32,
    pub(crate) air_control: f32,
}

impl Default for VehicleTuning {
    fn default() -> Self {
        VehicleTuning {
            max_engine_force: VEHICLE_MAX_ENGINE_FORCE,
            max_reverse_force: VEHICLE_MAX_REVERSE_FORCE,
            footbrake_impulse: VEHICLE_FOOTBRAKE_IMPULSE,
            engine_brake_impulse: VEHICLE_ENGINE_BRAKE_IMPULSE,
            handbrake_impulse: VEHICLE_HANDBRAKE_IMPULSE,
            direction_switch_speed: VEHICLE_DIRECTION_SWITCH_SPEED,
            max_steer: VEHICLE_MAX_STEER,
            rear_steer: VEHICLE_REAR_STEER,
            anti_roll: VEHICLE_ANTI_ROLL,
            reverse_stability: VEHICLE_REVERSE_STABILITY,
            air_control: VEHICLE_AIR_CONTROL,
        }
    }
}

pub(crate) struct VehicleState {
    pub(crate) body_handle: RigidBodyHandle,
    pub(crate) controller: DynamicRayCastVehicleController,
    pub(crate) spawn_rotation: Rotation,
    pub(crate) tuning: VehicleTuning,
}

pub(crate) fn create_vehicle(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
) -> VehicleState {
    // The 1200 kg sits below the chassis center so lateral grip has less
    // leverage to roll the car over; the principal inertia mirrors what
    // `additional_mass(1200.0)` used to derive from the cuboid, keeping the
    // pre-existing yaw/pitch response.
    let car_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 1.4, 0.0))
        .additional_mass_properties(MassProperties::new(
            Vector::new(0.0, -0.5, 0.0),
            1200.0,
            Vector::new(4500.0, 5560.0, 1600.0),
        ))
        .linear_damping(0.28)
        .angular_damping(2.8)
        .build();
    let car_collider = ColliderBuilder::cuboid(CAR_HALF_WIDTH, CAR_HALF_HEIGHT, CAR_HALF_LENGTH)
        .restitution(0.1)
        .friction(1.2)
        .build();
    let car_handle = rigid_body_set.insert(car_body);
    collider_set.insert_with_parent(car_collider, car_handle, rigid_body_set);

    let mut controller = DynamicRayCastVehicleController::new(car_handle);
    controller.index_up_axis = 1;
    controller.index_forward_axis = 2;

    let wheel_radius = 0.5;
    let suspension_rest = 0.45;
    let tuning = WheelTuning {
        suspension_stiffness: 20.0,
        suspension_compression: 4.5,
        suspension_damping: 4.5,
        max_suspension_travel: 0.24,
        side_friction_stiffness: 1.2,
        friction_slip: 9.0,
        max_suspension_force: 18000.0,
    };
    let half_width = CAR_HALF_WIDTH * 0.85;
    let half_length = CAR_HALF_LENGTH * 0.8;
    let connection_height = -0.5;
    let direction = Vector::new(0.0, -1.0, 0.0);
    // Rapier derives the wheel's forward as contact_normal × axle; with -X the
    // forward comes out +Z, matching index_forward_axis so positive engine
    // force drives the chassis toward its steering wheels.
    let axle = Vector::new(-1.0, 0.0, 0.0);

    controller.add_wheel(
        Vector::new(-half_width, connection_height, half_length),
        direction,
        axle,
        suspension_rest,
        wheel_radius,
        &tuning,
    );
    controller.add_wheel(
        Vector::new(half_width, connection_height, half_length),
        direction,
        axle,
        suspension_rest,
        wheel_radius,
        &tuning,
    );
    controller.add_wheel(
        Vector::new(-half_width, connection_height, -half_length),
        direction,
        axle,
        suspension_rest,
        wheel_radius,
        &tuning,
    );
    controller.add_wheel(
        Vector::new(half_width, connection_height, -half_length),
        direction,
        axle,
        suspension_rest,
        wheel_radius,
        &tuning,
    );

    VehicleState {
        body_handle: car_handle,
        controller,
        spawn_rotation: Rotation::IDENTITY,
        tuning: VehicleTuning::default(),
    }
}

impl SimEngine {
    pub(crate) fn step_vehicle(&mut self, dt: f32) {
        let (throttle, steer, handbrake) = if self.player.in_car {
            (
                self.input.forward,
                self.input.right,
                self.input.handbrake > 0.5,
            )
        } else {
            (0.0, 0.0, false)
        };

        if (throttle.abs() > 0.01 || steer.abs() > 0.01 || handbrake)
            && let Some(body) = self.rigid_body_set.get_mut(self.vehicle.body_handle)
        {
            body.wake_up(true);
        }

        if self.rigid_body_set[self.vehicle.body_handle].is_sleeping() {
            for wheel in self.vehicle.controller.wheels_mut() {
                wheel.engine_force = 0.0;
                wheel.brake = 0.0;
            }
            return;
        }

        let forward_speed = {
            let body = &self.rigid_body_set[self.vehicle.body_handle];
            let forward = body.rotation() * Vector::new(0.0, 0.0, 1.0);
            forward.dot(body.linvel())
        };

        let tuning = self.vehicle.tuning;
        let mut engine_force = 0.0;
        let mut brake = 0.0;
        if throttle > 0.01 {
            if forward_speed < -tuning.direction_switch_speed {
                brake = tuning.footbrake_impulse * throttle;
            } else {
                engine_force = tuning.max_engine_force * throttle;
            }
        } else if throttle < -0.01 {
            if forward_speed > tuning.direction_switch_speed {
                brake = tuning.footbrake_impulse * -throttle;
            } else {
                engine_force = tuning.max_reverse_force * throttle;
            }
        } else {
            brake = tuning.engine_brake_impulse;
        }

        let mut steer_angle = steer * tuning.max_steer;
        // Reverse stability: scale steering authority down with reverse speed
        // (unassisted reverse yaws ~1.5 rad/s — twitchier than forward).
        if tuning.reverse_stability > 0.0 && forward_speed < -0.5 {
            steer_angle /= 1.0 + tuning.reverse_stability * -forward_speed;
        }

        for (index, wheel) in self.vehicle.controller.wheels_mut().iter_mut().enumerate() {
            let rear = index >= 2;
            if handbrake && rear {
                wheel.engine_force = 0.0;
                wheel.brake = tuning.handbrake_impulse;
            } else {
                wheel.engine_force = engine_force;
                wheel.brake = brake;
            }
            // Counter-phase rear steering swings the tail out with the turn —
            // the arcade drift feel (a tamed version of the old inverted-
            // steering bug the handling notes call out).
            wheel.steering = if rear {
                -steer_angle * tuning.rear_steer
            } else {
                steer_angle
            };
        }

        let queries = self.broad_phase.as_query_pipeline_mut(
            self.narrow_phase.query_dispatcher(),
            &mut self.rigid_body_set,
            &mut self.collider_set,
            QueryFilter::default().exclude_rigid_body(self.vehicle.body_handle),
        );
        self.vehicle.controller.update_vehicle(dt, queries);
        self.apply_drive_assists(dt, throttle, steer);
    }

    /// Arcade stability assists, applied as torque impulses on top of the
    /// wheel forces. Each one is a tunable in `VehicleTuning` (0 disables):
    /// roll-only anti-flip righting, and throttle/steer torque authority
    /// while no wheel touches the ground.
    fn apply_drive_assists(&mut self, dt: f32, throttle: f32, steer: f32) {
        let tuning = self.vehicle.tuning;
        let grounded_wheels = self
            .vehicle
            .controller
            .wheels()
            .iter()
            .filter(|wheel| wheel.raycast_info().is_in_contact)
            .count();

        let body = &mut self.rigid_body_set[self.vehicle.body_handle];
        let rotation = *body.rotation();
        let up = rotation * Vector::Y;
        let forward = rotation * Vector::Z;
        let angvel = body.angvel();
        let mut torque_impulse = Vector::ZERO;

        // Anti-roll: spring the car upright about its forward axis only, so
        // legitimate ramp pitch is never fought. atan2 keeps the torque
        // pointing upright even past 90° (self-rights a car on its side);
        // the clamp bounds the righting rate and the rate term damps ringing.
        if tuning.anti_roll > 0.0 {
            let roll = up.cross(Vector::Y).dot(forward).atan2(up.y);
            if roll.abs() > 0.02 {
                let roll_rate = angvel.dot(forward);
                let spring = roll.clamp(-0.8, 0.8) * tuning.anti_roll;
                let damping = roll_rate * tuning.anti_roll * 0.2;
                torque_impulse += forward * ((spring - damping) * dt);
            }
        }

        // Air control: with no wheel contact (jump, or beached on a ramp
        // edge) throttle pitches and steer yaws, so the player can rock the
        // car free or level a jump. Capped so it can't spin the car up.
        if tuning.air_control > 0.0 && grounded_wheels == 0 && angvel.length_squared() < 4.0 {
            let right = rotation * Vector::X;
            torque_impulse += right * (throttle * tuning.air_control * dt);
            torque_impulse += up * (steer * tuning.air_control * dt);
        }

        if torque_impulse.length_squared() > 1e-8 {
            body.apply_torque_impulse(torque_impulse, true);
        }
    }
}
