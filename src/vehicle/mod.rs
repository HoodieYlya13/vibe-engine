mod spec;

use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;
use serde::Deserialize;

pub(crate) use spec::VehicleSpec;

use crate::constants::{CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH};
use crate::engine::SimEngine;

/// Runtime-mutable driving parameters. Defaults come from the active class in
/// `data/vehicles.ron`; the dev console can overwrite them live so iterating
/// on car feel never requires a wasm rebuild (PRD §8.3/§8.6).
#[derive(Clone, Copy, Deserialize)]
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
    pub(crate) handbrake_grip: f32,
}

pub(crate) struct VehicleState {
    pub(crate) body_handle: RigidBodyHandle,
    pub(crate) controller: DynamicRayCastVehicleController,
    pub(crate) spawn_rotation: Rotation,
    pub(crate) tuning: VehicleTuning,
    pub(crate) classes: Vec<VehicleSpec>,
    pub(crate) class_index: usize,
    /// 1.0 = pristine, 0.0 = destroyed (drivetrain dead until reset).
    pub(crate) health: f32,
}

pub(crate) fn create_vehicle(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
) -> VehicleState {
    let classes = spec::load_vehicle_classes();
    let class_index = 1;
    let class = classes[class_index].clone();
    let (body_handle, controller) = build_vehicle_body(rigid_body_set, collider_set, &class);

    VehicleState {
        body_handle,
        controller,
        spawn_rotation: Rotation::IDENTITY,
        tuning: class.drive,
        classes,
        class_index,
        health: 1.0,
    }
}

/// Builds the chassis body + collider + wheel controller for one class spec.
/// Also used when the dev console swaps classes at runtime.
pub(crate) fn build_vehicle_body(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    class: &VehicleSpec,
) -> (RigidBodyHandle, DynamicRayCastVehicleController) {
    // The class mass sits below the chassis center so lateral grip has less
    // leverage to roll the car over. The principal inertia is the full-size
    // cuboid's at that mass, so yaw/pitch response scales with the class.
    let (w, h, l) = (
        2.0 * CAR_HALF_WIDTH,
        2.0 * CAR_HALF_HEIGHT,
        2.0 * CAR_HALF_LENGTH,
    );
    let inertia = Vector::new(
        class.chassis.mass / 12.0 * (h * h + l * l),
        class.chassis.mass / 12.0 * (w * w + l * l),
        class.chassis.mass / 12.0 * (w * w + h * h),
    );
    let car_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 1.4, 0.0))
        .additional_mass_properties(MassProperties::new(
            Vector::new(0.0, -class.chassis.com_drop, 0.0),
            class.chassis.mass,
            inertia,
        ))
        .linear_damping(class.chassis.linear_damping)
        .angular_damping(class.chassis.angular_damping)
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

    let wheels = &class.wheels;
    let tuning = WheelTuning {
        suspension_stiffness: wheels.suspension_stiffness,
        suspension_compression: wheels.suspension_compression,
        suspension_damping: wheels.suspension_damping,
        max_suspension_travel: wheels.max_suspension_travel,
        side_friction_stiffness: wheels.side_friction_stiffness,
        friction_slip: wheels.friction_slip,
        max_suspension_force: wheels.max_suspension_force,
    };
    let half_width = CAR_HALF_WIDTH * 0.85;
    let half_length = CAR_HALF_LENGTH * 0.8;
    let connection_height = -0.5;
    let direction = Vector::new(0.0, -1.0, 0.0);
    // Rapier derives the wheel's forward as contact_normal × axle; with -X the
    // forward comes out +Z, matching index_forward_axis so positive engine
    // force drives the chassis toward its steering wheels.
    let axle = Vector::new(-1.0, 0.0, 0.0);

    for (x_sign, z_sign) in [(-1.0, 1.0), (1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
        controller.add_wheel(
            Vector::new(x_sign * half_width, connection_height, z_sign * half_length),
            direction,
            axle,
            wheels.suspension_rest,
            wheels.radius,
            &tuning,
        );
    }

    (car_handle, controller)
}

impl SimEngine {
    pub(crate) fn step_vehicle(&mut self, dt: f32) {
        // A destroyed car (health 0) has no drivetrain: inputs are ignored
        // and the wreck coasts on engine braking until a reset repairs it.
        let (throttle, steer, handbrake) = if self.player.in_car && self.vehicle.health > 0.0 {
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

        // Handbrake drift: while the handbrake is held the rear tires keep
        // only `handbrake_grip` of their grip, so the tail breaks loose and
        // swings instead of the brake just scrubbing speed. Both knobs must
        // shrink: `friction_slip` bounds the saturated (sliding) grip that
        // dominates hard cornering, `side_friction_stiffness` the rest.
        let wheels_spec = &self.vehicle.classes[self.vehicle.class_index].wheels;
        let grip = if handbrake {
            tuning.handbrake_grip
        } else {
            1.0
        };
        let rear_side_friction = wheels_spec.side_friction_stiffness * grip;
        let rear_friction_slip = wheels_spec.friction_slip * grip;

        for (index, wheel) in self.vehicle.controller.wheels_mut().iter_mut().enumerate() {
            let rear = index >= 2;
            if rear {
                wheel.side_friction_stiffness = rear_side_friction;
                wheel.friction_slip = rear_friction_slip;
            }
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

    /// Chassis impact damage (PRD §7.1 "cosmetic-plus"), measured as the
    /// velocity change across the physics solver step. Wheel and suspension
    /// impulses are applied earlier in `update_vehicle`, so the solver's Δv
    /// is gravity (≈0.16 m/s per step) plus chassis contact impulses — wall
    /// hits, bottoming out, landing on the nose or roof. A clean wheels-first
    /// landing is absorbed by the suspension and never shows up here.
    pub(crate) fn apply_impact_damage(&mut self, linvel_before: Vector) {
        let spec = self.vehicle.classes[self.vehicle.class_index].damage;
        let body = &self.rigid_body_set[self.vehicle.body_handle];
        let overshoot = (body.linvel() - linvel_before).length() - spec.impact_threshold;
        if overshoot > 0.0 {
            self.damage_vehicle(overshoot * spec.impact_scale);
        }
    }

    /// Removes `amount` health; crossing zero "explodes" the car — an upward
    /// pop on the wreck, and `step_vehicle` keeps the drivetrain dead until
    /// a reset (or class switch) repairs it.
    pub(crate) fn damage_vehicle(&mut self, amount: f32) {
        if self.vehicle.health <= 0.0 || amount <= 0.0 {
            return;
        }
        self.vehicle.health = (self.vehicle.health - amount).max(0.0);
        if self.vehicle.health <= 0.0
            && let Some(body) = self.rigid_body_set.get_mut(self.vehicle.body_handle)
        {
            let pop = body.mass() * 4.0;
            body.apply_impulse(Vector::Y * pop, true);
            body.wake_up(true);
        }
    }
}
