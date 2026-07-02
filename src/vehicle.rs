use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

use crate::constants::{
    CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, VEHICLE_DIRECTION_SWITCH_SPEED,
    VEHICLE_ENGINE_BRAKE_IMPULSE, VEHICLE_FOOTBRAKE_IMPULSE, VEHICLE_HANDBRAKE_IMPULSE,
    VEHICLE_MAX_ENGINE_FORCE, VEHICLE_MAX_REVERSE_FORCE, VEHICLE_MAX_STEER,
};
use crate::state::{SimEngine, VehicleState};

pub(crate) fn create_vehicle(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
) -> VehicleState {
    let car_body = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 1.4, 0.0))
        .additional_mass(1200.0)
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

        let mut engine_force = 0.0;
        let mut brake = 0.0;
        if throttle > 0.01 {
            if forward_speed < -VEHICLE_DIRECTION_SWITCH_SPEED {
                brake = VEHICLE_FOOTBRAKE_IMPULSE * throttle;
            } else {
                engine_force = VEHICLE_MAX_ENGINE_FORCE * throttle;
            }
        } else if throttle < -0.01 {
            if forward_speed > VEHICLE_DIRECTION_SWITCH_SPEED {
                brake = VEHICLE_FOOTBRAKE_IMPULSE * -throttle;
            } else {
                engine_force = VEHICLE_MAX_REVERSE_FORCE * throttle;
            }
        } else {
            brake = VEHICLE_ENGINE_BRAKE_IMPULSE;
        }

        let steer_angle = steer * VEHICLE_MAX_STEER;

        for (index, wheel) in self.vehicle.controller.wheels_mut().iter_mut().enumerate() {
            let rear = index >= 2;
            if handbrake && rear {
                wheel.engine_force = 0.0;
                wheel.brake = VEHICLE_HANDBRAKE_IMPULSE;
            } else {
                wheel.engine_force = engine_force;
                wheel.brake = brake;
            }
            wheel.steering = if rear { 0.0 } else { steer_angle };
        }

        let queries = self.broad_phase.as_query_pipeline_mut(
            self.narrow_phase.query_dispatcher(),
            &mut self.rigid_body_set,
            &mut self.collider_set,
            QueryFilter::default().exclude_rigid_body(self.vehicle.body_handle),
        );
        self.vehicle.controller.update_vehicle(dt, queries);
    }
}
