mod spec;

use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;
use serde::Deserialize;

pub(crate) use spec::VehicleSpec;

use crate::constants::{CAR_CHASSIS_DROP, CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH};
use crate::engine::SimEngine;

/// Where the wheels attach, from the chassis center line and from its middle.
const WHEEL_HALF_TRACK: f32 = 0.8075;
const WHEEL_HALF_BASE: f32 = 1.52;
const WHEELBASE: f32 = 2.0 * WHEEL_HALF_BASE;
/// Width, height and length of the box the class masses were tuned to turn like.
const INERTIA_BOX: (f32, f32, f32) = (1.9, 1.6, 3.8);
/// Seconds the steering takes from center to full lock, and from full lock back to center.
const STEER_ENGAGE_TIME: f32 = 0.3;
const STEER_RELEASE_TIME: f32 = 0.12;
/// Below this speed (m/s) the car is parked: steering is not limited and nothing assists it.
const ROLLING_SPEED: f32 = 0.5;
/// Turn rate full lock commands in a handbrake slide (rad/s), reached at `SLIDE_FULL_SPEED`.
const SLIDE_TURN_RATE: f32 = 2.0;
const SLIDE_FULL_SPEED: f32 = 8.0;
/// Seconds the rear tires take to break loose under the handbrake, and to grip again after it.
const SLIDE_ENGAGE_TIME: f32 = 0.12;
const SLIDE_RECOVERY_TIME: f32 = 0.45;
/// How fast the turn rate follows the steering (1/s).
const YAW_ASSIST_RATE: f32 = 60.0;
/// How fast the car's momentum swings round to its nose (1/s): gripping, and in a full slide.
const GRIP_ALIGN_RATE: f32 = 15.0;
const SLIDE_ALIGN_RATE: f32 = 2.0;

fn approach(current: f32, target: f32, max_step: f32) -> f32 {
    current + (target - current).clamp(-max_step, max_step)
}

fn blend(gripping: f32, sliding: f32, looseness: f32) -> f32 {
    gripping + (sliding - gripping) * looseness
}

/// Moves the steering toward the input: unhurried into a turn, quick back to straight.
fn sweep_steering(position: f32, input: f32, dt: f32) -> f32 {
    let is_returning = input.abs() < position.abs() || input * position < 0.0;
    let sweep_time = if is_returning {
        STEER_RELEASE_TIME
    } else {
        STEER_ENGAGE_TIME
    };
    approach(position, input, dt / sweep_time)
}

/// The turn rate (rad/s) the wheels trace at `steer_angle` when nothing slips.
fn kinematic_turn_rate(steer_angle: f32, rear_steer: f32, forward_speed: f32) -> f32 {
    steer_angle * (1.0 + rear_steer) * forward_speed / WHEELBASE
}

/// The steering angle whose turn at `speed` uses all the cornering grip the tires have.
fn grip_limited_steer(cornering_grip: f32, rear_steer: f32, speed: f32) -> f32 {
    cornering_grip * WHEELBASE / ((1.0 + rear_steer) * speed * speed)
}

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
    /// Where the steering is: -1 full left, 0 straight, 1 full right.
    pub(crate) steering: f32,
    /// How far the rear tires have broken loose: 0 gripping, 1 sliding under the handbrake.
    pub(crate) slide: f32,
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
        steering: 0.0,
        slide: 0.0,
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
    let (w, h, l) = INERTIA_BOX;
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
        .translation(Vector::new(0.0, -CAR_CHASSIS_DROP, 0.0))
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
    let connection_height = -0.5;
    let direction = Vector::new(0.0, -1.0, 0.0);
    // Rapier derives the wheel's forward as contact_normal × axle; with -X the
    // forward comes out +Z, matching index_forward_axis so positive engine
    // force drives the chassis toward its steering wheels.
    let axle = Vector::new(-1.0, 0.0, 0.0);

    for (x_sign, z_sign) in [(-1.0, 1.0), (1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
        controller.add_wheel(
            Vector::new(
                x_sign * WHEEL_HALF_TRACK,
                connection_height,
                z_sign * WHEEL_HALF_BASE,
            ),
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

        self.vehicle.steering = sweep_steering(self.vehicle.steering, steer, dt);
        self.vehicle.slide = self.swept_slide(handbrake, dt);
        let steer_angle = self.vehicle.steering * self.steering_lock(forward_speed);

        // Anti-wheelie traction control: when the nose rises away from the
        // ROAD SURFACE, sustained thrust torque is what FEEDS the lift — cut
        // it instead of asking the righting springs to out-muscle the engine
        // (they can't; the overpowered classes creep into a stable 25-30°
        // wheelie under accelerate+turn). Lift is measured against the
        // averaged wheel-contact normal, not world up, so a ramp climb (nose
        // high but parallel to the slope) keeps full power even if a wheel
        // skips. The axle-planted gate keeps ordinary launch squat (both
        // lifted-axle wheels still touching) at full power; a fast corner
        // that floats the inner front does trip it — which is exactly the
        // regime that used to creep into a lift. Uses last frame's raycasts
        // (one frame of lag is nothing against a wheelie taking a second to
        // build).
        if engine_force != 0.0 {
            let wheels = self.vehicle.controller.wheels();
            let planted = |i: usize| wheels[i].raycast_info().is_in_contact;
            // The axle thrust lifts: front under forward drive, rear in reverse.
            let lifting_axle_planted = if engine_force > 0.0 {
                planted(0) && planted(1)
            } else {
                planted(2) && planted(3)
            };
            if !lifting_axle_planted {
                let mut normal = Vector::ZERO;
                for wheel in wheels {
                    let info = wheel.raycast_info();
                    if info.is_in_contact {
                        normal += info.contact_normal_ws;
                    }
                }
                if let Some(normal) = normal.try_normalize() {
                    let body = &self.rigid_body_set[self.vehicle.body_handle];
                    let lift = (body.rotation() * Vector::Z).dot(normal) * engine_force.signum();
                    if lift > 0.04 {
                        engine_force *= (1.0 - (lift - 0.04) / 0.06).clamp(0.0, 1.0);
                    }
                }
            }
        }

        // Handbrake slide: the tires keep only `handbrake_grip` of their grip
        // while the rear is loose, so the car slides instead of the brake just
        // scrubbing speed. Both knobs must shrink: `friction_slip` bounds the
        // saturated (sliding) grip, `side_friction_stiffness` the rest.
        let wheels_spec = &self.vehicle.classes[self.vehicle.class_index].wheels;
        let grip = blend(1.0, tuning.handbrake_grip, self.vehicle.slide);
        let side_friction = wheels_spec.side_friction_stiffness * grip;
        let friction_slip = wheels_spec.friction_slip * grip;

        for (index, wheel) in self.vehicle.controller.wheels_mut().iter_mut().enumerate() {
            let rear = index >= 2;
            wheel.side_friction_stiffness = side_friction;
            wheel.friction_slip = friction_slip;
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
        self.apply_drive_assists(dt, throttle, steer, steer_angle, forward_speed);
    }

    fn swept_slide(&self, handbrake: bool, dt: f32) -> f32 {
        if handbrake {
            approach(self.vehicle.slide, 1.0, dt / SLIDE_ENGAGE_TIME)
        } else {
            approach(self.vehicle.slide, 0.0, dt / SLIDE_RECOVERY_TIME)
        }
    }

    /// The widest angle the front wheels may take at this speed: full lock when
    /// parking, then only as much as the tires can hold, and less in reverse.
    fn steering_lock(&self, forward_speed: f32) -> f32 {
        let tuning = self.vehicle.tuning;
        let cornering_grip = self.vehicle.classes[self.vehicle.class_index]
            .wheels
            .cornering_grip;
        let speed = forward_speed.abs();
        if speed <= ROLLING_SPEED {
            return tuning.max_steer;
        }
        let lock =
            tuning
                .max_steer
                .min(grip_limited_steer(cornering_grip, tuning.rear_steer, speed));
        if forward_speed < 0.0 {
            return lock / (1.0 + tuning.reverse_stability * speed);
        }
        lock
    }

    /// The turn rate the steering asks for: what the wheels trace while the
    /// tires grip, and a fixed fast swing while the rear slides.
    fn commanded_turn_rate(&self, steer_angle: f32, forward_speed: f32, looseness: f32) -> f32 {
        let gripping =
            kinematic_turn_rate(steer_angle, self.vehicle.tuning.rear_steer, forward_speed);
        let slide_speed = (forward_speed / SLIDE_FULL_SPEED).clamp(-1.0, 1.0);
        let sliding = self.vehicle.steering * SLIDE_TURN_RATE * slide_speed;
        blend(gripping, sliding, looseness)
    }

    /// Arcade stability assists, applied as torque impulses on top of the
    /// wheel forces. Each one is a tunable in `VehicleTuning` (0 disables):
    /// anti-flip righting (roll always, pitch only past a deadzone no ramp
    /// reaches), and throttle/steer torque authority while no wheel touches
    /// the ground.
    fn apply_drive_assists(
        &mut self,
        dt: f32,
        throttle: f32,
        steer: f32,
        steer_angle: f32,
        forward_speed: f32,
    ) {
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

        // Anti-wheelie/stoppie. The visual-envelope chassis (short wheelbase,
        // light pitch inertia) can ride a stable wheelie under an overpowered
        // launch, and the roll-only spring can't touch that. Two bands, both
        // grounded-gated (in full flight air control keeps pitch authority):
        // pitch-RATE damping once pitch leaves the flat-driving band — it
        // kills the rotation a wheelie/stoppie needs to form, but stays out
        // of normal suspension motion (an always-on damper resonates with
        // the truck's suspension and pumps it airborne) — plus a righting
        // spring beyond a deadzone steeper than any drivable ramp, so
        // legitimate slope pitch is never fought.
        const PITCH_DAMP_START: f32 = 0.08;
        const PITCH_DEADZONE: f32 = 0.45;
        if tuning.anti_roll > 0.0 && grounded_wheels > 0 {
            let right = rotation * Vector::X;
            let pitch = up.cross(Vector::Y).dot(right).atan2(up.y);
            let pitch_rate = angvel.dot(right);
            if pitch.abs() > PITCH_DAMP_START {
                let mut pitch_torque = -pitch_rate * tuning.anti_roll * 0.3;
                let excess = pitch.abs() - PITCH_DEADZONE;
                if excess > 0.0 {
                    // 3x the roll gain: it must out-muscle sustained thrust
                    // torque, not just settle a transient.
                    pitch_torque += (excess.min(0.8) * pitch.signum()) * tuning.anti_roll * 3.0;
                }
                torque_impulse += right * (pitch_torque * dt);
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

        // Yaw + grip assist: wheel grip scales with suspension load, so hard
        // throttle (which unloads the front axle) kills turn-in, and lifting
        // off snaps the grip — and the yaw rate — right back. Torque-based
        // correction loses that fight (saturated tires out-torque any sane
        // spring), so this works at the velocity level, after all torque
        // impulses above: (a) blend the yaw component of angvel toward the
        // turn rate the steering commands; (b) rotate the momentum vector
        // toward the nose (magnitude preserved — tires redirecting momentum).
        // A handbrake slide loosens both, so the nose swings ahead of the
        // momentum. Needs a grounded wheel and a roughly level car: the model
        // means nothing mid-wheelie or on a wall.
        if grounded_wheels == 0 || forward_speed.abs() <= ROLLING_SPEED || up.y <= 0.8 {
            return;
        }
        let looseness = self.vehicle.slide * (1.0 - tuning.handbrake_grip);
        let target = self.commanded_turn_rate(steer_angle, forward_speed, looseness);
        let align_rate = blend(GRIP_ALIGN_RATE, SLIDE_ALIGN_RATE, looseness);

        let body = &mut self.rigid_body_set[self.vehicle.body_handle];
        let angvel = body.angvel();
        let err = target - angvel.dot(up);
        body.set_angvel(angvel + up * (err * (YAW_ASSIST_RATE * dt).min(1.0)), true);

        let linvel = body.linvel();
        let v_h = Vector::new(linvel.x, 0.0, linvel.z);
        let speed_h = v_h.length();
        let fwd_h = Vector::new(forward.x, 0.0, forward.z);
        if speed_h > ROLLING_SPEED
            && let Some(travel_dir) = (fwd_h * forward_speed.signum()).try_normalize()
        {
            let dir = v_h / speed_h;
            // Signed slip angle about +Y from velocity to travel heading.
            let slip = (dir.z * travel_dir.x - dir.x * travel_dir.z).atan2(dir.dot(travel_dir));
            let (s, c) = (slip * (align_rate * dt).min(1.0)).sin_cos();
            body.set_linvel(
                Vector::new(v_h.x * c + v_h.z * s, linvel.y, -v_h.x * s + v_h.z * c),
                true,
            );
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
