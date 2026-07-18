//! Ambient rail traffic (Phase 4, PRD §7.4): decorative cars that ride the
//! per-chunk lane polylines authored in Blender (`LANE*` curves in the chunk
//! sidecars). Each car is a kinematic body following its lane at street
//! speed, keeping a gap to whatever is ahead, and hopping onto the nearest
//! onward lane at intersections. When the player car is about to hit one it
//! converts to a plain dynamic body ("dynamic conversion") so the crash is a
//! shove against ~1.3 t of sedan, not a wall — after that the solver owns it
//! until it despawns.
//!
//! Everything is deterministic: lane pick and continuation use an xorshift
//! RNG stepped only by sim events, and every iteration over the lane map
//! sorts its chunk keys first.

use std::collections::HashMap;

use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::{CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, CHUNK_SIZE, MAX_TRAFFIC};
use crate::engine::SimEngine;

/// Cruising speed on an open lane (m/s) — city-street pace.
const CRUISE_SPEED: f32 = 9.0;
const ACCEL: f32 = 5.0;
const BRAKE: f32 = 9.0;
/// Center-to-center distance the follower aims to keep to whatever is ahead.
const FOLLOW_GAP: f32 = 7.0;
/// Spawn annulus around the focus (player / player car) and the cull radius.
const SPAWN_MIN_DIST: f32 = 45.0;
const SPAWN_MAX_DIST: f32 = 160.0;
const DESPAWN_DIST: f32 = 220.0;
/// One spawn attempt per interval while a slot is free.
const SPAWN_INTERVAL: f32 = 0.4;
/// Max gap between a lane's end and a continuation lane's start (lanes are
/// inset ~7 m from intersection centers, so crossings stay under this).
const CONTINUATION_RADIUS: f32 = 25.0;
/// Kinematic ride height: body center over flat ground.
const RIDE_HEIGHT: f32 = CAR_HALF_HEIGHT + 0.02;
/// Collider density giving the cuboid a sedan-ish ~1.3 t once dynamic.
const BODY_DENSITY: f32 = 115.0;
/// A loose (converted) car lives at least this long before it can despawn.
const LOOSE_LIFETIME: f32 = 12.0;

pub(crate) struct Lane {
    points: Vec<Vector>,
    /// Cumulative arc length up to each point; last entry = lane length.
    cum: Vec<f32>,
}

impl Lane {
    fn new(points: Vec<Vector>) -> Self {
        let mut cum = Vec::with_capacity(points.len());
        let mut total = 0.0;
        cum.push(0.0);
        for pair in points.windows(2) {
            total += (pair[1] - pair[0]).length();
            cum.push(total);
        }
        Lane { points, cum }
    }

    fn length(&self) -> f32 {
        *self.cum.last().unwrap_or(&0.0)
    }

    fn start(&self) -> Vector {
        self.points[0]
    }

    fn end(&self) -> Vector {
        *self.points.last().unwrap()
    }

    fn start_dir(&self) -> Vector {
        self.dir_at_segment(0)
    }

    fn end_dir(&self) -> Vector {
        self.dir_at_segment(self.points.len().saturating_sub(2))
    }

    fn dir_at_segment(&self, i: usize) -> Vector {
        let i = i.min(self.points.len().saturating_sub(2));
        (self.points[i + 1] - self.points[i])
            .try_normalize()
            .unwrap_or(Vector::Z)
    }

    /// Position and travel direction at arc distance `s`.
    fn sample(&self, s: f32) -> (Vector, Vector) {
        let s = s.clamp(0.0, self.length());
        let seg = self
            .cum
            .windows(2)
            .position(|w| s <= w[1])
            .unwrap_or(self.points.len().saturating_sub(2));
        let seg_len = (self.cum[seg + 1] - self.cum[seg]).max(1e-4);
        let t = (s - self.cum[seg]) / seg_len;
        (
            self.points[seg] + (self.points[seg + 1] - self.points[seg]) * t,
            self.dir_at_segment(seg),
        )
    }
}

#[derive(Clone, Copy)]
enum Ride {
    /// Following lane `lane` of chunk `chunk`, at arc distance `s`.
    Lane {
        chunk: (i32, i32),
        lane: usize,
        s: f32,
    },
    /// Crossing an intersection: straight hop from `from` to the start of
    /// the target lane.
    Bridge {
        from: Vector,
        chunk: (i32, i32),
        lane: usize,
        t: f32,
        len: f32,
    },
    /// Converted to a dynamic body — the solver owns it now.
    Loose { age: f32 },
}

struct Car {
    body: RigidBodyHandle,
    ride: Ride,
    speed: f32,
}

#[derive(Default)]
pub(crate) struct TrafficState {
    lanes: HashMap<(i32, i32), Vec<Lane>>,
    slots: Vec<Option<Car>>,
    spawn_timer: f32,
    rng: u32,
}

impl TrafficState {
    fn ensure_slots(&mut self) {
        if self.slots.is_empty() {
            self.slots = (0..MAX_TRAFFIC).map(|_| None).collect();
        }
        if self.rng == 0 {
            self.rng = 0x9E37_79B9;
        }
    }

    fn next_rng(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        x
    }

    fn sorted_keys(&self) -> Vec<(i32, i32)> {
        let mut keys: Vec<_> = self.lanes.keys().copied().collect();
        keys.sort_unstable();
        keys
    }
}

#[wasm_bindgen]
impl SimEngine {
    /// Stream in one chunk's lane polylines. `points` is the concatenation of
    /// every lane's (x, y, z) rows in chunk-local coordinates; `counts` gives
    /// the number of points per lane. Reloading a chunk replaces its lanes.
    pub fn load_chunk_lanes(&mut self, cx: i32, cz: i32, points: &[f32], counts: &[u32]) {
        let origin = Vector::new(cx as f32 * CHUNK_SIZE, 0.0, cz as f32 * CHUNK_SIZE);
        let mut lanes = Vec::with_capacity(counts.len());
        let mut cursor = 0usize;
        for &count in counts {
            let count = count as usize;
            let pts: Vec<Vector> = points[cursor..]
                .chunks_exact(3)
                .take(count)
                .map(|row| origin + Vector::new(row[0], row[1], row[2]))
                .collect();
            cursor += count * 3;
            if pts.len() >= 2 {
                lanes.push(Lane::new(pts));
            }
        }
        self.traffic.lanes.insert((cx, cz), lanes);
    }

    pub fn traffic_count(&self) -> u32 {
        self.traffic.slots.iter().flatten().count() as u32
    }
}

impl SimEngine {
    /// Despawn hook for chunk streaming: cars riding (or bridging into)
    /// lanes of this chunk vanish with it. Loose cars stay — they are no
    /// longer tied to any lane and get culled by distance.
    pub(crate) fn unload_chunk_traffic(&mut self, cx: i32, cz: i32) {
        self.traffic.lanes.remove(&(cx, cz));
        for i in 0..self.traffic.slots.len() {
            let on_chunk = match &self.traffic.slots[i] {
                Some(car) => match car.ride {
                    Ride::Lane { chunk, .. } | Ride::Bridge { chunk, .. } => chunk == (cx, cz),
                    Ride::Loose { .. } => false,
                },
                None => false,
            };
            if on_chunk {
                self.despawn_traffic_slot(i);
            }
        }
    }

    fn despawn_traffic_slot(&mut self, slot: usize) {
        if let Some(car) = self.traffic.slots[slot].take() {
            self.remove_traffic_body(car);
        }
    }

    fn remove_traffic_body(&mut self, car: Car) {
        self.rigid_body_set.remove(
            car.body,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );
    }

    pub(crate) fn update_traffic(&mut self, dt: f32) {
        self.traffic.ensure_slots();

        let focus = if self.player.in_car {
            self.rigid_body_set[self.vehicle.body_handle].translation()
        } else {
            self.player.pos
        };

        self.try_spawn_traffic(dt, focus);

        let player_car = &self.rigid_body_set[self.vehicle.body_handle];
        let player_car_pos = player_car.translation();
        let player_car_vel = player_car.linvel();

        // Pose snapshot for gap-keeping (avoids borrowing during the walk).
        let positions: Vec<Option<Vector>> = self
            .traffic
            .slots
            .iter()
            .map(|slot| {
                slot.as_ref()
                    .map(|car| self.rigid_body_set[car.body].translation())
            })
            .collect();

        for i in 0..self.traffic.slots.len() {
            let Some(car) = self.traffic.slots[i].take() else {
                continue;
            };
            self.traffic.slots[i] =
                self.step_traffic_car(i, car, dt, &positions, player_car_pos, player_car_vel);
            if let Some(pos) = positions[i]
                && (pos - focus).length() > DESPAWN_DIST
            {
                self.despawn_traffic_slot(i);
            }
        }
    }

    fn try_spawn_traffic(&mut self, dt: f32, focus: Vector) {
        self.traffic.spawn_timer += dt;
        if self.traffic.spawn_timer < SPAWN_INTERVAL || self.traffic.lanes.is_empty() {
            return;
        }
        self.traffic.spawn_timer = 0.0;
        let Some(slot) = self.traffic.slots.iter().position(Option::is_none) else {
            return;
        };

        let keys = self.traffic.sorted_keys();
        // A handful of deterministic random picks; give up quietly otherwise.
        for _ in 0..6 {
            let key_roll = self.traffic.next_rng() as usize;
            let lane_roll = self.traffic.next_rng() as usize;
            let key = keys[key_roll % keys.len()];
            let lanes = &self.traffic.lanes[&key];
            if lanes.is_empty() {
                continue;
            }
            let lane_index = lane_roll % lanes.len();
            let lane = &lanes[lane_index];
            let mut start = lane.start();
            start.y = RIDE_HEIGHT;
            let dir = lane.start_dir();
            let dist = (start - focus).length();
            if !(SPAWN_MIN_DIST..=SPAWN_MAX_DIST).contains(&dist) {
                continue;
            }
            let blocked = self.traffic.slots.iter().flatten().any(|other| {
                (self.rigid_body_set[other.body].translation() - start).length() < 12.0
            });
            if blocked {
                continue;
            }

            let mut body = RigidBodyBuilder::kinematic_position_based()
                .translation(start)
                .build();
            body.set_rotation(Rotation::from_rotation_y(dir.x.atan2(dir.z)), true);
            let handle = self.rigid_body_set.insert(body);
            let collider =
                ColliderBuilder::cuboid(CAR_HALF_WIDTH, CAR_HALF_HEIGHT, CAR_HALF_LENGTH)
                    .density(BODY_DENSITY)
                    .build();
            self.collider_set
                .insert_with_parent(collider, handle, &mut self.rigid_body_set);
            self.traffic.slots[slot] = Some(Car {
                body: handle,
                ride: Ride::Lane {
                    chunk: key,
                    lane: lane_index,
                    s: 0.0,
                },
                speed: CRUISE_SPEED * 0.5,
            });
            return;
        }
    }

    /// Advance one traffic car; returns the car unless it despawned.
    fn step_traffic_car(
        &mut self,
        slot: usize,
        mut car: Car,
        dt: f32,
        positions: &[Option<Vector>],
        player_car_pos: Vector,
        player_car_vel: Vector,
    ) -> Option<Car> {
        // Loose cars are the solver's problem; just age them out once settled.
        if let Ride::Loose { age } = &mut car.ride {
            *age += dt;
            if *age > LOOSE_LIFETIME
                && self.rigid_body_set[car.body].linvel().length_squared() < 0.5
            {
                self.remove_traffic_body(car);
                return None;
            }
            return Some(car);
        }

        let pos = self.rigid_body_set[car.body].translation();

        // Dynamic conversion: the player car is closing in — hand the body to
        // the solver *before* the hit so the contact is car-vs-car, not
        // car-vs-wall. Predictive range grows with closing speed. Overlap
        // converts unconditionally: a teleport can drop the player car
        // inside a traffic car, and a kinematic body would crush it.
        let to_car = pos - player_car_pos;
        let dist = to_car.length();
        let closing = if dist > 1e-3 {
            player_car_vel.dot(to_car / dist)
        } else {
            0.0
        };
        if dist < 3.5 || (closing > 3.0 && dist < 4.2 + closing * 0.25) {
            let dir = self.traffic_ride_dir(&car).unwrap_or(Vector::Z);
            let body = &mut self.rigid_body_set[car.body];
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_linvel(dir * car.speed, true);
            car.ride = Ride::Loose { age: 0.0 };
            return Some(car);
        }

        // Gap-keeping: slow for anything ahead in our travel corridor —
        // other traffic or the player's car (moving or parked).
        let Some(dir) = self.traffic_ride_dir(&car) else {
            self.remove_traffic_body(car);
            return None;
        };
        let mut target_speed = CRUISE_SPEED;
        let mut consider = |other: Vector| {
            let d = other - pos;
            let along = d.dot(dir);
            let lateral = (d - dir * along).length();
            if along > 0.0 && along < FOLLOW_GAP * 3.0 && lateral < 2.5 {
                let crawl = ((along - FOLLOW_GAP) / FOLLOW_GAP).clamp(0.0, 1.0);
                target_speed = target_speed.min(CRUISE_SPEED * crawl);
            }
        };
        for (j, other) in positions.iter().enumerate() {
            if j != slot
                && let Some(other) = other
            {
                consider(*other);
            }
        }
        consider(player_car_pos);

        let rate = if target_speed > car.speed {
            ACCEL
        } else {
            BRAKE
        };
        car.speed += (target_speed - car.speed).clamp(-rate * dt, rate * dt);

        // Advance along the ride and compute the new kinematic pose.
        let step = car.speed * dt;
        let (new_ride, (mut next_pos, next_dir)) = match car.ride {
            Ride::Lane { chunk, lane, s } => {
                let s = s + step;
                let lane_ref = &self.traffic.lanes[&chunk][lane];
                if s < lane_ref.length() {
                    (Ride::Lane { chunk, lane, s }, lane_ref.sample(s))
                } else {
                    // Lane finished: hop toward a continuation, or vanish.
                    let end = lane_ref.end();
                    let end_dir = lane_ref.end_dir();
                    match self.pick_continuation(end, end_dir) {
                        Some((to_chunk, to_lane, len)) => {
                            let to = self.traffic.lanes[&to_chunk][to_lane].start();
                            let dir = (to - end).try_normalize().unwrap_or(end_dir);
                            (
                                Ride::Bridge {
                                    from: end,
                                    chunk: to_chunk,
                                    lane: to_lane,
                                    t: 0.0,
                                    len,
                                },
                                (end, dir),
                            )
                        }
                        None => {
                            self.remove_traffic_body(car);
                            return None;
                        }
                    }
                }
            }
            Ride::Bridge {
                from,
                chunk,
                lane,
                t,
                len,
            } => {
                let t = t + step;
                let to = self.traffic.lanes[&chunk][lane].start();
                let dir = (to - from).try_normalize().unwrap_or(Vector::Z);
                if t >= len {
                    (
                        Ride::Lane {
                            chunk,
                            lane,
                            s: 0.0,
                        },
                        self.traffic.lanes[&chunk][lane].sample(0.0),
                    )
                } else {
                    (
                        Ride::Bridge {
                            from,
                            chunk,
                            lane,
                            t,
                            len,
                        },
                        (from + dir * t, dir),
                    )
                }
            }
            Ride::Loose { .. } => unreachable!("handled above"),
        };
        car.ride = new_ride;

        next_pos.y = RIDE_HEIGHT;
        let body = &mut self.rigid_body_set[car.body];
        body.set_next_kinematic_translation(next_pos);
        body.set_next_kinematic_rotation(Rotation::from_rotation_y(next_dir.x.atan2(next_dir.z)));
        Some(car)
    }

    /// Travel direction of a rail car (None if its lane vanished mid-frame).
    fn traffic_ride_dir(&self, car: &Car) -> Option<Vector> {
        match car.ride {
            Ride::Lane { chunk, lane, s } => {
                let lane = self.traffic.lanes.get(&chunk)?.get(lane)?;
                Some(lane.sample(s).1)
            }
            Ride::Bridge {
                from, chunk, lane, ..
            } => {
                let to = self.traffic.lanes.get(&chunk)?.get(lane)?.start();
                (to - from).try_normalize()
            }
            Ride::Loose { .. } => None,
        }
    }

    /// Nearest onward lane whose start sits across the intersection from
    /// `end` and doesn't double straight back.
    fn pick_continuation(
        &mut self,
        end: Vector,
        end_dir: Vector,
    ) -> Option<((i32, i32), usize, f32)> {
        let mut candidates = Vec::new();
        for key in self.traffic.sorted_keys() {
            for (index, lane) in self.traffic.lanes[&key].iter().enumerate() {
                let gap = lane.start() - end;
                let len = gap.length();
                if !(1e-3..=CONTINUATION_RADIUS).contains(&len) {
                    continue;
                }
                // No U-turns onto the opposite-direction twin of the same
                // street, and the hop itself must head roughly onward.
                if lane.start_dir().dot(end_dir) < -0.3 || (gap / len).dot(end_dir) < 0.1 {
                    continue;
                }
                candidates.push((key, index, len));
            }
        }
        if candidates.is_empty() {
            return None;
        }
        let pick = self.traffic.next_rng() as usize % candidates.len();
        Some(candidates[pick])
    }

    /// Snapshot support: pose of slot `i`, if occupied.
    pub(crate) fn traffic_slot_pose(&self, i: usize) -> Option<(Vector, Rotation)> {
        let car = self.traffic.slots.get(i)?.as_ref()?;
        let body = &self.rigid_body_set[car.body];
        Some((body.translation(), *body.rotation()))
    }

    /// Active traffic positions for the crowd's avoidance pass.
    pub(crate) fn traffic_positions(&self) -> Vec<Vector> {
        self.traffic
            .slots
            .iter()
            .flatten()
            .map(|car| self.rigid_body_set[car.body].translation())
            .collect()
    }
}
