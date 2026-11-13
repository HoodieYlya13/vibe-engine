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
/// How far down its own ride a car looks for obstacles.
const LOOKAHEAD: f32 = FOLLOW_GAP * 3.0;
const PATH_SAMPLE_SPACING: f32 = 1.0;
/// Side/bumper clearance a rail car keeps to any obstacle footprint.
const CLEARANCE: f32 = 0.3;
/// Free travel left when a queued car stops: FOLLOW_GAP center-to-center
/// behind a parallel car.
const STOP_GAP: f32 = FOLLOW_GAP - 2.0 * CAR_HALF_LENGTH - CLEARANCE;
/// A car stuck this long behind crossing traffic stops yielding to it.
const YIELD_PATIENCE: f32 = 5.0;
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
/// Lateral distance from a lane's centerline to the middle of its sidewalk.
/// Kit geometry: the road tile is 12 m wide (curb at ±6 m) with a 1.5 m
/// sidewalk behind the curb, and lanes sit 2.6 m off the road center — so
/// the sidewalk middle lands 4.15 m to the lane's outer side.
const SIDEWALK_OFFSET: f32 = 4.15;

pub(crate) struct Lane {
    points: Vec<Vector>,
    /// Cumulative arc length up to each point; last entry = lane length.
    cum: Vec<f32>,
}

impl Lane {
    pub(crate) fn new(points: Vec<Vector>) -> Self {
        let mut cum = Vec::with_capacity(points.len());
        let mut total = 0.0;
        cum.push(0.0);
        for pair in points.windows(2) {
            total += (pair[1] - pair[0]).length();
            cum.push(total);
        }
        Lane { points, cum }
    }

    pub(crate) fn length(&self) -> f32 {
        *self.cum.last().unwrap_or(&0.0)
    }

    pub(crate) fn start(&self) -> Vector {
        self.points[0]
    }

    pub(crate) fn end(&self) -> Vector {
        *self.points.last().unwrap()
    }

    pub(crate) fn start_dir(&self) -> Vector {
        self.dir_at_segment(0)
    }

    pub(crate) fn end_dir(&self) -> Vector {
        self.dir_at_segment(self.points.len().saturating_sub(2))
    }

    fn dir_at_segment(&self, i: usize) -> Vector {
        let i = i.min(self.points.len().saturating_sub(2));
        (self.points[i + 1] - self.points[i])
            .try_normalize()
            .unwrap_or(Vector::Z)
    }

    /// Closest point on the polyline to `p` (horizontal distance): returns
    /// the arc distance and that distance.
    pub(crate) fn closest(&self, p: Vector) -> (f32, f32) {
        let mut best = (0.0, f32::MAX);
        for (i, pair) in self.points.windows(2).enumerate() {
            let seg = pair[1] - pair[0];
            let len_sq = seg.length_squared().max(1e-8);
            let t = ((p - pair[0]).dot(seg) / len_sq).clamp(0.0, 1.0);
            let mut to = p - (pair[0] + seg * t);
            to.y = 0.0;
            let dist = to.length();
            if dist < best.1 {
                best = (self.cum[i] + seg.length() * t, dist);
            }
        }
        best
    }

    /// Position and travel direction at arc distance `s`.
    pub(crate) fn sample(&self, s: f32) -> (Vector, Vector) {
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

/// Ground-plane rectangle a car-sized chassis covers.
#[derive(Clone, Copy)]
pub(crate) struct Footprint {
    center: Vector,
    forward: Vector,
}

impl Footprint {
    fn new(center: Vector, forward: Vector) -> Self {
        let flat = Vector::new(forward.x, 0.0, forward.z);
        Footprint {
            center,
            forward: flat.try_normalize().unwrap_or(Vector::Z),
        }
    }

    fn of_body(body: &RigidBody) -> Self {
        Footprint::new(body.translation(), *body.rotation() * Vector::Z)
    }

    fn right(&self) -> Vector {
        Vector::new(self.forward.z, 0.0, -self.forward.x)
    }

    fn reach_along(&self, axis: Vector) -> f32 {
        axis.dot(self.right()).abs() * CAR_HALF_WIDTH
            + axis.dot(self.forward).abs() * CAR_HALF_LENGTH
    }

    /// Separating-axis test of the two rectangles, each side kept `margin` apart.
    fn overlaps(&self, other: &Footprint, margin: f32) -> bool {
        let offset = other.center - self.center;
        [self.forward, self.right(), other.forward, other.right()]
            .into_iter()
            .all(|axis| {
                offset.dot(axis).abs() < self.reach_along(axis) + other.reach_along(axis) + margin
            })
    }
}

/// Another car as seen by a rail car's gap-keeping.
#[derive(Clone, Copy)]
struct Obstacle {
    footprint: Footprint,
    is_on_rail: bool,
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
    /// Seconds spent stopped behind other traffic.
    blocked_time: f32,
}

#[derive(Default)]
pub(crate) struct TrafficState {
    pub(crate) lanes: HashMap<(i32, i32), Vec<Lane>>,
    /// Sidewalk polylines derived from the lanes at load — the ped navgraph.
    pub(crate) walks: HashMap<(i32, i32), Vec<Lane>>,
    slots: Vec<Option<Car>>,
    spawn_timer: f32,
    rng: u32,
}

/// One sidewalk per lane, offset to the lane's outer side — the side away
/// from its opposite-direction twin (both directions of a street export as
/// separate lanes a few meters apart). A lane with no twin (one-way) gets an
/// arbitrary side.
fn derive_sidewalks(lanes: &[Lane]) -> Vec<Lane> {
    lanes
        .iter()
        .map(|lane| {
            let dir = (lane.end() - lane.start())
                .try_normalize()
                .unwrap_or(Vector::Z);
            let perp = Vector::new(-dir.z, 0.0, dir.x);
            let mid = (lane.start() + lane.end()) * 0.5;
            let mut side = 1.0f32;
            let mut best = f32::MAX;
            for other in lanes {
                let odir = (other.end() - other.start())
                    .try_normalize()
                    .unwrap_or(Vector::Z);
                if odir.dot(dir) > -0.7 {
                    continue;
                }
                let to = (other.start() + other.end()) * 0.5 - mid;
                let lateral = to.dot(perp);
                if !(0.5..=8.0).contains(&lateral.abs()) || to.dot(dir).abs() > lane.length() {
                    continue;
                }
                if lateral.abs() < best {
                    best = lateral.abs();
                    side = -lateral.signum();
                }
            }
            Lane::new(
                lane.points
                    .iter()
                    .map(|p| *p + perp * (side * SIDEWALK_OFFSET))
                    .collect(),
            )
        })
        .collect()
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

    pub(crate) fn sorted_keys(&self) -> Vec<(i32, i32)> {
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
                .as_chunks::<3>()
                .0
                .iter()
                .take(count)
                .map(|&[x, y, z]| origin + Vector::new(x, y, z))
                .collect();
            cursor += count * 3;
            if pts.len() >= 2 {
                lanes.push(Lane::new(pts));
            }
        }
        self.traffic
            .walks
            .insert((cx, cz), derive_sidewalks(&lanes));
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
        self.traffic.walks.remove(&(cx, cz));
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
        let player_car_footprint = Footprint::of_body(player_car);
        let player_car_vel = player_car.linvel();

        // Pose snapshot for gap-keeping (avoids borrowing during the walk).
        let obstacles: Vec<Option<Obstacle>> = self
            .traffic
            .slots
            .iter()
            .map(|slot| {
                slot.as_ref().map(|car| Obstacle {
                    footprint: Footprint::of_body(&self.rigid_body_set[car.body]),
                    is_on_rail: !matches!(car.ride, Ride::Loose { .. }),
                })
            })
            .collect();

        for i in 0..self.traffic.slots.len() {
            let Some(car) = self.traffic.slots[i].take() else {
                continue;
            };
            self.traffic.slots[i] =
                self.step_traffic_car(i, car, dt, &obstacles, player_car_footprint, player_car_vel);
            if let Some(obstacle) = obstacles[i]
                && (obstacle.footprint.center - focus).length() > DESPAWN_DIST
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
                blocked_time: 0.0,
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
        obstacles: &[Option<Obstacle>],
        player_car: Footprint,
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

        let Some(dir) = self.traffic_ride_dir(&car) else {
            self.remove_traffic_body(car);
            return None;
        };
        let own = Footprint::new(self.rigid_body_set[car.body].translation(), dir);

        // Dynamic conversion: the player car is closing in — hand the body to
        // the solver *before* the hit so the contact is car-vs-car, not
        // car-vs-wall. Predictive range grows with closing speed. Actual
        // overlap converts unconditionally: a teleport can drop the player
        // car inside a traffic car, and a kinematic body would crush it.
        let to_car = own.center - player_car.center;
        let dist = to_car.length();
        let closing = if dist > 1e-3 {
            player_car_vel.dot(to_car / dist)
        } else {
            0.0
        };
        let is_rammed = closing > 3.0 && dist < 4.2 + closing * 0.25;
        if is_rammed || own.overlaps(&player_car, CLEARANCE) {
            let body = &mut self.rigid_body_set[car.body];
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_linvel(dir * car.speed, true);
            car.ride = Ride::Loose { age: 0.0 };
            // Ramming a civilian car is a witnessed crime (PRD §7.3) — the
            // driver's crime: a driverless car shoved into traffic is not.
            if is_rammed && self.player.in_car {
                self.police
                    .report_crime(crate::police::CRIME_TRAFFIC_RAM, player_car.center);
            }
            return Some(car);
        }

        // Gap-keeping along the car's actual upcoming path (lane, then the
        // intersection hop, then the next lane): the player's car always
        // counts; other traffic counts when it is ahead on our heading, or
        // when it crosses our path and outranks us (lower slot).
        let is_patient = car.blocked_time < YIELD_PATIENCE;
        let mut free = self.free_distance(car.ride, &player_car);
        let mut is_blocked_by_traffic = false;
        for (j, other) in obstacles.iter().enumerate() {
            let Some(other) = other else { continue };
            if j == slot {
                continue;
            }
            let is_ahead_on_heading = dir.dot(other.footprint.forward) > 0.5;
            let must_yield = !other.is_on_rail || is_ahead_on_heading || (j < slot && is_patient);
            if !must_yield {
                continue;
            }
            // Overlapping traffic (spawn stacking) is ghosted through, as
            // rail cars do not collide with each other.
            if let Some(gap) = self.free_distance(car.ride, &other.footprint)
                && gap > 0.0
                && free.is_none_or(|current| gap < current)
            {
                free = Some(gap);
                is_blocked_by_traffic = true;
            }
        }

        let target_speed = match free {
            Some(gap) => CRUISE_SPEED * ((gap - STOP_GAP) / FOLLOW_GAP).clamp(0.0, 1.0),
            None => CRUISE_SPEED,
        };
        let rate = if target_speed > car.speed {
            ACCEL
        } else {
            BRAKE
        };
        car.speed += (target_speed - car.speed).clamp(-rate * dt, rate * dt);
        // Never bulldoze: a kinematic body is an unstoppable wall.
        if free.is_some_and(|gap| gap <= PATH_SAMPLE_SPACING) {
            car.speed = 0.0;
        }
        car.blocked_time = if is_blocked_by_traffic && car.speed < 0.1 {
            car.blocked_time + dt
        } else if car.speed > 1.0 {
            0.0
        } else {
            car.blocked_time
        };

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

    /// Position and heading `ahead` meters further down a rail ride. Past a
    /// lane's end the path is assumed to continue straight (the onward lane
    /// is only picked on arrival).
    fn pose_ahead(&self, ride: Ride, ahead: f32) -> Option<(Vector, Vector)> {
        match ride {
            Ride::Lane { chunk, lane, s } => {
                let lane = self.traffic.lanes.get(&chunk)?.get(lane)?;
                let overshoot = s + ahead - lane.length();
                if overshoot <= 0.0 {
                    Some(lane.sample(s + ahead))
                } else {
                    Some((lane.end() + lane.end_dir() * overshoot, lane.end_dir()))
                }
            }
            Ride::Bridge {
                from,
                chunk,
                lane,
                t,
                len,
            } => {
                let lane = self.traffic.lanes.get(&chunk)?.get(lane)?;
                let overshoot = t + ahead - len;
                if overshoot <= 0.0 {
                    let dir = (lane.start() - from).try_normalize()?;
                    Some((from + dir * (t + ahead), dir))
                } else {
                    Some(lane.sample(overshoot))
                }
            }
            Ride::Loose { .. } => None,
        }
    }

    /// Distance a rail car can still travel before its footprint would come
    /// within CLEARANCE of `obstacle`; None when the path ahead is clear.
    fn free_distance(&self, ride: Ride, obstacle: &Footprint) -> Option<f32> {
        let (origin, _) = self.pose_ahead(ride, 0.0)?;
        let reach = LOOKAHEAD + 2.0 * CAR_HALF_LENGTH + CLEARANCE;
        if (obstacle.center - origin).length_squared() > reach * reach {
            return None;
        }
        let mut ahead = 0.0;
        while ahead <= LOOKAHEAD {
            let (pos, dir) = self.pose_ahead(ride, ahead)?;
            if Footprint::new(pos, dir).overlaps(obstacle, CLEARANCE) {
                return Some(ahead);
            }
            ahead += PATH_SAMPLE_SPACING;
        }
        None
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
