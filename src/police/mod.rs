//! Wanted / heat system (Phase 4, PRD §7.3 — ★1–3 scope). Crimes the crowd
//! and traffic report (running peds over, ramming cars) build a continuous
//! heat value; the star count derives from thresholds. While wanted, police
//! cruisers spawn on the streamed lanes inside the density bubble and chase
//! the player by riding the traffic rails with target-biased routing — at
//! every intersection they pick the continuation pointing most toward the
//! last known player position — so they never clip through buildings. With
//! line-of-sight (a rapier raycast against the fixed world) and the player
//! close, a ★2+ cop leaves the rails and lunges straight down the sighted
//! line; contact converts it to a dynamic body exactly like ambient traffic,
//! so a ram is a solver crash, not a kinematic wall. Losing every cop's
//! line-of-sight starts the escape clock: after a grace period the heat
//! decays to zero and the pursuit dissolves. At ★3 pairs of cruisers park
//! across the road ahead of the player's travel as roadblocks.
//!
//! Deterministic like traffic/crowd: own xorshift RNG stepped only by sim
//! events, sorted chunk keys, fixed iteration order; the LOS raycast runs on
//! rapier's BVH which is rebuilt identically every step.

use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::{CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, MAX_POLICE};
use crate::engine::SimEngine;

/// Heat added per crime. Stars: ★1 at 1.0, ★2 at 3.0, ★3 at 6.0.
pub(crate) const CRIME_PED_HIT: f32 = 1.0;
pub(crate) const CRIME_TRAFFIC_RAM: f32 = 0.7;
const STAR_THRESHOLDS: [f32; 3] = [1.0, 3.0, 6.0];
const HEAT_MAX: f32 = 8.0;
/// Unseen by every cop this long → heat starts draining.
const SEARCH_GRACE: f32 = 8.0;
const HEAT_DECAY: f32 = 0.2;

/// Pursuit cruisers per star level (the rest of the pool is roadblock budget).
const COPS_FOR_STARS: [usize; 4] = [0, 1, 3, 4];
const ROADBLOCK_CARS: usize = 2;
/// Cruiser speed by star level: ★1 tails at street pace and disengages
/// easily; ★2+ genuinely chase (the player still outruns them flat out).
const PURSUIT_SPEED: [f32; 4] = [0.0, 9.0, 16.0, 19.0];
const ACCEL: f32 = 7.0;
const BRAKE: f32 = 10.0;
/// ★1 keeps this distance to the player instead of making contact.
const FOLLOW_GAP: f32 = 7.0;
/// Off-rail steering rate while lunging (rad/s).
const TURN_RATE: f32 = 1.8;
/// A ★2+ cop with line-of-sight inside this range leaves the rails and
/// drives the sighted straight line (which the LOS ray proved clear).
const LUNGE_RANGE: f32 = 40.0;
/// Predicted-position lead while the player is in sight (seconds of travel).
const LEAD_TIME: f32 = 0.5;

/// Eye height for the line-of-sight ray: above curbs, below buildings.
const EYE_HEIGHT: f32 = 1.3;
/// Longer than the spawn ring's inner edge, so a fresh cruiser heading in
/// can pin the player before the heat starts decaying.
const SIGHT_RANGE: f32 = 80.0;

const SPAWN_MIN_DIST: f32 = 50.0;
const SPAWN_MAX_DIST: f32 = 140.0;
const DESPAWN_DIST: f32 = 240.0;
const SPAWN_INTERVAL: f32 = 0.7;
/// Roadblocks drop ahead of a fast player at ★3, this far down their travel.
const ROADBLOCK_INTERVAL: f32 = 9.0;
const ROADBLOCK_LEAD: f32 = 90.0;

const RIDE_HEIGHT: f32 = CAR_HALF_HEIGHT + 0.02;
/// Same sedan-ish mass as traffic once converted to dynamic.
const BODY_DENSITY: f32 = 115.0;
/// A converted (dynamic) cruiser recycles once it has settled or fallen far
/// behind, freeing the slot for a fresh spawn.
const LOOSE_LIFETIME: f32 = 6.0;

#[derive(Clone, Copy)]
enum Duty {
    /// Riding a lane toward the target (traffic-style, biased routing).
    Rail {
        chunk: (i32, i32),
        lane: usize,
        s: f32,
    },
    /// Hopping an intersection onto the next lane.
    Bridge {
        from: Vector,
        chunk: (i32, i32),
        lane: usize,
        t: f32,
        len: f32,
    },
    /// Off the rails, steering straight at the sighted player.
    Lunge,
    /// Parked across the road as part of a ★3 roadblock.
    Block,
    /// Converted to a dynamic body — the solver owns it.
    Loose { age: f32 },
}

struct Cop {
    body: RigidBodyHandle,
    duty: Duty,
    speed: f32,
    yaw: f32,
}

pub(crate) struct PoliceState {
    slots: Vec<Option<Cop>>,
    pub(crate) heat: f32,
    /// Where the pursuit believes the player is: live while any cop has
    /// line-of-sight, frozen at the last sighting otherwise.
    last_seen: Vector,
    seen: bool,
    unseen_time: f32,
    spawn_timer: f32,
    roadblock_timer: f32,
    rng: u32,
}

impl Default for PoliceState {
    fn default() -> Self {
        PoliceState {
            slots: Vec::new(),
            heat: 0.0,
            last_seen: Vector::ZERO,
            seen: false,
            unseen_time: 0.0,
            spawn_timer: 0.0,
            roadblock_timer: 0.0,
            rng: 0x5EED_C0DE,
        }
    }
}

impl PoliceState {
    fn ensure_slots(&mut self) {
        if self.slots.is_empty() {
            self.slots = (0..MAX_POLICE).map(|_| None).collect();
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

    pub(crate) fn stars(&self) -> u32 {
        STAR_THRESHOLDS.iter().filter(|&&t| self.heat >= t).count() as u32
    }

    /// A witnessed crime: adds heat and pins the pursuit onto the scene.
    pub(crate) fn report_crime(&mut self, weight: f32, pos: Vector) {
        self.heat = (self.heat + weight).min(HEAT_MAX);
        self.last_seen = pos;
        self.unseen_time = 0.0;
    }
}

#[wasm_bindgen]
impl SimEngine {
    pub fn wanted_stars(&self) -> u32 {
        self.police.stars()
    }

    pub fn police_count(&self) -> u32 {
        self.police.slots.iter().flatten().count() as u32
    }

    /// Dev tool: pins the wanted level (0–3). Setting a level plants the
    /// pursuit on the player's current position, as if a crime just happened.
    pub fn set_wanted(&mut self, stars: u32) {
        let stars = stars.min(3) as usize;
        self.police.heat = [
            0.0,
            STAR_THRESHOLDS[0],
            STAR_THRESHOLDS[1],
            STAR_THRESHOLDS[2],
        ][stars];
        if stars > 0 {
            self.police.last_seen = self.focus_pos();
            self.police.unseen_time = 0.0;
        }
    }
}

impl SimEngine {
    fn focus_pos(&self) -> Vector {
        if self.player.in_car {
            self.rigid_body_set[self.vehicle.body_handle].translation()
        } else {
            self.player.pos
        }
    }

    /// True if an eye at `from` sees the player focus: within range and no
    /// fixed world collider (building, wall) blocks the straight line.
    fn cop_sees(&self, from: Vector, focus: Vector) -> bool {
        let mut origin = from;
        origin.y = EYE_HEIGHT;
        let mut target = focus;
        target.y = EYE_HEIGHT;
        let to = target - origin;
        let dist = to.length();
        if dist > SIGHT_RANGE {
            return false;
        }
        if dist < 1e-3 {
            return true;
        }
        let pipeline = self.broad_phase.as_query_pipeline(
            self.narrow_phase.query_dispatcher(),
            &self.rigid_body_set,
            &self.collider_set,
            QueryFilter::only_fixed(),
        );
        pipeline
            .cast_ray(&Ray::new(origin, to / dist), dist, true)
            .is_none()
    }

    pub(crate) fn update_police(&mut self, dt: f32) {
        self.police.ensure_slots();

        let focus = self.focus_pos();
        let focus_vel = if self.player.in_car {
            self.rigid_body_set[self.vehicle.body_handle].linvel()
        } else {
            Vector::ZERO
        };

        // Sight pass (immutable): does any cop have line-of-sight?
        let cop_positions: Vec<Option<Vector>> = self
            .police
            .slots
            .iter()
            .map(|slot| {
                slot.as_ref()
                    .map(|cop| self.rigid_body_set[cop.body].translation())
            })
            .collect();
        let sights: Vec<bool> = cop_positions
            .iter()
            .map(|pos| pos.is_some_and(|p| self.cop_sees(p, focus)))
            .collect();

        if sights.iter().any(|&s| s) {
            self.police.seen = true;
            self.police.last_seen = focus;
            self.police.unseen_time = 0.0;
        } else {
            self.police.seen = false;
            self.police.unseen_time += dt;
        }

        // Escape: unseen past the grace period drains the heat away.
        if self.police.heat > 0.0 && self.police.unseen_time > SEARCH_GRACE {
            self.police.heat = (self.police.heat - HEAT_DECAY * dt).max(0.0);
        }

        let stars = self.police.stars() as usize;
        if stars == 0 {
            // Pursuit over (any sub-star residual heat just cools quietly).
            for i in 0..self.police.slots.len() {
                self.despawn_police_slot(i);
            }
            return;
        }

        self.try_spawn_police(dt, focus, stars);
        self.try_spawn_roadblock(dt, focus, focus_vel, stars);

        // Pursuit target: predicted position while sighted, last known
        // position while searching.
        let target = if self.police.seen {
            self.police.last_seen + focus_vel * LEAD_TIME
        } else {
            self.police.last_seen
        };

        for i in 0..self.police.slots.len() {
            let Some(cop) = self.police.slots[i].take() else {
                continue;
            };
            let pos = cop_positions[i].unwrap_or(focus);
            if (pos - focus).length() > DESPAWN_DIST {
                self.remove_police_body(cop);
                continue;
            }
            self.police.slots[i] = self.step_cop(
                i,
                cop,
                dt,
                stars,
                target,
                focus,
                focus_vel,
                sights[i],
                &cop_positions,
            );
        }
    }

    fn despawn_police_slot(&mut self, slot: usize) {
        if let Some(cop) = self.police.slots[slot].take() {
            self.remove_police_body(cop);
        }
    }

    fn remove_police_body(&mut self, cop: Cop) {
        self.rigid_body_set.remove(
            cop.body,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );
    }

    fn spawn_cop_body(&mut self, pos: Vector, yaw: f32) -> RigidBodyHandle {
        let mut body = RigidBodyBuilder::kinematic_position_based()
            .translation(Vector::new(pos.x, RIDE_HEIGHT, pos.z))
            .build();
        body.set_rotation(Rotation::from_rotation_y(yaw), true);
        let handle = self.rigid_body_set.insert(body);
        let collider = ColliderBuilder::cuboid(CAR_HALF_WIDTH, CAR_HALF_HEIGHT, CAR_HALF_LENGTH)
            .density(BODY_DENSITY)
            .build();
        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        handle
    }

    fn try_spawn_police(&mut self, dt: f32, focus: Vector, stars: usize) {
        self.police.spawn_timer += dt;
        if self.police.spawn_timer < SPAWN_INTERVAL || self.traffic.lanes.is_empty() {
            return;
        }
        let pursuit_count = self
            .police
            .slots
            .iter()
            .flatten()
            .filter(|cop| !matches!(cop.duty, Duty::Block))
            .count();
        if pursuit_count >= COPS_FOR_STARS[stars] {
            return;
        }
        self.police.spawn_timer = 0.0;
        let Some(slot) = self.police.slots.iter().position(Option::is_none) else {
            return;
        };

        let keys = self.traffic.sorted_keys();
        for _ in 0..6 {
            let key_roll = self.police.next_rng() as usize;
            let lane_roll = self.police.next_rng() as usize;
            let s_roll = self.police.next_rng();
            let key = keys[key_roll % keys.len()];
            let lanes = &self.traffic.lanes[&key];
            if lanes.is_empty() {
                continue;
            }
            let lane_index = lane_roll % lanes.len();
            let lane = &lanes[lane_index];
            // Anywhere along the lane, not just its start — and only where
            // riding it heads toward the player: a cruiser that boots facing
            // away wastes its whole approach.
            let s = (s_roll % 1000) as f32 / 1000.0 * lane.length();
            let (point, dir) = lane.sample(s);
            let dist = (point - focus).length();
            if !(SPAWN_MIN_DIST..=SPAWN_MAX_DIST).contains(&dist) {
                continue;
            }
            if dir.dot((focus - point) / dist) < 0.2 {
                continue;
            }
            let handle = self.spawn_cop_body(point, dir.x.atan2(dir.z));
            self.police.slots[slot] = Some(Cop {
                body: handle,
                duty: Duty::Rail {
                    chunk: key,
                    lane: lane_index,
                    s,
                },
                speed: PURSUIT_SPEED[stars] * 0.5,
                yaw: dir.x.atan2(dir.z),
            });
            return;
        }
    }

    /// ★3: park a pair of cruisers across the road ahead of a fast player.
    fn try_spawn_roadblock(&mut self, dt: f32, focus: Vector, focus_vel: Vector, stars: usize) {
        self.police.roadblock_timer += dt;
        if stars < 3
            || self.police.roadblock_timer < ROADBLOCK_INTERVAL
            || self.traffic.lanes.is_empty()
        {
            return;
        }
        let speed = focus_vel.length();
        if speed < 8.0 {
            return;
        }
        let block_count = self
            .police
            .slots
            .iter()
            .flatten()
            .filter(|cop| matches!(cop.duty, Duty::Block))
            .count();
        if block_count >= ROADBLOCK_CARS {
            return;
        }
        self.police.roadblock_timer = 0.0;

        // Nearest lane point to the predicted player position down the road.
        let predicted = focus + focus_vel / speed * ROADBLOCK_LEAD;
        let mut best: Option<(Vector, Vector, f32)> = None;
        for key in self.traffic.sorted_keys() {
            for lane in &self.traffic.lanes[&key] {
                let (s, dist) = lane.closest(predicted);
                if best.is_none_or(|b| dist < b.2) {
                    let (point, dir) = lane.sample(s);
                    best = Some((point, dir, dist));
                }
            }
        }
        let Some((point, dir, dist)) = best else {
            return;
        };
        if dist > 15.0 {
            return;
        }

        // Two cars nose-to-tail across the lane, perpendicular to travel.
        let perp = Vector::new(-dir.z, 0.0, dir.x);
        let yaw = perp.x.atan2(perp.z);
        for offset in [-1.0f32, 1.0] {
            let Some(slot) = self.police.slots.iter().position(Option::is_none) else {
                return;
            };
            let pos = point + perp * (offset * (CAR_HALF_LENGTH + 0.15));
            let handle = self.spawn_cop_body(pos, yaw);
            self.police.slots[slot] = Some(Cop {
                body: handle,
                duty: Duty::Block,
                speed: 0.0,
                yaw,
            });
        }
    }

    /// Advance one cruiser; returns it unless it despawned.
    #[allow(clippy::too_many_arguments)]
    fn step_cop(
        &mut self,
        slot: usize,
        mut cop: Cop,
        dt: f32,
        stars: usize,
        target: Vector,
        focus: Vector,
        focus_vel: Vector,
        sighted: bool,
        cop_positions: &[Option<Vector>],
    ) -> Option<Cop> {
        // Converted cruisers belong to the solver; recycle once settled.
        if let Duty::Loose { age } = &mut cop.duty {
            *age += dt;
            let body = &self.rigid_body_set[cop.body];
            let settled = body.linvel().length_squared() < 0.5;
            let behind = (body.translation() - focus).length() > 50.0;
            if *age > LOOSE_LIFETIME && (settled || behind) {
                self.remove_police_body(cop);
                return None;
            }
            return Some(cop);
        }

        let pos = self.rigid_body_set[cop.body].translation();

        // Defensive conversion, exactly like traffic: the player car closing
        // in (or overlapping) flips the kinematic cruiser to dynamic so the
        // crash is a shove, not a wall. This is also how a ★2+ ram lands.
        let to_cop = pos - focus;
        let dist = to_cop.length();
        let closing = if dist > 1e-3 {
            focus_vel.dot(to_cop / dist)
        } else {
            0.0
        };
        let local = self.rigid_body_set[cop.body].rotation().inverse() * -to_cop;
        let overlapping = local.x.abs() < CAR_HALF_WIDTH * 2.0 + 0.3
            && local.z.abs() < CAR_HALF_LENGTH * 2.0 + 0.3;
        if overlapping || (closing > 3.0 && dist < 4.2 + closing * 0.25) {
            let (sin, cos) = cop.yaw.sin_cos();
            let body = &mut self.rigid_body_set[cop.body];
            body.set_body_type(RigidBodyType::Dynamic, true);
            body.set_linvel(Vector::new(sin, 0.0, cos) * cop.speed, true);
            cop.duty = Duty::Loose { age: 0.0 };
            return Some(cop);
        }

        // Roadblock cars just sit there (conversion above handles the hit).
        if matches!(cop.duty, Duty::Block) {
            if stars < 3 {
                self.remove_police_body(cop);
                return None;
            }
            return Some(cop);
        }

        // ★2+ with line-of-sight and the player close: leave the rails and
        // drive the sighted straight line (the LOS ray proved it clear).
        if stars >= 2 && sighted && dist < LUNGE_RANGE {
            cop.duty = Duty::Lunge;
        } else if matches!(cop.duty, Duty::Lunge) && !sighted {
            // Lost sight mid-lunge: get back on the road grid.
            cop.duty = match self.nearest_police_lane(pos) {
                Some((chunk, lane, s)) => Duty::Rail { chunk, lane, s },
                None => Duty::Lunge,
            };
        }

        // Speed control: chase pace, tempered by the cruiser ahead (they
        // don't rear-end each other) and by the player at ★1 — the lone ★1
        // cruiser tails, it doesn't ram. Civilian traffic is deliberately
        // ignored: a pursuit doesn't queue behind a 9 m/s sedan.
        let mut target_speed = PURSUIT_SPEED[stars];
        if !matches!(cop.duty, Duty::Lunge) {
            let (sin, cos) = cop.yaw.sin_cos();
            let heading = Vector::new(sin, 0.0, cos);
            let mut consider = |other: Vector| {
                let d = other - pos;
                let along = d.dot(heading);
                let lateral = (d - heading * along).length();
                if along > 0.0 && along < FOLLOW_GAP * 3.0 && lateral < 2.5 {
                    let crawl = ((along - FOLLOW_GAP) / FOLLOW_GAP).clamp(0.0, 1.0);
                    target_speed = target_speed.min(PURSUIT_SPEED[stars] * crawl);
                }
            };
            for (j, other) in cop_positions.iter().enumerate() {
                if j != slot
                    && let Some(other) = other
                {
                    consider(*other);
                }
            }
            if stars == 1 {
                consider(focus);
            }
        }
        let rate = if target_speed > cop.speed {
            ACCEL
        } else {
            BRAKE
        };
        cop.speed += (target_speed - cop.speed).clamp(-rate * dt, rate * dt);

        let step = cop.speed * dt;
        let (new_duty, next_pos, next_dir) = match cop.duty {
            Duty::Lunge => {
                let to = Vector::new(target.x - pos.x, 0.0, target.z - pos.z);
                let desired = to.x.atan2(to.z);
                let diff = (desired - cop.yaw + core::f32::consts::PI)
                    .rem_euclid(core::f32::consts::TAU)
                    - core::f32::consts::PI;
                cop.yaw += diff.clamp(-TURN_RATE * dt, TURN_RATE * dt);
                let (sin, cos) = cop.yaw.sin_cos();
                let heading = Vector::new(sin, 0.0, cos);
                (Duty::Lunge, pos + heading * step, heading)
            }
            Duty::Rail { chunk, lane, s } => {
                let s = s + step;
                let Some(lane_ref) = self
                    .traffic
                    .lanes
                    .get(&chunk)
                    .and_then(|list| list.get(lane))
                else {
                    // Lane streamed out underneath us: lunge or vanish.
                    self.remove_police_body(cop);
                    return None;
                };
                if s < lane_ref.length() {
                    let (p, d) = lane_ref.sample(s);
                    (Duty::Rail { chunk, lane, s }, p, d)
                } else {
                    let end = lane_ref.end();
                    let end_dir = lane_ref.end_dir();
                    match self.pick_police_continuation(end, end_dir, target) {
                        Some((to_chunk, to_lane, len)) => {
                            let to = self.traffic.lanes[&to_chunk][to_lane].start();
                            let dir = (to - end).try_normalize().unwrap_or(end_dir);
                            (
                                Duty::Bridge {
                                    from: end,
                                    chunk: to_chunk,
                                    lane: to_lane,
                                    t: 0.0,
                                    len,
                                },
                                end,
                                dir,
                            )
                        }
                        // Dead end: cut across open ground toward the target.
                        None => {
                            let (sin, cos) = cop.yaw.sin_cos();
                            (Duty::Lunge, pos, Vector::new(sin, 0.0, cos))
                        }
                    }
                }
            }
            Duty::Bridge {
                from,
                chunk,
                lane,
                t,
                len,
            } => {
                let Some(lane_ref) = self
                    .traffic
                    .lanes
                    .get(&chunk)
                    .and_then(|list| list.get(lane))
                else {
                    self.remove_police_body(cop);
                    return None;
                };
                let t = t + step;
                let to = lane_ref.start();
                let dir = (to - from).try_normalize().unwrap_or(Vector::Z);
                if t >= len {
                    let (p, d) = lane_ref.sample(0.0);
                    (
                        Duty::Rail {
                            chunk,
                            lane,
                            s: 0.0,
                        },
                        p,
                        d,
                    )
                } else {
                    (
                        Duty::Bridge {
                            from,
                            chunk,
                            lane,
                            t,
                            len,
                        },
                        from + dir * t,
                        dir,
                    )
                }
            }
            Duty::Block | Duty::Loose { .. } => unreachable!("handled above"),
        };
        cop.duty = new_duty;
        cop.yaw = next_dir.x.atan2(next_dir.z);

        let body = &mut self.rigid_body_set[cop.body];
        body.set_next_kinematic_translation(Vector::new(next_pos.x, RIDE_HEIGHT, next_pos.z));
        body.set_next_kinematic_rotation(Rotation::from_rotation_y(cop.yaw));
        Some(cop)
    }

    /// Continuation pick for a chasing cruiser: any lane start across the
    /// intersection is fair game — including the U-turn onto the opposite
    /// twin, which traffic forbids — and the winner is the lane heading most
    /// toward the pursuit target. This is the routing bias that makes rail
    /// cops converge on (and turn around after) the player.
    fn pick_police_continuation(
        &self,
        end: Vector,
        end_dir: Vector,
        target: Vector,
    ) -> Option<((i32, i32), usize, f32)> {
        let to_target = (target - end).try_normalize().unwrap_or(end_dir);
        let mut best: Option<((i32, i32), usize, f32, f32)> = None;
        for key in self.traffic.sorted_keys() {
            for (index, lane) in self.traffic.lanes[&key].iter().enumerate() {
                let gap = lane.start() - end;
                let len = gap.length();
                if !(1e-3..=25.0).contains(&len) {
                    continue;
                }
                let score = lane.start_dir().dot(to_target);
                if best.is_none_or(|b| score > b.3) {
                    best = Some((key, index, len, score));
                }
            }
        }
        best.map(|(key, index, len, _)| (key, index, len))
    }

    /// Nearest lane point to `p` in the 3×3 chunk neighborhood (re-attach
    /// point after a lunge loses sight).
    fn nearest_police_lane(&self, p: Vector) -> Option<((i32, i32), usize, f32)> {
        let (cx, cz) = (
            (p.x / crate::constants::CHUNK_SIZE).round() as i32,
            (p.z / crate::constants::CHUNK_SIZE).round() as i32,
        );
        let mut best: Option<((i32, i32), usize, f32, f32)> = None;
        for dcx in -1..=1 {
            for dcz in -1..=1 {
                let key = (cx + dcx, cz + dcz);
                let Some(list) = self.traffic.lanes.get(&key) else {
                    continue;
                };
                for (li, lane) in list.iter().enumerate() {
                    let (s, dist) = lane.closest(p);
                    if best.is_none_or(|b| dist < b.3) {
                        best = Some((key, li, s, dist));
                    }
                }
            }
        }
        best.map(|(key, li, s, _)| (key, li, s))
    }

    /// Snapshot support: pose of police slot `i`, if occupied.
    pub(crate) fn police_slot_pose(&self, i: usize) -> Option<(Vector, Rotation)> {
        let cop = self.police.slots.get(i)?.as_ref()?;
        let body = &self.rigid_body_set[cop.body];
        Some((body.translation(), *body.rotation()))
    }

    pub(crate) fn police_seen(&self) -> bool {
        self.police.seen
    }
}
