//! Decorative pedestrian crowd (Phase 4, PRD §7.4). In city worlds peds walk
//! the sidewalk polylines derived from the streamed traffic lanes, cross
//! streets at intersection corners, flee from the speeding player car, and
//! live in a density bubble around the player — peds that fall far behind
//! respawn onto sidewalks at the bubble's edge. The arena keeps the original
//! wander-with-obstacles behavior (it has no lanes). Peds carry no physical
//! colliders — cars shove them aside geometrically — and everything is
//! deterministic: xorshift RNG stepped only by sim events, fixed iteration
//! order, sorted chunk keys.

use std::collections::HashMap;

use rapier3d::prelude::*;

use crate::constants::{
    BLOCKS, CAR_CHASSIS_DROP, CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, CHUNK_SIZE,
    CITY_GROUND_HALF, PLAYER_FOOT_RADIUS, PLAYER_STAND_HEIGHT, RAMPS, WORLD_HALF,
};
use crate::engine::SimEngine;
use crate::traffic::Lane;
use crate::world::WorldKind;

const PED_CENTER_HEIGHT: f32 = 0.8;
/// City peds re-probe the ground under them every Nth frame (staggered by
/// ped index) — sidewalks are raised slabs, not the flat road plane.
const GROUND_PROBE_EVERY: u32 = 8;
/// The probe ray starts this far above the road plane and reaches as far below.
const GROUND_PROBE_REACH: f32 = 1.5;
/// Scatter radius in the city: covers the built kit grid (streets reach
/// ±168 m) without stranding peds on empty ground beyond it.
const CITY_PED_SPREAD: f32 = 170.0;
/// Keep the spawn point itself clear so the player never boots inside a ped.
const SPAWN_KEEPOUT: f32 = 10.0;
const GOLDEN_ANGLE: f32 = 2.399_963;
const PED_RADIUS: f32 = 0.4;

/// A detached ped latches onto a sidewalk this close, walking straight to it.
const ATTACH_RADIUS: f32 = 40.0;
/// Only every Nth frame per ped tries to attach (staggered by ped index).
const ATTACH_EVERY: u32 = 30;
/// Max hop between sidewalk endpoints at a corner: reaches the opposite
/// corner across the street (a crosswalk-ish 13–14 m in the kit city) but
/// not the diagonal corner.
const CROSS_RADIUS: f32 = 16.0;
/// Catch-up factor when a ped drifts off its path point (after pushes).
const CATCH_UP: f32 = 1.6;
/// Pushed or blocked further than this off the path — give up and re-attach.
const STRAY_LIMIT: f32 = 5.0;
const FLEE_SPEED: f32 = 5.0;
const FLEE_TIME: f32 = 4.0;
/// Facing turn rate (rad/s) toward the movement direction.
const TURN_RATE: f32 = 8.0;
/// Density bubble: peds beyond the outer radius respawn onto a sidewalk
/// inside the annulus around the focus (spend the budget off-screen-ish).
const PED_DESPAWN_DIST: f32 = 200.0;
const RESPAWN_MIN_DIST: f32 = 60.0;
const RESPAWN_MAX_DIST: f32 = 170.0;
/// Respawn attempts are capped per step so a mass teleport amortizes.
const RESPAWNS_PER_STEP: u32 = 8;

pub(crate) struct Ped {
    pub(crate) pos: Vector,
    pub(crate) yaw: f32,
    state: PedState,
    walk_speed: f32,
    /// Per-ped lateral offset on the sidewalk, so peds don't walk single file.
    jitter: f32,
    /// Crime-report cooldown: getting clipped by the car counts as one
    /// witnessed crime, not one per overlapping frame.
    hit_cool: f32,
    /// Height of the surface under the ped's feet (city worlds).
    ground: f32,
}

#[derive(Clone, Copy)]
enum PedState {
    /// Ambling with no path: the arena behavior, and the city fallback while
    /// no sidewalk is within reach.
    Wander,
    /// Walking straight toward a point on a sidewalk to latch onto it.
    Seek {
        chunk: (i32, i32),
        walk: usize,
        s: f32,
    },
    /// On a sidewalk, at arc distance `s`, heading toward its end or start.
    Walk {
        chunk: (i32, i32),
        walk: usize,
        s: f32,
        forward: bool,
    },
    /// Corner hop: straight line from `from` to an endpoint of the target
    /// sidewalk (same corner = a turn; across the street = a crossing).
    Cross {
        from: Vector,
        chunk: (i32, i32),
        walk: usize,
        forward: bool,
        t: f32,
        len: f32,
    },
    /// Scattering from the player car.
    Flee { dir: Vector, time: f32 },
}

pub(crate) struct CrowdState {
    pub(crate) peds: Vec<Ped>,
    /// Wander clamp: arena walls or the city ground extent.
    bound: f32,
    /// The static BLOCKS/RAMPS only exist in the obstacle arena.
    arena_obstacles: bool,
    /// Sidewalk/flee/bubble behavior only runs where lanes stream in.
    city: bool,
    rng: u32,
    frame: u32,
}

pub(crate) fn spawn_peds(count: usize, kind: WorldKind) -> CrowdState {
    let (spread, bound, arena_obstacles, city) = match kind {
        WorldKind::Arena { obstacles } => (WORLD_HALF - 2.0, WORLD_HALF, obstacles, false),
        WorldKind::City => (CITY_PED_SPREAD, CITY_GROUND_HALF, false, true),
    };
    // Deterministic sunflower scatter (golden-angle spiral): even coverage
    // with no grid rows. City peds start scattered too and converge onto the
    // sidewalks as chunks (and their lanes) stream in.
    let mut peds = Vec::with_capacity(count);
    for i in 0..count {
        let t = (i as f32 + 0.5) / count.max(1) as f32;
        let r = SPAWN_KEEPOUT + (spread - SPAWN_KEEPOUT).max(0.0) * t.sqrt();
        let theta = i as f32 * GOLDEN_ANGLE;
        let h = (i as u32).wrapping_mul(2_654_435_761);
        peds.push(Ped {
            pos: Vector::new(r * theta.cos(), PED_CENTER_HEIGHT, r * theta.sin()),
            yaw: theta,
            state: PedState::Wander,
            walk_speed: 1.1 + (h % 100) as f32 / 100.0 * 0.6,
            jitter: ((h >> 8) % 100) as f32 / 100.0 * 1.1 - 0.55,
            hit_cool: 0.0,
            ground: 0.0,
        });
    }
    CrowdState {
        peds,
        bound,
        arena_obstacles,
        city,
        rng: 0x51ED_270B,
        frame: 0,
    }
}

fn next_rng(rng: &mut u32) -> u32 {
    let mut x = *rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *rng = x;
    x
}

fn horiz(v: Vector) -> Vector {
    Vector::new(v.x, 0.0, v.z)
}

/// Move `pos` horizontally toward `target` by at most `max_step`; true on
/// arrival.
fn move_toward(pos: &mut Vector, target: Vector, max_step: f32) -> bool {
    let to = horiz(target - *pos);
    let dist = to.length();
    if dist <= max_step.max(0.3) {
        pos.x = target.x;
        pos.z = target.z;
        return true;
    }
    *pos += to * (max_step / dist);
    false
}

fn chunk_of(pos: Vector) -> (i32, i32) {
    (
        (pos.x / CHUNK_SIZE).round() as i32,
        (pos.z / CHUNK_SIZE).round() as i32,
    )
}

/// Nearest sidewalk point within `radius` of `p`, searching the 3×3 chunk
/// neighborhood in fixed order.
fn nearest_walk(
    walks: &HashMap<(i32, i32), Vec<Lane>>,
    p: Vector,
    radius: f32,
) -> Option<((i32, i32), usize, f32)> {
    let (cx, cz) = chunk_of(p);
    let mut best: Option<((i32, i32), usize, f32, f32)> = None;
    for dcx in -1..=1 {
        for dcz in -1..=1 {
            let key = (cx + dcx, cz + dcz);
            let Some(list) = walks.get(&key) else {
                continue;
            };
            for (wi, walk) in list.iter().enumerate() {
                let (s, dist) = walk.closest(p);
                if dist <= radius && best.is_none_or(|b| dist < b.3) {
                    best = Some((key, wi, s, dist));
                }
            }
        }
    }
    best.map(|(key, wi, s, _)| (key, wi, s))
}

/// Random corner continuation from a sidewalk endpoint: any other sidewalk
/// endpoint within CROSS_RADIUS. `forward` says which end was matched, i.e.
/// which way to walk the target path after arriving.
fn pick_crossing(
    walks: &HashMap<(i32, i32), Vec<Lane>>,
    rng: &mut u32,
    from: Vector,
    exclude: ((i32, i32), usize),
) -> Option<((i32, i32), usize, bool, f32)> {
    let (cx, cz) = chunk_of(from);
    let mut candidates = Vec::new();
    for dcx in -1..=1 {
        for dcz in -1..=1 {
            let key = (cx + dcx, cz + dcz);
            let Some(list) = walks.get(&key) else {
                continue;
            };
            for (wi, walk) in list.iter().enumerate() {
                if (key, wi) == exclude {
                    continue;
                }
                for (endpoint, forward) in [(walk.start(), true), (walk.end(), false)] {
                    let dist = horiz(endpoint - from).length();
                    if (0.5..=CROSS_RADIUS).contains(&dist) {
                        candidates.push((key, wi, forward, dist));
                    }
                }
            }
        }
    }
    if candidates.is_empty() {
        return None;
    }
    let pick = next_rng(rng) as usize % candidates.len();
    Some(candidates[pick])
}

impl SimEngine {
    pub(crate) fn update_pedestrians(&mut self, dt: f32) {
        if self.crowd.peds.is_empty() {
            return;
        }

        let car_body = &self.rigid_body_set[self.vehicle.body_handle];
        let car_pos = car_body.translation();
        let car_vel = car_body.linvel();
        let car_speed = car_vel.length();
        let car_rot = *car_body.rotation();
        let player_pos = self.player.pos;
        let in_car = self.player.in_car;
        let focus = if in_car { car_pos } else { player_pos };
        let traffic_positions = self.traffic_positions();
        let sim_time = self.runtime.sim_time;

        let walks = &self.traffic.walks;
        let ground_query = self.broad_phase.as_query_pipeline(
            self.narrow_phase.query_dispatcher(),
            &self.rigid_body_set,
            &self.collider_set,
            QueryFilter::only_fixed(),
        );
        let CrowdState {
            peds,
            bound,
            arena_obstacles,
            city,
            rng,
            frame,
        } = &mut self.crowd;
        *frame = frame.wrapping_add(1);
        let mut walk_keys: Vec<(i32, i32)> = walks.keys().copied().collect();
        walk_keys.sort_unstable();

        let world_half = *bound;
        let player_radius = PLAYER_FOOT_RADIUS;
        let car_radius = 2.4;
        let car_min = car_radius + PED_RADIUS;
        let car_push_scale = 1.2 + (car_speed * 0.1).min(2.0);
        let mut player_push = Vector::new(0.0, 0.0, 0.0);
        let mut respawns = 0u32;
        let mut ped_hits = 0u32;

        for (i, ped) in peds.iter_mut().enumerate() {
            let prev_pos = ped.pos;
            ped.hit_cool = (ped.hit_cool - dt).max(0.0);

            // Threat check: the player car bearing down on this ped. Escape
            // perpendicular to the car's travel, not straight down its path.
            if *city && !matches!(ped.state, PedState::Flee { .. }) {
                let to_ped = horiz(ped.pos - car_pos);
                let dist = to_ped.length().max(1e-3);
                let bearing = if car_speed > 1e-3 {
                    car_vel.dot(to_ped) / (car_speed * dist)
                } else {
                    0.0
                };
                if (car_speed > 6.0 && dist < 14.0 && bearing > 0.5)
                    || (car_speed > 3.0 && dist < 5.0)
                {
                    let along = if car_speed > 1e-3 {
                        car_vel / car_speed
                    } else {
                        Vector::Z
                    };
                    let escape = horiz(to_ped - along * to_ped.dot(along));
                    let dir = escape
                        .try_normalize()
                        .unwrap_or(Vector::new(-along.z, 0.0, along.x));
                    ped.state = PedState::Flee {
                        dir,
                        time: FLEE_TIME,
                    };
                }
            }

            ped.state = match ped.state {
                PedState::Wander => {
                    let angle = sim_time * 0.35 + i as f32 * 0.37;
                    let mut dir = Vector::new(angle.cos(), 0.0, angle.sin());
                    let avoid = horiz(ped.pos - car_pos);
                    if avoid.length_squared() < 36.0 {
                        dir += avoid.normalize_or_zero() * 2.5;
                    }
                    let speed = 0.8 + (i % 7) as f32 * 0.1;
                    ped.pos += dir * speed * dt;
                    // Staggered attach: latch onto a nearby sidewalk.
                    if *city && (i as u32).wrapping_add(*frame) % ATTACH_EVERY == 0 {
                        match nearest_walk(walks, ped.pos, ATTACH_RADIUS) {
                            Some((chunk, walk, s)) => PedState::Seek { chunk, walk, s },
                            None => PedState::Wander,
                        }
                    } else {
                        PedState::Wander
                    }
                }
                PedState::Seek { chunk, walk, s } => {
                    match walks.get(&chunk).and_then(|list| list.get(walk)) {
                        None => PedState::Wander,
                        Some(lane) => {
                            let (point, ldir) = lane.sample(s);
                            let perp = Vector::new(-ldir.z, 0.0, ldir.x);
                            let target = point + perp * ped.jitter;
                            if move_toward(&mut ped.pos, target, ped.walk_speed * dt) {
                                PedState::Walk {
                                    chunk,
                                    walk,
                                    s,
                                    forward: next_rng(rng) & 1 == 0,
                                }
                            } else {
                                PedState::Seek { chunk, walk, s }
                            }
                        }
                    }
                }
                PedState::Walk {
                    chunk,
                    walk,
                    s,
                    forward,
                } => match walks.get(&chunk).and_then(|list| list.get(walk)) {
                    None => PedState::Wander,
                    Some(lane) => {
                        let step = ped.walk_speed * dt;
                        let s2 = if forward { s + step } else { s - step };
                        if (0.0..=lane.length()).contains(&s2) {
                            let (point, ldir) = lane.sample(s2);
                            let perp = Vector::new(-ldir.z, 0.0, ldir.x);
                            let target = point + perp * ped.jitter;
                            if horiz(target - ped.pos).length() > STRAY_LIMIT {
                                PedState::Wander
                            } else {
                                move_toward(&mut ped.pos, target, step * CATCH_UP);
                                PedState::Walk {
                                    chunk,
                                    walk,
                                    s: s2,
                                    forward,
                                }
                            }
                        } else {
                            // Sidewalk ends at a corner: hop to another
                            // endpoint (turn or crossing), or turn around.
                            let end = if forward { lane.end() } else { lane.start() };
                            match pick_crossing(walks, rng, end, (chunk, walk)) {
                                Some((to_chunk, to_walk, to_forward, len)) => PedState::Cross {
                                    from: end,
                                    chunk: to_chunk,
                                    walk: to_walk,
                                    forward: to_forward,
                                    t: 0.0,
                                    len,
                                },
                                None => PedState::Walk {
                                    chunk,
                                    walk,
                                    s: s2.clamp(0.0, lane.length()),
                                    forward: !forward,
                                },
                            }
                        }
                    }
                },
                PedState::Cross {
                    from,
                    chunk,
                    walk,
                    forward,
                    t,
                    len,
                } => match walks.get(&chunk).and_then(|list| list.get(walk)) {
                    None => PedState::Wander,
                    Some(lane) => {
                        let endpoint = if forward { lane.start() } else { lane.end() };
                        let dir = horiz(endpoint - from).try_normalize().unwrap_or(Vector::Z);
                        let step = ped.walk_speed * dt;
                        let t2 = t + step;
                        if t2 >= len {
                            PedState::Walk {
                                chunk,
                                walk,
                                s: if forward { 0.0 } else { lane.length() },
                                forward,
                            }
                        } else {
                            let target = from + dir * t2;
                            if horiz(target - ped.pos).length() > STRAY_LIMIT {
                                PedState::Wander
                            } else {
                                move_toward(&mut ped.pos, target, step * CATCH_UP);
                                PedState::Cross {
                                    from,
                                    chunk,
                                    walk,
                                    forward,
                                    t: t2,
                                    len,
                                }
                            }
                        }
                    }
                },
                PedState::Flee { dir, time } => {
                    ped.pos += dir * FLEE_SPEED * dt;
                    let time = time - dt;
                    if time <= 0.0 {
                        PedState::Wander
                    } else {
                        PedState::Flee { dir, time }
                    }
                }
            };

            // Overlap pushes: the player on foot, the player car, traffic.
            if !in_car {
                let px = ped.pos.x - player_pos.x;
                let pz = ped.pos.z - player_pos.z;
                let pdist_sq = px * px + pz * pz;
                let min_dist = PED_RADIUS + player_radius;
                if pdist_sq < min_dist * min_dist {
                    let dist = pdist_sq.sqrt().max(0.001);
                    let n = Vector::new(px / dist, 0.0, pz / dist);
                    let push = min_dist - dist;
                    ped.pos += n * push;
                    player_push -= n * push;
                }
            }

            let dx = ped.pos.x - car_pos.x;
            let dz = ped.pos.z - car_pos.z;
            let dist_sq = dx * dx + dz * dz;
            if dist_sq < car_min * car_min {
                let dist = dist_sq.sqrt().max(0.001);
                let n = Vector::new(dx / dist, 0.0, dz / dist);
                ped.pos += n * (car_min - dist) * car_push_scale;
                // Getting clipped panics the ped — and at speed it's a
                // witnessed crime (PRD §7.3), but only with a driver: a
                // driverless car shoved into the crowd is nobody's crime.
                if *city {
                    if in_car && car_speed > 3.0 && ped.hit_cool <= 0.0 {
                        ped.hit_cool = 2.0;
                        ped_hits += 1;
                    }
                    ped.state = PedState::Flee {
                        dir: n,
                        time: FLEE_TIME,
                    };
                }
            }

            // Ambient traffic shoves peds aside the same way the player car
            // does (they are decorative — no physical colliders to do it).
            for tp in &traffic_positions {
                let tx = ped.pos.x - tp.x;
                let tz = ped.pos.z - tp.z;
                let t_sq = tx * tx + tz * tz;
                if t_sq < car_min * car_min {
                    let dist = t_sq.sqrt().max(0.001);
                    ped.pos += Vector::new(tx / dist, 0.0, tz / dist) * (car_min - dist);
                }
            }

            let mut ramp_ground = 0.0;
            if *arena_obstacles {
                Self::resolve_agent_block_collision(&mut ped.pos, PED_RADIUS);
                ramp_ground = Self::resolve_agent_ramp_collision(&mut ped.pos, PED_RADIUS);
            }
            if *city
                && (*frame)
                    .wrapping_add(i as u32)
                    .is_multiple_of(GROUND_PROBE_EVERY)
            {
                // Non-solid cast: a ped clipping through a building reads the
                // slab under it, not the inside of the building.
                let origin = Vector::new(ped.pos.x, GROUND_PROBE_REACH, ped.pos.z);
                let down = Ray::new(origin, -Vector::Y);
                ped.ground = ground_query
                    .cast_ray(&down, 2.0 * GROUND_PROBE_REACH, false)
                    .map_or(0.0, |(_, distance)| GROUND_PROBE_REACH - distance);
            }
            ped.pos.y = PED_CENTER_HEIGHT + ramp_ground + ped.ground;

            let limit = world_half - PED_RADIUS;
            ped.pos.x = ped.pos.x.clamp(-limit, limit);
            ped.pos.z = ped.pos.z.clamp(-limit, limit);

            // Density bubble: far peds respawn onto a sidewalk in the
            // annulus around the focus (attempts capped per step).
            if *city
                && respawns < RESPAWNS_PER_STEP
                && !walk_keys.is_empty()
                && horiz(ped.pos - focus).length() > PED_DESPAWN_DIST
            {
                respawns += 1;
                for _ in 0..6 {
                    let key = walk_keys[next_rng(rng) as usize % walk_keys.len()];
                    let list = &walks[&key];
                    if list.is_empty() {
                        continue;
                    }
                    let wi = next_rng(rng) as usize % list.len();
                    let lane = &list[wi];
                    let s = (next_rng(rng) % 1000) as f32 / 1000.0 * lane.length();
                    let (point, ldir) = lane.sample(s);
                    if !(RESPAWN_MIN_DIST..=RESPAWN_MAX_DIST)
                        .contains(&horiz(point - focus).length())
                    {
                        continue;
                    }
                    let perp = Vector::new(-ldir.z, 0.0, ldir.x);
                    let spot = point + perp * ped.jitter;
                    ped.pos = Vector::new(spot.x, PED_CENTER_HEIGHT, spot.z);
                    ped.state = PedState::Walk {
                        chunk: key,
                        walk: wi,
                        s,
                        forward: next_rng(rng) & 1 == 0,
                    };
                    break;
                }
            }

            // Face the way we actually moved (respawn keeps the old yaw for
            // a frame — invisible, it snaps client-side anyway).
            let delta = horiz(ped.pos - prev_pos);
            if delta.length_squared() > 1e-8 {
                let target_yaw = delta.x.atan2(delta.z);
                let diff = (target_yaw - ped.yaw + core::f32::consts::PI)
                    .rem_euclid(core::f32::consts::TAU)
                    - core::f32::consts::PI;
                ped.yaw += diff.clamp(-TURN_RATE * dt, TURN_RATE * dt);
            }
        }

        if !in_car {
            let dx = player_pos.x - car_pos.x;
            let dz = player_pos.z - car_pos.z;
            let roof_center_y =
                car_pos.y - CAR_CHASSIS_DROP + CAR_HALF_HEIGHT + PLAYER_STAND_HEIGHT;
            let player_above_roof = player_pos.y >= roof_center_y - 0.1;
            if !player_above_roof {
                let mut right_h = car_rot * Vector::new(1.0, 0.0, 0.0);
                right_h.y = 0.0;
                let mut forward_h = car_rot * Vector::new(0.0, 0.0, 1.0);
                forward_h.y = 0.0;
                if right_h.length_squared() < 0.0001 || forward_h.length_squared() < 0.0001 {
                    right_h = Vector::new(1.0, 0.0, 0.0);
                    forward_h = Vector::new(0.0, 0.0, 1.0);
                } else {
                    right_h = right_h.normalize();
                    forward_h = forward_h.normalize();
                }

                let local_x = dx * right_h.x + dz * right_h.z;
                let local_z = dx * forward_h.x + dz * forward_h.z;
                let overlap_x = CAR_HALF_WIDTH + player_radius - local_x.abs();
                let overlap_z = CAR_HALF_LENGTH + player_radius - local_z.abs();
                if overlap_x > 0.0 && overlap_z > 0.0 {
                    if overlap_x < overlap_z {
                        let sign = if local_x >= 0.0 { 1.0 } else { -1.0 };
                        player_push += right_h * (overlap_x * sign);
                    } else {
                        let sign = if local_z >= 0.0 { 1.0 } else { -1.0 };
                        player_push += forward_h * (overlap_z * sign);
                    }
                }
            }

            self.player.pos += player_push;
        }

        if ped_hits > 0 {
            self.police
                .report_crime(ped_hits as f32 * crate::police::CRIME_PED_HIT, car_pos);
        }
    }

    fn resolve_agent_block_collision(pos: &mut Vector, radius: f32) {
        for block in BLOCKS {
            let min_y = block.1 - block.4;
            let max_y = block.1 + block.4 + 1.8;
            if pos.y < min_y || pos.y > max_y {
                continue;
            }

            let dx = pos.x - block.0;
            let dz = pos.z - block.2;
            let overlap_x = block.3 + radius - dx.abs();
            let overlap_z = block.5 + radius - dz.abs();
            if overlap_x <= 0.0 || overlap_z <= 0.0 {
                continue;
            }

            if overlap_x < overlap_z {
                pos.x += if dx >= 0.0 { overlap_x } else { -overlap_x };
            } else {
                pos.z += if dz >= 0.0 { overlap_z } else { -overlap_z };
            }
        }
    }

    fn resolve_agent_ramp_collision(pos: &mut Vector, radius: f32) -> f32 {
        let mut ground = 0.0;

        for ramp in RAMPS {
            // Rows are (z_start, z_end, x_center, half_width, height) — the same
            // interpretation as the Rapier collider in physics.rs.
            let z0 = ramp.0;
            let z1 = ramp.1;
            let x0 = ramp.2 - ramp.3;
            let x1 = ramp.2 + ramp.3;
            let h = ramp.4;
            let z_min = z0.min(z1);
            let z_max = z0.max(z1);

            // Side walls: keep agents from clipping through ramp sides.
            if pos.z >= z_min && pos.z <= z_max {
                if pos.x > x1 && pos.x < x1 + radius {
                    pos.x = x1 + radius;
                } else if pos.x < x0 && pos.x > x0 - radius {
                    pos.x = x0 - radius;
                }
            }

            // Back wall: prevent clipping through the high-end face at z1.
            if pos.x >= x0 - radius && pos.x <= x1 + radius {
                let dir = (z1 - z0).signum();
                let outside = (pos.z - z1) * dir;
                if outside > 0.0 && outside < radius {
                    pos.z = z1 + radius * dir;
                }
            }

            // Stand on ramp surface when inside top footprint.
            if pos.x >= x0 && pos.x <= x1 && pos.z >= z_min && pos.z <= z_max {
                let run = (z1 - z0).abs().max(0.001);
                let t = ((pos.z - z0).abs() / run).clamp(0.0, 1.0);
                ground = ground.max(t * h);
            }
        }

        ground
    }
}
