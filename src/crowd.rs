use rapier3d::prelude::*;

use crate::constants::{
    BLOCKS, CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, PLAYER_FOOT_RADIUS,
    PLAYER_STAND_HEIGHT, RAMPS, WORLD_HALF,
};
use crate::state::SimEngine;

const PED_CENTER_HEIGHT: f32 = 0.8;

impl SimEngine {
    pub(crate) fn update_pedestrians(&mut self, dt: f32) {
        if self.crowd.ped_positions.is_empty() {
            return;
        }

        let world_half = WORLD_HALF;
        let car_body = &self.rigid_body_set[self.vehicle.body_handle];
        let car_pos = car_body.translation();
        let player_pos = self.player.pos;
        let mut player_push = Vector::new(0.0, 0.0, 0.0);
        let ped_radius = 0.4;
        let player_radius = PLAYER_FOOT_RADIUS;
        let car_radius = 2.4;
        let car_speed = car_body.linvel().length();
        let car_push_scale = 1.2 + (car_speed * 0.1).min(2.0);

        for (i, pos) in self.crowd.ped_positions.iter_mut().enumerate() {
            let angle = self.runtime.sim_time * 0.35 + i as f32 * 0.37;
            let mut dir = Vector::new(angle.cos(), 0.0, angle.sin());

            let dx = pos.x - car_pos.x;
            let dz = pos.z - car_pos.z;
            let dist_sq = dx * dx + dz * dz;
            if dist_sq < 36.0 {
                let avoid = Vector::new(dx, 0.0, dz);
                dir += avoid.normalize_or_zero() * 2.5;
            }

            if !self.player.in_car {
                let px = pos.x - player_pos.x;
                let pz = pos.z - player_pos.z;
                let pdist_sq = px * px + pz * pz;
                let min_dist = ped_radius + player_radius;
                if pdist_sq < min_dist * min_dist {
                    let dist = pdist_sq.sqrt().max(0.001);
                    let n = Vector::new(px / dist, 0.0, pz / dist);
                    let push = min_dist - dist;
                    *pos += n * push;
                    player_push -= n * push;
                }
            }

            let car_min = car_radius + ped_radius;
            if dist_sq < car_min * car_min {
                let dist = dist_sq.sqrt().max(0.001);
                let n = Vector::new(dx / dist, 0.0, dz / dist);
                let push = (car_min - dist) * car_push_scale;
                *pos += n * push;
            }

            let speed = 0.8 + (i % 7) as f32 * 0.1;
            *pos += dir * speed * dt;
            Self::resolve_agent_block_collision(pos, ped_radius);
            let ramp_ground = Self::resolve_agent_ramp_collision(pos, ped_radius);
            pos.y = PED_CENTER_HEIGHT + ramp_ground;

            let limit = world_half - ped_radius;
            pos.x = pos.x.clamp(-limit, limit);
            pos.z = pos.z.clamp(-limit, limit);
        }

        if !self.player.in_car {
            let dx = player_pos.x - car_pos.x;
            let dz = player_pos.z - car_pos.z;
            let roof_center_y = car_pos.y + CAR_HALF_HEIGHT + PLAYER_STAND_HEIGHT;
            let player_above_roof = player_pos.y >= roof_center_y - 0.1;
            if !player_above_roof {
                let rot = car_body.rotation();
                let mut right_h = rot * Vector::new(1.0, 0.0, 0.0);
                right_h.y = 0.0;
                let mut forward_h = rot * Vector::new(0.0, 0.0, 1.0);
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
            let x0 = ramp.0;
            let x1 = ramp.1;
            let z0 = ramp.2 - ramp.3;
            let z1 = ramp.2 + ramp.3;
            let h = ramp.4;

            // Side walls: keep agents from clipping through ramp sides.
            if pos.x >= x0 && pos.x <= x1 {
                if pos.z > z1 && pos.z < z1 + radius {
                    pos.z = z1 + radius;
                } else if pos.z < z0 && pos.z > z0 - radius {
                    pos.z = z0 - radius;
                }
            }

            // Back wall: prevent clipping through the high-end face.
            if pos.z >= z0 - radius && pos.z <= z1 + radius && pos.x > x1 && pos.x < x1 + radius {
                pos.x = x1 + radius;
            }

            // Stand on ramp surface when inside top footprint.
            if pos.x >= x0 && pos.x <= x1 && pos.z >= z0 && pos.z <= z1 {
                let run = (x1 - x0).max(0.001);
                let t = ((pos.x - x0) / run).clamp(0.0, 1.0);
                ground = ground.max(t * h);
            }
        }

        ground
    }
}
