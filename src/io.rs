use wasm_bindgen::prelude::*;

use crate::constants::{
    CAM_STATE_OFFSET, HUD_HINT_ENTER, HUD_HINT_EXIT, HUD_HINT_NONE, HUD_STATE_OFFSET,
    PED_STATE_OFFSET, PED_STATE_STRIDE, PLAYER_STATE_OFFSET,
};
use crate::state::SimEngine;

#[wasm_bindgen]
impl SimEngine {
    pub fn write_state(&self, out: &mut [f32]) {
        let needed =
            PED_STATE_OFFSET as usize + self.crowd.ped_positions.len() * PED_STATE_STRIDE as usize;
        if out.len() < needed {
            return;
        }

        let body = &self.rigid_body_set[self.vehicle.body_handle];
        let translation = body.translation();
        let rotation = body.rotation();

        out[0] = translation.x;
        out[1] = translation.y;
        out[2] = translation.z;
        out[3] = rotation.x;
        out[4] = rotation.y;
        out[5] = rotation.z;
        out[6] = rotation.w;

        let player_offset = PLAYER_STATE_OFFSET as usize;
        out[player_offset] = self.player.pos.x;
        out[player_offset + 1] = self.player.pos.y;
        out[player_offset + 2] = self.player.pos.z;
        out[player_offset + 3] = self.player.yaw;
        out[player_offset + 4] = if self.player.in_car { 1.0 } else { 0.0 };

        let dx = self.player.pos.x - translation.x;
        let dz = self.player.pos.z - translation.z;
        let enter_radius = 6.0;
        let can_enter = !self.player.in_car && (dx * dx + dz * dz <= enter_radius * enter_radius);
        let hud_hint = if self.player.in_car {
            HUD_HINT_EXIT
        } else if can_enter {
            HUD_HINT_ENTER
        } else {
            HUD_HINT_NONE
        };

        out[HUD_STATE_OFFSET as usize] = hud_hint as f32;

        let cam_offset = CAM_STATE_OFFSET as usize;
        let (target, target_y, radius) = if self.player.in_car {
            (translation, 0.6, 20.0)
        } else {
            (self.player.pos, 0.9, 8.0)
        };
        out[cam_offset] = target.x;
        out[cam_offset + 1] = target.y + target_y;
        out[cam_offset + 2] = target.z;
        out[cam_offset + 3] = radius;

        let mut cursor = PED_STATE_OFFSET as usize;
        for pos in &self.crowd.ped_positions {
            out[cursor] = 1.0;
            out[cursor + 1] = 0.0;
            out[cursor + 2] = 0.0;
            out[cursor + 3] = 0.0;
            out[cursor + 4] = 0.0;
            out[cursor + 5] = 1.0;
            out[cursor + 6] = 0.0;
            out[cursor + 7] = 0.0;
            out[cursor + 8] = 0.0;
            out[cursor + 9] = 0.0;
            out[cursor + 10] = 1.0;
            out[cursor + 11] = 0.0;
            out[cursor + 12] = pos.x;
            out[cursor + 13] = pos.y;
            out[cursor + 14] = pos.z;
            out[cursor + 15] = 1.0;
            cursor += PED_STATE_STRIDE as usize;
        }
    }

    pub fn state_len(&self) -> usize {
        PED_STATE_OFFSET as usize + self.crowd.ped_positions.len() * PED_STATE_STRIDE as usize
    }

    pub fn ped_count(&self) -> u32 {
        self.crowd.ped_positions.len() as u32
    }

    pub fn dt(&self) -> f32 {
        self.integration_parameters.dt
    }
}
