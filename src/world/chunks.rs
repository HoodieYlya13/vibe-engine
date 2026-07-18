//! Chunk collider streaming (Phase 3): the client decides which chunks are
//! near the player, fetches their collider sidecars, and streams them in and
//! out over the worker bridge. Each loaded chunk is one fixed body carrying
//! cuboid colliders, keyed by grid coordinates — chunk (cx, cz) is centered
//! at (cx·CHUNK_SIZE, cz·CHUNK_SIZE) and box positions are chunk-local.

use std::collections::HashMap;

use rapier3d::prelude::*;
use wasm_bindgen::prelude::*;

use crate::constants::CHUNK_SIZE;
use crate::engine::SimEngine;

/// Floats per collider box in a chunk payload: (x, y, z, hx, hy, hz).
pub const CHUNK_BOX_FLOATS: usize = 6;

#[derive(Default)]
pub(crate) struct ChunkStore {
    loaded: HashMap<(i32, i32), RigidBodyHandle>,
}

#[wasm_bindgen]
impl SimEngine {
    /// Stream in one chunk's static colliders. `boxes` is flattened rows of
    /// (x, y, z, hx, hy, hz) in chunk-local coordinates. Re-loading an
    /// already-loaded chunk replaces its colliders.
    pub fn load_chunk(&mut self, cx: i32, cz: i32, boxes: &[f32]) {
        self.unload_chunk(cx, cz);

        let origin = Vector::new(cx as f32 * CHUNK_SIZE, 0.0, cz as f32 * CHUNK_SIZE);
        let body = RigidBodyBuilder::fixed().translation(origin).build();
        let handle = self.rigid_body_set.insert(body);
        for row in boxes.chunks_exact(CHUNK_BOX_FLOATS) {
            let collider = ColliderBuilder::cuboid(row[3], row[4], row[5])
                .translation(Vector::new(row[0], row[1], row[2]))
                .build();
            self.collider_set
                .insert_with_parent(collider, handle, &mut self.rigid_body_set);
        }
        self.chunks.loaded.insert((cx, cz), handle);
    }

    /// Stream a chunk back out, freeing its body and every collider on it,
    /// plus its lanes and any traffic riding them.
    pub fn unload_chunk(&mut self, cx: i32, cz: i32) {
        self.unload_chunk_traffic(cx, cz);
        let Some(handle) = self.chunks.loaded.remove(&(cx, cz)) else {
            return;
        };
        self.rigid_body_set.remove(
            handle,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true,
        );
    }

    pub fn loaded_chunk_count(&self) -> u32 {
        self.chunks.loaded.len() as u32
    }
}
