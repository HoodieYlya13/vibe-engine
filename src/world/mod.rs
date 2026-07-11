//! Static world colliders. Two world kinds exist: the greybox test arena
//! (ground + walls + blocks/ramps from `constants.rs` — the world every
//! native test drives in) and the streamed city (flat ground slab only; all
//! other geometry arrives per-chunk through `chunks.rs`).

use rapier3d::prelude::*;

use crate::constants::{BLOCKS, CITY_GROUND_HALF, RAMPS, WALL_HEIGHT, WALL_THICKNESS, WORLD_HALF};

pub(crate) mod chunks;
pub(crate) use chunks::ChunkStore;

#[derive(Clone, Copy, PartialEq)]
pub(crate) enum WorldKind {
    Arena { obstacles: bool },
    City,
}

pub(crate) fn build_static_world(
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
    kind: WorldKind,
) {
    let ground_half = match kind {
        WorldKind::Arena { .. } => WORLD_HALF,
        WorldKind::City => CITY_GROUND_HALF,
    };
    let ground_body = RigidBodyBuilder::fixed()
        .translation(Vector::new(0.0, -0.5, 0.0))
        .build();
    let ground_collider = ColliderBuilder::cuboid(ground_half, 0.5, ground_half).build();
    let ground_handle = rigid_body_set.insert(ground_body);
    collider_set.insert_with_parent(ground_collider, ground_handle, rigid_body_set);

    let WorldKind::Arena { obstacles } = kind else {
        return;
    };

    let wall_body = RigidBodyBuilder::fixed().build();
    let wall_handle = rigid_body_set.insert(wall_body);
    let wall_x = WORLD_HALF + WALL_THICKNESS;
    let wall_z = WORLD_HALF + WALL_THICKNESS;
    let wall_depth = WORLD_HALF + WALL_THICKNESS;

    let wall_x_pos = ColliderBuilder::cuboid(WALL_THICKNESS, WALL_HEIGHT, wall_depth)
        .translation(Vector::new(wall_x, WALL_HEIGHT, 0.0))
        .build();
    let wall_x_neg = ColliderBuilder::cuboid(WALL_THICKNESS, WALL_HEIGHT, wall_depth)
        .translation(Vector::new(-wall_x, WALL_HEIGHT, 0.0))
        .build();
    let wall_z_pos = ColliderBuilder::cuboid(wall_depth, WALL_HEIGHT, WALL_THICKNESS)
        .translation(Vector::new(0.0, WALL_HEIGHT, wall_z))
        .build();
    let wall_z_neg = ColliderBuilder::cuboid(wall_depth, WALL_HEIGHT, WALL_THICKNESS)
        .translation(Vector::new(0.0, WALL_HEIGHT, -wall_z))
        .build();
    collider_set.insert_with_parent(wall_x_pos, wall_handle, rigid_body_set);
    collider_set.insert_with_parent(wall_x_neg, wall_handle, rigid_body_set);
    collider_set.insert_with_parent(wall_z_pos, wall_handle, rigid_body_set);
    collider_set.insert_with_parent(wall_z_neg, wall_handle, rigid_body_set);

    if obstacles {
        insert_obstacles(rigid_body_set, collider_set);
    }
}

fn insert_obstacles(rigid_body_set: &mut RigidBodySet, collider_set: &mut ColliderSet) {
    let block_body = RigidBodyBuilder::fixed().build();
    let block_handle = rigid_body_set.insert(block_body);
    for block in BLOCKS {
        let collider = ColliderBuilder::cuboid(block.3, block.4, block.5)
            .translation(Vector::new(block.0, block.1, block.2))
            .build();
        collider_set.insert_with_parent(collider, block_handle, rigid_body_set);
    }

    let ramp_body = RigidBodyBuilder::fixed().build();
    let ramp_handle = rigid_body_set.insert(ramp_body);
    for ramp in RAMPS {
        let z0 = ramp.0;
        let z1 = ramp.1;
        let x0 = ramp.2 - ramp.3;
        let x1 = ramp.2 + ramp.3;
        let h = ramp.4;
        let points = [
            Vector::new(x0, 0.0, z0),
            Vector::new(x1, 0.0, z0),
            Vector::new(x0, 0.0, z1),
            Vector::new(x1, 0.0, z1),
            Vector::new(x0, h, z1),
            Vector::new(x1, h, z1),
        ];
        if let Some(collider) = ColliderBuilder::convex_hull(&points) {
            collider_set.insert_with_parent(collider.build(), ramp_handle, rigid_body_set);
        }
    }
}
