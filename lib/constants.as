// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of include/box3d/constants.h

import { b3GetLengthUnitsPerMeter } from "./core";
import { B3_PI } from "./math_functions";

export { b3SetLengthUnitsPerMeter, b3GetLengthUnitsPerMeter } from "./core";

// Used to detect bad values. In float mode positions greater than about 16km have precision
// problems, so 100km is a safe limit.
@inline
export function B3_HUGE(): f32 {
	return 1.0e5 * b3GetLengthUnitsPerMeter();
}

/// Maximum parallel workers. Used for some fixed size arrays.
export const B3_MAX_WORKERS: i32 = 32;

/// Maximum number of tasks queued per world step.
export const B3_MAX_TASKS: i32 = 256;

// Maximum number of colors in the constraint graph. Constraints that cannot
// find a color are added to the overflow set which are solved single-threaded.
// The compound barrel benchmark has minor overflow with 24 colors
export const B3_GRAPH_COLOR_COUNT: i32 = 24;

// Number of contact point buckets for counting the number of contact points per
// shape contact pair. This is just for reporting and doesn't affect simulation.
export const B3_CONTACT_MANIFOLD_COUNT_BUCKETS: i32 = 8;

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// @warning modifying this can have a significant impact on stability
@inline
export function B3_LINEAR_SLOP(): f32 {
	return 0.005 * b3GetLengthUnitsPerMeter();
}

/// The minimum length of a capsules. Very short capsules should be created as spheres
/// to avoid numerical problems.
@inline
export function B3_MIN_CAPSULE_LENGTH(): f32 {
	return B3_LINEAR_SLOP();
}

/// Minimum contact point friction weight, lower bound for speculative points. Made small
/// enough to be washed away by weights that hit 1.
export const B3_MIN_FRICTION_WEIGHT: f32 = 1e-10;

/// The distance between shapes where they are considered overlapped. This is needed
/// because GJK may return small positive values for overlapped shapes in degenerate
/// configurations.
@inline
export function B3_OVERLAP_SLOP(): f32 {
	return 0.1 * B3_LINEAR_SLOP();
}

/// Maximum number of simultaneous worlds that can be allocated
export const B3_MAX_WORLDS: i32 = 128;

/// The maximum rotation of a body per time step. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
/// @warning increasing this to 0.5f * B3_PI or greater will break continuous collision.
export const B3_MAX_ROTATION: f32 = 0.25 * B3_PI;

/// @warning modifying this can have a significant impact on performance and stability
@inline
export function B3_SPECULATIVE_DISTANCE(): f32 {
	return 4.0 * B3_LINEAR_SLOP();
}

/// The rest offset is used for mesh contact to reduce ghost collisions and assist with CCD.
/// The rest offset adjusts the contact point separation value, making the solver push the shapes
/// apart by this distance.
/// Must be at least B3_LINEAR_SLOP and less than B3_SPECULATIVE_DISTANCE.
@inline
export function B3_MESH_REST_OFFSET(): f32 {
	return 1.0 * B3_LINEAR_SLOP();
}

/// The default contact recycling distance.
@inline
export function B3_CONTACT_RECYCLE_DISTANCE(): f32 {
	return 10.0 * B3_LINEAR_SLOP();
}

/// The default contact recycling world angle threshold. For performance this value
/// is cos(angle/2)^2. This value corresponds to 10 degrees.
export const B3_CONTACT_RECYCLE_ANGULAR_DISTANCE: f32 = 0.99240388;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment. This is in meters.
/// @warning modifying this can have a significant impact on performance
@inline
export function B3_MAX_AABB_MARGIN(): f32 {
	return 0.05 * b3GetLengthUnitsPerMeter();
}

/// Per-shape AABB margin is a fraction of the shape extent (capped by B3_MAX_AABB_MARGIN).
/// Small shapes get small margins; large shapes are clamped to the cap.
export const B3_AABB_MARGIN_FRACTION: f32 = 0.125;

/// The time that a body must be still before it will go to sleep. In seconds.
export const B3_TIME_TO_SLEEP: f32 = 0.5;

/// The maximum number of restitution iterations. Needed to avoid a solver overflow.
export const B3_MAX_RESTITUTION_ITERATIONS: i32 = 63;

/// The maximum number of contact points between two touching shapes.
export const B3_MAX_MANIFOLD_POINTS: i32 = 4;

/// The number of iterations for gyroscopic torques.
export const B3_GYROSCOPIC_ITERATIONS: i32 = 1;

/// The maximum number of convex hull vertices. This is fixed for performance reasons.
export const B3_MAX_HULL_VERTICES: i32 = 128;

/// The maximum number of convex hull faces.
export const B3_MAX_HULL_FACES: i32 = 128;

/// The maximum number of convex hull edges. Full edges, not half-edges.
export const B3_MAX_HULL_EDGES: i32 = 128;

/// Relative tolerance used to determine if two edges are parallel.
export const B3_PARALLEL_EDGE_TOL: f32 = 0.005;

/// The maximum number points to use for shape cast proxies (swept point cloud).
export const B3_MAX_SHAPE_CAST_POINTS: i32 = B3_MAX_HULL_VERTICES;

/// These generous limits allow for easy hashing. See b3ShapePairKey.
export const B3_SHAPE_POWER: i32 = 22;
export const B3_CHILD_POWER: i32 = 64 - 2 * B3_SHAPE_POWER;
export const B3_MAX_SHAPES: i32 = 1 << B3_SHAPE_POWER;
export const B3_MAX_CHILD_SHAPES: i32 = 1 << B3_CHILD_POWER;

/// This is the limit on how many mesh or heightfield triangles a single convex shape can collide with.
export const B3_MAX_MESH_CONTACT_TRIANGLES: i32 = 512;
