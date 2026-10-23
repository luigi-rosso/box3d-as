// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/aabb.h and src/aabb.c

import { FLT_EPSILON } from "./core";
import { b3AABB, b3AbsFloat, b3ClampFloat, b3Length, b3MaxFloat, b3MinFloat, b3MulSV, b3Sub, b3Vec3 } from "./math_functions";

// Get the surface area (perimeter)
export function b3Perimeter(a: b3AABB): f32 {
	let wx = a.upperBound.x - a.lowerBound.x;
	let wy = a.upperBound.y - a.lowerBound.y;
	let wz = a.upperBound.z - a.lowerBound.z;
	return 2.0 * (wx * wz + wy * wx + wz * wy);
}

/// Enlarge a to contain b
/// @return true if the AABB grew
export function b3EnlargeAABB(a: ref b3AABB, b: b3AABB): bool {
	let changed = false;
	if (b.lowerBound.x < a.lowerBound.x) {
		a.lowerBound.x = b.lowerBound.x;
		changed = true;
	}

	if (b.lowerBound.y < a.lowerBound.y) {
		a.lowerBound.y = b.lowerBound.y;
		changed = true;
	}

	if (b.lowerBound.z < a.lowerBound.z) {
		a.lowerBound.z = b.lowerBound.z;
		changed = true;
	}

	if (a.upperBound.x < b.upperBound.x) {
		a.upperBound.x = b.upperBound.x;
		changed = true;
	}

	if (a.upperBound.y < b.upperBound.y) {
		a.upperBound.y = b.upperBound.y;
		changed = true;
	}

	if (a.upperBound.z < b.upperBound.z) {
		a.upperBound.z = b.upperBound.z;
		changed = true;
	}

	return changed;
}

export function b3FarthestPointOnAABB(b: b3AABB, p: b3Vec3): b3Vec3 {
	return b3Vec3(
		p.x - b.lowerBound.x > b.upperBound.x - p.x ? b.lowerBound.x : b.upperBound.x,
		p.y - b.lowerBound.y > b.upperBound.y - p.y ? b.lowerBound.y : b.upperBound.y,
		p.z - b.lowerBound.z > b.upperBound.z - p.z ? b.lowerBound.z : b.upperBound.z,
	);
}

// Similar to Real-time Collision Detection, p179.
// Ray cast an AABB. This is a custom function used by height fields.
export function b3RayCastAABB(a: b3AABB, p1: b3Vec3, p2: b3Vec3, minFraction: ref f32, maxFraction: ref f32): bool {
	// Ray direction and length
	let d = b3Sub(p2, p1);
	let rayLength = b3Length(d);

	// Handle degenerate ray
	if (rayLength < FLT_EPSILON) {
		// Check if point is inside AABB
		if (
			p1.x >= a.lowerBound.x &&
			p1.x <= a.upperBound.x &&
			p1.y >= a.lowerBound.y &&
			p1.y <= a.upperBound.y &&
			p1.z >= a.lowerBound.z &&
			p1.z <= a.upperBound.z
		) {
			minFraction = 0.0;
			maxFraction = 0.0;
			return true;
		}

		return false;
	}

	let rayDir = b3MulSV(1.0 / rayLength, d);

	// Slab method for ray-AABB intersection
	let tMin: f32 = 0.0;
	let tMax = rayLength;

	// x-axis
	{
		let rayComponent = rayDir.x;
		let rayStart = p1.x;
		let boxMin = a.lowerBound.x;
		let boxMax = a.upperBound.x;

		if (b3AbsFloat(rayComponent) < FLT_EPSILON) {
			// Ray is parallel to slab, check if ray origin is within slab
			if (rayStart < boxMin || rayStart > boxMax) {
				return false;
			}
		} else {
			// Compute intersection distances
			let t1 = (boxMin - rayStart) / rayComponent;
			let t2 = (boxMax - rayStart) / rayComponent;

			// Ensure t1 <= t2
			if (t1 > t2) {
				let temp = t1;
				t1 = t2;
				t2 = temp;
			}

			// Update intersection interval
			tMin = b3MaxFloat(tMin, t1);
			tMax = b3MinFloat(tMax, t2);

			// Check for no intersection
			if (tMin > tMax) {
				return false;
			}
		}
	}

	// y-axis
	{
		let rayComponent = rayDir.y;
		let rayStart = p1.y;
		let boxMin = a.lowerBound.y;
		let boxMax = a.upperBound.y;

		if (b3AbsFloat(rayComponent) < FLT_EPSILON) {
			// Ray is parallel to slab, check if ray origin is within slab
			if (rayStart < boxMin || rayStart > boxMax) {
				return false;
			}
		} else {
			// Compute intersection distances
			let t1 = (boxMin - rayStart) / rayComponent;
			let t2 = (boxMax - rayStart) / rayComponent;

			// Ensure t1 <= t2
			if (t1 > t2) {
				let temp = t1;
				t1 = t2;
				t2 = temp;
			}

			// Update intersection interval
			tMin = b3MaxFloat(tMin, t1);
			tMax = b3MinFloat(tMax, t2);

			// Check for no intersection
			if (tMin > tMax) {
				return false;
			}
		}
	}

	// z-axis
	{
		let rayComponent = rayDir.z;
		let rayStart = p1.z;
		let boxMin = a.lowerBound.z;
		let boxMax = a.upperBound.z;

		if (b3AbsFloat(rayComponent) < FLT_EPSILON) {
			// Ray is parallel to slab, check if ray origin is within slab
			if (rayStart < boxMin || rayStart > boxMax) {
				return false;
			}
		} else {
			// Compute intersection distances
			let t1 = (boxMin - rayStart) / rayComponent;
			let t2 = (boxMax - rayStart) / rayComponent;

			// Ensure t1 <= t2
			if (t1 > t2) {
				let temp = t1;
				t1 = t2;
				t2 = temp;
			}

			// Update intersection interval
			tMin = b3MaxFloat(tMin, t1);
			tMax = b3MinFloat(tMax, t2);

			// Check for no intersection
			if (tMin > tMax) {
				return false;
			}
		}
	}

	// Check if intersection is behind ray start
	if (tMax < 0.0) {
		return false;
	}

	// Convert distances to fractions
	minFraction = b3ClampFloat(tMin / rayLength, 0.0, 1.0);
	maxFraction = b3ClampFloat(tMax / rayLength, 0.0, 1.0);

	return true;
}
