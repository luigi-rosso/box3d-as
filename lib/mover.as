// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/mover.c

import { b3AbsFloat, b3ClampFloat, b3Dot, b3MinFloat, b3MulAdd, b3MulSub, b3Vec3 } from "./math_functions";
import { b3PlaneSeparation } from "./math_internal";
import { b3CollisionPlane, b3PlaneSolverResult } from "./collision";
import { B3_LINEAR_SLOP } from "./constants";

/// Solves the position of a mover that satisfies the given collision planes.
/// @param targetDelta the desired translation from the position used to generate the collision planes
/// @param planes the collision planes
/// @param count the number of collision planes
export function b3SolvePlanes(
	targetDelta: b3Vec3,
	planes: StaticArray<b3CollisionPlane>,
	count: i32,
): b3PlaneSolverResult {
	for (let i = 0; i < count; ++i) {
		planes[i].push = 0.0;
	}

	let delta = targetDelta;
	let tolerance = B3_LINEAR_SLOP();

	let iteration: i32;
	for (iteration = 0; iteration < 20; ++iteration) {
		let totalPush: f32 = 0.0;
		for (let planeIndex = 0; planeIndex < count; ++planeIndex) {
			ref plane = planes[planeIndex];

			// Add slop to prevent jitter
			let separation = b3PlaneSeparation(plane.plane, delta) + B3_LINEAR_SLOP();

			let push = -separation;

			// Clamp accumulated push
			let accumulatedPush = plane.push;
			plane.push = b3ClampFloat(plane.push + push, 0.0, plane.pushLimit);
			push = plane.push - accumulatedPush;
			delta = b3MulAdd(delta, push, plane.plane.normal);

			// Track maximum push for convergence
			totalPush += b3AbsFloat(push);
		}

		if (totalPush < tolerance) {
			break;
		}
	}

	let result: b3PlaneSolverResult;
	result.delta = delta;
	result.iterationCount = iteration;
	return result;
}

/// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
/// set to false are skipped.
export function b3ClipVector(vector: b3Vec3, planes: StaticArray<b3CollisionPlane>, count: i32): b3Vec3 {
	let v = vector;

	for (let planeIndex = 0; planeIndex < count; ++planeIndex) {
		let plane = planes[planeIndex];
		if (plane.push == 0.0 || plane.clipVelocity == false) {
			continue;
		}

		v = b3MulSub(v, b3MinFloat(0.0, b3Dot(v, plane.plane.normal)), plane.plane.normal);
	}

	return v;
}
