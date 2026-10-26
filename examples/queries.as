// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// World queries over three boxes in a row: the closest ray hit, every ray hit, a sphere cast, and overlaps of a
// box and of a shape. Each box's shape carries its index as user data.

import {
	b3AABB,
	b3CreateBody,
	b3CreateHullShape,
	b3CreateWorld,
	b3DefaultBodyDef,
	b3DefaultQueryFilter,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3DestroyWorld,
	b3MakeBoxHull,
	b3Pos,
	b3ShapeId,
	b3ShapeProxy,
	b3Shape_GetUserData,
	b3Vec3,
	b3World_CastRay,
	b3World_CastRayClosest,
	b3World_CastShape,
	b3World_OverlapAABB,
	b3World_OverlapShape,
} from "../lib/box3d";

export class QueriesResult {
	closestBox: i32 = -1;
	closestHeight: f32 = 0;
	rayHitCount: i32 = 0;
	castBox: i32 = -1;
	castFraction: f32 = 0;
	aabbOverlaps: i32 = 0;
	shapeOverlaps: i32 = 0;
}

// The user data of boxes is 1 + index, so the ground's 0 reads as -1
function boxIndex(shapeId: b3ShapeId): i32 {
	return <i32>b3Shape_GetUserData(shapeId) - 1;
}

export function queries(): QueriesResult {
	let worldId = b3CreateWorld(b3DefaultWorldDef());

	let groundDef = b3DefaultBodyDef();
	groundDef.position = b3Vec3(0.0, -1.0, 0.0);
	let groundId = b3CreateBody(worldId, groundDef);
	b3CreateHullShape(groundId, b3DefaultShapeDef(), b3MakeBoxHull(20.0, 1.0, 20.0).base);

	// 1 m boxes on the ground at x = -4, 0 and 4. Static, so the queries see them without a step.
	let box = b3MakeBoxHull(0.5, 0.5, 0.5);
	for (let i = 0; i < 3; ++i) {
		let bodyDef = b3DefaultBodyDef();
		bodyDef.position = b3Vec3(<f32>(4 * i - 4), 0.5, 0.0);
		let bodyId = b3CreateBody(worldId, bodyDef);
		let shapeDef = b3DefaultShapeDef();
		shapeDef.userData = <u64>(i + 1);
		b3CreateHullShape(bodyId, shapeDef, box.base);
	}

	let filter = b3DefaultQueryFilter();
	let result = new QueriesResult();

	// Straight down onto the middle box
	let ray = b3World_CastRayClosest(worldId, b3Vec3(0.0, 10.0, 0.0), b3Vec3(0.0, -20.0, 0.0), filter);
	if (ray.hit) {
		result.closestBox = boxIndex(ray.shapeId);
		result.closestHeight = ray.point.y;
	}

	// Along the row through all three boxes. Returning 1 keeps the full ray, so every shape on it reports.
	b3World_CastRay(worldId, b3Vec3(-10.0, 0.5, 0.0), b3Vec3(20.0, 0.0, 0.0), filter,
		(shapeId: b3ShapeId, point: b3Pos, normal: b3Vec3, fraction: f32, userMaterialId: u64, triangleIndex: i32,
			childIndex: i32): f32 => {
			result.rayHitCount += 1;
			return 1.0;
		});

	// A sphere of radius 0.25, one point at the origin, swept along the same line. Returning the fraction clips the
	// cast there, so the last report is the closest hit.
	let sphere = new b3ShapeProxy(new StaticArray<b3Vec3>(1), 1, 0.25);
	b3World_CastShape(worldId, b3Vec3(-10.0, 0.5, 0.0), sphere, b3Vec3(20.0, 0.0, 0.0), filter,
		(shapeId: b3ShapeId, point: b3Pos, normal: b3Vec3, fraction: f32, userMaterialId: u64, triangleIndex: i32,
			childIndex: i32): f32 => {
			result.castBox = boxIndex(shapeId);
			result.castFraction = fraction;
			return fraction;
		});

	// Bounding boxes overlapping a box around the last box, clear of the ground
	let aabb = b3AABB(b3Vec3(3.0, 0.2, -1.0), b3Vec3(5.0, 2.0, 1.0));
	b3World_OverlapAABB(worldId, aabb, filter, (shapeId: b3ShapeId): bool => {
		result.aabbOverlaps += 1;
		return true;
	});

	// Shapes touching a 1 m sphere centered on the last box: the box and the ground under it
	let ball = new b3ShapeProxy(new StaticArray<b3Vec3>(1), 1, 1.0);
	b3World_OverlapShape(worldId, b3Vec3(4.0, 0.5, 0.0), ball, filter, (shapeId: b3ShapeId): bool => {
		result.shapeOverlaps += 1;
		return true;
	});

	b3DestroyWorld(worldId);
	return result;
}
