// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The character mover: a capsule that is not a body, moved by queries. Each step it gathers the planes it touches
// with b3World_CollideMover, finds a move that respects them with b3SolvePlanes, sweeps that move with
// b3World_CastMover, and clips its velocity against the planes with b3ClipVector. Here it falls onto the ground
// and walks into a wall. demo/mover.as adds a pogo spring, jumping and pushing bodies.

import {
	B3_HUGE,
	b3Add,
	b3Capsule,
	b3ClipVector,
	b3CollisionPlane,
	b3CreateBody,
	b3CreateHullShape,
	b3CreateWorld,
	b3DefaultBodyDef,
	b3DefaultQueryFilter,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3DestroyWorld,
	b3LengthSquared,
	b3MakeBoxHull,
	b3MulSV,
	b3PlaneResult,
	b3Pos,
	b3ShapeId,
	b3SolvePlanes,
	b3Sub,
	b3Vec3,
	b3WorldId,
	b3World_CastMover,
	b3World_CollideMover,
	b3World_Step,
} from "../lib/box3d";

const PLANE_CAPACITY: i32 = 8;
const GRAVITY: f32 = 15.0;
const WALK_SPEED: f32 = 3.0;

export class Mover {
	position: b3Pos = b3Vec3(0.0, 1.0, 0.0);
	velocity: b3Vec3 = b3Vec3();
	// Relative to position: the capsule's bottom is 0.8 m below it
	capsule: b3Capsule = b3Capsule(b3Vec3(0.0, -0.5, 0.0), b3Vec3(0.0, 0.5, 0.0), 0.3);
	planes: StaticArray<b3CollisionPlane> = new StaticArray<b3CollisionPlane>(PLANE_CAPACITY);
	planeCount: i32 = 0;
}

function stepMover(worldId: b3WorldId, mover: Mover, timeStep: f32): void {
	mover.velocity.x = WALK_SPEED;
	mover.velocity.y -= GRAVITY * timeStep;
	let target = b3Add(mover.position, b3MulSV(timeStep, mover.velocity));

	let filter = b3DefaultQueryFilter();
	let collectPlanes = (shapeId: b3ShapeId, results: StaticArray<b3PlaneResult>, count: i32): bool => {
		for (let i = 0; i < count && mover.planeCount < PLANE_CAPACITY; ++i) {
			let plane = b3CollisionPlane();
			plane.plane = results[i].plane;
			// A wall or the ground may push the mover as far as it needs
			plane.pushLimit = B3_HUGE();
			plane.clipVelocity = true;
			mover.planes[mover.planeCount] = plane;
			mover.planeCount += 1;
		}
		return true;
	};

	let tolerance: f32 = 0.01;
	for (let iteration = 0; iteration < 5; ++iteration) {
		mover.planeCount = 0;
		b3World_CollideMover(worldId, mover.position, mover.capsule, filter, collectPlanes);

		let solved = b3SolvePlanes(b3Sub(target, mover.position), mover.planes, mover.planeCount);
		let fraction = b3World_CastMover(worldId, mover.position, mover.capsule, solved.delta, filter, null);
		let delta = b3MulSV(fraction, solved.delta);
		mover.position = b3Add(mover.position, delta);

		if (b3LengthSquared(delta) < tolerance * tolerance) {
			break;
		}
	}

	// Drops the velocity into the ground and the wall, so it does not build up while the planes hold the mover
	mover.velocity = b3ClipVector(mover.velocity, mover.planes, mover.planeCount);
}

export function moverExample(): Mover {
	let worldId = b3CreateWorld(b3DefaultWorldDef());

	let groundDef = b3DefaultBodyDef();
	groundDef.position = b3Vec3(0.0, -1.0, 0.0);
	let groundId = b3CreateBody(worldId, groundDef);
	b3CreateHullShape(groundId, b3DefaultShapeDef(), b3MakeBoxHull(10.0, 1.0, 10.0).base);

	// A wall whose face is at x = 4
	let wallDef = b3DefaultBodyDef();
	wallDef.position = b3Vec3(4.5, 1.0, 0.0);
	let wallId = b3CreateBody(worldId, wallDef);
	b3CreateHullShape(wallId, b3DefaultShapeDef(), b3MakeBoxHull(0.5, 1.0, 5.0).base);

	let mover = new Mover();
	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 180; ++i) {
		// The mover reads the world, so it moves between steps, against the bodies where the step left them
		b3World_Step(worldId, timeStep, 4);
		stepMover(worldId, mover, timeStep);
	}

	b3DestroyWorld(worldId);
	return mover;
}
