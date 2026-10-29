// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Threading: the same pyramid stepped on the main lane alone, with its narrow phase and solver stages on four lanes,
// and on parallel calls over the world's own memory. Results do not depend on the thread count, so the three runs end
// bit for bit the same. Lanes and parallel calls serve one world at a time; notes/threading.md says what each costs.

import {
	b3BodyId,
	b3Body_GetPosition,
	b3CreateBody,
	b3CreateHullShape,
	b3CreateWorld,
	b3DefaultBodyDef,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3DestroyWorld,
	b3MakeBoxHull,
	b3Vec3,
	b3World_Step,
	b3_dynamicBody,
} from "../lib/box3d";
import { b3World_EnableLanes, b3_laneBackendWorkers, b3_laneCollide, b3_laneSolve } from "../lib/lanes";
import { b3World_EnableParallel, b3_parallelCollide, b3_parallelSolve, b3_parallelStats } from "../lib/parallel_step";

const BASE_COUNT: i32 = 10;

export class PyramidRun {
	// Every box's final position
	positions: StaticArray<f32> = new StaticArray<f32>(0);
	// Steps whose solver stages and narrow phase ran on the lanes or parallel calls
	solves: i32 = 0;
	collides: i32 = 0;
}

export const RUN_SERIAL: i32 = 0;
export const RUN_LANES: i32 = 1;
export const RUN_PARALLEL: i32 = 2;

// Steps a pyramid of boxes serially, on four lanes or on parallel calls
export function stepPyramid(how: i32): PyramidRun {
	// Awake for the whole run, so every step has work for the lanes
	let worldDef = b3DefaultWorldDef();
	worldDef.enableSleep = false;
	let worldId = b3CreateWorld(worldDef);

	let groundDef = b3DefaultBodyDef();
	groundDef.position = b3Vec3(0.0, -1.0, 0.0);
	let groundId = b3CreateBody(worldId, groundDef);
	b3CreateHullShape(groundId, b3DefaultShapeDef(), b3MakeBoxHull(20.0, 1.0, 20.0).base);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	let box = b3MakeBoxHull(0.5, 0.5, 0.5);
	let bodies = new Array<b3BodyId>();
	for (let row = 0; row < BASE_COUNT; ++row) {
		for (let i = row; i < BASE_COUNT; ++i) {
			bodyDef.position = b3Vec3(<f32>(2 * i - row) * 0.5 - <f32>BASE_COUNT * 0.5, <f32>row + 0.5, 0.0);
			let bodyId = b3CreateBody(worldId, bodyDef);
			b3CreateHullShape(bodyId, b3DefaultShapeDef(), box.base);
			bodies.push(bodyId);
		}
	}

	// Lanes run on workers here, b3_laneBackendJobs puts them on the job pool instead
	let lanes = how == RUN_LANES ? b3World_EnableLanes(worldId, 4, b3_laneBackendWorkers, b3_laneCollide | b3_laneSolve) : null;
	// Parallel calls run as many parts as Parallel.size(), one where the runtime has no threads
	if (how == RUN_PARALLEL) {
		b3World_EnableParallel(worldId, b3_parallelCollide | b3_parallelSolve);
	}
	let solves = b3_parallelStats.solves;
	let collides = b3_parallelStats.collides;

	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 120; ++i) {
		b3World_Step(worldId, timeStep, 4);
	}

	let run = new PyramidRun();
	if (lanes != null) {
		run.solves = lanes.stats.solves;
		run.collides = lanes.collide.steps;
		// Zero lanes stops the lanes and gives the world back its own task callbacks
		b3World_EnableLanes(worldId, 0, b3_laneBackendWorkers, 0);
	}
	if (how == RUN_PARALLEL) {
		run.solves = b3_parallelStats.solves - solves;
		run.collides = b3_parallelStats.collides - collides;
		b3World_EnableParallel(worldId, 0);
	}

	let positions = new StaticArray<f32>(3 * bodies.length);
	for (let i = 0; i < bodies.length; ++i) {
		let p = b3Body_GetPosition(bodies[i]);
		positions[3 * i] = p.x;
		positions[3 * i + 1] = p.y;
		positions[3 * i + 2] = p.z;
	}
	run.positions = positions;
	b3DestroyWorld(worldId);
	return run;
}

export class ThreadingResult {
	// Coordinates whose bits differ from the serial run's
	differ: i32 = 0;
	solves: i32 = 0;
	collides: i32 = 0;
}

// Runs the pyramid serially and then as how says
export function threading(how: i32): ThreadingResult {
	let serial = stepPyramid(RUN_SERIAL).positions;
	let threaded = stepPyramid(how);
	let result = new ThreadingResult();
	for (let i = 0; i < serial.length; ++i) {
		if (reinterpret<u32>(serial[i]) != reinterpret<u32>(threaded.positions[i])) {
			result.differ += 1;
		}
	}
	result.solves = threaded.solves;
	result.collides = threaded.collides;
	return result;
}
