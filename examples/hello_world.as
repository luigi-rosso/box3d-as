// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Hello world: a world, a ground box and a falling box, stepped until the box rests on the ground.

import {
	b3Body_GetPosition,
	b3Body_IsAwake,
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

export class HelloWorldResult {
	height: f32 = 0;
	awake: bool = true;
}

export function helloWorld(): HelloWorldResult {
	let worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3(0.0, -10.0, 0.0);
	let worldId = b3CreateWorld(worldDef);

	// A static body is the default; its box is 20 m wide with the top face at y = 0
	let groundDef = b3DefaultBodyDef();
	groundDef.position = b3Vec3(0.0, -1.0, 0.0);
	let groundId = b3CreateBody(worldId, groundDef);
	let groundBox = b3MakeBoxHull(10.0, 1.0, 10.0);
	b3CreateHullShape(groundId, b3DefaultShapeDef(), groundBox.base);

	// A dynamic 1 m cube, 4 m up
	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = b3Vec3(0.0, 4.0, 0.0);
	let bodyId = b3CreateBody(worldId, bodyDef);
	let box = b3MakeBoxHull(0.5, 0.5, 0.5);
	let shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0;
	b3CreateHullShape(bodyId, shapeDef, box.base);

	// A constant time step and 4 sub-steps, as Box3D recommends
	let timeStep: f32 = <f32>(1.0 / 60.0);
	let subStepCount = 4;
	for (let i = 0; i < 180; ++i) {
		b3World_Step(worldId, timeStep, subStepCount);
	}

	let result = new HelloWorldResult();
	result.height = b3Body_GetPosition(bodyId).y;
	result.awake = b3Body_IsAwake(bodyId);

	// Destroying the world destroys its bodies, shapes and joints
	b3DestroyWorld(worldId);
	return result;
}
