// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Events: a box falls through a sensor volume onto the ground. The sensor reports it entering and leaving, and the
// landing reports a contact begin and a hit. Events are read after each step and last until the next one.

import {
	b3CreateBody,
	b3CreateHullShape,
	b3CreateWorld,
	b3DefaultBodyDef,
	b3DefaultShapeDef,
	b3DefaultWorldDef,
	b3DestroyWorld,
	b3MakeBoxHull,
	b3Vec3,
	b3World_GetContactEvents,
	b3World_GetSensorEvents,
	b3World_Step,
	b3_dynamicBody,
} from "../lib/box3d";

export class EventsResult {
	sensorBegins: i32 = 0;
	sensorEnds: i32 = 0;
	contactBegins: i32 = 0;
	hits: i32 = 0;
	fastestHit: f32 = 0;
}

export function events(): EventsResult {
	let worldId = b3CreateWorld(b3DefaultWorldDef());

	// Contact and hit events are off by default; either shape of a pair can ask for them
	let groundDef = b3DefaultBodyDef();
	groundDef.position = b3Vec3(0.0, -1.0, 0.0);
	let groundId = b3CreateBody(worldId, groundDef);
	let groundShapeDef = b3DefaultShapeDef();
	groundShapeDef.enableContactEvents = true;
	groundShapeDef.enableHitEvents = true;
	b3CreateHullShape(groundId, groundShapeDef, b3MakeBoxHull(10.0, 1.0, 10.0).base);

	// A sensor 2 m thick between y = 2 and y = 4. It detects shapes that enable sensor events, and it needs them on
	// itself to report.
	let sensorDef = b3DefaultBodyDef();
	sensorDef.position = b3Vec3(0.0, 3.0, 0.0);
	let sensorId = b3CreateBody(worldId, sensorDef);
	let sensorShapeDef = b3DefaultShapeDef();
	sensorShapeDef.isSensor = true;
	sensorShapeDef.enableSensorEvents = true;
	b3CreateHullShape(sensorId, sensorShapeDef, b3MakeBoxHull(2.0, 1.0, 2.0).base);

	let bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = b3Vec3(0.0, 6.0, 0.0);
	let boxId = b3CreateBody(worldId, bodyDef);
	let shapeDef = b3DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b3CreateHullShape(boxId, shapeDef, b3MakeBoxHull(0.5, 0.5, 0.5).base);

	let result = new EventsResult();
	let timeStep: f32 = <f32>(1.0 / 60.0);
	for (let i = 0; i < 120; ++i) {
		b3World_Step(worldId, timeStep, 4);

		let sensorEvents = b3World_GetSensorEvents(worldId);
		result.sensorBegins += sensorEvents.beginCount;
		result.sensorEnds += sensorEvents.endCount;

		let contactEvents = b3World_GetContactEvents(worldId);
		result.contactBegins += contactEvents.beginCount;
		for (let j = 0; j < contactEvents.hitCount; ++j) {
			// A hit is a contact that began faster than the world's hitEventThreshold, 1 m/s by default
			let hit = contactEvents.hitEvents[j];
			result.hits += 1;
			result.fastestHit = Mathf.max(result.fastestHit, hit.approachSpeed);
		}
	}

	b3DestroyWorld(worldId);
	return result;
}
