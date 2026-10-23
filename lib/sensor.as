// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/sensor.c

import { b3BitSet } from "./bitset";

// Used to track shapes that hit sensors using time of impact
export struct b3SensorHit {
	sensorId: i32 = 0;
	visitorId: i32 = 0;

	constructor(sensorId: i32, visitorId: i32) {
		this.sensorId = sensorId;
		this.visitorId = visitorId;
	}
}

export struct b3Visitor {
	shapeId: i32 = 0;
	generation: u16 = 0;

	constructor(shapeId: i32, generation: u16) {
		this.shapeId = shapeId;
		this.generation = generation;
	}
}

export class b3Sensor {
	hits: Array<b3Visitor> = new Array<b3Visitor>();
	overlaps1: Array<b3Visitor> = new Array<b3Visitor>();
	overlaps2: Array<b3Visitor> = new Array<b3Visitor>();
	shapeId: i32 = 0;
}

export class b3SensorTaskContext {
	eventBits: b3BitSet = new b3BitSet();
}

// functions from sensor.c follow
