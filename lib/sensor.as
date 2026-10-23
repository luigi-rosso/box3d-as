// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/sensor.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX } from "./core";
import { B3_MAX_SHAPE_CAST_POINTS } from "./constants";
import { b3BitSet, b3InPlaceUnion, b3SetBit, b3SetBitCountAndClear } from "./bitset";
import { b3CTZ64 } from "./ctz";
import {
	b3InvMulTransforms,
	b3MinInt,
	b3Pos_zero,
	b3ToRelativeTransform,
	b3Transform,
	b3TransformPoint,
	b3Transform_identity,
	b3Vec3,
} from "./math_functions";
import { b3ShapeProxy } from "./collision";
import {
	b3SensorBeginTouchEvent,
	b3SensorEndTouchEvent,
	b3ShapeId,
	b3_capsuleShape,
	b3_compoundShape,
	b3_heightShape,
	b3_hullShape,
	b3_meshShape,
	b3_sphereShape,
} from "./types";
import { b3Array_Append, b3Array_Emplace, b3Array_RemoveSwap } from "./container";
import { Q_LOOP } from "./qsort";
import { b3DynamicTree_Query } from "./dynamic_tree";
import { b3OverlapCapsule } from "./capsule";
import { b3OverlapSphere } from "./sphere";
import { b3OverlapHull } from "./hull";
import { b3OverlapMesh } from "./mesh";
import { b3OverlapHeightField } from "./height_field";
import { b3OverlapCompound } from "./compound";
import { b3GetBodyTransform, b3GetBodyTransformQuick } from "./body";
import { b3ParallelFor } from "./parallel_for";
import { b3World, b3_disabledSet } from "./physics_world";
import {
	b3IsConvex,
	b3SetShapeProxy,
	b3Shape,
	b3ShouldShapesCollide,
	b3_enableCustomFiltering,
	b3_enableSensorEvents,
} from "./shape";

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

// C's per sensor stack context. The sensor task never nests, so one context serves every sensor
// and the tree callback captures nothing.
class b3SensorQueryContext {
	world: b3World? = null;
	taskContext: b3SensorTaskContext? = null;
	sensor: b3Sensor? = null;
	sensorShape: b3Shape? = null;
	transform: b3Transform = b3Transform();
}

const s_sensorQueryContext = new b3SensorQueryContext();

function b3SensorTreeCallback(proxyId: i32, userData: u64): bool {
	return b3SensorQueryCallback(proxyId, userData, s_sensorQueryContext);
}

// C's stack proxies of b3OverlapSensor, which never nests
const s_visitorPoints = new StaticArray<b3Vec3>(2);
const s_visitorProxy = new b3ShapeProxy();
const s_sensorLocalPoints = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);
const s_sensorLocalProxy = new b3ShapeProxy();

function b3OverlapSensor(
	sensorShape: b3Shape,
	sensorTransform: b3Transform,
	visitorShape: b3Shape,
	visitorTransform: b3Transform,
): bool {
	let type = sensorShape.type;

	let proxy = b3SetShapeProxy(s_visitorProxy, visitorShape, s_visitorPoints);

	// Get the visitor shape in the frame of the sensor
	let relativeTransform = b3InvMulTransforms(sensorTransform, visitorTransform);

	let localPoints = s_sensorLocalPoints;
	let localProxy = s_sensorLocalProxy;

	localProxy.count = b3MinInt(proxy.count, B3_MAX_SHAPE_CAST_POINTS);
	for (let i = 0; i < localProxy.count; ++i) {
		localPoints[i] = b3TransformPoint(relativeTransform, proxy.points[i]);
	}

	localProxy.points = localPoints;
	localProxy.radius = proxy.radius;

	switch (type) {
		case b3_capsuleShape:
			return b3OverlapCapsule(sensorShape.capsule, b3Transform_identity, localProxy);

		case b3_compoundShape:
			return b3OverlapCompound(sensorShape.compound!, b3Transform_identity, localProxy);

		case b3_heightShape:
			return b3OverlapHeightField(sensorShape.heightField!, b3Transform_identity, localProxy);

		case b3_hullShape:
			return b3OverlapHull(sensorShape.hull!, b3Transform_identity, localProxy);

		case b3_meshShape:
			return b3OverlapMesh(sensorShape.mesh!, b3Transform_identity, localProxy);

		case b3_sphereShape:
			return b3OverlapSphere(sensorShape.sphere, b3Transform_identity, localProxy);

		default:
			B3_ASSERT(false);
			return false;
	}
}

// Sensor shapes need to
// - detect begin and end overlap events
// - events must be reported in deterministic order
// - maintain an active list of overlaps for query

// Assumption
// - sensors don't detect shapes on the same body

// Algorithm
// Query all sensors for overlaps
// Check against previous overlaps

// Data structures
// Each sensor has an double buffered array of overlaps
// These overlaps use a shape reference with index and generation

function b3SensorQueryCallback(_proxyId: i32, userData: u64, queryContext: b3SensorQueryContext): bool {
	let shapeId = <i32>userData;

	let sensorShape = queryContext.sensorShape!;
	let sensorShapeId = sensorShape.id;

	if (shapeId == sensorShapeId) {
		return true;
	}

	let world = queryContext.world!;
	let otherShape = world.shapes[shapeId];

	// Visitors must be convex.
	if (b3IsConvex(otherShape.type) == false) {
		return true;
	}

	// Are sensor events enabled on the other shape?
	if ((otherShape.flags & b3_enableSensorEvents) == 0) {
		return true;
	}

	// Skip shapes on the same body
	if (otherShape.bodyId == sensorShape.bodyId) {
		return true;
	}

	// Check filter
	if (b3ShouldShapesCollide(sensorShape.filter, otherShape.filter) == false) {
		return true;
	}

	// Custom user filter
	if ((sensorShape.flags & b3_enableCustomFiltering) != 0 || (otherShape.flags & b3_enableCustomFiltering) != 0) {
		let customFilterFcn = world.customFilterFcn;
		if (customFilterFcn != null) {
			let idA = b3ShapeId();
			idA.index1 = sensorShapeId + 1;
			idA.world0 = world.worldId;
			idA.generation = sensorShape.generation;
			let idB = b3ShapeId();
			idB.index1 = shapeId + 1;
			idB.world0 = world.worldId;
			idB.generation = otherShape.generation;
			let shouldCollide = customFilterFcn(idA, idB);
			if (shouldCollide == false) {
				return true;
			}
		}
	}

	let otherTransform = b3ToRelativeTransform(b3GetBodyTransform(world, otherShape.bodyId), b3Pos_zero);

	let overlap = b3OverlapSensor(sensorShape, queryContext.transform, otherShape, otherTransform);
	if (overlap == false) {
		return true;
	}

	// Record the overlap
	let sensor = queryContext.sensor!;
	let shapeRef = b3Array_Emplace(sensor.overlaps2);
	sensor.overlaps2[shapeRef].shapeId = shapeId;
	sensor.overlaps2[shapeRef].generation = otherShape.generation;

	return true;
}

// b3CompareVisitors never returns 0, so it is the less than below. libc qsort breaks ties its own
// way, which only shows when two visitors share a shape id but not a generation.
class b3VisitorSorter {
	visitors: Array<b3Visitor> = new Array<b3Visitor>();

	@inline less(i: i32, j: i32): bool {
		return this.visitors[i].shapeId < this.visitors[j].shapeId;
	}

	@inline swap(i: i32, j: i32): void {
		let tmp = this.visitors[i];
		this.visitors[i] = this.visitors[j];
		this.visitors[j] = tmp;
	}
}

const s_visitorSorter = new b3VisitorSorter();

function b3SensorTask(startIndex: i32, endIndex: i32, workerIndex: i32, world: b3World): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(workerIndex < world.workerCount);
	let taskContext = world.sensorTaskContexts[workerIndex];

	if (B3_ASSERT_ENABLED) B3_ASSERT(startIndex < endIndex);

	let trees = world.broadPhase.trees;
	for (let sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex) {
		let sensor = world.sensors[sensorIndex];
		let sensorShape = world.shapes[sensor.shapeId];

		// Swap overlap arrays
		let temp = sensor.overlaps1;
		sensor.overlaps1 = sensor.overlaps2;
		sensor.overlaps2 = temp;
		sensor.overlaps2.length = 0;

		// Append sensor hits
		b3Array_Append(sensor.overlaps2, sensor.hits, sensor.hits.length);

		// Clear the hits
		sensor.hits.length = 0;

		let body = world.bodies[sensorShape.bodyId];
		if (body.setIndex == b3_disabledSet || (sensorShape.flags & b3_enableSensorEvents) == 0) {
			if (sensor.overlaps1.length != 0) {
				// This sensor is dropping all overlaps because it has been disabled.
				b3SetBit(taskContext.eventBits, <u32>sensorIndex);
			}
			continue;
		}

		let transform = b3ToRelativeTransform(b3GetBodyTransformQuick(world, sensorShape.bodyId), b3Pos_zero);

		let queryContext = s_sensorQueryContext;
		queryContext.world = world;
		queryContext.taskContext = taskContext;
		queryContext.sensor = sensor;
		queryContext.sensorShape = sensorShape;
		queryContext.transform = transform;

		if (B3_ASSERT_ENABLED) B3_ASSERT(sensorShape.sensorIndex == sensorIndex);
		let queryBounds = sensorShape.aabb;

		let callback = b3SensorTreeCallback;

		// Query all trees
		b3DynamicTree_Query(trees[0], queryBounds, sensorShape.filter.maskBits, false, callback);
		b3DynamicTree_Query(trees[1], queryBounds, sensorShape.filter.maskBits, false, callback);
		b3DynamicTree_Query(trees[2], queryBounds, sensorShape.filter.maskBits, false, callback);

		// Sort the overlaps to enable finding begin and end events.
		if (sensor.overlaps2.length > 1) {
			s_visitorSorter.visitors = sensor.overlaps2;
			Q_LOOP<b3VisitorSorter>(sensor.overlaps2.length, s_visitorSorter);
		}

		// Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
		let uniqueCount = 0;
		let overlapCount = sensor.overlaps2.length;
		let overlapData = sensor.overlaps2;
		for (let i = 0; i < overlapCount; ++i) {
			if (uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId) {
				overlapData[uniqueCount] = overlapData[i];
				uniqueCount += 1;
			}
		}
		sensor.overlaps2.length = uniqueCount;

		let count1 = sensor.overlaps1.length;
		let count2 = sensor.overlaps2.length;
		if (count1 != count2) {
			// something changed
			b3SetBit(taskContext.eventBits, <u32>sensorIndex);
		} else {
			for (let i = 0; i < count1; ++i) {
				let s1 = sensor.overlaps1[i];
				let s2 = sensor.overlaps2[i];

				if (s1.shapeId != s2.shapeId || s1.generation != s2.generation) {
					// something changed
					b3SetBit(taskContext.eventBits, <u32>sensorIndex);
					break;
				}
			}
		}
	}
}

@inline function b3MakeSensorShapeId(index1: i32, world0: u16, generation: u16): b3ShapeId {
	let id = b3ShapeId();
	id.index1 = index1;
	id.world0 = world0;
	id.generation = generation;
	return id;
}

@inline function b3MakeSensorEndEvent(sensorShapeId: b3ShapeId, visitorShapeId: b3ShapeId): b3SensorEndTouchEvent {
	let event = b3SensorEndTouchEvent();
	event.sensorShapeId = sensorShapeId;
	event.visitorShapeId = visitorShapeId;
	return event;
}

@inline function b3MakeSensorBeginEvent(sensorShapeId: b3ShapeId, visitorShapeId: b3ShapeId): b3SensorBeginTouchEvent {
	let event = b3SensorBeginTouchEvent();
	event.sensorShapeId = sensorShapeId;
	event.visitorShapeId = visitorShapeId;
	return event;
}

export function b3OverlapSensors(world: b3World): void {
	let sensorCount = world.sensors.length;
	if (sensorCount == 0) {
		return;
	}

	B3_ASSERT(world.workerCount > 0);

	for (let i = 0; i < world.workerCount; ++i) {
		b3SetBitCountAndClear(world.sensorTaskContexts[i].eventBits, <u32>sensorCount);
	}

	// Parallel-for sensors overlaps
	let minRange = 16;
	b3ParallelFor(
		world,
		(startIndex: i32, endIndex: i32, workerIndex: i32): void => b3SensorTask(startIndex, endIndex, workerIndex, world),
		sensorCount,
		minRange,
		"sensors",
	);

	let bitSet = world.sensorTaskContexts[0].eventBits;
	for (let i = 1; i < world.workerCount; ++i) {
		b3InPlaceUnion(bitSet, world.sensorTaskContexts[i].eventBits);
	}

	// Iterate sensors bits and publish events
	// Process sensor state changes. Iterate over set bits
	let bits = bitSet.bits;
	let blockCount = bitSet.blockCount;

	for (let k: u32 = 0; k < blockCount; ++k) {
		let word = bits[<i32>k];
		while (word != 0) {
			let ctz = b3CTZ64(word);
			let sensorIndex = <i32>(64 * k + ctz);

			let sensor = world.sensors[sensorIndex];
			let sensorShape = world.shapes[sensor.shapeId];
			let sensorId = b3MakeSensorShapeId(sensor.shapeId + 1, world.worldId, sensorShape.generation);

			let count1 = sensor.overlaps1.length;
			let count2 = sensor.overlaps2.length;
			let refs1 = sensor.overlaps1;
			let refs2 = sensor.overlaps2;

			// overlaps1 can have overlaps that end
			// overlaps2 can have overlaps that begin
			let index1 = 0;
			let index2 = 0;
			while (index1 < count1 && index2 < count2) {
				let r1 = refs1[index1];
				let r2 = refs2[index2];
				if (r1.shapeId == r2.shapeId) {
					if (r1.generation < r2.generation) {
						// end
						let visitorId = b3MakeSensorShapeId(r1.shapeId + 1, world.worldId, r1.generation);
						let event = b3MakeSensorEndEvent(sensorId, visitorId);
						world.sensorEndEvents[world.endEventArrayIndex].push(event);
						index1 += 1;
					} else if (r1.generation > r2.generation) {
						// begin
						let visitorId = b3MakeSensorShapeId(r2.shapeId + 1, world.worldId, r2.generation);
						let event = b3MakeSensorBeginEvent(sensorId, visitorId);
						world.sensorBeginEvents.push(event);
						index2 += 1;
					} else {
						// persisted
						index1 += 1;
						index2 += 1;
					}
				} else if (r1.shapeId < r2.shapeId) {
					// end
					let visitorId = b3MakeSensorShapeId(r1.shapeId + 1, world.worldId, r1.generation);
					let event = b3MakeSensorEndEvent(sensorId, visitorId);
					world.sensorEndEvents[world.endEventArrayIndex].push(event);
					index1 += 1;
				} else {
					// begin
					let visitorId = b3MakeSensorShapeId(r2.shapeId + 1, world.worldId, r2.generation);
					let event = b3MakeSensorBeginEvent(sensorId, visitorId);
					world.sensorBeginEvents.push(event);
					index2 += 1;
				}
			}

			while (index1 < count1) {
				// end
				let r1 = refs1[index1];
				let visitorId = b3MakeSensorShapeId(r1.shapeId + 1, world.worldId, r1.generation);
				let event = b3MakeSensorEndEvent(sensorId, visitorId);
				world.sensorEndEvents[world.endEventArrayIndex].push(event);
				index1 += 1;
			}

			while (index2 < count2) {
				// begin
				let r2 = refs2[index2];
				let visitorId = b3MakeSensorShapeId(r2.shapeId + 1, world.worldId, r2.generation);
				let event = b3MakeSensorBeginEvent(sensorId, visitorId);
				world.sensorBeginEvents.push(event);
				index2 += 1;
			}

			// Clear the smallest set bit
			word = word & (word - 1);
		}
	}
}

export function b3DestroySensor(world: b3World, sensorShape: b3Shape): void {
	let sensor = world.sensors[sensorShape.sensorIndex];
	for (let i = 0; i < sensor.overlaps2.length; ++i) {
		let ref_ = sensor.overlaps2[i];
		let event = b3MakeSensorEndEvent(
			b3MakeSensorShapeId(sensorShape.id + 1, world.worldId, sensorShape.generation),
			b3MakeSensorShapeId(ref_.shapeId + 1, world.worldId, ref_.generation),
		);

		world.sensorEndEvents[world.endEventArrayIndex].push(event);
	}

	// Destroy sensor
	sensor.hits.length = 0;
	sensor.overlaps1.length = 0;
	sensor.overlaps2.length = 0;

	let movedIndex = b3Array_RemoveSwap(world.sensors, sensorShape.sensorIndex);
	if (movedIndex != B3_NULL_INDEX) {
		// Fixup moved sensor
		let movedSensor = world.sensors[sensorShape.sensorIndex];
		let otherSensorShape = world.shapes[movedSensor.shapeId];
		otherSensorShape.sensorIndex = sensorShape.sensorIndex;
	}
}
