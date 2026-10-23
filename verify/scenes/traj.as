// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Trajectory driver, the twin of verify/scenes/traj.c. create() builds a scene, step() runs one
// Scene_Step and writes that step's record (step index, body count, counters, bodies, mover) into
// a word buffer the host reads through recordPointer() and recordLength() before the next frame
// boundary. The host writes the header.

import { Scene, SceneMover, Scene_Create, Scene_GetCount, Scene_GetName, Scene_Step, SCENE_MOVER_PLANE_CAPACITY } from "./scenes";
import {
	B3_CONTACT_MANIFOLD_COUNT_BUCKETS,
	b3BodyId,
	b3Body_GetAngularVelocity,
	b3Body_GetLinearVelocity,
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_GetType,
	b3Body_IsAwake,
	b3Body_IsValid,
	b3WorldId,
	b3World_GetAwakeBodyCount,
	b3World_GetBodyEvents,
	b3World_GetContactEvents,
	b3World_GetCounters,
} from "../../lib/box3d";

export { frameBoundary } from "../as/common";

const TRAJ_BODY_WORDS = 17;
const TRAJ_COUNTER_COUNT = 56;
const TRAJ_PLANE_WORDS = 12;
const TRAJ_MOVER_WORDS = 16 + TRAJ_PLANE_WORDS * SCENE_MOVER_PLANE_CAPACITY;

let g_scene = new Scene();
let g_words = new Uint32Array(1024);
let g_count = 0;
let g_error = 0;

function PushWord(value: u32): void {
	if (g_count == g_words.length) {
		let grown = new Uint32Array(2 * g_words.length);
		for (let i = 0; i < g_count; ++i) {
			grown[i] = g_words[i];
		}
		g_words = grown;
	}
	g_words[g_count] = value;
	g_count += 1;
}

function PushFloat(value: f32): void {
	PushWord(reinterpret<u32>(value));
}

function PushInt(value: i32): void {
	PushWord(<u32>value);
}

function WriteCounters(worldId: b3WorldId): void {
	let c = b3World_GetCounters(worldId);
	let contactEvents = b3World_GetContactEvents(worldId);
	let bodyEvents = b3World_GetBodyEvents(worldId);
	let start = g_count;

	PushInt(c.bodyCount);
	PushInt(c.shapeCount);
	PushInt(c.contactCount);
	PushInt(c.jointCount);
	PushInt(c.islandCount);
	PushInt(b3World_GetAwakeBodyCount(worldId));
	PushInt(c.awakeContactCount);
	PushInt(c.recycledContactCount);
	PushInt(c.staticTreeHeight);
	PushInt(c.treeHeight);
	PushInt(c.satCallCount);
	PushInt(c.satCacheHitCount);
	PushInt(c.distanceIterations);
	PushInt(c.pushBackIterations);
	PushInt(c.rootIterations);
	PushInt(contactEvents.beginCount);
	PushInt(contactEvents.endCount);
	PushInt(contactEvents.hitCount);
	PushInt(bodyEvents.moveCount);
	for (let i = 0; i < 24; ++i) {
		PushInt(c.colorCounts[i]);
	}
	for (let i = 0; i < B3_CONTACT_MANIFOLD_COUNT_BUCKETS; ++i) {
		PushInt(c.manifoldCounts[i]);
	}

	// Build and worker dependent
	PushInt(c.taskCount);
	PushInt(c.stackUsed);
	PushInt(c.arenaCapacity);
	PushWord(<u32>(<u64>c.byteCount & 0xffffffff));
	PushWord(<u32>(<u64>c.byteCount >> 32));

	if (g_count - start != TRAJ_COUNTER_COUNT) {
		g_error = 1;
	}
}

function WriteBody(index: i32, bodyId: b3BodyId): void {
	PushInt(index);

	if (b3Body_IsValid(bodyId) == false) {
		for (let i = 1; i < TRAJ_BODY_WORDS; ++i) {
			PushWord(0);
		}
		return;
	}

	let flags: u32 = 1;
	if (b3Body_IsAwake(bodyId)) {
		flags |= 2;
	}
	flags |= <u32>b3Body_GetType(bodyId) << 4;
	PushWord(flags);
	PushInt(bodyId.index1);
	PushWord(<u32>bodyId.generation);

	let p = b3Body_GetPosition(bodyId);
	let q = b3Body_GetRotation(bodyId);
	let v = b3Body_GetLinearVelocity(bodyId);
	let omega = b3Body_GetAngularVelocity(bodyId);

	PushFloat(p.x);
	PushFloat(p.y);
	PushFloat(p.z);
	PushFloat(q.v.x);
	PushFloat(q.v.y);
	PushFloat(q.v.z);
	PushFloat(q.s);
	PushFloat(v.x);
	PushFloat(v.y);
	PushFloat(v.z);
	PushFloat(omega.x);
	PushFloat(omega.y);
	PushFloat(omega.z);
}

function WriteMover(m: SceneMover): void {
	let start = g_count;
	PushFloat(m.transform.p.x);
	PushFloat(m.transform.p.y);
	PushFloat(m.transform.p.z);
	PushFloat(m.velocity.x);
	PushFloat(m.velocity.y);
	PushFloat(m.velocity.z);
	PushFloat(m.pogoVelocity);
	PushWord(m.onGround ? 1 : 0);
	PushWord(m.sprint ? 1 : 0);
	PushInt(m.planeCount);
	PushInt(m.totalIterations);
	PushInt(m.moveIterations);
	PushWord(m.rayHit ? 1 : 0);
	PushFloat(m.rayFraction);
	PushFloat(m.castFraction);
	PushWord(0);

	for (let i = 0; i < SCENE_MOVER_PLANE_CAPACITY; ++i) {
		if (i >= m.planeCount) {
			for (let k = 0; k < TRAJ_PLANE_WORDS; ++k) {
				PushWord(0);
			}
			continue;
		}

		let plane = m.planes[i];
		PushFloat(plane.plane.normal.x);
		PushFloat(plane.plane.normal.y);
		PushFloat(plane.plane.normal.z);
		PushFloat(plane.plane.offset);
		PushFloat(plane.pushLimit);
		PushFloat(plane.push);
		PushWord(plane.clipVelocity ? 1 : 0);
		PushFloat(m.planePoints[i].x);
		PushFloat(m.planePoints[i].y);
		PushFloat(m.planePoints[i].z);
		PushInt(m.planeShapes[i].index1);
		PushWord(<u32>m.planeShapes[i].generation);
	}

	if (g_count - start != TRAJ_MOVER_WORDS) {
		g_error = 2;
	}
}

export function sceneCount(): i32 {
	return Scene_GetCount();
}

// Returns false for an unknown scene index
export function create(sceneIndex: i32, workerCount: i32): bool {
	if (sceneIndex < 0 || sceneIndex >= Scene_GetCount()) {
		return false;
	}
	g_scene = new Scene();
	return Scene_Create(g_scene, Scene_GetName(sceneIndex), workerCount);
}

export function moverWords(): i32 {
	return g_scene.hasMover ? TRAJ_MOVER_WORDS : 0;
}

export function counterCount(): i32 {
	return TRAJ_COUNTER_COUNT;
}

export function bodyCount(): i32 {
	return g_scene.bodyCount;
}

export function step(stepIndex: i32): void {
	Scene_Step(g_scene, stepIndex);

	g_count = 0;
	PushInt(stepIndex);
	PushInt(g_scene.bodyCount);
	WriteCounters(g_scene.worldId);
	for (let i = 0; i < g_scene.bodyCount; ++i) {
		WriteBody(i, g_scene.bodies[i]);
	}
	if (g_scene.hasMover) {
		WriteMover(g_scene.mover);
	}
}

export function recordError(): i32 {
	return g_error;
}

export function recordLength(): i32 {
	return g_count;
}

// Valid until the next frame boundary, which may move the buffer
export unsafe function recordPointer(): usize {
	return g_words.dataStart;
}
