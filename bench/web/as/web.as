// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The port on wasm, driven from JS: the same exports as bench/web/c/driver.c.

import { Scene, Scene_Create, Scene_Destroy, Scene_GetCount, Scene_GetName, Scene_Step } from "../../../verify/scenes/scenes";
import {
	b3Body_GetAngularVelocity,
	b3Body_GetLinearVelocity,
	b3Body_GetPosition,
	b3Body_GetRotation,
	b3Body_IsValid,
	b3World_GetProfile,
} from "../../../lib/box3d";

export { frameBoundary } from "../../../verify/as/common";

let g_scene = new Scene();
let g_live = false;

export function sceneCount(): i32 {
	return Scene_GetCount();
}

export function create(sceneIndex: i32, workerCount: i32): i32 {
	if (g_live) {
		Scene_Destroy(g_scene);
		g_live = false;
	}
	if (sceneIndex < 0 || sceneIndex >= Scene_GetCount()) {
		return 0;
	}
	g_scene = new Scene();
	g_live = Scene_Create(g_scene, Scene_GetName(sceneIndex), workerCount);
	return g_live ? 1 : 0;
}

export function step(stepIndex: i32): void {
	Scene_Step(g_scene, stepIndex);
}

function hashWord(h: u32, bits: u32): u32 {
	for (let s = 0; s < 4; ++s) {
		h = (h ^ ((bits >> (8 * <u32>s)) & 0xff)) * 16777619;
	}
	return h;
}

// FNV-1a over each body's pose and velocities as verify/scenes/hash.mjs reads a recording; a destroyed
// body hashes the recording's zero words
export function stateHash(): u32 {
	let h: u32 = 2166136261;
	for (let i = 0; i < g_scene.bodyCount; ++i) {
		let id = g_scene.bodies[i];
		if (!b3Body_IsValid(id)) {
			for (let k = 0; k < 13; ++k) {
				h = hashWord(h, 0);
			}
			continue;
		}
		let p = b3Body_GetPosition(id);
		let q = b3Body_GetRotation(id);
		let v = b3Body_GetLinearVelocity(id);
		let w = b3Body_GetAngularVelocity(id);
		let words: StaticArray<f32> = [p.x, p.y, p.z, q.v.x, q.v.y, q.v.z, q.s, v.x, v.y, v.z, w.x, w.y, w.z];
		for (let k = 0; k < words.length; ++k) {
			h = hashWord(h, reinterpret<u32>(words[k]));
		}
	}
	return h;
}

// The b3Profile fields in the order of bench/native/bench.c
export function profileField(k: i32): f32 {
	let p = b3World_GetProfile(g_scene.worldId);
	let fields: StaticArray<f32> = [
		p.step, p.pairs, p.collide, p.solve, p.solverSetup, p.constraints, p.prepareConstraints,
		p.integrateVelocities, p.warmStart, p.solveImpulses, p.integratePositions, p.relaxImpulses,
		p.restitution, p.storeImpulses, p.splitIslands, p.transforms, p.sensorHits, p.jointEvents,
		p.hitEvents, p.refit, p.bullets, p.sleepIslands, p.sensors,
	];
	return 0 <= k && k < fields.length ? fields[k] : 0;
}

export function queryMs(k: i32): f64 {
	return g_scene.queryMs[k];
}

export function queryHash(k: i32): u32 {
	return g_scene.queryHash[k];
}

export function bodyCount(): i32 {
	return g_scene.bodyCount;
}
