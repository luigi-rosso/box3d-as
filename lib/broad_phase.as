// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/broad_phase.c

import { b3AABB } from "./math_functions";
import { b3DynamicTree } from "./collision";
import { b3DynamicTree_MarkProxyMoved, b3DynamicTree_MarkProxyMovedSerial } from "./dynamic_tree";
import { b3HashSet } from "./table";
import { b3BodyType, b3_bodyTypeCount } from "./types";

// Store the proxy type in the lower 2 bits of the proxy key. This leaves 30 bits for the id.
@inline export function B3_PROXY_TYPE(key: i32): b3BodyType {
	return key & 3;
}

@inline export function B3_PROXY_ID(key: i32): i32 {
	return key >> 2;
}

@inline export function B3_PROXY_KEY(id: i32, type: b3BodyType): i32 {
	return (id << 2) | type;
}

const b3_emptySiblings = new StaticArray<i32>(0);

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
export class b3BroadPhase {
	// One tree for each body type.
	trees: StaticArray<b3DynamicTree> = new StaticArray<b3DynamicTree>(b3_bodyTypeCount);

	// The moved siblings gathered from the dynamic body tree.
	movedSiblings: StaticArray<i32> = b3_emptySiblings;

	// Tracks shape pairs that have a b3Contact
	// todo pairSet can grow quite large on the first time step and remain large
	pairSet: b3HashSet = new b3HashSet();

	constructor() {
		for (let i = 0; i < b3_bodyTypeCount; ++i) {
			this.trees[i] = new b3DynamicTree();
		}
	}
}

@inline export function b3BroadPhase_MarkProxyMovedSerial(bp: b3BroadPhase, proxyKey: i32): void {
	let proxyType = B3_PROXY_TYPE(proxyKey);
	let proxyId = B3_PROXY_ID(proxyKey);
	b3DynamicTree_MarkProxyMovedSerial(bp.trees[proxyType], proxyId);
}

@inline export function b3BroadPhase_MarkProxyMoved(bp: b3BroadPhase, proxyKey: i32, aabb: b3AABB): void {
	let proxyType = B3_PROXY_TYPE(proxyKey);
	let proxyId = B3_PROXY_ID(proxyKey);
	b3DynamicTree_MarkProxyMoved(bp.trees[proxyType], proxyId, aabb);
}

// functions from broad_phase.c follow
