// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// The broad-phase tree rebuild on a job, through Box3D's own task callbacks: b3EnqueueTreeUpdate's closure only
// carries the world, so the enqueue callback can recognize it by name and send the two trees it rebuilds, and the
// finish callback copies them back. It runs beside the narrow phase and the solver, as in C.

import { Blob, SharedRegion } from "rive/shared";

import { b3DynamicTree, b3TreeNode, b3TreeProxy } from "./collision";
import { b3DynamicTree_Rebuild } from "./dynamic_tree";
import { b3_dynamicBody, b3_kinematicBody } from "./types";
import { b3World } from "./physics_world";
import { b3GrowU8, b3LaneNow, b3LaneTraceOpen, b3LaneTraceSend, b3LaneWriter, b3ReadI32 } from "./lane_common";
import { b3SpanBegin, b3SpanEnd, b3_traceTree } from "./trace";

// Scalars of a tree after its three arrays: nodeEnd, nodeCapacity, pairFreeList, proxyCount, proxyCapacity,
// proxyFreeList, rebuildCapacity, dfsOrdered
const TREE_WORDS: i32 = 8;

unsafe function b3WriteTree(w: b3LaneWriter, tree: b3DynamicTree): void {
	w.i32(tree.nodes.length);
	w.bytes(changetype<usize>(tree.nodes), tree.nodes.length * <i32>sizeof<b3TreeNode>());
	w.i32(tree.parents.length);
	w.bytes(changetype<usize>(tree.parents), tree.parents.length * 4);
	w.i32(tree.proxies.length);
	w.bytes(changetype<usize>(tree.proxies), tree.proxies.length * <i32>sizeof<b3TreeProxy>());
	w.i32(tree.nodeEnd);
	w.i32(tree.nodeCapacity);
	w.i32(tree.pairFreeList);
	w.i32(tree.proxyCount);
	w.i32(tree.proxyCapacity);
	w.i32(tree.proxyFreeList);
	w.i32(tree.rebuildCapacity);
	w.i32(tree.dfsOrdered ? 1 : 0);
}

// Reads a tree written by b3WriteTree into tree, reusing its arrays where they are long enough
unsafe function b3ReadTree(base: usize, at: i32, tree: b3DynamicTree): i32 {
	let nodeCount = b3ReadI32(base, at);
	if (tree.nodes.length != nodeCount) {
		tree.nodes = new StaticArray<b3TreeNode>(nodeCount);
	}
	let bytes = nodeCount * <i32>sizeof<b3TreeNode>();
	memory.copy(changetype<usize>(tree.nodes), base + <usize>(at + 4), <usize>bytes);
	at += 4 + bytes;
	let parentCount = b3ReadI32(base, at);
	if (tree.parents.length != parentCount) {
		tree.parents = new StaticArray<i32>(parentCount);
	}
	memory.copy(changetype<usize>(tree.parents), base + <usize>(at + 4), <usize>(4 * parentCount));
	at += 4 + 4 * parentCount;
	let proxyCount = b3ReadI32(base, at);
	if (tree.proxies.length != proxyCount) {
		tree.proxies = new StaticArray<b3TreeProxy>(proxyCount);
	}
	bytes = proxyCount * <i32>sizeof<b3TreeProxy>();
	memory.copy(changetype<usize>(tree.proxies), base + <usize>(at + 4), <usize>bytes);
	at += 4 + bytes;
	tree.nodeEnd = b3ReadI32(base, at);
	tree.nodeCapacity = b3ReadI32(base, at + 4);
	tree.pairFreeList = b3ReadI32(base, at + 8);
	tree.proxyCount = b3ReadI32(base, at + 12);
	tree.proxyCapacity = b3ReadI32(base, at + 16);
	tree.proxyFreeList = b3ReadI32(base, at + 20);
	tree.rebuildCapacity = b3ReadI32(base, at + 24);
	tree.dfsOrdered = b3ReadI32(base, at + 28) != 0;
	return at + 4 * TREE_WORDS;
}

// The rebuild scratch is sized by rebuildCapacity, which the lane's fresh tree does not have
function b3ClearScratch(tree: b3DynamicTree): void {
	tree.rebuildCapacity = 0;
}

@job export function b3LaneTreeJob(input: Blob, traceRegion: SharedRegion): Blob {
	let trace = b3LaneTraceOpen(traceRegion);
	let traceBegin = b3SpanBegin(trace);
	let bytes = new Uint8Array(<i32>input.byteLength);
	input.readInto(0, bytes);
	let base = bytes.dataStart;
	let dynamicTree = new b3DynamicTree();
	let kinematicTree = new b3DynamicTree();
	let at = b3ReadTree(base, 0, dynamicTree);
	b3ReadTree(base, at, kinematicTree);
	b3ClearScratch(dynamicTree);
	b3ClearScratch(kinematicTree);
	b3DynamicTree_Rebuild(dynamicTree, false);
	b3DynamicTree_Rebuild(kinematicTree, false);
	let w = new b3LaneWriter();
	b3WriteTree(w, dynamicTree);
	b3WriteTree(w, kinematicTree);
	let output = Blob.fromArray(w.buffer, 0, w.length);
	b3SpanEnd(trace, b3_traceTree, traceBegin);
	b3LaneTraceSend(traceRegion, trace);
	return output;
}

export class b3LaneTreeMain {
	writer: b3LaneWriter = new b3LaneWriter();
	staging: Uint8Array = new Uint8Array(0);
	pack: f64 = 0;
	apply: f64 = 0;
	wait: f64 = 0;
	tasks: i32 = 0;
	bytes: f64 = 0;
}

export unsafe function b3LaneTreePack(m: b3LaneTreeMain, world: b3World): Blob {
	let ticks = b3LaneNow();
	let w = m.writer;
	w.reset();
	let trees = world.broadPhase.trees;
	b3WriteTree(w, trees[b3_dynamicBody]);
	b3WriteTree(w, trees[b3_kinematicBody]);
	let blob = Blob.fromArray(w.buffer, 0, w.length);
	m.bytes += <f64>w.length;
	m.pack += b3LaneNow() - ticks;
	return blob;
}

// The rebuilt trees replace the world's. Their rebuild scratch stays the world's own, which a rebuild fully writes
// before it reads, so the capacity it was sized for stays too.
export unsafe function b3LaneTreeApply(m: b3LaneTreeMain, world: b3World, output: Blob): void {
	let ticks = b3LaneNow();
	let length = <i32>output.byteLength;
	m.staging = b3GrowU8(m.staging, length);
	let staging = m.staging;
	output.readInto(0, staging, 0, length);
	m.bytes += <f64>length;
	let trees = world.broadPhase.trees;
	let base = staging.dataStart;
	let at = 0;
	for (let k = 0; k < 2; ++k) {
		let tree = trees[k == 0 ? b3_dynamicBody : b3_kinematicBody];
		let rebuildCapacity = tree.rebuildCapacity;
		at = b3ReadTree(base, at, tree);
		tree.rebuildCapacity = rebuildCapacity;
	}
	m.apply += b3LaneNow() - ticks;
}

