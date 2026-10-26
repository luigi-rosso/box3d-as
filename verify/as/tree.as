// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/dynamic_tree.as, mirrors verify/cref/tree.c

import { vtag, vi32, vu32, vu64, vbool, vf32, vend, vseed, vrand64, vrandint, vrandf } from "./common";
import { FLT_MAX, b3OnMain } from "../../lib/core";
import { b3AABB, b3AABB_Center, b3Add, b3DistanceSquared, b3MulSV, b3Sub, b3Vec3, b3Vec3_zero } from "../../lib/math_functions";
import {
	b3BoxCastInput,
	b3DynamicTree,
	b3DynamicTree_GetAABB,
	b3DynamicTree_GetUserData,
	b3RayCastInput,
	b3TreeNode,
	b3TreeStats,
} from "../../lib/collision";
import { B3_DEFAULT_CATEGORY_BITS, B3_DEFAULT_MASK_BITS } from "../../lib/types";
import { b3RayCastAABB } from "../../lib/aabb";
import {
	B3_LEAF_NODE,
	b3CreateTreeProxyInternal,
	b3DynamicTree_BoxCast,
	b3DynamicTree_ClearMoved,
	b3DynamicTree_Create,
	b3DynamicTree_CreateProxy,
	b3DynamicTree_Destroy,
	b3DynamicTree_DestroyProxy,
	b3DynamicTree_EnlargeProxy,
	b3DynamicTree_GatherMovedProxies,
	b3DynamicTree_GetAreaRatio,
	b3DynamicTree_GetByteCount,
	b3DynamicTree_GetCategoryBits,
	b3DynamicTree_GetHeight,
	b3DynamicTree_GetProxyCount,
	b3DynamicTree_GetRootBounds,
	b3DynamicTree_MarkProxyMoved,
	b3DynamicTree_MarkProxyMovedSerial,
	b3DynamicTree_MoveProxy,
	b3DynamicTree_MoveProxyInternal,
	b3DynamicTree_Query,
	b3DynamicTree_QueryClosest,
	b3DynamicTree_RayCast,
	b3DynamicTree_Rebuild,
	b3DynamicTree_Refit,
	b3DynamicTree_SetCategoryBits,
	b3DynamicTree_Validate,
	b3HasTreeMoved,
	b3NeedsRebuild,
	b3TreeView,
	b3TreeView_BoxCast,
	b3TreeView_Query,
	b3TreeView_QueryClosest,
	b3TreeView_RayCast,
} from "../../lib/dynamic_tree";

export { frameBoundary } from "./common";

const MAX_LIVE = 4096;
const PHASE_COUNT = 10;

let tree = new b3DynamicTree();
let treeAlive = false;
const live = new StaticArray<i32>(MAX_LIVE);
let liveCount = 0;
let lastLeafCount = 0;

let boxMode = 0;
let worldSize: f32 = 0;
let boxSize: f32 = 0;
let createWeight = 0;
let destroyWeight = 0;
let opsLo = 0;
let opsHi = 0;
let phaseIndex = 0;

// FNV-1a over the tree state
let h_state: u32 = 0;

@inline function hmix(x: u32): void {
	h_state ^= x;
	h_state *= 16777619;
}

@inline function hmixf(x: f32): void {
	hmix(reinterpret<u32>(x));
}

function hmix64(x: u64): void {
	hmix(<u32>x);
	hmix(<u32>(x >> 32));
}

function hmixAABB(a: b3AABB): void {
	hmixf(a.lowerBound.x);
	hmixf(a.lowerBound.y);
	hmixf(a.lowerBound.z);
	hmixf(a.upperBound.x);
	hmixf(a.upperBound.y);
	hmixf(a.upperBound.z);
}

// C's union: a leaf reads the shape index, an internal node its height
@inline function nodeUnion(n: b3TreeNode): i32 {
	return (n.flagIndex & B3_LEAF_NODE) == B3_LEAF_NODE ? n.shapeIndex : n.height;
}

// Counts nodes whose unused union member is nonzero, which C cannot represent
let aliasCount = 0;

function checkAlias(n: b3TreeNode): void {
	let leaf = (n.flagIndex & B3_LEAF_NODE) == B3_LEAF_NODE;
	if ((leaf && n.height != 0) || (leaf == false && n.shapeIndex != 0)) {
		aliasCount += 1;
	}
}

function treeHash(): u32 {
	h_state = 2166136261;
	hmix(<u32>tree.nodeEnd);
	hmix(<u32>tree.nodeCapacity);
	hmix(<u32>tree.pairFreeList);
	hmix(<u32>tree.proxyCount);
	hmix(<u32>tree.proxyCapacity);
	hmix(<u32>tree.proxyFreeList);
	hmix(tree.dfsOrdered ? 1 : 0);
	let nodes = tree.nodes;
	let parents = tree.parents;
	for (let i = 0; i < tree.nodeEnd; ++i) {
		ref n = nodes[i];
		hmixAABB(n.aabb);
		hmix(n.flagIndex);
		hmix(<u32>nodeUnion(n));
		hmix(<u32>parents[i]);
	}
	let proxies = tree.proxies;
	for (let i = 0; i < tree.proxyCapacity; ++i) {
		ref p = proxies[i];
		hmix64(p.userData);
		hmix64(p.categoryBits);
		hmix(<u32>p.node);
		hmix(<u32>p.next);
	}
	return h_state;
}

function vaabb(a: b3AABB): void {
	vf32(a.lowerBound.x);
	vf32(a.lowerBound.y);
	vf32(a.lowerBound.z);
	vf32(a.upperBound.x);
	vf32(a.upperBound.y);
	vf32(a.upperBound.z);
}

function vvec(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

function vstats(tag: string, s: b3TreeStats): void {
	vtag(tag);
	vi32(s.nodeVisits);
	vi32(s.leafVisits);
	vend();
}

function vnode(n: b3TreeNode): void {
	vaabb(n.aabb);
	vu32(n.flagIndex);
	vi32(nodeUnion(n));
	checkAlias(n);
}

function emitHeader(tag: string): void {
	vtag("tree");
	vtag(tag);
	vu64(tree.version);
	vi32(tree.nodeEnd);
	vi32(tree.nodeCapacity);
	vi32(tree.pairFreeList);
	vi32(tree.proxyCount);
	vi32(tree.proxyCapacity);
	vi32(tree.proxyFreeList);
	vi32(tree.rebuildCapacity);
	vbool(tree.dfsOrdered);
	vbool(tree.swapNodes.length != 0);
	vend();
}

function emitStats(): void {
	vtag("stats");
	vi32(b3DynamicTree_GetHeight(tree));
	vf32(b3DynamicTree_GetAreaRatio(tree));
	vaabb(b3DynamicTree_GetRootBounds(tree));
	vi32(b3DynamicTree_GetProxyCount(tree));
	vi32(b3DynamicTree_GetByteCount(tree));
	vbool(b3HasTreeMoved(tree));
	vbool(b3NeedsRebuild(tree));
	vend();
}

// Every slot of every array, the stale tails included
function emitTree(tag: string): void {
	emitHeader(tag);
	emitStats();

	aliasCount = 0;
	for (let i = 0; i < tree.nodeCapacity; ++i) {
		vtag("n");
		vi32(i);
		vnode(tree.nodes[i]);
		vi32(tree.parents[i]);
		vend();
	}

	if (tree.swapNodes.length != 0) {
		for (let i = 0; i < tree.nodeCapacity; ++i) {
			vtag("s");
			vi32(i);
			vnode(tree.swapNodes[i]);
			vend();
		}
	}

	for (let i = 0; i < tree.proxyCapacity; ++i) {
		ref p = tree.proxies[i];
		vtag("p");
		vi32(i);
		vu64(p.userData);
		vu64(p.categoryBits);
		vi32(p.node);
		vi32(p.next);
		vend();
	}

	for (let i = 0; i < lastLeafCount; ++i) {
		vtag("l");
		vi32(i);
		vi32(tree.leafIndices[i]);
		vnode(tree.leafNodes[i]);
		vvec(tree.leafCenters[i]);
		vend();
	}

	vtag("alias");
	vi32(aliasCount);
	vend();
}

function emitOp(tag: string, a: i32, b: i32): void {
	vtag(tag);
	vi32(a);
	vi32(b);
	vu32(treeHash());
	vend();
}

function makeBox(c: b3Vec3, e: b3Vec3): b3AABB {
	return b3AABB(b3Sub(c, e), b3Add(c, e));
}

function randomBox(): b3AABB {
	let c = b3Vec3();
	let e = b3Vec3();
	let zero: f32 = 0.0;
	let mode = boxMode;
	if (mode == 5 && vrandint(0, 8) == 0) {
		mode = 0;
	}

	if (mode == 0) {
		c.x = vrandf(-worldSize, worldSize);
		c.y = vrandf(-worldSize, worldSize);
		c.z = vrandf(-worldSize, worldSize);
		e.x = vrandf(0.05, boxSize);
		e.y = vrandf(0.05, boxSize);
		e.z = vrandf(0.05, boxSize);
	} else if (mode == 1) {
		c.x = <f32>vrandint(-6, 7);
		c.y = <f32>vrandint(-6, 7);
		c.z = <f32>vrandint(-6, 7);
		e.x = 0.5 * <f32>vrandint(1, 3);
		e.y = 0.5 * <f32>vrandint(1, 3);
		e.z = 0.5 * <f32>vrandint(1, 3);
	} else if (mode == 2) {
		c.x = vrandf(-worldSize, worldSize);
		c.y = vrandf(-worldSize, worldSize);
		c.z = vrandf(-worldSize, worldSize);
		e.x = vrandint(0, 2) == 0 ? zero : vrandf(0.0, boxSize);
		e.y = vrandint(0, 2) == 0 ? zero : vrandf(0.0, boxSize);
		e.z = vrandint(0, 2) == 0 ? zero : vrandf(0.0, boxSize);
	} else if (mode == 3) {
		c.x = vrandf(-worldSize, worldSize);
		c.y = 0.0;
		c.z = 0.0;
		e.x = vrandf(0.05, boxSize);
		e.y = vrandf(0.05, boxSize);
		e.z = vrandf(0.05, boxSize);
	} else if (mode == 4) {
		c.x = 100000.0 + vrandf(-worldSize, worldSize);
		c.y = -50000.0 + vrandf(-worldSize, worldSize);
		c.z = vrandf(-worldSize, worldSize);
		e.x = vrandf(0.01, boxSize);
		e.y = vrandf(0.01, boxSize);
		e.z = vrandf(0.01, boxSize);
	} else {
		c = b3Vec3_zero;
		e = b3Vec3(1.0, 1.0, 1.0);
	}

	return makeBox(c, e);
}

let aimTarget = b3Vec3();

function aimAtProxy(): bool {
	if (liveCount == 0 || vrandint(0, 2) == 0) {
		return false;
	}

	aimTarget = b3AABB_Center(b3DynamicTree_GetAABB(tree, live[vrandint(0, liveCount)]));
	return true;
}

function randomQueryBox(): b3AABB {
	let c = b3Vec3();
	let e = b3Vec3();
	if (aimAtProxy()) {
		c = aimTarget;
	} else {
		c.x = vrandf(-worldSize, worldSize);
		c.y = vrandf(-worldSize, worldSize);
		c.z = vrandf(-worldSize, worldSize);
		if (boxMode == 4) {
			c.x += 100000.0;
			c.y -= 50000.0;
		}
	}
	let s = vrandf(0.0, 0.5 * worldSize);
	e.x = vrandf(0.0, s);
	e.y = vrandf(0.0, s);
	e.z = vrandf(0.0, s);
	return makeBox(c, e);
}

function randomPoint(): b3Vec3 {
	let p = b3Vec3();
	p.x = vrandf(-1.2 * worldSize, 1.2 * worldSize);
	p.y = vrandf(-1.2 * worldSize, 1.2 * worldSize);
	p.z = vrandf(-1.2 * worldSize, 1.2 * worldSize);
	if (boxMode == 4) {
		p.x += 100000.0;
		p.y -= 50000.0;
	}
	return p;
}

function randomBits(): u64 {
	let pick = vrandint(0, 4);
	if (pick == 0) {
		return B3_DEFAULT_CATEGORY_BITS;
	}
	if (pick == 1) {
		let shift = vrandint(0, 64);
		return (<u64>1) << <u64>shift;
	}
	if (pick == 2) {
		return vrand64();
	}
	return 3;
}

function enlarged(a: b3AABB): b3AABB {
	let zero: f32 = 0.0;
	let g: f32 = 0.25 * boxSize;
	let d0 = vrandf(0.0, g);
	let d1 = vrandf(0.0, g);
	let d2 = vrandf(0.0, g);
	let d3 = vrandf(0.0, g);
	let d4 = vrandf(0.0, g);
	let d5 = vrandf(0.0, g);
	let skip = vrandint(0, 6);
	a.lowerBound.x -= skip == 0 ? zero : d0;
	a.lowerBound.y -= skip == 1 ? zero : d1;
	a.lowerBound.z -= skip == 2 ? zero : d2;
	a.upperBound.x += skip == 3 ? zero : d3 + <f32>0.001;
	a.upperBound.y += skip == 4 ? zero : d4;
	a.upperBound.z += skip == 5 ? zero : d5;
	return a;
}

// Query callbacks. All context is global so both sides read it the same way.
let q_calls = 0;
let q_stopAfter = 0;
let q_nest = false;
let q_depth = 0;
let q_point = b3Vec3();
let q_policy = 0;

function nestedQueryCallback(proxyId: i32, userData: u64): bool {
	vtag("nq");
	vi32(proxyId);
	vu64(userData);
	vend();
	return true;
}

function nestedQuery(): void {
	q_depth += 1;
	let box = randomQueryBox();
	let s = b3DynamicTree_Query(tree, box, B3_DEFAULT_MASK_BITS, false, nestedQueryCallback);
	vstats("nqs", s);
	q_depth -= 1;
}

function queryCallback(proxyId: i32, userData: u64): bool {
	vtag("q");
	vi32(proxyId);
	vu64(userData);
	vend();
	q_calls += 1;
	if (q_nest && q_depth == 0 && q_calls % 3 == 1) {
		nestedQuery();
	}
	return q_stopAfter < 0 || q_calls < q_stopAfter;
}

function nestedClosestCallback(minSqr: f32, proxyId: i32, userData: u64): f32 {
	vtag("nk");
	vf32(minSqr);
	vi32(proxyId);
	vu64(userData);
	vend();
	return FLT_MAX;
}

function closestCallback(minSqr: f32, proxyId: i32, userData: u64): f32 {
	let a = b3DynamicTree_GetAABB(tree, proxyId);
	let d = b3DistanceSquared(q_point, b3AABB_Center(a));
	let pick = vrandint(0, 8);
	let value = pick == 0 ? FLT_MAX : d;
	vtag("k");
	vf32(minSqr);
	vi32(proxyId);
	vu64(userData);
	vf32(value);
	vend();
	q_calls += 1;
	if (q_nest && q_depth == 0 && q_calls % 4 == 1) {
		q_depth += 1;
		let p = randomPoint();
		let m: FixedArray<f32, 1>;
		m[0] = 0.25 * worldSize * worldSize;
		let s = b3DynamicTree_QueryClosest(tree, p, B3_DEFAULT_MASK_BITS, false, nestedClosestCallback, m[0]);
		vtag("nks");
		vf32(m[0]);
		vend();
		vstats("nkst", s);
		q_depth -= 1;
	}
	return value;
}

function castValue(maxFraction: f32, hit: bool, fraction: f32): f32 {
	if (q_policy == 1) {
		return hit ? fraction : -1.0;
	}

	if (q_policy == 2) {
		return maxFraction;
	}

	let pick = vrandint(0, 16);
	if (pick == 0) {
		return 0.0;
	}
	if (pick < 4) {
		return -1.0;
	}
	if (pick < 8) {
		return maxFraction;
	}
	let t = vrandf(0.0, 1.0);
	return maxFraction * t;
}

function rayCallback(input: b3RayCastInput, proxyId: i32, userData: u64): f32 {
	let fraction: f32 = 0.0;
	let hit = false;
	if (q_policy == 1) {
		let a = b3DynamicTree_GetAABB(tree, proxyId);
		let f: FixedArray<f32, 2>;
		hit = b3RayCastAABB(a, input.origin, b3Add(input.origin, input.translation), f[0], f[1]);
		fraction = f[0];
	}
	let value = castValue(input.maxFraction, hit, fraction);
	vtag("r");
	vi32(proxyId);
	vu64(userData);
	vvec(input.origin);
	vvec(input.translation);
	vf32(input.maxFraction);
	vf32(value);
	vend();
	q_calls += 1;
	if (q_nest && q_depth == 0 && q_calls % 3 == 1) {
		nestedQuery();
	}
	return value;
}

function boxCallback(input: b3BoxCastInput, proxyId: i32, userData: u64): f32 {
	let value = castValue(input.maxFraction, false, 0.0);
	vtag("b");
	vi32(proxyId);
	vu64(userData);
	vaabb(input.box);
	vvec(input.translation);
	vf32(input.maxFraction);
	vf32(value);
	vend();
	q_calls += 1;
	if (q_nest && q_depth == 0 && q_calls % 3 == 1) {
		nestedQuery();
	}
	return value;
}

// 0 runs through the tree API, 1 through a view without proxies, 2 through a view with proxies
function pickPath(): i32 {
	let pick = vrandint(0, 8);
	return pick < 5 ? 0 : (pick < 7 ? 1 : 2);
}

function makeView(path: i32): b3TreeView {
	if (path == 2) {
		return new b3TreeView(tree.nodes, tree.proxies);
	}
	return new b3TreeView(tree.nodes, null);
}

function runQuery(): void {
	let box = randomQueryBox();
	let mask = randomBits();
	let requireAll = vrandint(0, 3) == 0;
	q_stopAfter = vrandint(0, 4) == 0 ? vrandint(1, 6) : -1;
	q_nest = vrandint(0, 4) == 0;
	q_calls = 0;
	let path = pickPath();
	vtag("Q");
	vaabb(box);
	vu64(mask);
	vbool(requireAll);
	vi32(q_stopAfter);
	vbool(q_nest);
	vi32(path);
	vend();
	let s = b3TreeStats();
	if (path == 0) {
		s = b3DynamicTree_Query(tree, box, mask, requireAll, queryCallback);
	} else {
		s = b3TreeView_Query(makeView(path), box, mask, requireAll, queryCallback);
	}
	vstats("Qs", s);
}

function runClosest(): void {
	q_point = randomPoint();
	let mask = randomBits();
	let requireAll = vrandint(0, 3) == 0;
	let minSqr: FixedArray<f32, 1>;
	minSqr[0] = vrandint(0, 2) == 0 ? FLT_MAX : vrandf(0.0, worldSize * worldSize);
	q_nest = vrandint(0, 4) == 0;
	q_calls = 0;
	let path = pickPath();
	vtag("K");
	vvec(q_point);
	vu64(mask);
	vbool(requireAll);
	vf32(minSqr[0]);
	vbool(q_nest);
	vi32(path);
	vend();
	let s = b3TreeStats();
	if (path == 0) {
		s = b3DynamicTree_QueryClosest(tree, q_point, mask, requireAll, closestCallback, minSqr[0]);
	} else {
		s = b3TreeView_QueryClosest(makeView(path), q_point, mask, requireAll, closestCallback, minSqr[0]);
	}
	vtag("Km");
	vf32(minSqr[0]);
	vend();
	vstats("Ks", s);
}

function runRayCast(): void {
	let input = b3RayCastInput();
	input.origin = randomPoint();
	let span: f32 = 2.4 * worldSize;
	input.translation.x = vrandf(-span, span);
	input.translation.y = vrandf(-span, span);
	input.translation.z = vrandf(-span, span);
	if (aimAtProxy()) {
		input.translation = b3MulSV(1.5, b3Sub(aimTarget, input.origin));
	}
	let axis = vrandint(0, 6);
	if (axis == 0) {
		// axis aligned rays hit the parallel slab paths
		input.translation.y = 0.0;
		input.translation.z = 0.0;
	} else if (axis == 1) {
		input.translation = b3Vec3_zero;
	}
	input.maxFraction = vrandint(0, 4) == 0 ? vrandf(0.1, 1.0) : <f32>1.0;
	let mask = randomBits();
	let requireAll = vrandint(0, 3) == 0;
	q_policy = vrandint(0, 3);
	q_nest = vrandint(0, 4) == 0;
	q_calls = 0;
	let path = pickPath();
	vtag("R");
	vvec(input.origin);
	vvec(input.translation);
	vf32(input.maxFraction);
	vu64(mask);
	vbool(requireAll);
	vi32(q_policy);
	vbool(q_nest);
	vi32(path);
	vend();
	let s = b3TreeStats();
	if (path == 0) {
		s = b3DynamicTree_RayCast(tree, input, mask, requireAll, rayCallback);
	} else {
		s = b3TreeView_RayCast(makeView(path), input, mask, requireAll, rayCallback);
	}
	vstats("Rs", s);
}

function runBoxCast(): void {
	let input = b3BoxCastInput();
	let c = randomPoint();
	let e = b3Vec3();
	e.x = vrandf(0.0, boxSize);
	e.y = vrandf(0.0, boxSize);
	e.z = vrandf(0.0, boxSize);
	input.box = makeBox(c, e);
	let span: f32 = 2.4 * worldSize;
	input.translation.x = vrandf(-span, span);
	input.translation.y = vrandf(-span, span);
	input.translation.z = vrandf(-span, span);
	if (aimAtProxy()) {
		input.translation = b3MulSV(1.5, b3Sub(aimTarget, c));
	}
	if (vrandint(0, 6) == 0) {
		input.translation = b3Vec3_zero;
	}
	input.maxFraction = vrandint(0, 4) == 0 ? vrandf(0.1, 1.0) : <f32>1.0;
	let mask = randomBits();
	let requireAll = vrandint(0, 3) == 0;
	q_policy = vrandint(0, 2) == 0 ? 2 : 0;
	q_nest = vrandint(0, 4) == 0;
	q_calls = 0;
	let path = pickPath();
	vtag("B");
	vaabb(input.box);
	vvec(input.translation);
	vf32(input.maxFraction);
	vu64(mask);
	vbool(requireAll);
	vi32(q_policy);
	vbool(q_nest);
	vi32(path);
	vend();
	let s = b3TreeStats();
	if (path == 0) {
		s = b3DynamicTree_BoxCast(tree, input, mask, requireAll, boxCallback);
	} else {
		s = b3TreeView_BoxCast(makeView(path), input, mask, requireAll, boxCallback);
	}
	vstats("Bs", s);
}

function runQueries(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let kind = vrandint(0, 4);
		if (kind == 0) {
			runQuery();
		} else if (kind == 1) {
			runClosest();
		} else if (kind == 2) {
			runRayCast();
		} else {
			runBoxCast();
		}
	}
}

function doRebuild(full: bool): void {
	let count = b3DynamicTree_Rebuild(tree, full);
	if (count > 0) {
		lastLeafCount = count;
	}
	b3DynamicTree_Validate(tree);
	emitOp(full ? "rebuildFull" : "rebuild", count, 0);
}

const gatherIds = new StaticArray<i32>(MAX_LIVE);

function doGather(): void {
	let count = b3DynamicTree_GatherMovedProxies(tree, gatherIds);
	vtag("gather");
	vi32(count);
	for (let i = 0; i < count; ++i) {
		vi32(gatherIds[i]);
	}
	vend();
}

function doCreate(): void {
	let box = randomBox();
	let category = randomBits();
	let userData = vrand64();
	let internal = vrandint(0, 3);
	let proxyId = 0;
	if (internal == 0) {
		let mark = vrandint(0, 2) == 1;
		proxyId = b3CreateTreeProxyInternal(tree, box, category, userData, mark);
	} else {
		proxyId = b3DynamicTree_CreateProxy(tree, box, category, userData);
	}
	live[liveCount++] = proxyId;
	emitOp("create", proxyId, internal);
}

function doDestroy(index: i32): void {
	let proxyId = live[index];
	live[index] = live[liveCount - 1];
	liveCount -= 1;
	b3DynamicTree_DestroyProxy(tree, proxyId);
	emitOp("destroy", proxyId, liveCount);
}

function doMove(proxyId: i32): void {
	let box = randomBox();
	let internal = vrandint(0, 3);
	if (internal == 0) {
		let mark = vrandint(0, 2) == 1;
		b3DynamicTree_MoveProxyInternal(tree, proxyId, box, mark);
	} else {
		b3DynamicTree_MoveProxy(tree, proxyId, box);
	}
	emitOp("move", proxyId, internal);
}

function doEnlarge(proxyId: i32): void {
	let box = enlarged(b3DynamicTree_GetAABB(tree, proxyId));
	b3DynamicTree_EnlargeProxy(tree, proxyId, box);
	emitOp("enlarge", proxyId, 0);
}

function doMarkMoved(proxyId: i32): void {
	let box = enlarged(b3DynamicTree_GetAABB(tree, proxyId));
	b3DynamicTree_MarkProxyMoved<b3OnMain>(tree, proxyId, box);
	emitOp("mark", proxyId, 0);
}

function doMarkSerial(proxyId: i32): void {
	b3DynamicTree_MarkProxyMovedSerial(tree, proxyId);
	emitOp("markSerial", proxyId, 0);
}

function doSetCategory(proxyId: i32): void {
	let bits = randomBits();
	b3DynamicTree_SetCategoryBits(tree, proxyId, bits);
	vtag("category");
	vi32(proxyId);
	vu64(b3DynamicTree_GetCategoryBits(tree, proxyId));
	vu64(b3DynamicTree_GetUserData(tree, proxyId));
	vaabb(b3DynamicTree_GetAABB(tree, proxyId));
	vend();
}

function createTree(capacity: i32): void {
	tree = b3DynamicTree_Create(capacity);
	treeAlive = true;
	liveCount = 0;
	lastLeafCount = 0;
}

function destroyTree(): void {
	b3DynamicTree_Destroy(tree);
	treeAlive = false;
	liveCount = 0;
	lastLeafCount = 0;
	emitHeader("destroyed");
}

function createFixed(box: b3AABB, userData: u64): void {
	let proxyId = b3DynamicTree_CreateProxy(tree, box, B3_DEFAULT_CATEGORY_BITS, userData);
	live[liveCount++] = proxyId;
	emitOp("create", proxyId, 0);
}

function unitBox(x: f32, y: f32, z: f32): b3AABB {
	return makeBox(b3Vec3(x, y, z), b3Vec3(0.5, 0.5, 0.5));
}

export function scripted(): void {
	vseed(77);
	boxMode = 0;
	worldSize = 10.0;
	boxSize = 1.0;

	// Capacities
	let capacities: StaticArray<i32> = [-5, 0, 1, 15, 16, 17, 100];
	for (let i = 0; i < 7; ++i) {
		createTree(capacities[i]);
		emitHeader("cap");
		emitStats();
		destroyTree();
	}

	// Empty tree
	createTree(0);
	emitTree("empty");
	runQueries(12);
	doRebuild(false);
	doRebuild(true);
	b3DynamicTree_Refit(tree);
	b3DynamicTree_ClearMoved(tree);
	doGather();
	emitTree("empty2");

	// One proxy: the root is a leaf
	createFixed(unitBox(1.0, 2.0, 3.0), 0xabcdef0123456789);
	emitTree("one");
	runQueries(12);
	doRebuild(false);
	doRebuild(true);
	emitTree("oneRebuilt");
	doMarkSerial(live[0]);
	doGather();
	doRebuild(false);
	doEnlarge(live[0]);
	doGather();
	b3DynamicTree_Refit(tree);
	emitOp("refit", 0, 0);
	b3DynamicTree_ClearMoved(tree);
	emitOp("clear", 0, 0);
	doMarkMoved(live[0]);
	b3DynamicTree_Refit(tree);
	emitOp("refit", 0, 0);
	emitTree("oneMoved");
	doDestroy(0);
	emitTree("oneDestroyed");

	// A handful of proxies, rotations and removals by hand
	for (let i = 0; i < 6; ++i) {
		createFixed(unitBox(<f32>i, 0.0, 0.0), <u64>i);
		emitTree("row");
	}
	runQueries(16);
	doDestroy(2);
	doDestroy(0);
	emitTree("rowRemoved");
	createFixed(unitBox(2.5, 0.0, 0.0), 100);
	createFixed(unitBox(2.5, 0.0, 0.0), 101);
	emitTree("rowReuse");
	doRebuild(false);
	emitTree("rowRebuilt");
	doMove(live[1]);
	doMove(live[3]);
	doRebuild(false);
	emitTree("rowPartial");
	doRebuild(true);
	emitTree("rowFull");
	while (liveCount > 0) {
		doDestroy(liveCount - 1);
	}
	emitTree("rowEmpty");
	destroyTree();

	// Growth of the node and proxy pools, with the spare dropped on node growth
	createTree(1);
	for (let i = 0; i < 60; ++i) {
		doCreate();
		if (i % 8 == 7) {
			doRebuild(i % 16 == 15);
			emitTree("grow");
		}
	}
	runQueries(24);
	for (let i = 0; i < 40; ++i) {
		doDestroy(vrandint(0, liveCount));
	}
	emitTree("shrunk");
	for (let i = 0; i < 40; ++i) {
		doCreate();
	}
	emitTree("regrown");
	doRebuild(false);
	doRebuild(false);
	emitTree("settled");
	destroyTree();

	// Identical boxes: every partition and sibling cost ties
	createTree(8);
	for (let i = 0; i < 33; ++i) {
		createFixed(unitBox(0.0, 0.0, 0.0), <u64>(1000 + i));
	}
	emitTree("same");
	runQueries(12);
	doRebuild(true);
	emitTree("sameRebuilt");
	destroyTree();

	// Compound style: create then a full build, then queries
	createTree(50);
	for (let i = 0; i < 50; ++i) {
		doCreate();
	}
	doRebuild(true);
	emitTree("compound");
	runQueries(40);
	destroyTree();
}

export function beginRound(round: i32): void {
	vseed(<u64>(1000 + round));
	boxMode = round % 6;
	worldSize = vrandf(5.0, 60.0);
	boxSize = vrandf(0.2, 4.0);
	let capacity = vrandint(0, 300);
	createWeight = vrandint(35, 55);
	destroyWeight = vrandint(10, 20);
	opsLo = round % 3 == 0 ? 300 : 60;
	opsHi = round % 3 == 0 ? 700 : 200;
	phaseIndex = 0;
	createTree(capacity);
	vtag("round");
	vi32(round);
	vi32(boxMode);
	vf32(worldSize);
	vf32(boxSize);
	vi32(capacity);
	vend();
	emitTree("begin");
}

export function phase(): void {
	let growing = phaseIndex < PHASE_COUNT / 2 + 1;
	let cw = growing ? createWeight : destroyWeight;
	let dw = growing ? destroyWeight : createWeight;
	let ops = vrandint(opsLo, opsHi);
	for (let k = 0; k < ops; ++k) {
		let kind = vrandint(0, 100);
		if (liveCount == 0 && kind >= cw) {
			kind = 0;
		}

		if (kind < cw) {
			if (liveCount < MAX_LIVE) {
				doCreate();
			}
			continue;
		}
		kind -= cw;

		if (kind < dw) {
			doDestroy(vrandint(0, liveCount));
			continue;
		}
		kind -= dw;

		let target = live[vrandint(0, liveCount)];
		if (kind < 14) {
			doMove(target);
		} else if (kind < 22) {
			doEnlarge(target);
		} else if (kind < 26) {
			doMarkMoved(target);
		} else if (kind < 28) {
			doMarkSerial(target);
		} else if (kind < 30) {
			doSetCategory(target);
		} else if (kind < 33) {
			runQueries(1);
		} else if (kind < 35) {
			doRebuild(vrandint(0, 4) == 0);
		} else if (kind < 37) {
			b3DynamicTree_Refit(tree);
			emitOp("refit", 0, 0);
		} else if (kind < 38) {
			// Clearing without a refit leaves stale ancestors that validation rejects
			b3DynamicTree_Refit(tree);
			b3DynamicTree_ClearMoved(tree);
			emitOp("clear", 0, 0);
		} else if (kind < 39) {
			doGather();
		} else {
			emitOp("noop", target, 0);
		}
	}

	emitTree("phase");
	runQueries(vrandint(10, 40));

	// The solver pattern: refit what moved, then a partial rebuild
	if (phaseIndex % 3 == 2) {
		doGather();
		b3DynamicTree_Refit(tree);
		emitOp("refit", 0, 0);
		doRebuild(false);
		emitTree("settled");
		runQueries(vrandint(5, 20));
	}

	phaseIndex += 1;
}

export function endRound(): void {
	doRebuild(true);
	emitTree("end");
	runQueries(20);
	destroyTree();
}
