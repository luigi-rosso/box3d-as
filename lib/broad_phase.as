// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/broad_phase.c

import { B3_ASSERT, B3_ENABLE_VALIDATION, B3_NULL_INDEX } from "./core";
import { B3_MAX_TASKS, B3_SHAPE_POWER } from "./constants";
import {
	b3AABB,
	b3AABB_Overlaps,
	b3AABB_Transform,
	b3InvertTransform,
	b3MaxInt,
	b3MinInt,
	b3Pos_zero,
	b3ToRelativeTransform,
} from "./math_functions";
import { b3DynamicTree, b3DynamicTree_GetAABB, b3DynamicTree_GetUserData, b3TreeNode } from "./collision";
import {
	B3_MOVED_NODE,
	B3_ROOT_NODE,
	B3_TREE_STACK_SIZE,
	b3CreateTreeProxyInternal,
	b3DynamicTree_ClearMoved,
	b3DynamicTree_Create,
	b3DynamicTree_Destroy,
	b3DynamicTree_DestroyProxy,
	b3DynamicTree_MarkProxyMoved,
	b3DynamicTree_MarkProxyMovedSerial,
	b3DynamicTree_MoveProxyInternal,
	b3DynamicTree_Rebuild,
	b3DynamicTree_Validate,
	b3DynamicTree_ValidateNoMoved,
	b3GetLeftChild,
	b3HasTreeMoved,
	b3IsLeaf,
	b3NeedsRebuild,
	b3TreeView_Query,
} from "./dynamic_tree";
import {
	B3_CHILD_MASK,
	B3_SHAPE_MASK,
	b3ContainsHashedKey,
	b3ContainsKey,
	b3CreateSet,
	b3DestroySet,
	b3HashSet,
	b3KeyHash,
	b3ShapePairKey,
} from "./table";
import {
	B3_DEFAULT_MASK_BITS,
	b3BodyType,
	b3Capacity,
	b3ShapeId,
	b3_bodyTypeCount,
	b3_compoundShape,
	b3_dynamicBody,
	b3_kinematicBody,
	b3_staticBody,
} from "./types";
import { b3LoadAABBV, b3OverlapNode, b3OverlapV } from "./simd";
import { Q_LOOP } from "./qsort";
import { b3Array_Resize } from "./container";
import { b3GetBodyTransform, b3ShouldBodiesCollide } from "./body";
import { b3GetCompoundTreeView } from "./compound";
import { b3CreateContact } from "./contact";
import { b3ParallelFor } from "./parallel_for";
import { b3World } from "./physics_world";
import { b3ShouldShapesCollide, b3_enableCustomFiltering } from "./shape";

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

	// C's per step stack buffers, kept and grown across steps
	siblingBuffer: StaticArray<i32> = b3_emptySiblings;
	pairKeyBuffer: StaticArray<u64> = new StaticArray<u64>(0);
	pairContexts: Array<b3PairContext> = new Array<b3PairContext>();

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

@inline export unsafe function b3BroadPhase_MarkProxyMoved(bp: b3BroadPhase, proxyKey: i32, aabb: b3AABB): void {
	let proxyType = B3_PROXY_TYPE(proxyKey);
	let proxyId = B3_PROXY_ID(proxyKey);
	b3DynamicTree_MarkProxyMoved(bp.trees[proxyType], proxyId, aabb);
}

// functions from broad_phase.c follow

export function b3CreateBroadPhase(bp: b3BroadPhase, capacity: b3Capacity): void {
	bp.pairSet = b3CreateSet(b3MaxInt(32, 2 * capacity.contactCount));

	let staticCapacity = b3MaxInt(16, capacity.staticShapeCount);
	bp.trees[b3_staticBody] = b3DynamicTree_Create(staticCapacity);

	let kinematicCapacity = 16;
	bp.trees[b3_kinematicBody] = b3DynamicTree_Create(kinematicCapacity);

	let dynamicCapacity = b3MaxInt(16, capacity.dynamicShapeCount);
	bp.trees[b3_dynamicBody] = b3DynamicTree_Create(dynamicCapacity);

	bp.movedSiblings = b3_emptySiblings;
}

export function b3DestroyBroadPhase(bp: b3BroadPhase): void {
	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		b3DynamicTree_Destroy(bp.trees[i]);
	}

	b3DestroySet(bp.pairSet);

	// memset
	for (let i = 0; i < b3_bodyTypeCount; ++i) {
		bp.trees[i] = new b3DynamicTree();
	}
	bp.movedSiblings = b3_emptySiblings;
	bp.pairSet = new b3HashSet();
}

export function b3BroadPhase_CreateProxy(
	bp: b3BroadPhase,
	proxyType: b3BodyType,
	aabb: b3AABB,
	categoryBits: u64,
	shapeIndex: i32,
	forcePairCreation: bool,
): i32 {
	B3_ASSERT(0 <= proxyType && proxyType < b3_bodyTypeCount);

	let mark = proxyType != b3_staticBody || forcePairCreation;

	let proxyId = b3CreateTreeProxyInternal(bp.trees[proxyType], aabb, categoryBits, <u64>shapeIndex, mark);
	let proxyKey = B3_PROXY_KEY(proxyId, proxyType);
	return proxyKey;
}

export function b3BroadPhase_DestroyProxy(bp: b3BroadPhase, proxyKey: i32): void {
	let proxyType = B3_PROXY_TYPE(proxyKey);
	let proxyId = B3_PROXY_ID(proxyKey);

	B3_ASSERT(0 <= proxyType && proxyType < b3_bodyTypeCount);
	b3DynamicTree_DestroyProxy(bp.trees[proxyType], proxyId);
}

export function b3BroadPhase_MoveProxy(bp: b3BroadPhase, proxyKey: i32, aabb: b3AABB): void {
	let proxyType = B3_PROXY_TYPE(proxyKey);
	let proxyId = B3_PROXY_ID(proxyKey);

	let mark = true;
	b3DynamicTree_MoveProxyInternal(bp.trees[proxyType], proxyId, aabb, mark);
}

// Gather the sibling pairs with a moved node. This is done serially, it is cache friendly.
function b3GatherMovedSiblings(tree: b3DynamicTree, pairIndices: StaticArray<i32>): i32 {
	let nodes = tree.nodes;
	let nodeEnd = tree.nodeEnd;

	let count = 0;

	// Skip the root.
	for (let pair = 2; pair < nodeEnd; pair += 2) {
		// Push when either sibling moved.
		if (((nodes[pair].flagIndex | nodes[pair + 1].flagIndex) & B3_MOVED_NODE) != 0) {
			pairIndices[count++] = pair;
		}
	}

	return count;
}

const B3_CANDIDATE_BATCH: i32 = 32;

struct b3CandidatePair {
	shapeIdA: i32 = 0;
	shapeIdB: i32 = 0;
}

// C keeps the flush buffers and the traversal stacks on the stack; here the per task context owns them.
class b3PairContext {
	world: b3World;
	pairKeys: Array<u64>;
	batch: StaticArray<b3CandidatePair> = new StaticArray<b3CandidatePair>(B3_CANDIDATE_BATCH);
	batchCount: i32 = 0;

	// Compound shapes are static only, so only the static cross pass can meet one. Elsewhere
	// this stays false and the batch never reads a shape type.
	checkCompounds: bool = false;

	plain: StaticArray<b3CandidatePair> = new StaticArray<b3CandidatePair>(B3_CANDIDATE_BATCH);
	keys: StaticArray<u64> = new StaticArray<u64>(B3_CANDIDATE_BATCH);
	hashes: StaticArray<u32> = new StaticArray<u32>(B3_CANDIDATE_BATCH);
	candidates: StaticArray<b3CandidatePair> = new StaticArray<b3CandidatePair>(B3_CANDIDATE_BATCH);
	proxyStack: StaticArray<i32> = new StaticArray<i32>(B3_TREE_STACK_SIZE);
	pairStack: StaticArray<b3IndexPair> = new StaticArray<b3IndexPair>(B3_TREE_STACK_SIZE);

	constructor(world: b3World, pairKeys: Array<u64>) {
		this.world = world;
		this.pairKeys = pairKeys;
	}
}

struct b3NodePair {
	a: b3TreeNode = b3TreeNode();
	b: b3TreeNode = b3TreeNode();

	constructor(a: b3TreeNode, b: b3TreeNode) {
		this.a = a;
		this.b = b;
	}
}

struct b3IndexPair {
	a: i32 = 0;
	b: i32 = 0;

	constructor(a: i32, b: i32) {
		this.a = a;
		this.b = b;
	}
}

// b3PrefetchHash is a cache hint only and is not ported.

// The shape pair gauntlet. Everything here is a property of the pair, not of a compound child,
// so a compound runs it once for the whole pair.
function b3ShouldCreatePair(world: b3World, shapeIdA: i32, shapeIdB: i32): bool {
	let shapeA = world.shapes[shapeIdA];
	let shapeB = world.shapes[shapeIdB];

	let bodyIdA = shapeA.bodyId;
	let bodyIdB = shapeB.bodyId;

	// Are the shapes on the same body?
	if (bodyIdA == bodyIdB) {
		return false;
	}

	// Sensors are handled elsewhere
	if (shapeA.sensorIndex != B3_NULL_INDEX || shapeB.sensorIndex != B3_NULL_INDEX) {
		return false;
	}

	if (b3ShouldShapesCollide(shapeA.filter, shapeB.filter) == false) {
		return false;
	}

	// Does a joint override collision?
	if (b3ShouldBodiesCollide(world, bodyIdA, bodyIdB) == false) {
		return false;
	}

	// Custom user filter
	if ((shapeA.flags & b3_enableCustomFiltering) != 0 || (shapeB.flags & b3_enableCustomFiltering) != 0) {
		let customFilterFcn = world.customFilterFcn;
		if (customFilterFcn != null) {
			let idA = b3ShapeId();
			idA.index1 = shapeIdA + 1;
			idA.world0 = world.worldId;
			idA.generation = shapeA.generation;
			let idB = b3ShapeId();
			idB.index1 = shapeIdB + 1;
			idB.world0 = world.worldId;
			idB.generation = shapeB.generation;
			if (customFilterFcn(idA, idB) == false) {
				return false;
			}
		}
	}

	return true;
}

// The child query runs no user code, so one context serves every compound pair and the tree
// callback captures nothing
class b3CompoundPairContext {
	pairContext: b3PairContext? = null;
	pairSet: b3HashSet? = null;
	compoundShapeId: i32 = 0;
	otherShapeId: i32 = 0;
}

const s_compoundPairContext = new b3CompoundPairContext();

function b3CompoundChildTreeCallback(proxyId: i32, userData: u64): bool {
	return b3CompoundChildCallback(proxyId, userData, s_compoundPairContext);
}

function b3CompoundChildCallback(_proxyId: i32, userData: u64, compoundContext: b3CompoundPairContext): bool {
	let childIndex = <i32>userData;

	let key = b3ShapePairKey(compoundContext.compoundShapeId, compoundContext.otherShapeId, childIndex);
	if (b3ContainsKey(compoundContext.pairSet!, key) == false) {
		compoundContext.pairContext!.pairKeys.push(key);
	}

	return true;
}

// One of the two shapes is a compound, so the pair becomes one pair per overlapping child. The
// child boxes live in the compound frame, so the other shape's box has to be demoted and pulled
// into that frame first.
function b3EmitCompoundPairs(context: b3PairContext, compoundShapeId: i32, otherShapeId: i32): void {
	let world = context.world;

	let compoundShape = world.shapes[compoundShapeId];
	let otherShape = world.shapes[otherShapeId];

	let otherProxyKey = otherShape.proxyKey;
	B3_ASSERT(otherProxyKey != B3_NULL_INDEX);
	let otherAABB = b3DynamicTree_GetAABB(world.broadPhase.trees[B3_PROXY_TYPE(otherProxyKey)], B3_PROXY_ID(otherProxyKey));

	// Query bounds are float world space, so the demoted transform is the matching float frame
	let compoundTransform = b3ToRelativeTransform(b3GetBodyTransform(world, compoundShape.bodyId), b3Pos_zero);
	let localAABB = b3AABB_Transform(b3InvertTransform(compoundTransform), otherAABB);

	let compoundContext = s_compoundPairContext;
	compoundContext.pairContext = context;
	compoundContext.pairSet = world.broadPhase.pairSet;
	compoundContext.compoundShapeId = compoundShapeId;
	compoundContext.otherShapeId = otherShapeId;

	let startCount = context.pairKeys.length;

	let view = b3GetCompoundTreeView(compoundShape.compound!);
	b3TreeView_Query(view, localAABB, B3_DEFAULT_MASK_BITS, false, b3CompoundChildTreeCallback);

	if (context.pairKeys.length > startCount && b3ShouldCreatePair(world, compoundShapeId, otherShapeId) == false) {
		b3Array_Resize(context.pairKeys, startCount);
	}
}

function b3FlushCandidatePairs(context: b3PairContext): void {
	let world = context.world;
	let bp = world.broadPhase;

	let count1 = context.batchCount;
	context.batchCount = 0;

	let plain = context.plain;
	let plainCount = 0;

	if (context.checkCompounds) {
		// A compound expands into one pair per child, so its key is not known yet and it cannot
		// ride the batched cull below.
		let shapes = world.shapes;
		for (let i = 0; i < count1; ++i) {
			let candidate = context.batch[i];
			let compoundA = shapes[candidate.shapeIdA].type == b3_compoundShape;
			let compoundB = shapes[candidate.shapeIdB].type == b3_compoundShape;

			if (compoundA) {
				// Compound versus compound is not supported
				B3_ASSERT(compoundB == false);
				b3EmitCompoundPairs(context, candidate.shapeIdA, candidate.shapeIdB);
			} else if (compoundB) {
				b3EmitCompoundPairs(context, candidate.shapeIdB, candidate.shapeIdA);
			} else {
				plain[plainCount] = candidate;
				plainCount += 1;
			}
		}
	} else {
		for (let i = 0; i < count1; ++i) {
			plain[i] = context.batch[i];
		}
		plainCount = count1;
	}

	let keys = context.keys;
	let hashes = context.hashes;
	for (let i = 0; i < plainCount; ++i) {
		keys[i] = b3ShapePairKey(plain[i].shapeIdA, plain[i].shapeIdB, 0);
		hashes[i] = b3KeyHash(keys[i]);
	}

	// Cull existing pairs.
	let candidates = context.candidates;
	let count2 = 0;
	for (let i = 0; i < plainCount; ++i) {
		if (b3ContainsHashedKey(bp.pairSet, keys[i], hashes[i]) == false) {
			candidates[count2] = plain[i];
			keys[count2] = keys[i];
			count2 += 1;
		}
	}

	// Filter candidates.
	for (let i = 0; i < count2; ++i) {
		if (b3ShouldCreatePair(world, candidates[i].shapeIdA, candidates[i].shapeIdB)) {
			// The pair passed the gauntlet. A new contact will be created.
			context.pairKeys.push(keys[i]);
		}
	}
}

@inline function b3AddCandidatePair(shapeIdA: i32, shapeIdB: i32, context: b3PairContext): void {
	// Follow shape index order.
	ref candidate = context.batch[context.batchCount];
	candidate.shapeIdA = b3MinInt(shapeIdA, shapeIdB);
	candidate.shapeIdB = b3MaxInt(shapeIdA, shapeIdB);
	context.batchCount += 1;
	if (context.batchCount == B3_CANDIDATE_BATCH) {
		b3FlushCandidatePairs(context);
	}
}

// Did either move and if so do they overlap?
@inline function b3TestPair(a: b3TreeNode, b: b3TreeNode): bool {
	if (((a.flagIndex | b.flagIndex) & B3_MOVED_NODE) == 0) {
		return false;
	}

	return b3OverlapV(a.aabb, b.aabb);
}

function b3CollideProxyAndSubtree(proxy: b3TreeNode, nodes: StaticArray<b3TreeNode>, pair: i32, context: b3PairContext): void {
	let proxyMark = proxy.flagIndex & B3_MOVED_NODE;
	let boxv = b3LoadAABBV(proxy.aabb);
	let shapeId = proxy.shapeIndex;

	let stack = context.proxyStack;
	let stackCount = 0;
	stack[stackCount++] = pair;

	while (stackCount > 0) {
		pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			let node = nodes[pair + i];
			if (((node.flagIndex | proxyMark) & B3_MOVED_NODE) == 0) {
				continue;
			}

			if (b3OverlapNode(boxv, node) == false) {
				continue;
			}

			if (b3IsLeaf(node)) {
				b3AddCandidatePair(shapeId, node.shapeIndex, context);
			} else {
				if (stackCount < B3_TREE_STACK_SIZE) {
					stack[stackCount++] = b3GetLeftChild(node);
				} else {
					B3_ASSERT(stackCount < B3_TREE_STACK_SIZE);
				}
			}
		}
	}
}

// Helper for b3CollideCrossPairs to avoid code duplication. Returns the new stack count, which C
// writes through a pointer.
@inline function b3VisitPair(
	arrayA: StaticArray<b3TreeNode>,
	arrayB: StaticArray<b3TreeNode>,
	nodeA: b3TreeNode,
	nodeB: b3TreeNode,
	stack: StaticArray<b3IndexPair>,
	stackCount: i32,
	context: b3PairContext,
): i32 {
	if (b3TestPair(nodeA, nodeB) == false) {
		return stackCount;
	}

	let leafA = b3IsLeaf(nodeA);
	let leafB = b3IsLeaf(nodeB);
	if (leafA && leafB) {
		b3AddCandidatePair(nodeA.shapeIndex, nodeB.shapeIndex, context);
	} else if (leafA) {
		b3CollideProxyAndSubtree(nodeA, arrayB, b3GetLeftChild(nodeB), context);
	} else if (leafB) {
		b3CollideProxyAndSubtree(nodeB, arrayA, b3GetLeftChild(nodeA), context);
	} else {
		if (stackCount < B3_TREE_STACK_SIZE) {
			stack[stackCount] = b3IndexPair(b3GetLeftChild(nodeA), b3GetLeftChild(nodeB));
			stackCount += 1;
		} else {
			B3_ASSERT(stackCount < B3_TREE_STACK_SIZE);
		}
	}

	return stackCount;
}

// This collides two sub-trees against each other. They can live in the same dynamic tree.
// This can only generate pairs cross sub-tree, but not within a sub-tree. This fact means
// this does not generate duplicate pairs.
// For example consider the full binary tree A (B (D  E) C (F G))
// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
// Then colliding children of B can give the pair (D,E) and for C (F,G).
// So no duplicates even when used for self-collision.
// Whenever a proxy is moved, the flag is propagated up the hierarchy to the root. So
// self collision gathers all those moved internal nodes and collides their subtrees together.
// See Real-time collision detection section 6.3.2. This is faster than querying every moved
// proxy against the whole tree. Scaling is linear instead of linear * log.
// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.
function b3CollideCrossPairs(
	arrayA: StaticArray<b3TreeNode>,
	arrayB: StaticArray<b3TreeNode>,
	subtreeA: b3TreeNode,
	subtreeB: b3TreeNode,
	context: b3PairContext,
): void {
	let stack = context.pairStack;
	let stackCount = 0;

	// Seed the stack.
	stackCount = b3VisitPair(arrayA, arrayB, subtreeA, subtreeB, stack, stackCount, context);

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			for (let j = 0; j < 2; ++j) {
				stackCount = b3VisitPair(arrayA, arrayB, arrayA[pair.a + i], arrayB[pair.b + j], stack, stackCount, context);
			}
		}
	}
}

// This takes moved internal nodes and collides their sub-trees against each other.
// One pair context per worker, reset for each task
function b3AcquirePairContext(world: b3World, workerIndex: i32): b3PairContext {
	let contexts = world.broadPhase.pairContexts;
	while (contexts.length <= workerIndex) {
		contexts.push(new b3PairContext(world, world.taskContexts[contexts.length].pairKeys));
	}

	let context = contexts[workerIndex];
	context.world = world;
	context.pairKeys = world.taskContexts[workerIndex].pairKeys;
	context.batchCount = 0;
	context.checkCompounds = false;
	return context;
}

function b3SelfPairsTask(startIndex: i32, endIndex: i32, workerIndex: i32, world: b3World): void {
	let bp = world.broadPhase;
	let tree = bp.trees[b3_dynamicBody];
	let nodes = tree.nodes;
	let siblingIndices = bp.movedSiblings;

	let pairContext = b3AcquirePairContext(world, workerIndex);

	for (let i = startIndex; i < endIndex; ++i) {
		let nodeIndex = siblingIndices[i];
		b3CollideCrossPairs(nodes, nodes, nodes[nodeIndex], nodes[nodeIndex + 1], pairContext);
	}

	b3FlushCandidatePairs(pairContext);
}

const B3_CROSS_SEED_COUNT: i32 = 64;

// C's stack buffers for the cross seeds and their search queue
const s_crossQueue = new StaticArray<b3NodePair>(2 * B3_CROSS_SEED_COUNT);
const s_crossSeeds = new StaticArray<b3NodePair>(2 * B3_CROSS_SEED_COUNT);

// This does a serial cross-tree breadth first search until the queue is full. Then it returns
// the queue pairs as seeds for a parallel search. C's seeds pointer is seeds plus seedBase.
function b3GatherCrossSeeds(
	treeA: b3DynamicTree,
	treeB: b3DynamicTree,
	seeds: StaticArray<b3NodePair>,
	seedBase: i32,
): i32 {
	let nodesA = treeA.nodes;
	let nodesB = treeB.nodes;

	// Breadth-first search from the two roots as one pair. An empty tree has a sentinel root that
	// survives nothing.
	let queue = s_crossQueue;
	let mask = 2 * B3_CROSS_SEED_COUNT - 1;
	let head = 0;
	let tail = 0;

	let rootA = nodesA[B3_ROOT_NODE];
	let rootB = nodesB[B3_ROOT_NODE];
	if (b3TestPair(rootA, rootB)) {
		queue[tail & mask] = b3NodePair(rootA, rootB);
		tail += 1;
	}

	let seedCount = 0;
	while (head < tail && seedCount + (tail - head) + 3 < B3_CROSS_SEED_COUNT) {
		let pair = queue[head & mask];
		head += 1;

		if (b3IsLeaf(pair.a) || b3IsLeaf(pair.b)) {
			seeds[seedBase + seedCount++] = pair;
			continue;
		}

		let a = b3GetLeftChild(pair.a);
		let b = b3GetLeftChild(pair.b);

		// Nodes have two children each, so four combinations.
		for (let i = 0; i < 2; ++i) {
			for (let j = 0; j < 2; ++j) {
				if (b3TestPair(nodesA[a + i], nodesB[b + j])) {
					queue[tail & mask] = b3NodePair(nodesA[a + i], nodesB[b + j]);
					tail += 1;
				}
			}
		}
	}

	while (head < tail) {
		seeds[seedBase + seedCount] = queue[head & mask];
		seedCount += 1;
		head += 1;
	}

	return seedCount;
}

class b3CrossContext {
	world: b3World;
	seeds: StaticArray<b3NodePair>;
	staticSeedCount: i32;
	checkCompounds: bool;

	constructor(world: b3World, seeds: StaticArray<b3NodePair>, staticSeedCount: i32, checkCompounds: bool) {
		this.world = world;
		this.seeds = seeds;
		this.staticSeedCount = staticSeedCount;
		this.checkCompounds = checkCompounds;
	}
}

function b3CrossPairsTask(startIndex: i32, endIndex: i32, workerIndex: i32, crossContext: b3CrossContext): void {
	let world = crossContext.world;
	let bp = world.broadPhase;
	let staticNodes = bp.trees[b3_staticBody].nodes;
	let kinematicNodes = bp.trees[b3_kinematicBody].nodes;
	let dynamicNodes = bp.trees[b3_dynamicBody].nodes;

	let pairContext = b3AcquirePairContext(world, workerIndex);

	// Static seeds come first, so the two passes are contiguous sub-ranges. The batch is drained
	// between them because only the static pass can meet a compound.
	let staticEnd = b3MinInt(endIndex, crossContext.staticSeedCount);

	pairContext.checkCompounds = crossContext.checkCompounds;
	for (let i = startIndex; i < staticEnd; ++i) {
		let seed = crossContext.seeds[i];
		b3CollideCrossPairs(dynamicNodes, staticNodes, seed.a, seed.b, pairContext);
	}
	b3FlushCandidatePairs(pairContext);

	pairContext.checkCompounds = false;
	for (let i = b3MaxInt(startIndex, crossContext.staticSeedCount); i < endIndex; ++i) {
		let seed = crossContext.seeds[i];
		b3CollideCrossPairs(dynamicNodes, kinematicNodes, seed.a, seed.b, pairContext);
	}
	b3FlushCandidatePairs(pairContext);
}

function b3UpdateTreesTask(world: b3World): void {
	b3DynamicTree_Rebuild(world.broadPhase.trees[b3_dynamicBody], false);
	b3DynamicTree_Rebuild(world.broadPhase.trees[b3_kinematicBody], false);
}

// Task that can be done in parallel with the narrow-phase
// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
// The closure carries the world, so the task context argument is unused. A null task is 0.
function b3EnqueueTreeUpdate(world: b3World): void {
	if (world.taskCount < B3_MAX_TASKS) {
		world.userTreeTask = world.enqueueTaskFcn(
			(taskContext: u64): void => b3UpdateTreesTask(world),
			0,
			world.userTaskContext,
			"rebuild tree",
		);
		world.taskCount += 1;
		world.activeTaskCount += world.userTreeTask == 0 ? 0 : 1;
	} else {
		world.userTreeTask = 0;
		b3UpdateTreesTask(world);
	}
}

class b3PairKeySorter {
	pairKeys: StaticArray<u64> = new StaticArray<u64>(0);

	@inline less(i: i32, j: i32): bool {
		return this.pairKeys[i] < this.pairKeys[j];
	}

	@inline swap(i: i32, j: i32): void {
		let tmp_ = this.pairKeys[i];
		this.pairKeys[i] = this.pairKeys[j];
		this.pairKeys[j] = tmp_;
	}
}

const s_pairKeySorter = new b3PairKeySorter();

export function b3UpdateBroadPhasePairs(world: b3World): void {
	let bp = world.broadPhase;

	let needUpdate = b3HasTreeMoved(bp.trees[b3_staticBody]);
	needUpdate = needUpdate || b3NeedsRebuild(bp.trees[b3_kinematicBody]);
	needUpdate = needUpdate || b3NeedsRebuild(bp.trees[b3_dynamicBody]);

	if (needUpdate == false) {
		return;
	}

	for (let i = 0; i < world.workerCount; ++i) {
		world.taskContexts[i].pairKeys.length = 0;
	}

	// Generate pairs by querying the dynamic body tree against itself and against
	// the kinematic and static trees.
	{
		// Get the sibling pairs of the dynamic body tree that have moved.
		let dynamicTree = bp.trees[b3_dynamicBody];
		let pairCapacity = b3MaxInt(dynamicTree.nodeEnd / 2, 1);
		if (bp.siblingBuffer.length < pairCapacity) {
			bp.siblingBuffer = new StaticArray<i32>(b3MaxInt(pairCapacity, 2 * bp.siblingBuffer.length));
		}
		bp.movedSiblings = bp.siblingBuffer;
		let dynamicMoveCount = b3GatherMovedSiblings(dynamicTree, bp.movedSiblings);

		// Get seeds for colliding against the static and kinematic trees.
		let crossSeeds = s_crossSeeds;
		let staticSeedCount = b3GatherCrossSeeds(dynamicTree, bp.trees[b3_staticBody], crossSeeds, 0);
		B3_ASSERT(staticSeedCount <= B3_CROSS_SEED_COUNT);
		let kinematicSeedCount = b3GatherCrossSeeds(dynamicTree, bp.trees[b3_kinematicBody], crossSeeds, staticSeedCount);
		B3_ASSERT(kinematicSeedCount <= B3_CROSS_SEED_COUNT);
		let crossMoveCount = staticSeedCount + kinematicSeedCount;

		// Collide the dynamic body tree against the static and kinematic trees.
		let crossContext = new b3CrossContext(world, crossSeeds, staticSeedCount, world.compoundShapeCount > 0);
		b3ParallelFor(
			world,
			(startIndex: i32, endIndex: i32, workerIndex: i32): void =>
				b3CrossPairsTask(startIndex, endIndex, workerIndex, crossContext),
			crossMoveCount,
			1,
			"cross pairs",
		);

		// Collide the dynamic body tree against itself.
		b3ParallelFor(
			world,
			(startIndex: i32, endIndex: i32, workerIndex: i32): void => b3SelfPairsTask(startIndex, endIndex, workerIndex, world),
			dynamicMoveCount,
			64,
			"self pairs",
		);
	}

	b3DynamicTree_ClearMoved(bp.trees[b3_staticBody]);

	// Update stale trees.
	b3EnqueueTreeUpdate(world);

	// Pairs arrive in deterministic order but scrambled relative to body and shape order.
	// Sorting them here improves solver performance and makes contact order independent of
	// tree structure.
	let pairCount = 0;
	for (let i = 0; i < world.workerCount; ++i) {
		pairCount += world.taskContexts[i].pairKeys.length;
	}

	if (bp.pairKeyBuffer.length < pairCount) {
		bp.pairKeyBuffer = new StaticArray<u64>(b3MaxInt(pairCount, 2 * bp.pairKeyBuffer.length));
	}
	let pairKeys = bp.pairKeyBuffer;
	let keyCount = 0;
	for (let i = 0; i < world.workerCount; ++i) {
		let workerKeys = world.taskContexts[i].pairKeys;
		let workerCount = workerKeys.length;
		for (let k = 0; k < workerCount; ++k) {
			pairKeys[keyCount + k] = workerKeys[k];
		}
		keyCount += workerCount;
	}

	B3_ASSERT(keyCount == pairCount);

	if (pairCount > 1) {
		s_pairKeySorter.pairKeys = pairKeys;
		Q_LOOP<b3PairKeySorter>(pairCount, s_pairKeySorter);
	}

	for (let i = 0; i < keyCount; ++i) {
		let key = pairKeys[i];

		// The traversal is cross subtree and the seeds are disjoint, so a duplicate key means a
		// real bug. Trap it in a validation build, and skip rather than double add to the pair
		// set, which a later destroy would then remove once.
		let duplicate = i > 0 && key == pairKeys[i - 1];
		if (duplicate) {
			continue;
		}

		let shapeIdA = <i32>((key >> <u64>(64 - B3_SHAPE_POWER)) & <u64>B3_SHAPE_MASK);
		let shapeIdB = <i32>((key >> <u64>(64 - 2 * B3_SHAPE_POWER)) & <u64>B3_SHAPE_MASK);
		let childIndex = <i32>(key & <u64>B3_CHILD_MASK);

		let shapeA = world.shapes[shapeIdA];
		let shapeB = world.shapes[shapeIdB];

		b3CreateContact(world, shapeA, shapeB, childIndex);
	}

	bp.movedSiblings = b3_emptySiblings;
}

export function b3BroadPhase_TestOverlap(bp: b3BroadPhase, proxyKeyA: i32, proxyKeyB: i32): bool {
	let typeIndexA = B3_PROXY_TYPE(proxyKeyA);
	let proxyIdA = B3_PROXY_ID(proxyKeyA);
	let typeIndexB = B3_PROXY_TYPE(proxyKeyB);
	let proxyIdB = B3_PROXY_ID(proxyKeyB);

	let aabbA = b3DynamicTree_GetAABB(bp.trees[typeIndexA], proxyIdA);
	let aabbB = b3DynamicTree_GetAABB(bp.trees[typeIndexB], proxyIdB);
	return b3AABB_Overlaps(aabbA, aabbB);
}

export function b3BroadPhase_GetShapeIndex(bp: b3BroadPhase, proxyKey: i32): i32 {
	let typeIndex = B3_PROXY_TYPE(proxyKey);
	let proxyId = B3_PROXY_ID(proxyKey);

	return <i32>b3DynamicTree_GetUserData(bp.trees[typeIndex], proxyId);
}

export function b3ValidateBroadPhase(bp: b3BroadPhase): void {
	b3DynamicTree_Validate(bp.trees[b3_dynamicBody]);
	b3DynamicTree_Validate(bp.trees[b3_kinematicBody]);

	// todo validate every shape AABB is contained in tree AABB
}

export function b3ValidateNoMoved(bp: b3BroadPhase): void {
	if (B3_ENABLE_VALIDATION) {
		for (let j = 0; j < b3_bodyTypeCount; ++j) {
			let tree = bp.trees[j];
			b3DynamicTree_ValidateNoMoved(tree);
		}
	}
}
