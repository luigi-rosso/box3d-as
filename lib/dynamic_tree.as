// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/dynamic_tree.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_ENABLE_VALIDATION, B3_NULL_INDEX, FLT_MAX, b3OnMain } from "./core";
import {
	b3AABB,
	b3AABB_Center,
	b3AABB_Contains,
	b3AABB_Extents,
	b3AABB_Union,
	b3Add,
	b3Clamp,
	b3DistanceSquared,
	b3Dot,
	b3IsValidAABB,
	b3LengthSquared,
	b3Max,
	b3MaxInt,
	b3Min,
	b3MinFloat,
	b3MulAdd,
	b3MulSV,
	b3Sub,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import {
	B3_DYNAMIC_TREE_VERSION,
	b3BoxCastInput,
	b3DynamicTree,
	b3RayCastInput,
	b3TreeBoxCastCallbackFcn,
	b3TreeNode,
	b3TreeProxy,
	b3TreeQueryCallbackFcn,
	b3TreeQueryClosestCallbackFcn,
	b3TreeRayCastCallbackFcn,
	b3TreeStats,
} from "./collision";
import { b3EnlargeAABB, b3Perimeter } from "./aabb";
import {
	b3AddV,
	b3LoadAABBV,
	b3LoadV,
	b3OverlapNode,
	b3StoreAABBV,
	b3SubV,
	b3TestBoundsRayOverlap,
	b3UnionPairV,
	b3UnionV,
} from "./simd";

// dynamic_tree.h

export const B3_TREE_STACK_SIZE: i32 = 512;

// Used to mark a node as being moved such that pairs need to be generated and the tree
// may need to be rebuilt. This mark comes from a few sources:
// - updating the transform and AABB in the solver from movement
// - a joint disabling collision
// - creating a proxy
// - setting the transform on a body

export const B3_MOVED_NODE: u32 = 1 << 30;
export const B3_LEAF_NODE: u32 = 1 << 31;
export const B3_NODE_INDEX_MASK: u32 = 0xffffffff & ~(B3_MOVED_NODE | B3_LEAF_NODE);

// Used to indicate empty nodes. It also passes as a leaf so that internal node
// processing will naturally skip it.
export const B3_EMPTY_NODE: u32 = B3_NODE_INDEX_MASK | B3_LEAF_NODE;

export const B3_ROOT_NODE: i32 = 0;

// Nodes are taken by value; C takes a pointer.
@inline export function b3IsLeaf(node: b3TreeNode): bool {
	return (node.flagIndex & B3_LEAF_NODE) == B3_LEAF_NODE;
}

@inline export function b3IsNodeMoved(node: b3TreeNode): bool {
	return (node.flagIndex & B3_MOVED_NODE) == B3_MOVED_NODE;
}

@inline export function b3IsEmptyNode(node: b3TreeNode): bool {
	return node.flagIndex == B3_EMPTY_NODE;
}

@inline export function b3GetLeftChild(node: b3TreeNode): i32 {
	return <i32>(node.flagIndex & B3_NODE_INDEX_MASK);
}

@inline export function b3GetProxyId(node: b3TreeNode): i32 {
	return <i32>(node.flagIndex & B3_NODE_INDEX_MASK);
}

@inline export function b3GetRootPair(nodes: StaticArray<b3TreeNode>): i32 {
	let flags = nodes[B3_ROOT_NODE].flagIndex;
	return (flags & B3_LEAF_NODE) == B3_LEAF_NODE ? B3_ROOT_NODE : <i32>(flags & B3_NODE_INDEX_MASK);
}

// The union is two fields here, so the one a node does not use stays zero.
export function b3MakeEmptyNode(): b3TreeNode {
	let node = b3TreeNode();
	node.aabb.lowerBound = b3Vec3(Infinity, Infinity, Infinity);
	node.aabb.upperBound = b3Vec3(-Infinity, -Infinity, -Infinity);
	node.flagIndex = B3_EMPTY_NODE;
	node.height = 0;
	return node;
}

// proxies is null for a tree without proxies, such as a compound tree.
export class b3TreeView {
	nodes: StaticArray<b3TreeNode>;
	proxies: StaticArray<b3TreeProxy>?;

	constructor(nodes: StaticArray<b3TreeNode>, proxies: StaticArray<b3TreeProxy>?) {
		this.nodes = nodes;
		this.proxies = proxies;
	}
}

export function b3MakeTreeView(tree: b3DynamicTree): b3TreeView {
	return new b3TreeView(tree.nodes, tree.proxies);
}

@inline export function b3HasTreeMoved(tree: b3DynamicTree): bool {
	return (tree.nodes[B3_ROOT_NODE].flagIndex & B3_MOVED_NODE) == B3_MOVED_NODE;
}

@inline export function b3NeedsRebuild(tree: b3DynamicTree): bool {
	return (tree.nodes[B3_ROOT_NODE].flagIndex & B3_MOVED_NODE) == B3_MOVED_NODE || tree.dfsOrdered == false;
}

// dynamic_tree.c

// sizeof( b3DynamicTree ) on a 64 bit target, for b3DynamicTree_GetByteCount
const B3_DYNAMIC_TREE_BYTES: i32 = 112;
const B3_TREE_NODE_BYTES: i32 = 32;
const B3_TREE_PROXY_BYTES: i32 = 24;
const B3_AABB_BYTES: i32 = 24;
const B3_VEC3_BYTES: i32 = 12;

// Validation stays off as in C; -D B3_VALIDATE_TREE turns it on with -D B3_ENABLE_ASSERT.
const B3_TREE_VALIDATION: bool = B3_ENABLE_VALIDATION || isDefined(B3_VALIDATE_TREE);

function b3CopyArray<A>(dst: A, src: A, count: i32): void {
	for (let i = 0; i < count; ++i) {
		dst[i] = src[i];
	}
}

// The free sibling pair list is chained through the parent index of their first node. A pair comes from the
// free list when there is one and from the bump pointer otherwise. A rebuild empties the free list, so
// allocations can just bump.
function b3AllocateSiblingPair(tree: b3DynamicTree): i32 {
	// Allocate from free list.
	if (tree.pairFreeList != B3_NULL_INDEX) {
		let pair = tree.pairFreeList;
		tree.pairFreeList = tree.parents[pair];
		return pair;
	}

	// Extend the bump allocator if needed.
	if (tree.nodeEnd + 2 > tree.nodeCapacity) {
		let oldCapacity = tree.nodeCapacity;
		let newCapacity = oldCapacity + (oldCapacity >> 1);
		newCapacity += newCapacity & 1;
		let nodes = new StaticArray<b3TreeNode>(newCapacity);
		b3CopyArray(nodes, tree.nodes, oldCapacity);
		tree.nodes = nodes;
		let parents = new StaticArray<i32>(newCapacity);
		b3CopyArray(parents, tree.parents, oldCapacity);
		tree.parents = parents;
		tree.nodeCapacity = newCapacity;

		// The spare has to match, the rebuild allocates it again.
		tree.swapNodes = new StaticArray<b3TreeNode>(0);
	}

	let pair = tree.nodeEnd;
	tree.nodeEnd += 2;
	return pair;
}

function b3FreePair(tree: b3DynamicTree, pair: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd);
	tree.nodes[pair] = b3MakeEmptyNode();
	tree.nodes[pair + 1] = b3MakeEmptyNode();
	tree.parents[pair] = tree.pairFreeList;
	tree.parents[pair + 1] = B3_NULL_INDEX;
	tree.pairFreeList = pair;
}

/// Constructing the tree initializes the node pool.
export function b3DynamicTree_Create(proxyCapacity: i32): b3DynamicTree {
	let capacity = b3MaxInt(proxyCapacity, 16);

	let tree = new b3DynamicTree();

	tree.version = B3_DYNAMIC_TREE_VERSION;

	// A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
	tree.nodeCapacity = 2 * capacity;
	tree.nodes = new StaticArray<b3TreeNode>(tree.nodeCapacity);
	tree.parents = new StaticArray<i32>(tree.nodeCapacity);
	tree.pairFreeList = B3_NULL_INDEX;

	// The root and the empty node always exist
	tree.nodes[B3_ROOT_NODE] = b3MakeEmptyNode();
	tree.nodes[B3_ROOT_NODE + 1] = b3MakeEmptyNode();
	tree.parents[B3_ROOT_NODE] = B3_NULL_INDEX;
	tree.parents[B3_ROOT_NODE + 1] = B3_NULL_INDEX;

	// This is the bump index.
	tree.nodeEnd = 2;
	tree.dfsOrdered = true;

	tree.proxyCapacity = capacity;
	tree.proxyCount = 0;
	tree.proxies = new StaticArray<b3TreeProxy>(tree.proxyCapacity);

	// Build a linked list for the free list.
	let proxies = tree.proxies;
	for (let i = 0; i < tree.proxyCapacity - 1; ++i) {
		proxies[i].node = B3_NULL_INDEX;
		proxies[i].next = i + 1;
	}

	proxies[tree.proxyCapacity - 1].node = B3_NULL_INDEX;
	proxies[tree.proxyCapacity - 1].next = B3_NULL_INDEX;
	tree.proxyFreeList = 0;

	tree.rebuildCapacity = 0;

	return tree;
}

/// Destroy the tree, freeing the node pool.
export function b3DynamicTree_Destroy(tree: b3DynamicTree): void {
	tree.version = 0;
	tree.nodes = new StaticArray<b3TreeNode>(0);
	tree.parents = new StaticArray<i32>(0);
	tree.proxies = new StaticArray<b3TreeProxy>(0);
	tree.nodeEnd = 0;
	tree.nodeCapacity = 0;
	tree.pairFreeList = 0;
	tree.proxyCount = 0;
	tree.proxyCapacity = 0;
	tree.proxyFreeList = 0;
	tree.swapNodes = new StaticArray<b3TreeNode>(0);
	tree.leafIndices = new StaticArray<i32>(0);
	tree.leafNodes = new StaticArray<b3TreeNode>(0);
	tree.leafBoxes = new StaticArray<b3AABB>(0);
	tree.leafCenters = new StaticArray<b3Vec3>(0);
	tree.binIndices = new StaticArray<i32>(0);
	tree.rebuildCapacity = 0;
	tree.dfsOrdered = false;
}

// Allocate a proxy from the pool. Grow the pool if necessary.
function b3AllocateProxy(tree: b3DynamicTree): i32 {
	// Expand the pool as needed.
	if (tree.proxyFreeList == B3_NULL_INDEX) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(tree.proxyCount == tree.proxyCapacity);

		// The free list is empty. Rebuild a bigger pool.
		// A restored pool can have capacity 1, so guarantee strict growth.
		let oldCapacity = tree.proxyCapacity;
		tree.proxyCapacity += b3MaxInt(oldCapacity >> 1, 1);
		let grown = new StaticArray<b3TreeProxy>(tree.proxyCapacity);
		b3CopyArray(grown, tree.proxies, oldCapacity);
		tree.proxies = grown;

		// Build a linked list for the free list.
		for (let i = oldCapacity; i < tree.proxyCapacity - 1; ++i) {
			grown[i].node = B3_NULL_INDEX;
			grown[i].next = i + 1;
		}

		grown[tree.proxyCapacity - 1].node = B3_NULL_INDEX;
		grown[tree.proxyCapacity - 1].next = B3_NULL_INDEX;
		tree.proxyFreeList = oldCapacity;
	}

	// Peel a proxy off the free list.
	let proxyIndex = tree.proxyFreeList;
	ref proxy = tree.proxies[proxyIndex];
	tree.proxyFreeList = proxy.next;
	proxy = b3TreeProxy();
	proxy.node = B3_NULL_INDEX;
	proxy.next = B3_NULL_INDEX;
	++tree.proxyCount;
	return proxyIndex;
}

// Return a proxy to the pool.
function b3FreeProxy(tree: b3DynamicTree, proxyId: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 < tree.proxyCount);
	tree.proxies[proxyId].node = B3_NULL_INDEX;
	tree.proxies[proxyId].next = tree.proxyFreeList;
	tree.proxyFreeList = proxyId;
	--tree.proxyCount;
}

@inline function b3GetNodeHeight(node: b3TreeNode): i32 {
	return b3IsLeaf(node) ? 0 : node.height;
}

// The internal node above a children pair
function b3MakeInternalNode(nodes: StaticArray<b3TreeNode>, pair: i32): b3TreeNode {
	let flags1 = nodes[pair].flagIndex;
	let flags2 = nodes[pair + 1].flagIndex;
	let height1 = (flags1 & B3_LEAF_NODE) == B3_LEAF_NODE ? 0 : nodes[pair].height;
	let height2 = (flags2 & B3_LEAF_NODE) == B3_LEAF_NODE ? 0 : nodes[pair + 1].height;

	let node = b3TreeNode();
	node.aabb = b3UnionPairV(nodes, pair);
	node.flagIndex = <u32>pair | ((flags1 | flags2) & B3_MOVED_NODE);
	node.height = 1 + b3MaxInt(height1, height2);
	return node;
}

function b3MakeLeafNode(aabb: b3AABB, proxyId: i32, userData: u64, moved: bool): b3TreeNode {
	let node = b3TreeNode();
	node.aabb = aabb;
	node.flagIndex = <u32>proxyId | B3_LEAF_NODE | (moved ? B3_MOVED_NODE : 0);
	node.shapeIndex = <i32><u32>userData;
	return node;
}

// A node landed at a new index, so tell what hangs below it
@inline function b3LinkChildren(tree: b3DynamicTree, nodeIndex: i32): void {
	let flags = tree.nodes[nodeIndex].flagIndex;
	if ((flags & B3_LEAF_NODE) == B3_LEAF_NODE) {
		tree.proxies[<i32>(flags & B3_NODE_INDEX_MASK)].node = nodeIndex;
	} else {
		let pair = <i32>(flags & B3_NODE_INDEX_MASK);
		tree.parents[pair] = nodeIndex;
		tree.parents[pair + 1] = nodeIndex;
	}
}

// The sweep refit visits indices from high to low, so it needs every child above its parent
@inline function b3IsNodeOrdered(nodes: StaticArray<b3TreeNode>, nodeIndex: i32): bool {
	let flags = nodes[nodeIndex].flagIndex;
	return (flags & B3_LEAF_NODE) == B3_LEAF_NODE || nodeIndex < <i32>(flags & B3_NODE_INDEX_MASK);
}

// Greedy algorithm for sibling selection using the SAH
// We have three nodes A-(B,C) and want to add a leaf D, there are three choices.
// 1: make a new parent for A and D : E-(A-(B,C), D)
// 2: associate D with B
//   a: B is a leaf : A-(E-(B,D), C)
//   b: B is an internal node: A-(B{D},C)
// 3: associate D with C
//   a: C is a leaf : A-(B, E-(C,D))
//   b: C is an internal node: A-(B, C{D})
// All of these have a clear cost except when B or C is an internal node. Hence we need to be greedy.

// The cost for cases 1, 2a, and 3a can be computed using the sibling cost formula.
// cost of sibling H = area(union(H, D)) + increased area of ancestors

// Suppose B (or C) is an internal node, then the lowest cost would be one of two cases:
// case1: D becomes a sibling of B
// case2: D becomes a descendant of B along with a new internal node of area(D).
function b3FindBestSibling(tree: b3DynamicTree, boxD: b3AABB): i32 {
	let centerD = b3AABB_Center(boxD);
	let areaD = b3Perimeter(boxD);

	let nodes = tree.nodes;
	let nodeIndex = B3_ROOT_NODE;

	if (b3IsLeaf(nodes[nodeIndex])) {
		return nodeIndex;
	}

	let rootBox = nodes[nodeIndex].aabb;

	// Area of current node
	let areaBase = b3Perimeter(rootBox);

	// Area of inflated node
	let directCost = b3Perimeter(b3AABB_Union(rootBox, boxD));
	let inheritedCost: f32 = 0.0;

	let bestSibling = nodeIndex;
	let bestCost = directCost;

	// Descend the tree, following a single greedy path.
	for (;;) {
		let child1 = b3GetLeftChild(nodes[nodeIndex]);
		let child2 = child1 + 1;

		// Cost of creating a new parent for this node and the new leaf
		let cost = directCost + inheritedCost;

		// Sometimes there are multiple identical costs within tolerance.
		// This breaks the ties using the centroid distance.
		if (cost < bestCost) {
			bestSibling = nodeIndex;
			bestCost = cost;
		}

		// Inheritance cost seen by children
		inheritedCost += directCost - areaBase;

		let leaf1 = (nodes[child1].flagIndex & B3_LEAF_NODE) == B3_LEAF_NODE;
		let leaf2 = (nodes[child2].flagIndex & B3_LEAF_NODE) == B3_LEAF_NODE;

		// Cost of descending into child 1
		let lowerCost1 = FLT_MAX;
		let box1 = nodes[child1].aabb;
		let directCost1 = b3Perimeter(b3AABB_Union(box1, boxD));
		let area1: f32 = 0.0;
		if (leaf1) {
			// Child 1 is a leaf
			// Cost of creating new node and increasing area of node P
			let cost1 = directCost1 + inheritedCost;
			if (cost1 < bestCost) {
				bestSibling = child1;
				bestCost = cost1;
			}
		} else {
			// Child 1 is an internal node
			area1 = b3Perimeter(box1);

			// Lower bound cost of inserting under child 1. The minimum accounts for two possibilities:
			// 1. Child1 could be the sibling with cost1 = inheritedCost + directCost1
			// 2. A descendant of child1 could be the sibling with the lower bound cost of
			//       cost1 = inheritedCost + (directCost1 - area1) + areaD
			// This minimum here leads to the minimum of these two costs.
			lowerCost1 = inheritedCost + directCost1 + b3MinFloat(areaD - area1, 0.0);
		}

		// Cost of descending into child 2
		let lowerCost2 = FLT_MAX;
		let box2 = nodes[child2].aabb;
		let directCost2 = b3Perimeter(b3AABB_Union(box2, boxD));
		let area2: f32 = 0.0;
		if (leaf2) {
			let cost2 = directCost2 + inheritedCost;
			if (cost2 < bestCost) {
				bestSibling = child2;
				bestCost = cost2;
			}
		} else {
			area2 = b3Perimeter(box2);
			lowerCost2 = inheritedCost + directCost2 + b3MinFloat(areaD - area2, 0.0);
		}

		if (leaf1 && leaf2) {
			break;
		}

		// Can the cost possibly be decreased?
		if (bestCost <= lowerCost1 && bestCost <= lowerCost2) {
			break;
		}

		if (lowerCost1 == lowerCost2 && leaf1 == false) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(lowerCost1 < FLT_MAX);
			if (B3_ASSERT_ENABLED) B3_ASSERT(lowerCost2 < FLT_MAX);

			// No clear choice based on lower bound surface area. This can happen when both
			// children fully contain D. Fall back to node distance.
			let d1 = b3Sub(b3AABB_Center(box1), centerD);
			let d2 = b3Sub(b3AABB_Center(box2), centerD);
			lowerCost1 = b3LengthSquared(d1);
			lowerCost2 = b3LengthSquared(d2);
		}

		// Descend
		if (lowerCost1 < lowerCost2 && leaf1 == false) {
			nodeIndex = child1;
			areaBase = area1;
			directCost = directCost1;
		} else {
			nodeIndex = child2;
			areaBase = area2;
			directCost = directCost2;
		}
	}

	return bestSibling;
}

// Swap a child of A with a grandchild under the other child.
// Example swapping B with G:
// (A (B (D E) C (F G)) -> (A (G) C (F B (D E))
function b3SwapNodes(tree: b3DynamicTree, iDown: i32, iUp: i32): void {
	let nodes = tree.nodes;
	let temp = nodes[iDown];
	nodes[iDown] = nodes[iUp];
	nodes[iUp] = temp;
	b3LinkChildren(tree, iDown);
	b3LinkChildren(tree, iUp);

	if (b3IsNodeOrdered(nodes, iDown) == false || b3IsNodeOrdered(nodes, iUp) == false) {
		tree.dfsOrdered = false;
	}

	// The sibling of the node that went down holds a different subtree now
	let iC = iDown ^ 1;
	nodes[iC] = b3MakeInternalNode(nodes, b3GetLeftChild(nodes[iC]));
}

// Perform a left or right rotation if node A is imbalanced.
// Tree: (A (B (D E) C (F G))
function b3RotateNodes(tree: b3DynamicTree, iA: i32): void {
	let nodes = tree.nodes;
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsLeaf(nodes[iA]) == false);

	let iB = b3GetLeftChild(nodes[iA]);
	let iC = iB + 1;
	ref B = nodes[iB];
	ref C = nodes[iC];

	let leafB = b3IsLeaf(B);
	let leafC = b3IsLeaf(C);
	if (leafB && leafC) {
		return;
	}

	let bestDown = B3_NULL_INDEX;
	let bestUp = B3_NULL_INDEX;
	let bestDelta: f32 = 0.0;

	if (leafC == false) {
		// Swap B with F or G
		let iF = b3GetLeftChild(C);
		let iG = iF + 1;
		let areaC = b3Perimeter(C.aabb);

		// B <-> F then C (B G)
		let deltaBF = b3Perimeter(b3UnionV(B.aabb, nodes[iG].aabb)) - areaC;
		if (deltaBF < bestDelta) {
			bestDown = iB;
			bestUp = iF;
			bestDelta = deltaBF;
		}

		// B <-> G then C (F B)
		let deltaBG = b3Perimeter(b3UnionV(B.aabb, nodes[iF].aabb)) - areaC;
		if (deltaBG < bestDelta) {
			bestDown = iB;
			bestUp = iG;
			bestDelta = deltaBG;
		}
	}

	if (leafB == false) {
		// Swap C with D or E
		let iD = b3GetLeftChild(B);
		let iE = iD + 1;
		let areaB = b3Perimeter(B.aabb);

		// C <-> D then B (C E)
		let deltaCD = b3Perimeter(b3UnionV(C.aabb, nodes[iE].aabb)) - areaB;
		if (deltaCD < bestDelta) {
			bestDown = iC;
			bestUp = iD;
			bestDelta = deltaCD;
		}

		// C <-> E then B (D C)
		let deltaCE = b3Perimeter(b3UnionV(C.aabb, nodes[iD].aabb)) - areaB;
		if (deltaCE < bestDelta) {
			bestDown = iC;
			bestUp = iE;
			bestDelta = deltaCE;
		}
	}

	if (bestDown != B3_NULL_INDEX) {
		b3SwapNodes(tree, bestDown, bestUp);
	}
}

function b3InsertLeaf(tree: b3DynamicTree, aabb: b3AABB, proxyId: i32, moved: bool, shouldRotate: bool): void {
	let leaf = b3MakeLeafNode(aabb, proxyId, tree.proxies[proxyId].userData, moved);

	if (b3IsEmptyNode(tree.nodes[B3_ROOT_NODE])) {
		tree.nodes[B3_ROOT_NODE] = leaf;
		tree.proxies[proxyId].node = B3_ROOT_NODE;
		return;
	}

	let sibling = b3FindBestSibling(tree, aabb);

	// The sibling's position becomes the new parent. The sibling moves down into a new pair
	// beside the leaf, which puts it below its own children when it is internal.
	let pair = b3AllocateSiblingPair(tree);
	let nodes = tree.nodes;
	let parents = tree.parents;

	nodes[pair] = nodes[sibling];
	nodes[pair + 1] = leaf;
	parents[pair] = sibling;
	parents[pair + 1] = sibling;
	b3LinkChildren(tree, pair);
	tree.proxies[proxyId].node = pair + 1;

	nodes[sibling] = b3MakeInternalNode(nodes, pair);

	if (b3IsNodeOrdered(nodes, sibling) == false || b3IsNodeOrdered(nodes, pair) == false) {
		tree.dfsOrdered = false;
	}

	// Walk back up the tree refitting ancestors, the root included.
	let index = sibling;
	while (index != B3_NULL_INDEX) {
		if (shouldRotate) {
			b3RotateNodes(tree, index);
		}

		nodes[index] = b3MakeInternalNode(nodes, b3GetLeftChild(nodes[index]));
		index = parents[index];
	}
}

function b3RemoveLeaf(tree: b3DynamicTree, proxyId: i32): void {
	let nodes = tree.nodes;
	let parents = tree.parents;

	let leaf = tree.proxies[proxyId].node;
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= leaf && leaf < tree.nodeEnd);
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsLeaf(nodes[leaf]) && b3GetProxyId(nodes[leaf]) == proxyId);

	if (leaf == B3_ROOT_NODE) {
		nodes[B3_ROOT_NODE] = b3MakeEmptyNode();
		return;
	}

	// The sibling takes the parent's position and the pair is freed. This keeps the order,
	// the sibling only moves down.
	let parent = parents[leaf];
	nodes[parent] = nodes[leaf ^ 1];
	b3LinkChildren(tree, parent);
	b3FreePair(tree, leaf & ~1);

	// Update ancestors.
	let index = parents[parent];
	while (index != B3_NULL_INDEX) {
		nodes[index] = b3MakeInternalNode(nodes, b3GetLeftChild(nodes[index]));
		index = parents[index];
	}
}

// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can grow
// the node pool.
export function b3CreateTreeProxyInternal(
	tree: b3DynamicTree,
	aabb: b3AABB,
	categoryBits: u64,
	userData: u64,
	markMoved: bool,
): i32 {
	let proxyId = b3AllocateProxy(tree);

	ref proxy = tree.proxies[proxyId];
	proxy.categoryBits = categoryBits;
	proxy.userData = userData;

	let shouldRotate = true;
	b3InsertLeaf(tree, aabb, proxyId, markMoved, shouldRotate);

	return proxyId;
}

/// Create a proxy. Provide an AABB and a userData value.
export function b3DynamicTree_CreateProxy(tree: b3DynamicTree, aabb: b3AABB, categoryBits: u64, userData: u64): i32 {
	return b3CreateTreeProxyInternal(tree, aabb, categoryBits, userData, false);
}

/// Destroy a proxy. This asserts if the id is invalid.
export function b3DynamicTree_DestroyProxy(tree: b3DynamicTree, proxyId: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);

	b3RemoveLeaf(tree, proxyId);
	b3FreeProxy(tree, proxyId);
}

/// Get the number of proxies created
export function b3DynamicTree_GetProxyCount(tree: b3DynamicTree): i32 {
	return tree.proxyCount;
}

export function b3DynamicTree_MoveProxyInternal(tree: b3DynamicTree, proxyId: i32, aabb: b3AABB, markMoved: bool): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);

	b3RemoveLeaf(tree, proxyId);

	let shouldRotate = false;
	b3InsertLeaf(tree, aabb, proxyId, markMoved, shouldRotate);
}

/// Move a proxy to a new AABB by removing and reinserting into the tree.
export function b3DynamicTree_MoveProxy(tree: b3DynamicTree, proxyId: i32, aabb: b3AABB): void {
	b3DynamicTree_MoveProxyInternal(tree, proxyId, aabb, false);
}

/// Enlarge a proxy and enlarge ancestors as necessary.
export function b3DynamicTree_EnlargeProxy(tree: b3DynamicTree, proxyId: i32, aabb: b3AABB): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);

	let nodes = tree.nodes;
	let parents = tree.parents;

	let index = tree.proxies[proxyId].node;
	ref node = nodes[index];
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsLeaf(node));

	node.aabb = aabb;
	node.flagIndex |= B3_MOVED_NODE;

	index = parents[index];
	while (index != B3_NULL_INDEX) {
		ref ancestor = nodes[index];
		let changed = b3EnlargeAABB(ancestor.aabb, aabb);

		// This is marked to ensure the root is marked in this loop or the one below.
		ancestor.flagIndex |= B3_MOVED_NODE;

		index = parents[index];

		if (changed == false) {
			break;
		}
	}

	// Mark all the way up to the root.
	while (index != B3_NULL_INDEX) {
		ref ancestor = nodes[index];
		if (ancestor.flagIndex & B3_MOVED_NODE) {
			// Early out because this ancestor was previously ascended and marked as moved.
			break;
		}

		ancestor.flagIndex |= B3_MOVED_NODE;
		index = parents[index];
	}
}

/// Modify the category bits on a proxy. This is an expensive operation.
export function b3DynamicTree_SetCategoryBits(tree: b3DynamicTree, proxyId: i32, categoryBits: u64): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	tree.proxies[proxyId].categoryBits = categoryBits;
}

/// Get the category bits on a proxy.
export function b3DynamicTree_GetCategoryBits(tree: b3DynamicTree, proxyId: i32): u64 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
	return tree.proxies[proxyId].categoryBits;
}

/// Get the height of the binary tree.
export function b3DynamicTree_GetHeight(tree: b3DynamicTree): i32 {
	return b3GetNodeHeight(tree.nodes[B3_ROOT_NODE]);
}

// The area ratio is the thing that SAH seeks to minimize. SAH
// cannot do anything about leaf boxes or the root box. It seeks
// to minimize the area of all non-root internal nodes. Divide this
// by the root area to make the metric non-dimensional.
// So this becomes a meaningful measure of tree quality.
/// Get the ratio of the sum of the node areas to the root area.
export function b3DynamicTree_GetAreaRatio(tree: b3DynamicTree): f32 {
	if (tree.proxyCount == 0) {
		return 0.0;
	}

	let nodes = tree.nodes;
	let rootArea = b3Perimeter(nodes[B3_ROOT_NODE].aabb);
	if (rootArea <= 0.0) {
		return 0.0;
	}

	// Free nodes and the empty node are leaf tagged.
	let internalArea: f32 = 0.0;
	let nodeEnd = tree.nodeEnd;
	for (let i = 2; i < nodeEnd; ++i) {
		if (b3IsLeaf(nodes[i]) == false) {
			internalArea += b3Perimeter(nodes[i].aabb);
		}
	}

	return internalArea / rootArea;
}

/// Get the bounding box that contains the entire tree
export function b3DynamicTree_GetRootBounds(tree: b3DynamicTree): b3AABB {
	if (tree.proxyCount == 0) {
		return b3AABB(b3Vec3_zero, b3Vec3_zero);
	}

	return tree.nodes[B3_ROOT_NODE].aabb;
}

function b3ValidateSubtree(tree: b3DynamicTree, nodeIndex: i32, leafCount: ref i32): i32 {
	B3_ASSERT(0 <= nodeIndex && nodeIndex < tree.nodeEnd);
	let node = tree.nodes[nodeIndex];
	B3_ASSERT(b3IsEmptyNode(node) == false);
	B3_ASSERT(b3IsValidAABB(node.aabb));

	if (b3IsLeaf(node)) {
		let proxyId = b3GetProxyId(node);
		B3_ASSERT(0 <= proxyId && proxyId < tree.proxyCapacity);
		B3_ASSERT(tree.proxies[proxyId].node == nodeIndex);
		B3_ASSERT(<i32><u32>tree.proxies[proxyId].userData == node.shapeIndex);
		B3_ASSERT(node.height == 0);
		leafCount += 1;
		return 0;
	}

	let pair = b3GetLeftChild(node);
	B3_ASSERT((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd);
	B3_ASSERT(tree.parents[pair] == nodeIndex);
	B3_ASSERT(tree.parents[pair + 1] == nodeIndex);
	B3_ASSERT(tree.dfsOrdered == false || nodeIndex < pair);
	B3_ASSERT(node.shapeIndex == 0);

	let c1 = tree.nodes[pair];
	let c2 = tree.nodes[pair + 1];
	B3_ASSERT(b3AABB_Contains(node.aabb, c1.aabb));
	B3_ASSERT(b3AABB_Contains(node.aabb, c2.aabb));
	B3_ASSERT(b3IsNodeMoved(node) == (b3IsNodeMoved(c1) || b3IsNodeMoved(c2)));

	// A bad tree can stack overflow, but that is validation on its own.
	let height1 = b3ValidateSubtree(tree, pair, leafCount);
	let height2 = b3ValidateSubtree(tree, pair + 1, leafCount);
	let height = 1 + b3MaxInt(height1, height2);
	B3_ASSERT(node.height == height);
	return height;
}

/// Validate this tree. For testing.
export function b3DynamicTree_Validate(tree: b3DynamicTree): void {
	if (B3_TREE_VALIDATION) {
		B3_ASSERT(2 <= tree.nodeEnd && tree.nodeEnd <= tree.nodeCapacity);
		B3_ASSERT((tree.nodeEnd & 1) == 0);
		B3_ASSERT(tree.parents[B3_ROOT_NODE] == B3_NULL_INDEX);
		B3_ASSERT(b3IsEmptyNode(tree.nodes[B3_ROOT_NODE + 1]));

		let freePairCount = 0;
		let pair = tree.pairFreeList;
		while (pair != B3_NULL_INDEX) {
			B3_ASSERT((pair & 1) == 0 && 2 <= pair && pair < tree.nodeEnd);
			B3_ASSERT(b3IsEmptyNode(tree.nodes[pair]));
			B3_ASSERT(b3IsEmptyNode(tree.nodes[pair + 1]));
			pair = tree.parents[pair];
			++freePairCount;
			B3_ASSERT(2 * freePairCount < tree.nodeEnd);
		}

		// Validate proxy free list.
		let freeProxyCount = 0;
		let freeIndex = tree.proxyFreeList;
		while (freeIndex != B3_NULL_INDEX) {
			B3_ASSERT(0 <= freeIndex && freeIndex < tree.proxyCapacity);
			B3_ASSERT(tree.proxies[freeIndex].node == B3_NULL_INDEX);
			freeIndex = tree.proxies[freeIndex].next;
			++freeProxyCount;
		}
		B3_ASSERT(tree.proxyCount + freeProxyCount == tree.proxyCapacity);

		B3_ASSERT(tree.nodeEnd == 2 * b3MaxInt(tree.proxyCount, 1) + 2 * freePairCount);

		if (tree.proxyCount == 0) {
			B3_ASSERT(b3IsEmptyNode(tree.nodes[B3_ROOT_NODE]));
			return;
		}

		// A ref binds a fixed array element but not a plain local
		let leafCount: FixedArray<i32, 1>;
		b3ValidateSubtree(tree, B3_ROOT_NODE, leafCount[0]);
		B3_ASSERT(leafCount[0] == tree.proxyCount);
	}
}

/// Validate this tree has no moved nodes. For testing.
export function b3DynamicTree_ValidateNoMoved(tree: b3DynamicTree): void {
	if (B3_TREE_VALIDATION) {
		let nodes = tree.nodes;
		let nodeEnd = tree.nodeEnd;
		for (let i = 0; i < nodeEnd; ++i) {
			B3_ASSERT(b3IsNodeMoved(nodes[i]) == false);
		}
	}
}

/// Get the number of bytes used by this tree
export function b3DynamicTree_GetByteCount(tree: b3DynamicTree): i32 {
	let size = B3_DYNAMIC_TREE_BYTES;
	size += tree.nodeCapacity * B3_TREE_NODE_BYTES;
	size += tree.nodeCapacity * sizeof<i32>();
	size += tree.proxyCapacity * B3_TREE_PROXY_BYTES;
	size += tree.swapNodes.length == 0 ? 0 : tree.nodeCapacity * B3_TREE_NODE_BYTES;

	// leafIndices
	size += tree.rebuildCapacity * sizeof<i32>();
	// leafNodes
	size += tree.rebuildCapacity * B3_TREE_NODE_BYTES;
	// leafBoxes
	size += tree.rebuildCapacity * B3_AABB_BYTES;
	// leafCenters
	size += tree.rebuildCapacity * B3_VEC3_BYTES;
	// binIndices
	size += tree.rebuildCapacity * sizeof<i32>();

	return size;
}

@inline function b3TestCategory(categoryBits: u64, maskBits: u64, requireAllBits: bool): bool {
	return requireAllBits ? (categoryBits & maskBits) == maskBits : (categoryBits & maskBits) != 0;
}

// C keeps the query stacks on the C stack. Here each nesting depth keeps a static one, since a query nested inside
// a callback (a ray reaching a mesh's own tree) runs once per candidate shape.
let b3_queryStack = new StaticArray<i32>(B3_TREE_STACK_SIZE);
let b3_nestedQueryStacks = new Array<StaticArray<i32>>();
let b3_queryDepth = 0;

function b3AcquireQueryStack(): StaticArray<i32> {
	b3_queryDepth += 1;
	if (b3_queryDepth == 1) {
		return b3_queryStack;
	}
	while (b3_nestedQueryStacks.length < b3_queryDepth - 1) {
		b3_nestedQueryStacks.push(new StaticArray<i32>(B3_TREE_STACK_SIZE));
	}
	return b3_nestedQueryStacks[b3_queryDepth - 2];
}

struct b3QueryClosestItem {
	nodeIndex: i32 = 0;
	distanceToNodeSqr: f32 = 0;

	constructor(nodeIndex: i32, distanceToNodeSqr: f32) {
		this.nodeIndex = nodeIndex;
		this.distanceToNodeSqr = distanceToNodeSqr;
	}
}

let b3_closestStack = new StaticArray<b3QueryClosestItem>(B3_TREE_STACK_SIZE);
let b3_closestDepth = 0;

function b3AcquireClosestStack(): StaticArray<b3QueryClosestItem> {
	b3_closestDepth += 1;
	return b3_closestDepth == 1 ? b3_closestStack : new StaticArray<b3QueryClosestItem>(B3_TREE_STACK_SIZE);
}

// V is the callback function type, or a visitor class whose visit method is called directly, as parallel parts need
function b3QueryTreeStack<V>(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	aabb: b3AABB,
	maskBits: u64,
	requireAllBits: bool,
	callback: V,
	hasProxies: bool,
	stack: StaticArray<i32>,
): b3TreeStats {
	let result = b3TreeStats();

	let stackCount = 0;
	stack[stackCount++] = b3GetRootPair(nodes);

	let boxv = b3LoadAABBV(aabb);

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		result.nodeVisits += 1;

		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];
			if (b3OverlapNode(boxv, node) == false) {
				continue;
			}

			let flags = node.flagIndex;
			if ((flags & B3_LEAF_NODE) == B3_LEAF_NODE) {
				let proxyId = <i32>(flags & B3_NODE_INDEX_MASK);
				let userData = <u64><u32>node.shapeIndex;
				if (hasProxies) {
					ref proxy = proxies[proxyId];
					if (b3TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false) {
						continue;
					}
					userData = proxy.userData;
				}

				let proceed: bool;
				if (isFunction<V>()) {
					proceed = callback(proxyId, userData);
				} else {
					proceed = callback.visit(proxyId, userData);
				}
				result.leafVisits += 1;

				if (proceed == false) {
					return result;
				}
			} else {
				if (stackCount < B3_TREE_STACK_SIZE - 1) {
					stack[stackCount++] = <i32>(flags & B3_NODE_INDEX_MASK);
				} else {
					if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount < B3_TREE_STACK_SIZE - 1);
				}
			}
		}
	}

	return result;
}

function b3QueryTreeView(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	aabb: b3AABB,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeQueryCallbackFcn,
	hasProxies: bool,
): b3TreeStats {
	if (b3IsEmptyNode(nodes[B3_ROOT_NODE])) {
		return b3TreeStats();
	}

	let stack = b3AcquireQueryStack();
	let result = b3QueryTreeStack<b3TreeQueryCallbackFcn>(nodes, proxies, aabb, maskBits, requireAllBits, callback, hasProxies, stack);
	b3_queryDepth -= 1;
	return result;
}

@inline function b3DistanceToNodeSqr(point: b3Vec3, node: b3TreeNode): f32 {
	let r = b3Sub(point, b3Clamp(point, node.aabb.lowerBound, node.aabb.upperBound));
	return b3Dot(r, r);
}

// minDistanceSqr carries C's in and out pointer value.
function b3QueryClosestTreeStack(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	point: b3Vec3,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeQueryClosestCallbackFcn,
	minDistanceSqr: ref f32,
	hasProxies: bool,
	stack: StaticArray<b3QueryClosestItem>,
): b3TreeStats {
	let result = b3TreeStats();

	let minSqr = minDistanceSqr;
	let stackCount = 0;

	let rootPair = b3GetRootPair(nodes);
	let seed1 = b3QueryClosestItem(rootPair, b3DistanceToNodeSqr(point, nodes[rootPair]));

	if (rootPair == B3_ROOT_NODE) {
		stack[stackCount++] = seed1;
	} else {
		let seed2 = b3QueryClosestItem(rootPair + 1, b3DistanceToNodeSqr(point, nodes[rootPair + 1]));

		// Ensure we iterate the closest child first as we pop off the stack
		if (seed2.distanceToNodeSqr < seed1.distanceToNodeSqr) {
			stack[stackCount++] = seed1;
			stack[stackCount++] = seed2;
		} else {
			stack[stackCount++] = seed2;
			stack[stackCount++] = seed1;
		}
	}

	while (stackCount > 0) {
		let item = stack[--stackCount];
		ref node = nodes[item.nodeIndex];
		result.nodeVisits += 1;

		if (item.distanceToNodeSqr >= minSqr) {
			continue;
		}

		let flags = node.flagIndex;
		if ((flags & B3_LEAF_NODE) == B3_LEAF_NODE) {
			let proxyId = <i32>(flags & B3_NODE_INDEX_MASK);
			let userData = <u64><u32>node.shapeIndex;
			if (hasProxies) {
				ref proxy = proxies[proxyId];
				if (b3TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false) {
					continue;
				}
				userData = proxy.userData;
			}

			// callback to user code with minimum distance squared so far and proxy id
			let dd = callback(minSqr, proxyId, userData);

			if (dd < minSqr) {
				minSqr = dd;
			}

			result.leafVisits += 1;
			continue;
		}

		if (stackCount + 2 > B3_TREE_STACK_SIZE) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount + 2 <= B3_TREE_STACK_SIZE);
			continue;
		}

		let pair = <i32>(flags & B3_NODE_INDEX_MASK);

		// Store the distance to node in the stack instead of recomputing after pop
		let item1 = b3QueryClosestItem(pair, b3DistanceToNodeSqr(point, nodes[pair]));
		let item2 = b3QueryClosestItem(pair + 1, b3DistanceToNodeSqr(point, nodes[pair + 1]));

		// Ensure we iterate the closest child first as we pop off the stack
		if (item2.distanceToNodeSqr < item1.distanceToNodeSqr) {
			stack[stackCount++] = item1;
			stack[stackCount++] = item2;
		} else {
			stack[stackCount++] = item2;
			stack[stackCount++] = item1;
		}
	}

	minDistanceSqr = minSqr;

	return result;
}

function b3QueryClosestTreeView(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	point: b3Vec3,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeQueryClosestCallbackFcn,
	minDistanceSqr: ref f32,
	hasProxies: bool,
): b3TreeStats {
	if (b3IsEmptyNode(nodes[B3_ROOT_NODE])) {
		return b3TreeStats();
	}

	let stack = b3AcquireClosestStack();
	let result = b3QueryClosestTreeStack(
		nodes,
		proxies,
		point,
		maskBits,
		requireAllBits,
		callback,
		minDistanceSqr,
		hasProxies,
		stack,
	);
	b3_closestDepth -= 1;
	return result;
}

// A lot of optimization work went into this. The children of a popped pair are both tested
// before either is followed, then the survivors are ordered so the closer one narrows first.
function b3RayCastTreeStack(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	input: b3RayCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeRayCastCallbackFcn,
	hasProxies: bool,
	stack: StaticArray<i32>,
): b3TreeStats {
	let result = b3TreeStats();

	let p1 = input.origin;
	let d = input.translation;

	let pv1 = b3LoadV(p1);
	let dv = b3LoadV(d);

	let maxFraction = input.maxFraction;

	let p2 = b3MulAdd(p1, maxFraction, d);

	// Build a bounding box for the segment.
	let segmentAABB = b3AABB(b3Min(p1, p2), b3Max(p1, p2));
	let boxv = b3LoadAABBV(segmentAABB);

	let stackCount = 0;
	stack[stackCount++] = b3GetRootPair(nodes);

	let subInput = input;

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		result.nodeVisits += 1;

		// C's hit pointers become node indices.
		let hit0 = 0;
		let hit1 = 0;
		let isLeaf0 = false;
		let isLeaf1 = false;
		let hitCount = 0;
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];

			if (b3OverlapNode(boxv, node) == false) {
				continue;
			}

			let lower = b3LoadV(node.aabb.lowerBound);
			let upper = b3LoadV(node.aabb.upperBound);
			if (b3TestBoundsRayOverlap(lower, upper, pv1, dv) == false) {
				continue;
			}

			if (hitCount == 0) {
				isLeaf0 = b3IsLeaf(node);
				hit0 = pair + i;
			} else {
				isLeaf1 = b3IsLeaf(node);
				hit1 = pair + i;
			}
			hitCount += 1;
		}

		if (hitCount == 2 && isLeaf0 == false && isLeaf1 == false) {
			let center1 = b3AABB_Center(nodes[hit0].aabb);
			let center2 = b3AABB_Center(nodes[hit1].aabb);
			let d1 = b3DistanceSquared(center1, p1);
			let d2 = b3DistanceSquared(center2, p1);

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if (d1 < d2) {
				let temp = hit0;
				hit0 = hit1;
				hit1 = temp;
			}
		}

		for (let i = 0; i < hitCount; ++i) {
			let hitIndex = i == 0 ? hit0 : hit1;
			let hitLeaf = i == 0 ? isLeaf0 : isLeaf1;
			if (hitLeaf) {
				ref hit = nodes[hitIndex];
				let proxyId = b3GetProxyId(hit);
				let userData = <u64><u32>hit.shapeIndex;
				if (hasProxies) {
					ref proxy = proxies[proxyId];
					if (b3TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false) {
						continue;
					}
					userData = proxy.userData;
				}

				subInput.maxFraction = maxFraction;

				let value = callback(subInput, proxyId, userData);
				result.leafVisits += 1;

				// The user may return -1 to indicate this shape should be skipped

				if (value == 0.0) {
					// The client has terminated the ray cast.
					return result;
				}

				if (0.0 < value && value <= maxFraction) {
					// Update segment bounding box.
					maxFraction = value;
					p2 = b3MulAdd(p1, maxFraction, d);
					segmentAABB.lowerBound = b3Min(p1, p2);
					segmentAABB.upperBound = b3Max(p1, p2);
					boxv = b3LoadAABBV(segmentAABB);
				}
			} else {
				if (stackCount < B3_TREE_STACK_SIZE - 1) {
					stack[stackCount++] = b3GetLeftChild(nodes[hitIndex]);
				} else {
					if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount < B3_TREE_STACK_SIZE - 1);
				}
			}
		}
	}

	return result;
}

function b3RayCastTreeView(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	input: b3RayCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeRayCastCallbackFcn,
	hasProxies: bool,
): b3TreeStats {
	if (b3IsEmptyNode(nodes[B3_ROOT_NODE])) {
		return b3TreeStats();
	}

	let stack = b3AcquireQueryStack();
	let result = b3RayCastTreeStack(nodes, proxies, input, maskBits, requireAllBits, callback, hasProxies, stack);
	b3_queryDepth -= 1;
	return result;
}

// Follows structure of ray cast with small tweaks to handle a swept box.
function b3BoxCastTreeStack(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	input: b3BoxCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeBoxCastCallbackFcn,
	hasProxies: bool,
	stack: StaticArray<i32>,
): b3TreeStats {
	let result = b3TreeStats();

	// The caller folds the shape radius and the world origin into the box
	let originAABB = input.box;

	let p1 = b3AABB_Center(originAABB);
	let extension = b3AABB_Extents(originAABB);

	let d = input.translation;

	let pv1 = b3LoadV(p1);
	let dv = b3LoadV(d);
	let ev = b3LoadV(extension);

	let maxFraction = input.maxFraction;

	// Build total box for the cast
	let t = b3MulSV(maxFraction, input.translation);
	let totalAABB = b3AABB(
		b3Min(originAABB.lowerBound, b3Add(originAABB.lowerBound, t)),
		b3Max(originAABB.upperBound, b3Add(originAABB.upperBound, t)),
	);
	let boxv = b3LoadAABBV(totalAABB);

	let stackCount = 0;
	stack[stackCount++] = b3GetRootPair(nodes);

	let subInput = input;

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		result.nodeVisits += 1;

		let hit0 = 0;
		let hit1 = 0;
		let isLeaf0 = false;
		let isLeaf1 = false;
		let hitCount = 0;
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];

			if (b3OverlapNode(boxv, node) == false) {
				continue;
			}

			// radius extension is added to the node in this case
			let lower = b3SubV(b3LoadV(node.aabb.lowerBound), ev);
			let upper = b3AddV(b3LoadV(node.aabb.upperBound), ev);
			if (b3TestBoundsRayOverlap(lower, upper, pv1, dv) == false) {
				continue;
			}

			if (hitCount == 0) {
				isLeaf0 = b3IsLeaf(node);
				hit0 = pair + i;
			} else {
				isLeaf1 = b3IsLeaf(node);
				hit1 = pair + i;
			}
			hitCount += 1;
		}

		if (hitCount == 2 && isLeaf0 == false && isLeaf1 == false) {
			let center1 = b3AABB_Center(nodes[hit0].aabb);
			let center2 = b3AABB_Center(nodes[hit1].aabb);
			let d1 = b3DistanceSquared(center1, p1);
			let d2 = b3DistanceSquared(center2, p1);

			// Want to push the closest one last. Both have the same isLeaf, so they don't swap.
			if (d1 < d2) {
				let temp = hit0;
				hit0 = hit1;
				hit1 = temp;
			}
		}

		for (let i = 0; i < hitCount; ++i) {
			let hitIndex = i == 0 ? hit0 : hit1;
			let hitLeaf = i == 0 ? isLeaf0 : isLeaf1;
			if (hitLeaf) {
				ref hit = nodes[hitIndex];
				let proxyId = b3GetProxyId(hit);
				let userData = <u64><u32>hit.shapeIndex;
				if (hasProxies) {
					ref proxy = proxies[proxyId];
					if (b3TestCategory(proxy.categoryBits, maskBits, requireAllBits) == false) {
						continue;
					}
					userData = proxy.userData;
				}

				subInput.maxFraction = maxFraction;

				let value = callback(subInput, proxyId, userData);
				result.leafVisits += 1;

				if (value == 0.0) {
					// The client has terminated the cast.
					return result;
				}

				if (0.0 < value && value < maxFraction) {
					maxFraction = value;
					t = b3MulSV(maxFraction, input.translation);
					totalAABB.lowerBound = b3Min(originAABB.lowerBound, b3Add(originAABB.lowerBound, t));
					totalAABB.upperBound = b3Max(originAABB.upperBound, b3Add(originAABB.upperBound, t));
					boxv = b3LoadAABBV(totalAABB);
				}
			} else {
				if (stackCount < B3_TREE_STACK_SIZE - 1) {
					stack[stackCount++] = b3GetLeftChild(nodes[hitIndex]);
				} else {
					if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount < B3_TREE_STACK_SIZE - 1);
				}
			}
		}
	}

	return result;
}

function b3BoxCastTreeView(
	nodes: StaticArray<b3TreeNode>,
	proxies: StaticArray<b3TreeProxy>,
	input: b3BoxCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeBoxCastCallbackFcn,
	hasProxies: bool,
): b3TreeStats {
	if (b3IsEmptyNode(nodes[B3_ROOT_NODE])) {
		return b3TreeStats();
	}

	let stack = b3AcquireQueryStack();
	let result = b3BoxCastTreeStack(nodes, proxies, input, maskBits, requireAllBits, callback, hasProxies, stack);
	b3_queryDepth -= 1;
	return result;
}

/// Query an AABB for overlapping proxies. The callback function is called for each proxy that overlaps the supplied AABB.
///	@return performance data
export function b3DynamicTree_Query(
	tree: b3DynamicTree,
	aabb: b3AABB,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeQueryCallbackFcn,
): b3TreeStats {
	if (tree.proxyCount == 0) {
		return b3TreeStats();
	}

	return b3QueryTreeView(tree.nodes, tree.proxies, aabb, maskBits, requireAllBits, callback, true);
}

/// b3DynamicTree_Query with a visitor whose visit(proxyId, userData) method is the callback, over the caller's stack of
/// B3_TREE_STACK_SIZE, which parallel parts can run.
export function b3DynamicTree_QueryVisitor<V>(
	tree: b3DynamicTree,
	aabb: b3AABB,
	maskBits: u64,
	requireAllBits: bool,
	visitor: V,
	stack: StaticArray<i32>,
): b3TreeStats {
	if (tree.proxyCount == 0 || b3IsEmptyNode(tree.nodes[B3_ROOT_NODE])) {
		return b3TreeStats();
	}

	return b3QueryTreeStack<V>(tree.nodes, tree.proxies, aabb, maskBits, requireAllBits, visitor, true, stack);
}

/// b3TreeView_Query of a view without proxies, such as a compound's, with a visitor and the caller's stack, which parallel
/// parts can run
export function b3QueryNodesVisitor<V>(
	nodes: StaticArray<b3TreeNode>,
	aabb: b3AABB,
	maskBits: u64,
	requireAllBits: bool,
	visitor: V,
	stack: StaticArray<i32>,
): b3TreeStats {
	if (b3IsEmptyNode(nodes[B3_ROOT_NODE])) {
		return b3TreeStats();
	}

	return b3QueryTreeStack<V>(nodes, b3_noProxies, aabb, maskBits, requireAllBits, visitor, false, stack);
}

/// Query an AABB for the closest object. The callback function is called for each proxy that might be closest to the supplied
/// point.
/// @param minDistanceSqr the initial and final minimum squared distance. Provide a small initial to restrict the search and
/// improve performance. If the value is large this query has performance that scales linearly with the number of proxies and
/// would be slower than a brute force search.
///	@return performance data
export function b3DynamicTree_QueryClosest(
	tree: b3DynamicTree,
	point: b3Vec3,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeQueryClosestCallbackFcn,
	minDistanceSqr: ref f32,
): b3TreeStats {
	if (tree.proxyCount == 0) {
		return b3TreeStats();
	}

	return b3QueryClosestTreeView(
		tree.nodes,
		tree.proxies,
		point,
		maskBits,
		requireAllBits,
		callback,
		minDistanceSqr,
		true,
	);
}

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform an exact ray cast in the case where the proxy contains a shape.
/// The callback also performs any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
///	@return performance data
export function b3DynamicTree_RayCast(
	tree: b3DynamicTree,
	input: b3RayCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeRayCastCallbackFcn,
): b3TreeStats {
	if (tree.proxyCount == 0) {
		return b3TreeStats();
	}

	return b3RayCastTreeView(tree.nodes, tree.proxies, input, maskBits, requireAllBits, callback, true);
}

/// Sweep an AABB through the tree. The box is in the tree's world float frame and the callback
/// re-differences each shape at full precision against the query origin. Used by the large world
/// spatial queries so the tree traversal stays float while the narrow phase stays precise.
export function b3DynamicTree_BoxCast(
	tree: b3DynamicTree,
	input: b3BoxCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeBoxCastCallbackFcn,
): b3TreeStats {
	if (tree.proxyCount == 0) {
		return b3TreeStats();
	}

	return b3BoxCastTreeView(tree.nodes, tree.proxies, input, maskBits, requireAllBits, callback, true);
}

const b3_noProxies = new StaticArray<b3TreeProxy>(0);

export function b3TreeView_Query(
	view: b3TreeView,
	aabb: b3AABB,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeQueryCallbackFcn,
): b3TreeStats {
	let proxies = view.proxies;
	if (proxies == null) {
		return b3QueryTreeView(view.nodes, b3_noProxies, aabb, maskBits, requireAllBits, callback, false);
	}

	return b3QueryTreeView(view.nodes, proxies, aabb, maskBits, requireAllBits, callback, true);
}

export function b3TreeView_QueryClosest(
	view: b3TreeView,
	point: b3Vec3,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeQueryClosestCallbackFcn,
	minDistanceSqr: ref f32,
): b3TreeStats {
	let proxies = view.proxies;
	if (proxies == null) {
		return b3QueryClosestTreeView(
			view.nodes,
			b3_noProxies,
			point,
			maskBits,
			requireAllBits,
			callback,
			minDistanceSqr,
			false,
		);
	}

	return b3QueryClosestTreeView(view.nodes, proxies, point, maskBits, requireAllBits, callback, minDistanceSqr, true);
}

export function b3TreeView_RayCast(
	view: b3TreeView,
	input: b3RayCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeRayCastCallbackFcn,
): b3TreeStats {
	let proxies = view.proxies;
	if (proxies == null) {
		return b3RayCastTreeView(view.nodes, b3_noProxies, input, maskBits, requireAllBits, callback, false);
	}

	return b3RayCastTreeView(view.nodes, proxies, input, maskBits, requireAllBits, callback, true);
}

export function b3TreeView_BoxCast(
	view: b3TreeView,
	input: b3BoxCastInput,
	maskBits: u64,
	requireAllBits: bool,
	callback: b3TreeBoxCastCallbackFcn,
): b3TreeStats {
	let proxies = view.proxies;
	if (proxies == null) {
		return b3BoxCastTreeView(view.nodes, b3_noProxies, input, maskBits, requireAllBits, callback, false);
	}

	return b3BoxCastTreeView(view.nodes, proxies, input, maskBits, requireAllBits, callback, true);
}

// Median split == 0, Surface area heuristic == 1. C compiles only the median split.

// Median split heuristic. C's pointers into the leaf arrays become the arrays plus a base index.
// Unchecked: every index is in [base, base + count), which b3BuildTree keeps inside the rebuild's leaf arrays
@unchecked
function b3PartitionMid(indices: StaticArray<i32>, centers: StaticArray<b3Vec3>, base: i32, count: i32): i32 {
	// Handle trivial case
	if (count <= 2) {
		return count / 2;
	}

	let lowerBound = centers[base];
	let upperBound = centers[base];

	for (let i = 1; i < count; ++i) {
		lowerBound = b3Min(lowerBound, centers[base + i]);
		upperBound = b3Max(upperBound, centers[base + i]);
	}

	let d = b3Sub(upperBound, lowerBound);
	let c = b3MulSV(0.5, b3Add(lowerBound, upperBound));

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	let i1 = 0;
	let i2 = count;
	if (d.x >= d.y && d.x >= d.z) {
		let pivot = c.x;

		while (i1 < i2) {
			while (i1 < i2 && centers[base + i1].x < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && centers[base + i2 - 1].x >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				b3SwapLeaves(indices, centers, base + i1, base + i2 - 1);
				i1 += 1;
				i2 -= 1;
			}
		}
	} else if (d.y >= d.z) {
		let pivot = c.y;

		while (i1 < i2) {
			while (i1 < i2 && centers[base + i1].y < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && centers[base + i2 - 1].y >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				b3SwapLeaves(indices, centers, base + i1, base + i2 - 1);
				i1 += 1;
				i2 -= 1;
			}
		}
	} else {
		let pivot = c.z;

		while (i1 < i2) {
			while (i1 < i2 && centers[base + i1].z < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && centers[base + i2 - 1].z >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				b3SwapLeaves(indices, centers, base + i1, base + i2 - 1);
				i1 += 1;
				i2 -= 1;
			}
		}
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(i1 == i2);

	if (i1 > 0 && i1 < count) {
		return i1;
	}

	return count / 2;
}

// Swap indices and centers
// Only b3PartitionMid calls it, with indices in its range
@unchecked
@inline function b3SwapLeaves(indices: StaticArray<i32>, centers: StaticArray<b3Vec3>, a: i32, b: i32): void {
	let tempIndex = indices[a];
	indices[a] = indices[b];
	indices[b] = tempIndex;

	let tempCenter = centers[a];
	centers[a] = centers[b];
	centers[b] = tempCenter;
}

// Temporary data used to track the rebuild of a tree node.
struct b3RebuildItem {
	// Where this node is written and the pair its children go in.
	nodeIndex: i32 = 0;
	pair: i32 = 0;
	childCount: i32 = 0;

	// Leaf indices
	startIndex: i32 = 0;
	splitIndex: i32 = 0;
	endIndex: i32 = 0;
}

struct b3CopyItem {
	oldPair: i32 = 0;
	newIndex: i32 = 0;

	constructor(oldPair: i32, newIndex: i32) {
		this.oldPair = oldPair;
		this.newIndex = newIndex;
	}
}

// The rebuild, refit and moved flag walks never call out, so they share static stacks.
const b3_rebuildStack = new StaticArray<b3RebuildItem>(B3_TREE_STACK_SIZE);
const b3_copyStack = new StaticArray<b3CopyItem>(B3_TREE_STACK_SIZE);
const b3_walkStack = new StaticArray<i32>(B3_TREE_STACK_SIZE);

// Bump allocate a pair of sibling nodes. Returns index to the first one.
@inline function b3BumpPair(tree: b3DynamicTree, parent: i32): i32 {
	let pair = tree.nodeEnd;
	if (B3_ASSERT_ENABLED) B3_ASSERT(pair + 2 <= tree.nodeCapacity);
	tree.nodeEnd += 2;
	tree.parents[pair] = parent;
	tree.parents[pair + 1] = parent;
	return pair;
}

@inline function b3SetLeftChild(node: ref b3TreeNode, pair: i32): void {
	node.flagIndex = (node.flagIndex & ~B3_NODE_INDEX_MASK) | <u32>pair;
}

// Copy a retained subtree from the old array into the rebuilt DFS array using
// a bump allocator.
function b3CopySubtree(tree: b3DynamicTree, node: b3TreeNode, newIndex: i32): void {
	let oldNodes = tree.nodes;
	let newNodes = tree.swapNodes;
	let proxies = tree.proxies;

	if (b3IsLeaf(node)) {
		newNodes[newIndex] = node;
		proxies[b3GetProxyId(node)].node = newIndex;
		return;
	}

	let stack = b3_copyStack;
	let stackCount = 0;

	let oldPair = b3GetLeftChild(node);
	let newPair = b3BumpPair(tree, newIndex);
	b3SetLeftChild(node, newPair);

	// Copy the subtree root. This can be a left or right sibling (even or odd).
	newNodes[newIndex] = node;

	for (;;) {
		// Copy the siblings.
		newNodes[newPair] = oldNodes[oldPair];
		newNodes[newPair + 1] = oldNodes[oldPair + 1];

		let flags0 = newNodes[newPair].flagIndex;
		let flags1 = newNodes[newPair + 1].flagIndex;

		if ((flags1 & B3_LEAF_NODE) == B3_LEAF_NODE) {
			// Hook up proxy on right sibling.
			let proxyId = <i32>(flags1 & B3_NODE_INDEX_MASK);
			proxies[proxyId].node = newPair + 1;
		} else {
			// Push the left child of the right sibling.
			if (stackCount < B3_TREE_STACK_SIZE) {
				let leftChild = <i32>(flags1 & B3_NODE_INDEX_MASK);
				stack[stackCount++] = b3CopyItem(leftChild, newPair + 1);
			} else {
				// todo fail gracefully if the stack runs out in release
				if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount < B3_TREE_STACK_SIZE);
			}
		}

		if ((flags0 & B3_LEAF_NODE) != B3_LEAF_NODE) {
			// Descend into left subtree.
			let leftIndex = newPair;
			oldPair = <i32>(flags0 & B3_NODE_INDEX_MASK);
			newPair = b3BumpPair(tree, leftIndex);
			b3SetLeftChild(newNodes[leftIndex], newPair);
			continue;
		}

		// Hook up proxy on left sibling.
		proxies[<i32>(flags0 & B3_NODE_INDEX_MASK)].node = newPair;

		if (stackCount == 0) {
			break;
		}

		// Descend a right subtree that was previously pushed.
		let item = stack[--stackCount];
		oldPair = item.oldPair;
		newPair = b3BumpPair(tree, item.newIndex);
		b3SetLeftChild(newNodes[item.newIndex], newPair);
	}
}

@inline function b3PlaceLeaf(tree: b3DynamicTree, leaf: b3TreeNode, newIndex: i32): void {
	if (b3IsLeaf(leaf)) {
		tree.swapNodes[newIndex] = leaf;
		tree.proxies[b3GetProxyId(leaf)].node = newIndex;
	} else {
		b3CopySubtree(tree, leaf, newIndex);
	}
}

function b3BuildTree(tree: b3DynamicTree, leafCount: i32): void {
	let nodes = tree.swapNodes;
	let leaves = tree.leafNodes;
	let leafIndices = tree.leafIndices;
	let leafCenters = tree.leafCenters;

	// Bump allocation into the swap nodes.
	tree.nodeEnd = 2;
	nodes[B3_ROOT_NODE + 1] = b3MakeEmptyNode();
	tree.parents[B3_ROOT_NODE] = B3_NULL_INDEX;
	tree.parents[B3_ROOT_NODE + 1] = B3_NULL_INDEX;

	if (leafCount == 1) {
		b3PlaceLeaf(tree, leaves[leafIndices[0]], B3_ROOT_NODE);
		return;
	}

	let stack = b3_rebuildStack;
	let top = 0;

	stack[0].nodeIndex = B3_ROOT_NODE;
	stack[0].pair = b3BumpPair(tree, B3_ROOT_NODE);
	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
	stack[0].splitIndex = b3PartitionMid(leafIndices, leafCenters, 0, leafCount);

	while (true) {
		ref item = stack[top];
		item.childCount += 1;

		if (item.childCount == 2) {
			// Both children written, so the parent node can be finalized.
			nodes[item.nodeIndex] = b3MakeInternalNode(nodes, item.pair);
			if (top == 0) {
				break;
			}

			top -= 1;
			continue;
		}

		let slot = item.childCount;
		let startIndex = slot == 0 ? item.startIndex : item.splitIndex;
		let endIndex = slot == 0 ? item.splitIndex : item.endIndex;
		let count = endIndex - startIndex;
		if (B3_ASSERT_ENABLED) B3_ASSERT(count > 0);

		let nodeIndex = item.pair + slot;
		if (count == 1) {
			b3PlaceLeaf(tree, leaves[leafIndices[startIndex]], nodeIndex);
			continue;
		}

		// todo fail gracefully if the stack runs out in release
		if (B3_ASSERT_ENABLED) B3_ASSERT(top < B3_TREE_STACK_SIZE - 1);

		top += 1;
		ref newItem = stack[top];
		newItem.nodeIndex = nodeIndex;
		newItem.pair = b3BumpPair(tree, nodeIndex);
		newItem.childCount = -1;
		newItem.startIndex = startIndex;
		newItem.endIndex = endIndex;
		newItem.splitIndex = startIndex;
		newItem.splitIndex += b3PartitionMid(leafIndices, leafCenters, startIndex, count);
	}
}

// Rebuild the stale parts of the tree. The entire tree is put into DFS order. This makes
// refitting much faster. This is done async with threading, so the cost is hidden.
// Not safe to access tree during this operation.
export function b3DynamicTree_Rebuild(tree: b3DynamicTree, fullBuild: bool): i32 {
	return b3DynamicTree_RebuildAt<b3OnMain>(tree, fullBuild);
}

// Allocates what a rebuild of the tree needs, so a parallel part can run it
export function b3DynamicTree_ReserveRebuild(tree: b3DynamicTree): void {
	if (tree.swapNodes.length == 0) {
		tree.swapNodes = new StaticArray<b3TreeNode>(tree.nodeCapacity);
	}

	let proxyCount = tree.proxyCount;
	if (proxyCount > tree.rebuildCapacity) {
		let newCapacity = proxyCount + proxyCount / 2;
		tree.leafIndices = new StaticArray<i32>(newCapacity);
		tree.leafNodes = new StaticArray<b3TreeNode>(newCapacity);
		tree.leafCenters = new StaticArray<b3Vec3>(newCapacity);
		tree.rebuildCapacity = newCapacity;
	}
}

// Makes a parallel part's rebuild the tree; nothing reads the tree in between
export function b3DynamicTree_FinishRebuild(tree: b3DynamicTree, fullBuild: bool): void {
	if (tree.swapPending) {
		tree.swapPending = false;
		b3SwapRebuiltNodes(tree, fullBuild);
	}
}

function b3SwapRebuiltNodes(tree: b3DynamicTree, fullBuild: bool): void {
	// The spare is now the tree and the old array is the spare. The build is dense, so the
	// stale tail of the old array sits above the end and is never read.
	let temp = tree.nodes;
	tree.nodes = tree.swapNodes;
	tree.swapNodes = temp;
	tree.pairFreeList = B3_NULL_INDEX;
	tree.dfsOrdered = true;

	b3DynamicTree_Validate(tree);
	if (fullBuild == false) {
		b3DynamicTree_ValidateNoMoved(tree);
	}
}

// A parallel part, b3InParallel, runs it after b3DynamicTree_ReserveRebuild and leaves the swap to
// b3DynamicTree_FinishRebuild
export function b3DynamicTree_RebuildAt<M>(tree: b3DynamicTree, fullBuild: bool): i32 {
	let proxyCount = tree.proxyCount;
	if (proxyCount == 0) {
		return 0;
	}

	// An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
	let nodes = tree.nodes;
	let rootFlags = nodes[B3_ROOT_NODE].flagIndex;
	let rootMoved = (rootFlags & B3_MOVED_NODE) == B3_MOVED_NODE;
	if (fullBuild == false && rootMoved == false && tree.dfsOrdered) {
		return 0;
	}

	if (!isBoolean<M>()) {
		b3DynamicTree_ReserveRebuild(tree);
	}

	let leafIndices = tree.leafIndices;
	let leaves = tree.leafNodes;
	let leafCenters = tree.leafCenters;

	// Gather build leaves. A marked internal node is descended and abandoned in the old array.
	// Everything else is a build leaf, kept subtrees included. A leaf root, or an unmarked root
	// on an unordered array, is the one build leaf.
	let leafCount = 0;
	let stack = b3_walkStack;
	let stackCount = 0;
	if ((rootFlags & B3_LEAF_NODE) != B3_LEAF_NODE && (fullBuild || rootMoved)) {
		stack[stackCount++] = <i32>(rootFlags & B3_NODE_INDEX_MASK);
	} else {
		let node = nodes[B3_ROOT_NODE];
		if (fullBuild == false) {
			node.flagIndex &= ~B3_MOVED_NODE;
		}
		leafIndices[0] = 0;
		leaves[0] = node;
		leafCenters[0] = b3AABB_Center(node.aabb);
		leafCount = 1;
	}

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			let node = nodes[pair + i];
			if (b3IsLeaf(node) == false && (fullBuild || b3IsNodeMoved(node))) {
				// todo fail gracefully if the stack runs out in release
				if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount < B3_TREE_STACK_SIZE);
				stack[stackCount++] = b3GetLeftChild(node);
				continue;
			}

			if (fullBuild == false) {
				node.flagIndex &= ~B3_MOVED_NODE;
			}
			leafIndices[leafCount] = leafCount;
			leaves[leafCount] = node;
			leafCenters[leafCount] = b3AABB_Center(node.aabb);
			leafCount += 1;
		}
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(0 < leafCount && leafCount <= proxyCount);

	b3BuildTree(tree, leafCount);

	if (isBoolean<M>()) {
		tree.swapPending = true;
	} else {
		b3SwapRebuiltNodes(tree, fullBuild);
	}

	return leafCount;
}

// Set the moved flag on the ancestors of the proxy. Serial use case.
export function b3DynamicTree_MarkProxyMovedSerial(tree: b3DynamicTree, proxyId: i32): void {
	let nodes = tree.nodes;
	let parents = tree.parents;

	let index = tree.proxies[proxyId].node;

	while (index != B3_NULL_INDEX) {
		nodes[index].flagIndex |= B3_MOVED_NODE;
		index = parents[index];
	}
}

// Update a proxy AABB and flag the ancestors as moved. Thread-safe using atomics in the b3InParallel instantiation,
// which parallel parts run as they finalize bodies.
export unsafe function b3DynamicTree_MarkProxyMoved<M>(tree: b3DynamicTree, proxyId: i32, aabb: b3AABB): void {
	let nodes = tree.nodes;
	let parents = tree.parents;

	let index = tree.proxies[proxyId].node;

	ref node = nodes[index];

	// This is not raced since it is the leaf.
	node.aabb = aabb;
	node.flagIndex |= B3_MOVED_NODE;

	index = parents[index];
	while (index != B3_NULL_INDEX) {
		let flagIndex = changetype<usize>(nodes) + <usize>index * sizeof<b3TreeNode>() + offsetof<b3TreeNode>("flagIndex");

		// Read first to avoid the FetchOr if possible.
		if ((load<u32>(flagIndex) & B3_MOVED_NODE) != 0) {
			break;
		}

		let previousFlags: u32;
		if (isBoolean<M>()) {
			previousFlags = atomic.or<u32>(flagIndex, B3_MOVED_NODE);
		} else {
			previousFlags = load<u32>(flagIndex);
			store<u32>(flagIndex, previousFlags | B3_MOVED_NODE);
		}
		if ((previousFlags & B3_MOVED_NODE) != 0) {
			// Ancestor already visited.
			break;
		}

		index = parents[index];
	}
}

// Clear the moved flags from the entire tree.
export function b3DynamicTree_ClearMoved(tree: b3DynamicTree): void {
	let nodes = tree.nodes;

	ref root = nodes[B3_ROOT_NODE];
	if (b3IsNodeMoved(root) == false) {
		return;
	}

	root.flagIndex &= ~B3_MOVED_NODE;
	if (b3IsLeaf(root)) {
		return;
	}

	let stack = b3_walkStack;
	let stackCount = 0;
	stack[stackCount++] = b3GetLeftChild(root);

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];
			if (node.flagIndex & B3_MOVED_NODE) {
				node.flagIndex &= ~B3_MOVED_NODE;
				if (b3IsLeaf(node) == false) {
					if (stackCount < B3_TREE_STACK_SIZE) {
						stack[stackCount++] = b3GetLeftChild(node);
					} else {
						// Bad stuff will happen if the moved flags don't get cleared.
						if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount < B3_TREE_STACK_SIZE);
					}
				}
			}
		}
	}
}

// proxyIds is any i32 array with room for every moved proxy.
export function b3DynamicTree_GatherMovedProxies<A>(tree: b3DynamicTree, proxyIds: A): i32 {
	let nodes = tree.nodes;
	ref root = nodes[B3_ROOT_NODE];
	if (b3IsNodeMoved(root) == false) {
		return 0;
	}

	if (b3IsLeaf(root)) {
		proxyIds[0] = b3GetProxyId(root);
		return 1;
	}

	let count = 0;
	let stack = b3_walkStack;
	let stackCount = 0;
	stack[stackCount++] = b3GetLeftChild(root);

	while (stackCount > 0) {
		let pair = stack[--stackCount];
		for (let i = 0; i < 2; ++i) {
			ref node = nodes[pair + i];
			if (b3IsNodeMoved(node) == false) {
				continue;
			}

			if (b3IsLeaf(node)) {
				proxyIds[count++] = b3GetProxyId(node);
			} else {
				if (stackCount < B3_TREE_STACK_SIZE) {
					stack[stackCount++] = b3GetLeftChild(node);
				} else {
					if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount < B3_TREE_STACK_SIZE);
				}
			}
		}
	}

	return count;
}

// Slow refit for unit tests.
function b3RefitSubtree(nodes: StaticArray<b3TreeNode>, rootIndex: i32): void {
	ref root = nodes[rootIndex];
	if (b3IsLeaf(root) || b3IsNodeMoved(root) == false) {
		return;
	}

	// Negative entries indicate deferred unions. A parent is pushed before its children so the
	// LIFO order guarantees both children are finished when the union runs.
	let stack = b3_walkStack;
	let stackCount = 0;
	// ~0 == -1
	stack[stackCount++] = ~rootIndex;
	stack[stackCount++] = b3GetLeftChild(root);
	stack[stackCount++] = b3GetLeftChild(root) + 1;

	while (stackCount > 0) {
		let item = stack[--stackCount];
		if (item < 0) {
			ref parent = nodes[~item];
			let pair = b3GetLeftChild(parent);
			parent.aabb = b3UnionV(nodes[pair].aabb, nodes[pair + 1].aabb);
			continue;
		}

		ref node = nodes[item];
		if (b3IsLeaf(node) || b3IsNodeMoved(node) == false) {
			continue;
		}

		if (stackCount + 3 <= B3_TREE_STACK_SIZE) {
			let pair = b3GetLeftChild(node);
			stack[stackCount++] = ~item;
			stack[stackCount++] = pair;
			stack[stackCount++] = pair + 1;
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(stackCount + 3 <= B3_TREE_STACK_SIZE);
		}
	}
}

export function b3DynamicTree_Refit(tree: b3DynamicTree): void {
	if (b3HasTreeMoved(tree) == false) {
		return;
	}

	let nodes = tree.nodes;

	if (tree.dfsOrdered == false) {
		// This only happens in unit tests.
		b3RefitSubtree(nodes, B3_ROOT_NODE);
		return;
	}

	for (let pair = tree.nodeEnd - 2; pair >= 0; pair -= 2) {
		let flags1 = nodes[pair].flagIndex;
		let flags2 = nodes[pair + 1].flagIndex;

		// Did either move?
		if (((flags1 | flags2) & B3_MOVED_NODE) == 0) {
			continue;
		}

		// Is the node internal and moved?
		let refit1 = (flags1 & (B3_LEAF_NODE | B3_MOVED_NODE)) == B3_MOVED_NODE;
		let refit2 = (flags2 & (B3_LEAF_NODE | B3_MOVED_NODE)) == B3_MOVED_NODE;

		// If the node needs a refit then get the child index. Otherwise use a valid dummy index.
		// The store function won't actually modify the node at the dummy index. This
		// avoids an unpredictable branch. SIMD FTW.
		let children1 = refit1 ? <i32>(flags1 & B3_NODE_INDEX_MASK) : pair;
		let children2 = refit2 ? <i32>(flags2 & B3_NODE_INDEX_MASK) : pair;

		// Conditionally store the union.
		b3StoreAABBV(nodes[pair].aabb, b3UnionPairV(nodes, children1), refit1);
		b3StoreAABBV(nodes[pair + 1].aabb, b3UnionPairV(nodes, children2), refit2);
	}
}
