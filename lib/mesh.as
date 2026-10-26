// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/mesh.c

// Dirk Gregorius contributed portions of this code

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_EPSILON, FLT_MAX, b3FloatToIntSat, b3Hash64NonZero } from "./core";
import { B3_LINEAR_SLOP, B3_MAX_SHAPE_CAST_POINTS } from "./constants";
import {
	b3AABB,
	b3AABB_Area,
	b3AABB_Center,
	b3AABB_Extents,
	b3AABB_Transform,
	b3AABB_Union,
	b3Add,
	b3ComputeCosSin,
	b3Cos,
	b3Cross,
	b3Dot,
	b3IsSaneAABB,
	b3Length,
	b3Lerp,
	b3MakeAABB,
	b3Max,
	b3MaxInt,
	b3Min,
	b3MinInt,
	b3Mul,
	b3MulSV,
	b3Neg,
	b3Normalize,
	b3Plane,
	b3Quat_identity,
	b3Sin,
	b3Sub,
	b3Transform,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_zero,
	B3_PI,
} from "./math_functions";
import { B3_BOUNDS3_EMPTY, B3_TWO_PI, b3AABB_AddPoint, b3GetByIndex, b3MajorAxis, b3SignedVolume } from "./math_internal";
import {
	B3_MESH_DATA_SIZE,
	B3_MESH_VERSION,
	b3Capsule,
	b3CastOutput,
	b3DistanceInput,
	b3Mesh,
	b3MeshData,
	b3MeshDef,
	b3MeshNode,
	b3MeshQueryFcn,
	b3VisitMeshTriangle,
	b3MeshTriangle,
	b3PlaneResult,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeCastPairInput,
	b3ShapeProxy,
	b3SimplexCache,
	b3_concaveEdge1,
	b3_concaveEdge2,
	b3_concaveEdge3,
	b3_inverseConcaveEdge1,
	b3_inverseConcaveEdge2,
	b3_inverseConcaveEdge3,
} from "./collision";
import {
	b3AbsV,
	b3AddV,
	b3DivV,
	b3GetV,
	b3IntersectRayTriangle,
	b3LoadV,
	b3MaxV,
	b3MinV,
	b3MulV,
	b3SplatV,
	b3SubV,
	b3TestBoundsOverlap,
	b3TestBoundsRayOverlap,
	b3TestBoundsTriangleOverlap,
	b3_halfV,
	b3_oneV,
} from "./simd";
import { b3SetProxy, b3ShapeCast, b3ShapeDistance } from "./distance";
import { b3SetLocalProxy } from "./shape";
import { b3Array_AddIndex } from "./container";

const B3_BIN_COUNT: i32 = 8;
const B3_DESIRED_TRIANGLES_PER_LEAF: i32 = 4;
const B3_LEAF_NODE: u32 = 3;
const B3_MAXIMUM_TRIANGLES_PER_LEAF: i32 = 8;
export const B3_MESH_STACK_SIZE: i32 = 256;

// Node pointers are indices into the node array.
// Traversal stack shared by the queries that never call out to user code.
const b3_meshStack = new StaticArray<i32>(B3_MESH_STACK_SIZE);

@inline function b3IsLeaf(nodes: StaticArray<b3MeshNode>, node: i32): bool {
	return nodes[node].data.asLeaf.type == B3_LEAF_NODE;
}

@inline function b3GetLeftChild(nodes: StaticArray<b3MeshNode>, node: i32): i32 {
	// The left child follows its parent.
	if (B3_ASSERT_ENABLED) B3_ASSERT(!b3IsLeaf(nodes, node));
	return node + 1;
}

@inline function b3GetRightChild(nodes: StaticArray<b3MeshNode>, node: i32): i32 {
	// We store the offset of the right child relative to its parent
	if (B3_ASSERT_ENABLED) B3_ASSERT(!b3IsLeaf(nodes, node));
	return node + <i32>nodes[node].data.asNode.childOffset;
}

function b3GetNodeHeight(nodes: StaticArray<b3MeshNode>, node: i32): i32 {
	if (b3IsLeaf(nodes, node)) {
		return 0;
	}

	let leftHeight = b3GetNodeHeight(nodes, b3GetLeftChild(nodes, node));
	let rightHeight = b3GetNodeHeight(nodes, b3GetRightChild(nodes, node));

	return 1 + b3MaxInt(leftHeight, rightHeight);
}

export function b3GetHeight(mesh: b3MeshData): i32 {
	// C has no root when the node offset is zero
	if (mesh.nodeOffset == 0) {
		return 0;
	}

	return b3GetNodeHeight(mesh.nodes, 0);
}

export function b3IsValidMesh(meshData: b3MeshData?): bool {
	if (meshData == null) {
		return false;
	}

	if (meshData.version != B3_MESH_VERSION) {
		return false;
	}

	if (meshData.byteCount < B3_MESH_DATA_SIZE) {
		return false;
	}

	return true;
}

// Node for a vertex linked list
struct b3VertexNode {
	vertexIndex: i32 = 0;
	nextNodeIndex: i32 = 0;

	constructor(vertexIndex: i32, nextNodeIndex: i32) {
		this.vertexIndex = vertexIndex;
		this.nextNodeIndex = nextNodeIndex;
	}
}

class b3SpatialHash {
	nodes: Array<b3VertexNode> = new Array<b3VertexNode>();
	vertices: StaticArray<b3Vec3>;
	vertexCount: i32;
	vertexMap: Map<u64, i32> = new Map<u64, i32>();
	cellSize: f32;
	tolerance: f32;

	constructor(vertices: StaticArray<b3Vec3>, vertexCount: i32, tolerance: f32) {
		this.vertices = vertices;
		this.vertexCount = vertexCount;
		this.tolerance = tolerance;
		this.cellSize = 2.0 * tolerance;

		B3_ASSERT(this.cellSize > 0.0);
	}
}

@inline function b3HashCell(x: i32, y: i32, z: i32): u64 {
	let key: u64 = 0;
	key ^= <u64>(<i64>x) + 0x9e3779b9 + (key << 6) + (key >> 2);
	key ^= <u64>(<i64>y) + 0x9e3779b9 + (key << 6) + (key >> 2);
	key ^= <u64>(<i64>z) + 0x9e3779b9 + (key << 6) + (key >> 2);
	return key;
}

// Welding works by bucketing nearby vertices into identical keys in a hash table.
// Bucketing is done manually with an array.
function b3SpatialHash_FindDuplicate(h: b3SpatialHash, currentIndex: i32): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(currentIndex < h.vertexCount);
	let vertex = h.vertices[currentIndex];
	let cellSize = h.cellSize;
	let tolerance = h.tolerance;

	// Get the grid coordinates for the current vertex
	let baseX = b3FloatToIntSat(Mathf.floor(vertex.x / cellSize));
	let baseY = b3FloatToIntSat(Mathf.floor(vertex.y / cellSize));
	let baseZ = b3FloatToIntSat(Mathf.floor(vertex.z / cellSize));

	// Check the current cell and all 26 neighboring cells (3x3x3 - 1)
	for (let dx = -1; dx <= 1; ++dx) {
		for (let dy = -1; dy <= 1; ++dy) {
			for (let dz = -1; dz <= 1; ++dz) {
				// Compute hash for this neighboring cell (this is the key in the map)
				let key = b3HashCell(baseX + dx, baseY + dy, baseZ + dz);

				if (h.vertexMap.has(key)) {
					// Check all vertices in this key
					let nodeIndex = h.vertexMap.get(key);

					while (nodeIndex != B3_NULL_INDEX) {
						let node = h.nodes[nodeIndex];

						let existingIndex = node.vertexIndex;
						if (B3_ASSERT_ENABLED) B3_ASSERT(existingIndex < currentIndex);

						let other = h.vertices[existingIndex];

						// IsEqual inlined: check if vertices are within tolerance
						if (
							Mathf.abs(vertex.x - other.x) <= tolerance &&
							Mathf.abs(vertex.y - other.y) <= tolerance &&
							Mathf.abs(vertex.z - other.z) <= tolerance
						) {
							// Found duplicate
							return existingIndex;
						}

						nodeIndex = node.nextNodeIndex;
					}
				}
			}
		}
	}

	// No duplicate found, add to hash table
	let currentKey = b3HashCell(baseX, baseY, baseZ);

	if (h.vertexMap.has(currentKey)) {
		let nodeIndex = h.vertexMap.get(currentKey);
		h.vertexMap.set(currentKey, h.nodes.length);
		h.nodes.push(b3VertexNode(currentIndex, nodeIndex));
	} else {
		h.vertexMap.set(currentKey, h.nodes.length);
		h.nodes.push(b3VertexNode(currentIndex, B3_NULL_INDEX));
	}

	// Not welded
	return B3_NULL_INDEX;
}

// C bundles the arguments in b3WeldData
function b3WeldVertices(
	srcVertices: StaticArray<b3Vec3>,
	srcIndices: StaticArray<i32>,
	dstVertices: StaticArray<b3Vec3>,
	dstIndices: StaticArray<i32>,
	vertexCount: i32,
	indexCount: i32,
	tolerance: f32,
): i32 {
	let uniqueCount = 0;

	// Create spatial hash and find duplicates
	let spatialHash = new b3SpatialHash(srcVertices, vertexCount, tolerance);
	let vertexMapping = new StaticArray<i32>(vertexCount);

	for (let i = 0; i < vertexCount; ++i) {
		let duplicateIndex = b3SpatialHash_FindDuplicate(spatialHash, i);

		if (duplicateIndex == B3_NULL_INDEX) {
			// New unique vertex
			vertexMapping[i] = uniqueCount;
			dstVertices[uniqueCount] = srcVertices[i];
			uniqueCount += 1;
		} else {
			// Found duplicate, map to existing vertex
			vertexMapping[i] = vertexMapping[duplicateIndex];
		}
	}

	// Update indices to reference the new vertex array
	for (let i = 0; i < indexCount; ++i) {
		let srcIndex = srcIndices[i];
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= srcIndex && srcIndex < vertexCount);
		dstIndices[i] = vertexMapping[srcIndex];
	}

	return uniqueCount;
}

function b3StoreLeaf(nodes: Array<b3MeshNode>, index: i32, aabb: b3AABB, triangleCount: i32, triangleOffset: i32): void {
	nodes[index].data.asLeaf.type = B3_LEAF_NODE;
	nodes[index].data.asLeaf.triangleCount = <u32>triangleCount;
	nodes[index].triangleOffset = <u32>triangleOffset;
	nodes[index].lowerBound = aabb.lowerBound;
	nodes[index].upperBound = aabb.upperBound;
}

struct b3Primitive {
	aabb: b3AABB = b3AABB();
	center: b3Vec3 = b3Vec3();
	triangleIndex: i32 = 0;
}

struct b3Bucket {
	count: i32 = 0;
	bounds: b3AABB = b3AABB();
}

struct b3Split {
	leftBounds: b3AABB = b3AABB();
	rightBounds: b3AABB = b3AABB();
	axis: i32 = 0;
	index: i32 = 0;
}

// C passes a pointer into the primitive array; here the array and the first index.
function b3SplitBinnedSah(count: i32, primitives: StaticArray<b3Primitive>, base: i32): b3Split {
	let split = b3Split();
	split.axis = -1;
	split.index = -1;

	// Compute bounds of primitive centroids and choose split axis
	let bounds = b3AABB(primitives[base].center, primitives[base].center);
	for (let i = 1; i < count; ++i) {
		bounds = b3AABB_AddPoint(bounds, primitives[base + i].center);
	}

	// Compute costs for splitting after each bucket and keep track of best split
	// This is a small O(n^2) loop. This can be further optimized, but it is already
	// very fast and is kept for simplicity right now.
	let bestBucket = -1;
	let bestCost = FLT_MAX;

	for (let axis = 0; axis < 3; ++axis) {
		let extent = b3AABB_Extents(bounds);
		if (b3GetByIndex(extent, axis) < B3_LINEAR_SLOP()) {
			continue;
		}

		// Initialize buckets
		let buckets: FixedArray<b3Bucket, 8>;
		for (let i = 0; i < B3_BIN_COUNT; ++i) {
			buckets[i].count = 0;
			buckets[i].bounds = B3_BOUNDS3_EMPTY;
		}

		// Fill buckets
		let factor: f32 =
			(<f32>B3_BIN_COUNT * (1.0 - FLT_EPSILON)) / (b3GetByIndex(bounds.upperBound, axis) - b3GetByIndex(bounds.lowerBound, axis));
		for (let i = 0; i < count; ++i) {
			let center = primitives[base + i].center;
			let index = b3FloatToIntSat(factor * (b3GetByIndex(center, axis) - b3GetByIndex(bounds.lowerBound, axis)));
			if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < B3_BIN_COUNT);

			buckets[index].count++;
			buckets[index].bounds = b3AABB_Union(buckets[index].bounds, primitives[base + i].aabb);
		}

		// Evaluate splits
		for (let i = 0; i < B3_BIN_COUNT - 1; ++i) {
			let leftCount = 0;
			let leftBounds = B3_BOUNDS3_EMPTY;
			for (let k = 0; k <= i; ++k) {
				leftCount += buckets[k].count;
				leftBounds = b3AABB_Union(leftBounds, buckets[k].bounds);
			}

			let rightCount = 0;
			let rightBounds = B3_BOUNDS3_EMPTY;
			for (let k = i + 1; k < B3_BIN_COUNT; ++k) {
				rightCount += buckets[k].count;
				rightBounds = b3AABB_Union(rightBounds, buckets[k].bounds);
			}

			if (B3_ASSERT_ENABLED) B3_ASSERT(leftCount + rightCount == count);
			if (leftCount > 0 && rightCount > 0) {
				let cost = <f32>leftCount * b3AABB_Area(leftBounds) + <f32>rightCount * b3AABB_Area(rightBounds);

				if (cost < bestCost) {
					bestBucket = i;
					bestCost = cost;

					split.axis = axis;
					split.index = leftCount;
					split.leftBounds = leftBounds;
					split.rightBounds = rightBounds;
				}
			}
		}
	}

	// Partition
	if (bestBucket >= 0) {
		let axis = split.axis;
		let factor: f32 =
			(<f32>B3_BIN_COUNT * (1.0 - FLT_EPSILON)) / (b3GetByIndex(bounds.upperBound, axis) - b3GetByIndex(bounds.lowerBound, axis));

		let splitIndex = 0;
		for (let i = 0; i < count; ++i) {
			let center = primitives[base + i].center;
			let index = b3FloatToIntSat(factor * (b3GetByIndex(center, axis) - b3GetByIndex(bounds.lowerBound, axis)));

			if (index <= bestBucket) {
				let temp = primitives[base + i];
				primitives[base + i] = primitives[base + splitIndex];
				primitives[base + splitIndex] = temp;
				splitIndex++;
			}
		}
		if (B3_ASSERT_ENABLED) B3_ASSERT(splitIndex == split.index);
	}

	return split;
}

function b3SplitHalf(count: i32, primitives: StaticArray<b3Primitive>, base: i32): b3Split {
	// Split in the middle
	let splitIndex = count / 2;

	let leftBounds = B3_BOUNDS3_EMPTY;
	for (let i = 0; i < splitIndex; ++i) {
		leftBounds = b3AABB_Union(leftBounds, primitives[base + i].aabb);
	}

	let rightBounds = B3_BOUNDS3_EMPTY;
	for (let i = splitIndex; i < count; ++i) {
		rightBounds = b3AABB_Union(rightBounds, primitives[base + i].aabb);
	}

	let bounds = b3AABB_Union(leftBounds, rightBounds);
	let axis = b3MajorAxis(b3AABB_Extents(bounds));

	let split = b3Split();
	split.axis = axis;
	split.index = splitIndex;
	split.leftBounds = leftBounds;
	split.rightBounds = rightBounds;

	return split;
}

function b3SplitMedian(count: i32, primitives: StaticArray<b3Primitive>, base: i32): b3Split {
	if (B3_ASSERT_ENABLED) B3_ASSERT(count > 2);

	let lowerBound = primitives[base].center;
	let upperBound = primitives[base].center;

	for (let i = 1; i < count; ++i) {
		lowerBound = b3Min(lowerBound, primitives[base + i].center);
		upperBound = b3Max(upperBound, primitives[base + i].center);
	}

	let d = b3Sub(upperBound, lowerBound);
	let c = b3MulSV(0.5, b3Add(lowerBound, upperBound));

	let split = b3Split();
	split.index = -1;

	// Partition longest axis using the Hoare partition scheme
	// https://en.wikipedia.org/wiki/Quicksort
	// https://nicholasvadivelu.com/2021/01/11/array-partition/
	let i1 = 0;
	let i2 = count;
	if (d.x >= d.y && d.x >= d.z) {
		split.axis = 0;

		let pivot = c.x;

		while (i1 < i2) {
			while (i1 < i2 && primitives[base + i1].center.x < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && primitives[base + i2 - 1].center.x >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				// Swap primitives
				let temp = primitives[base + i1];
				primitives[base + i1] = primitives[base + i2 - 1];
				primitives[base + i2 - 1] = temp;

				i1 += 1;
				i2 -= 1;
			}
		}
	} else if (d.y >= d.z) {
		split.axis = 1;

		let pivot = c.y;

		while (i1 < i2) {
			while (i1 < i2 && primitives[base + i1].center.y < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && primitives[base + i2 - 1].center.y >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				// Swap primitives
				let temp = primitives[base + i1];
				primitives[base + i1] = primitives[base + i2 - 1];
				primitives[base + i2 - 1] = temp;

				i1 += 1;
				i2 -= 1;
			}
		}
	} else {
		split.axis = 2;

		let pivot = c.z;

		while (i1 < i2) {
			while (i1 < i2 && primitives[base + i1].center.z < pivot) {
				i1 += 1;
			}

			while (i1 < i2 && primitives[base + i2 - 1].center.z >= pivot) {
				i2 -= 1;
			}

			if (i1 < i2) {
				// Swap primitives
				let temp = primitives[base + i1];
				primitives[base + i1] = primitives[base + i2 - 1];
				primitives[base + i2 - 1] = temp;

				i1 += 1;
				i2 -= 1;
			}
		}
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(i1 == i2);
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= i1 && i1 < count);

	if (i1 == 0 || i1 == count - 1) {
		// failed to split
		i1 = count / 2;
	}

	let leftBounds = B3_BOUNDS3_EMPTY;
	for (let i = 0; i < i1; ++i) {
		leftBounds = b3AABB_Union(leftBounds, primitives[base + i].aabb);
	}

	let rightBounds = B3_BOUNDS3_EMPTY;
	for (let i = i1; i < count; ++i) {
		rightBounds = b3AABB_Union(rightBounds, primitives[base + i].aabb);
	}

	split.index = i1;
	split.leftBounds = leftBounds;
	split.rightBounds = rightBounds;
	return split;
}

// C's primitives - base is the base argument. height must be a struct leaf or FixedArray element.
function b3BuildRecursive(
	nodes: Array<b3MeshNode>,
	count: i32,
	primitives: StaticArray<b3Primitive>,
	base: i32,
	useMedianSplit: bool,
	height: ref i32,
): i32 {
	if (count > B3_DESIRED_TRIANGLES_PER_LEAF) {
		// Try to split the input set using the SAH
		let split: b3Split;
		if (useMedianSplit) {
			split = b3SplitMedian(count, primitives, base);
		} else {
			split = b3SplitBinnedSah(count, primitives, base);
		}

		if (split.axis < 0) {
			if (count > B3_MAXIMUM_TRIANGLES_PER_LEAF) {
				// Re-split. This is a less optimal split and can create more false positives!
				split = b3SplitHalf(count, primitives, base);
			} else {
				let bounds = B3_BOUNDS3_EMPTY;
				for (let i = 0; i < count; ++i) {
					bounds = b3AABB_Union(bounds, primitives[base + i].aabb);
				}

				// We have only a few triangles left. Create a leaf.
				let index = b3Array_AddIndex(nodes);
				b3StoreLeaf(nodes, index, bounds, count, base);

				return index;
			}
		}

		// Allocate node and recurse
		let index = b3Array_AddIndex(nodes);
		let childHeights: FixedArray<i32, 2>;
		let leftIndex = b3BuildRecursive(nodes, split.index, primitives, base, useMedianSplit, childHeights[0]);
		let rightIndex = b3BuildRecursive(nodes, count - split.index, primitives, base + split.index, useMedianSplit, childHeights[1]);

		height = b3MaxInt(childHeights[0], childHeights[1]) + 1;

		if (B3_ASSERT_ENABLED) B3_ASSERT(leftIndex - index == 1 && rightIndex - index > 1);

		let aabb = b3AABB_Union(split.leftBounds, split.rightBounds);
		nodes[index].data.asNode.axis = <u32>split.axis;
		nodes[index].data.asNode.childOffset = <u32>(rightIndex - index);
		nodes[index].lowerBound = aabb.lowerBound;
		nodes[index].upperBound = aabb.upperBound;

		// Zero so mesh->hash is deterministic
		nodes[index].triangleOffset = 0;

		return index;
	}

	let aabb = B3_BOUNDS3_EMPTY;
	for (let i = 0; i < count; ++i) {
		aabb = b3AABB_Union(aabb, primitives[base + i].aabb);
	}

	let index = b3Array_AddIndex(nodes);
	b3StoreLeaf(nodes, index, aabb, count, base);

	height = 1;

	return index;
}

function b3SortMeshTriangles(mesh: b3MeshData): bool {
	let nodes = mesh.nodes;
	let triangles = mesh.triangles;
	let materialIndices = mesh.materialIndices;

	// Sort triangles in depth-first-order
	let offset = 0;
	let tempTriangles = new StaticArray<b3MeshTriangle>(mesh.triangleCount);
	let tempMaterialIndices = new StaticArray<u8>(mesh.triangleCount);
	let tempCount = 0;

	let count = 0;
	let stack = new StaticArray<i32>(B3_MESH_STACK_SIZE);
	stack[count++] = 0;

	while (count > 0) {
		let node = stack[--count];

		if (b3IsLeaf(nodes, node) == false) {
			if (count >= B3_MESH_STACK_SIZE - 2) {
				return false;
			}

			stack[count++] = b3GetRightChild(nodes, node);
			stack[count++] = b3GetLeftChild(nodes, node);
		} else {
			let triangleCount = <i32>nodes[node].data.asLeaf.triangleCount;
			let triangleOffset = <i32>nodes[node].triangleOffset;

			for (let triangle = 0; triangle < triangleCount; ++triangle) {
				let index = triangleOffset + triangle;
				tempTriangles[tempCount] = triangles[index];
				tempMaterialIndices[tempCount] = materialIndices[index];
				tempCount += 1;
			}

			nodes[node].triangleOffset = <u32>offset;
			offset += triangleCount;
		}
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(offset == tempCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(tempCount == mesh.triangleCount);

	// Copy sorted triangle array back to tree
	mesh.triangles = tempTriangles;
	mesh.materialIndices = tempMaterialIndices;

	return true;
}

struct b3MeshEdge {
	vertex1: i32 = 0;
	vertex2: i32 = 0;
	triangle1: i32 = 0;
	triangle2: i32 = 0;
	triangleCount: u16 = 0;

	// The index of an edge within the parent triangle: 0, 1, or 2. 0xFF is unset
	triangleEdgeIndex1: u8 = 0;
	triangleEdgeIndex2: u8 = 0;
}

const b3_concaveEdgeFlags: StaticArray<i32> = [b3_concaveEdge1, b3_concaveEdge2, b3_concaveEdge3];
const b3_inverseConcaveEdgeFlags: StaticArray<i32> = [b3_inverseConcaveEdge1, b3_inverseConcaveEdge2, b3_inverseConcaveEdge3];

@inline function b3EdgeKey(vertex1: i32, vertex2: i32): u64 {
	return (<u64>(<i64>vertex1) << 32) | <u64>(<i64>vertex2);
}

function b3IdentifyEdges(mesh: b3MeshData): void {
	let triangles = mesh.triangles;
	let vertices = mesh.vertices;
	let flags = mesh.flags;

	let triangleCount = mesh.triangleCount;
	let edgeCount = 3 * triangleCount;
	let edges = new StaticArray<b3MeshEdge>(edgeCount);
	let normals = new StaticArray<b3Vec3>(triangleCount);

	for (let i = 0; i < triangleCount; ++i) {
		let triangle = triangles[i];
		let i1 = triangle.index1;
		let i2 = triangle.index2;
		let i3 = triangle.index3;

		{
			ref edge = edges[3 * i + 0];
			edge.vertex1 = b3MinInt(i1, i2);
			edge.vertex2 = b3MaxInt(i1, i2);
			edge.triangle1 = i;
			edge.triangle2 = B3_NULL_INDEX;
			edge.triangleEdgeIndex1 = 0;
			edge.triangleEdgeIndex2 = 0xff;
			edge.triangleCount = 1;
		}

		{
			ref edge = edges[3 * i + 1];
			edge.vertex1 = b3MinInt(i2, i3);
			edge.vertex2 = b3MaxInt(i2, i3);
			edge.triangle1 = i;
			edge.triangle2 = B3_NULL_INDEX;
			edge.triangleEdgeIndex1 = 1;
			edge.triangleEdgeIndex2 = 0xff;
			edge.triangleCount = 1;
		}

		{
			ref edge = edges[3 * i + 2];
			edge.vertex1 = b3MinInt(i3, i1);
			edge.vertex2 = b3MaxInt(i3, i1);
			edge.triangle1 = i;
			edge.triangle2 = B3_NULL_INDEX;
			edge.triangleEdgeIndex1 = 2;
			edge.triangleEdgeIndex2 = 0xff;
			edge.triangleCount = 1;
		}

		let v1 = vertices[i1];
		let v2 = vertices[i2];
		let v3 = vertices[i3];

		let e1 = b3Sub(v2, v1);
		let e2 = b3Sub(v3, v1);
		let n = b3Cross(e1, e2);

		normals[i] = b3Normalize(n);
	}

	let map = new Map<u64, i32>();

	let key = b3EdgeKey(edges[0].vertex1, edges[0].vertex2);
	map.set(key, 0);

	// Find unique edges and assign adjacency
	for (let i = 1; i < edgeCount; ++i) {
		key = b3EdgeKey(edges[i].vertex1, edges[i].vertex2);

		if (map.has(key) == false) {
			map.set(key, i);
		} else {
			let otherIndex = map.get(key);
			if (B3_ASSERT_ENABLED) B3_ASSERT(otherIndex < i);

			ref base = edges[otherIndex];
			if (base.triangleCount == 1) {
				base.triangle2 = edges[i].triangle1;
				base.triangleEdgeIndex2 = edges[i].triangleEdgeIndex1;
			}

			base.triangleCount += 1;
		}
	}

	for (let i = 0; i < edgeCount; ++i) {
		let edge = edges[i];
		if (edge.triangleCount != 2) {
			continue;
		}

		if (B3_ASSERT_ENABLED) B3_ASSERT(edge.triangleEdgeIndex1 < 3);
		if (B3_ASSERT_ENABLED) B3_ASSERT(edge.triangleEdgeIndex2 < 3);

		let triangle1 = triangles[edge.triangle1];
		let triangle2 = triangles[edge.triangle2];

		let j1 = triangle2.index1;
		let j2 = triangle2.index2;
		let j3 = triangle2.index3;

		let opposite = B3_NULL_INDEX;

		switch (edge.triangleEdgeIndex2) {
			case 0:
				opposite = j3;
				break;

			case 1:
				opposite = j1;
				break;

			case 2:
				opposite = j2;
				break;

			default:
				B3_ASSERT(false);
		}

		let v1 = vertices[triangle1.index1];
		let v2 = vertices[triangle1.index2];
		let v3 = vertices[triangle1.index3];
		let p = vertices[opposite];

		let cos5Deg: f32 = 0.9962;
		let signedVolume = b3SignedVolume(v1, v2, v3, p);
		let n1 = normals[edge.triangle1];
		let n2 = normals[edge.triangle2];
		let cosAngle = b3Dot(n1, n2);
		if (signedVolume > 0.0 || cosAngle > cos5Deg) {
			flags[edge.triangle1] |= <u8>b3_concaveEdgeFlags[edge.triangleEdgeIndex1];
			flags[edge.triangle2] |= <u8>b3_concaveEdgeFlags[edge.triangleEdgeIndex2];
		}

		if (signedVolume < 0.0 || cosAngle > cos5Deg) {
			flags[edge.triangle1] |= <u8>b3_inverseConcaveEdgeFlags[edge.triangleEdgeIndex1];
			flags[edge.triangle2] |= <u8>b3_inverseConcaveEdgeFlags[edge.triangleEdgeIndex2];
		}
	}
}

export function b3CreateGridMesh(xCount: i32, zCount: i32, cellWidth: f32, materialCount: i32, identifyEdges: bool): b3MeshData? {
	B3_ASSERT(0 <= materialCount && materialCount <= 255);

	// Create vertices
	let vertexCount = (xCount + 1) * (zCount + 1);

	let vertices = new StaticArray<b3Vec3>(vertexCount);
	let index = 0;

	let xWidth = cellWidth * <f32>xCount;
	let zWidth = cellWidth * <f32>zCount;

	let x: f32 = -0.5 * xWidth;
	for (let ix = 0; ix <= xCount; ++ix) {
		let z: f32 = -0.5 * zWidth;
		for (let iz = 0; iz <= zCount; ++iz) {
			vertices[index] = b3Vec3(x, 0.0, z);
			z += cellWidth;
			index += 1;
		}
		x += cellWidth;
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(index == vertexCount);

	// Triangles
	let triangleCount = 2 * xCount * zCount;

	let indices = new StaticArray<i32>(3 * triangleCount);
	let materialIndices = new StaticArray<u8>(triangleCount);

	let materialIndex = 0;
	index = 0;
	for (let ix = 0; ix < xCount; ++ix) {
		for (let iz = 0; iz < zCount; ++iz) {
			let index1 = iz + (zCount + 1) * ix;
			let index2 = index1 + 1;
			let index3 = index2 + (zCount + 1);
			let index4 = index3 - 1;

			indices[index + 0] = index1;
			indices[index + 1] = index2;
			indices[index + 2] = index3;

			indices[index + 3] = index3;
			indices[index + 4] = index4;
			indices[index + 5] = index1;

			if (materialCount > 0) {
				materialIndices[2 * materialIndex + 0] = <u8>(materialIndex % materialCount);
				materialIndices[2 * materialIndex + 1] = <u8>(materialIndex % materialCount);
			}

			materialIndex += 1;
			index += 6;
		}
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(index == 3 * triangleCount);

	let def = new b3MeshDef();
	def.vertexCount = vertexCount;
	def.vertices = vertices;
	def.triangleCount = triangleCount;
	def.indices = indices;
	def.materialIndices = materialCount > 0 ? materialIndices : null;
	def.useMedianSplit = true;
	def.identifyEdges = identifyEdges;

	return b3CreateMesh(def, null, 0);
}

export function b3CreateWaveMesh(
	xCount: i32,
	zCount: i32,
	cellWidth: f32,
	amplitude: f32,
	rowFrequency: f32,
	columnFrequency: f32,
): b3MeshData? {
	// Create vertices
	let vertexCount = (xCount + 1) * (zCount + 1);

	let vertices = new StaticArray<b3Vec3>(vertexCount);
	let index = 0;

	let xWidth = cellWidth * <f32>xCount;
	let zWidth = cellWidth * <f32>zCount;

	let omegaZ: f32 = 2.0 * B3_PI * rowFrequency * cellWidth;
	let omegaX: f32 = 2.0 * B3_PI * columnFrequency * cellWidth;

	let x: f32 = -0.5 * xWidth;
	for (let ix = 0; ix <= xCount; ++ix) {
		let cs = b3ComputeCosSin(omegaX * <f32>ix);
		let rowHeight = cs.sine;

		let z: f32 = -0.5 * zWidth;
		for (let iz = 0; iz <= zCount; ++iz) {
			cs = b3ComputeCosSin(omegaZ * <f32>iz);
			let columnHeight = cs.sine;

			let y = amplitude * rowHeight * columnHeight;
			vertices[index] = b3Vec3(x, y, z);
			z += cellWidth;
			index += 1;
		}
		x += cellWidth;
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(index == vertexCount);

	// Triangles
	let triangleCount = 2 * xCount * zCount;

	let indices = new StaticArray<i32>(3 * triangleCount);

	index = 0;
	for (let ix = 0; ix < xCount; ++ix) {
		for (let iz = 0; iz < zCount; ++iz) {
			let index1 = iz + (zCount + 1) * ix;
			let index2 = index1 + 1;
			let index3 = index2 + (zCount + 1);
			let index4 = index3 - 1;

			indices[index + 0] = index1;
			indices[index + 1] = index2;
			indices[index + 2] = index3;

			indices[index + 3] = index3;
			indices[index + 4] = index4;
			indices[index + 5] = index1;

			index += 6;
		}
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(index == 3 * triangleCount);

	let def = new b3MeshDef();
	def.vertexCount = vertexCount;
	def.vertices = vertices;
	def.triangleCount = triangleCount;
	def.indices = indices;
	def.useMedianSplit = true;
	def.identifyEdges = true;

	return b3CreateMesh(def, null, 0);
}

export function b3CreateTorusMesh(radialResolution: i32, tubularResolution: i32, radius: f32, thickness: f32): b3MeshData? {
	// Create vertices
	let vertices = new Array<b3Vec3>();

	for (let radialIndex = 0; radialIndex < radialResolution; radialIndex++) {
		for (let tubularIndex = 0; tubularIndex < tubularResolution; tubularIndex++) {
			let u = (<f32>tubularIndex / <f32>tubularResolution) * B3_TWO_PI;
			let v = (<f32>radialIndex / <f32>radialResolution) * B3_TWO_PI;

			let x = (radius + thickness * b3Cos(v)) * b3Cos(u);
			let y = (radius + thickness * b3Cos(v)) * b3Sin(u);
			let z = thickness * b3Sin(v);

			vertices.push(b3Vec3(x, y, z));
		}
	}

	// Triangles
	let indices = new Array<i32>();
	for (let radialIndex1 = 0; radialIndex1 < radialResolution; radialIndex1++) {
		let radialIndex2 = (radialIndex1 + 1) % radialResolution;
		for (let tubularIndex1 = 0; tubularIndex1 < tubularResolution; tubularIndex1++) {
			let tubularIndex2 = (tubularIndex1 + 1) % tubularResolution;
			let index1 = radialIndex1 * tubularResolution + tubularIndex1;
			let index2 = radialIndex1 * tubularResolution + tubularIndex2;
			let index3 = radialIndex2 * tubularResolution + tubularIndex2;
			let index4 = radialIndex2 * tubularResolution + tubularIndex1;

			indices.push(index1);
			indices.push(index2);
			indices.push(index3);

			indices.push(index3);
			indices.push(index4);
			indices.push(index1);
		}
	}

	let def = new b3MeshDef();
	def.vertexCount = vertices.length;
	def.vertices = b3ToStaticVec3(vertices);
	def.triangleCount = indices.length / 3;
	def.indices = b3ToStaticI32(indices);
	def.useMedianSplit = false;
	def.identifyEdges = true;

	return b3CreateMesh(def, null, 0);
}

function b3ToStaticVec3(a: Array<b3Vec3>): StaticArray<b3Vec3> {
	let s = new StaticArray<b3Vec3>(a.length);
	for (let i = 0; i < a.length; ++i) {
		s[i] = a[i];
	}
	return s;
}

function b3ToStaticI32(a: Array<i32>): StaticArray<i32> {
	let s = new StaticArray<i32>(a.length);
	for (let i = 0; i < a.length; ++i) {
		s[i] = a[i];
	}
	return s;
}

// Shared by the box generators: the 8 corners offset by center
function b3MakeBoxVertices(center: b3Vec3, x: f32, y: f32, z: f32): StaticArray<b3Vec3> {
	let vertices = new StaticArray<b3Vec3>(8);
	vertices[0] = b3Vec3(x, y, z);
	vertices[1] = b3Vec3(-x, y, z);
	vertices[2] = b3Vec3(-x, -y, z);
	vertices[3] = b3Vec3(x, -y, z);
	vertices[4] = b3Vec3(x, y, -z);
	vertices[5] = b3Vec3(-x, y, -z);
	vertices[6] = b3Vec3(-x, -y, -z);
	vertices[7] = b3Vec3(x, -y, -z);

	for (let i = 0; i < 8; ++i) {
		vertices[i] = b3Add(vertices[i], center);
	}

	return vertices;
}

const b3_boxMeshIndices: StaticArray<i32> = [
	0, 1, 3, 1, 2, 3, // front
	0, 4, 1, 1, 4, 5, // top
	0, 3, 7, 4, 0, 7, // right
	4, 7, 5, 6, 5, 7, // back
	1, 5, 2, 6, 2, 5, // left
	3, 2, 7, 6, 7, 2, // bottom
];

const b3_hollowBoxMeshIndices: StaticArray<i32> = [
	3, 1, 0, 3, 2, 1, // front
	1, 4, 0, 5, 4, 1, // top
	7, 3, 0, 7, 0, 4, // right
	5, 7, 4, 7, 5, 6, // back
	2, 5, 1, 5, 2, 6, // left
	7, 2, 3, 2, 7, 6, // bottom
];

export function b3CreateBoxMesh(center: b3Vec3, extent: b3Vec3, identifyEdges: bool): b3MeshData? {
	let def = new b3MeshDef();
	def.vertexCount = 8;
	def.vertices = b3MakeBoxVertices(center, extent.x, extent.y, extent.z);
	def.triangleCount = 12;
	def.indices = b3_boxMeshIndices;
	def.useMedianSplit = false;
	def.identifyEdges = identifyEdges;

	return b3CreateMesh(def, null, 0);
}

export function b3CreateHollowBoxMesh(center: b3Vec3, extent: b3Vec3): b3MeshData? {
	let def = new b3MeshDef();
	def.vertexCount = 8;
	def.vertices = b3MakeBoxVertices(center, extent.x, extent.y, extent.z);
	def.triangleCount = 12;
	def.indices = b3_hollowBoxMeshIndices;
	def.useMedianSplit = false;
	def.identifyEdges = true;

	return b3CreateMesh(def, null, 0);
}

export function b3CreatePlatformMesh(center: b3Vec3, height: f32, topWidth: f32, bottomWidth: f32): b3MeshData? {
	let hb: f32 = 0.5 * bottomWidth;
	let ht: f32 = 0.5 * topWidth;
	let hy: f32 = 0.5 * height;
	let vertices = new StaticArray<b3Vec3>(8);
	vertices[0] = b3Vec3(ht, hy, ht);
	vertices[1] = b3Vec3(-ht, hy, ht);
	vertices[2] = b3Vec3(-hb, -hy, hb);
	vertices[3] = b3Vec3(hb, -hy, hb);
	vertices[4] = b3Vec3(ht, hy, -ht);
	vertices[5] = b3Vec3(-ht, hy, -ht);
	vertices[6] = b3Vec3(-hb, -hy, -hb);
	vertices[7] = b3Vec3(hb, -hy, -hb);

	for (let i = 0; i < 8; ++i) {
		vertices[i] = b3Add(vertices[i], center);
	}

	let def = new b3MeshDef();
	def.vertexCount = 8;
	def.vertices = vertices;
	def.triangleCount = 12;
	def.indices = b3_boxMeshIndices;
	def.useMedianSplit = true;
	def.identifyEdges = true;

	return b3CreateMesh(def, null, 0);
}

// Stride larger than this likely indicates stride is uninitialized memory.
const B3_MAX_STRIDE: i32 = 4096;

// The C blob image, so the hash matches C byte for byte
function b3SerializeMesh(mesh: b3MeshData): Uint8Array {
	let bytes = new Uint8Array(mesh.byteCount);
	let view = new DataView(bytes.buffer);
	view.setUint64(0, mesh.version, true);
	view.setUint64(8, mesh.hash, true);
	view.setInt32(16, mesh.byteCount, true);
	view.setFloat32(20, mesh.bounds.lowerBound.x, true);
	view.setFloat32(24, mesh.bounds.lowerBound.y, true);
	view.setFloat32(28, mesh.bounds.lowerBound.z, true);
	view.setFloat32(32, mesh.bounds.upperBound.x, true);
	view.setFloat32(36, mesh.bounds.upperBound.y, true);
	view.setFloat32(40, mesh.bounds.upperBound.z, true);
	view.setFloat32(44, mesh.surfaceArea, true);
	view.setInt32(48, mesh.treeHeight, true);
	view.setInt32(52, mesh.degenerateCount, true);
	view.setInt32(56, mesh.nodeOffset, true);
	view.setInt32(60, mesh.nodeCount, true);
	view.setInt32(64, mesh.vertexOffset, true);
	view.setInt32(68, mesh.vertexCount, true);
	view.setInt32(72, mesh.triangleOffset, true);
	view.setInt32(76, mesh.triangleCount, true);
	view.setInt32(80, mesh.materialOffset, true);
	view.setInt32(84, mesh.materialCount, true);
	view.setInt32(88, mesh.flagsOffset, true);
	view.setInt32(92, mesh.padding, true);

	let offset = mesh.nodeOffset;
	for (let i = 0; i < mesh.nodeCount; ++i) {
		let node = mesh.nodes[i];
		view.setFloat32(offset + 0, node.lowerBound.x, true);
		view.setFloat32(offset + 4, node.lowerBound.y, true);
		view.setFloat32(offset + 8, node.lowerBound.z, true);
		view.setUint32(offset + 12, b3GetMeshNodeBits(node), true);
		view.setFloat32(offset + 16, node.upperBound.x, true);
		view.setFloat32(offset + 20, node.upperBound.y, true);
		view.setFloat32(offset + 24, node.upperBound.z, true);
		view.setUint32(offset + 28, node.triangleOffset, true);
		offset += 32;
	}

	offset = mesh.vertexOffset;
	for (let i = 0; i < mesh.vertexCount; ++i) {
		let v = mesh.vertices[i];
		view.setFloat32(offset + 0, v.x, true);
		view.setFloat32(offset + 4, v.y, true);
		view.setFloat32(offset + 8, v.z, true);
		offset += 12;
	}

	offset = mesh.triangleOffset;
	for (let i = 0; i < mesh.triangleCount; ++i) {
		let t = mesh.triangles[i];
		view.setInt32(offset + 0, t.index1, true);
		view.setInt32(offset + 4, t.index2, true);
		view.setInt32(offset + 8, t.index3, true);
		offset += 12;
	}

	for (let i = 0; i < mesh.triangleCount; ++i) {
		bytes[mesh.materialOffset + i] = mesh.materialIndices[i];
		bytes[mesh.flagsOffset + i] = mesh.flags[i];
	}

	return bytes;
}

/// The 32 bits C's b3MeshNode data union holds: the leaf view when the node is a leaf.
export function b3GetMeshNodeBits(node: b3MeshNode): u32 {
	if (node.data.asLeaf.type == B3_LEAF_NODE) {
		return B3_LEAF_NODE | (<u32>node.data.asLeaf.triangleCount << 2);
	}

	return <u32>node.data.asNode.axis | (<u32>node.data.asNode.childOffset << 2);
}

// todo this should fail if the mesh has a height greater than B3_MESH_STACK_SIZE
export function b3CreateMesh(def: b3MeshDef, degenerateTriangleIndices: StaticArray<i32>?, degenerateCapacity: i32): b3MeshData? {
	if (def.stride != 0 && (def.stride < 12 || B3_MAX_STRIDE < def.stride)) {
		return null;
	}

	if ((def.stride & 3) != 0) {
		return null;
	}

	if (def.vertexCount < 3 || def.triangleCount <= 0) {
		return null;
	}

	let triangleCount = def.triangleCount;
	if (triangleCount == 0) {
		return null;
	}

	let vertexCount = def.vertexCount;

	let meshBounds = B3_BOUNDS3_EMPTY;

	// Clone indices and vertices to support welding
	let indices = new StaticArray<i32>(3 * triangleCount);
	let vertices = new StaticArray<b3Vec3>(vertexCount);

	if (def.weldVertices && def.weldTolerance > 0.0) {
		vertexCount = b3WeldVertices(def.vertices, def.indices, vertices, indices, vertexCount, 3 * triangleCount, def.weldTolerance);
		if (B3_ASSERT_ENABLED) B3_ASSERT(vertexCount <= def.vertexCount);
	} else {
		for (let i = 0; i < vertexCount; ++i) {
			vertices[i] = def.vertices[i];
		}

		for (let i = 0; i < 3 * triangleCount; ++i) {
			indices[i] = def.indices[i];
		}
	}

	let primitives = new StaticArray<b3Primitive>(triangleCount);
	let primitiveCount = 0;
	let degenerateCount = 0;
	let minArea: f32 = 0.01 * B3_LINEAR_SLOP() * B3_LINEAR_SLOP();
	let surfaceArea: f32 = 0.0;
	let materialCount = 1;
	let clockWise = def.clockWiseWinding;
	let defMaterialIndices = def.materialIndices;

	for (let index = 0; index < triangleCount; ++index) {
		let index1 = indices[3 * index + 0];
		let index2 = indices[3 * index + 1];
		let index3 = indices[3 * index + 2];

		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index1 && index1 < vertexCount);
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index2 && index2 < vertexCount);
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index3 && index3 < vertexCount);

		let vertex1 = vertices[index1];
		let vertex2 = vertices[index2];
		let vertex3 = vertices[index3];

		let normal = b3Cross(b3Sub(vertex2, vertex1), b3Sub(vertex3, vertex1));
		let area: f32 = 0.5 * b3Length(normal);

		if (area < minArea) {
			if (index1 != index2 && index1 != index3 && index2 != index3) {
				if (degenerateTriangleIndices != null && degenerateCount < degenerateCapacity) {
					degenerateTriangleIndices[degenerateCount] = index;
				}

				degenerateCount += 1;
			}

			continue;
		}

		surfaceArea += area;

		let box = b3AABB(b3Min(vertex1, b3Min(vertex2, vertex3)), b3Max(vertex1, b3Max(vertex2, vertex3)));

		let center = b3AABB_Center(box);

		ref primitive = primitives[primitiveCount];
		primitive.aabb = box;
		primitive.center = center;
		primitive.triangleIndex = index;
		primitiveCount += 1;

		if (defMaterialIndices != null) {
			materialCount = b3MaxInt(materialCount, <i32>defMaterialIndices[index] + 1);
		}

		meshBounds = b3AABB_Union(meshBounds, box);
	}

	// Update triangle count due to degenerates being skipped
	triangleCount = primitiveCount;

	if (b3IsSaneAABB(meshBounds) == false) {
		return null;
	}

	// Build the tree (this reorders the builder triangles)
	let tempNodes = new Array<b3MeshNode>();

	let treeHeight: FixedArray<i32, 1>;
	b3BuildRecursive(tempNodes, triangleCount, primitives, 0, def.useMedianSplit, treeHeight[0]);

	// Allocate the mesh. The constructor sets the counts, offsets and byteCount.
	let mesh = new b3MeshData(tempNodes.length, vertexCount, triangleCount);

	mesh.version = B3_MESH_VERSION;
	mesh.bounds = meshBounds;
	mesh.surfaceArea = surfaceArea;
	mesh.treeHeight = treeHeight[0];
	mesh.degenerateCount = degenerateCount;
	mesh.materialCount = materialCount;

	let nodes = mesh.nodes;
	let triangles = mesh.triangles;
	let meshVertices = mesh.vertices;
	let materialIndices = mesh.materialIndices;
	let flags = mesh.flags;

	for (let i = 0; i < tempNodes.length; ++i) {
		nodes[i] = tempNodes[i];
	}

	for (let i = 0; i < vertexCount; ++i) {
		meshVertices[i] = vertices[i];
	}

	for (let index = 0; index < triangleCount; ++index) {
		let primitive = primitives[index];

		let i1 = 3 * primitive.triangleIndex + 0;
		let i2 = 3 * primitive.triangleIndex + 1;
		let i3 = 3 * primitive.triangleIndex + 2;
		if (clockWise) {
			let t = i2;
			i2 = i3;
			i3 = t;
		}

		triangles[index].index1 = indices[i1];
		triangles[index].index2 = indices[i2];
		triangles[index].index3 = indices[i3];
		flags[index] = 0;

		// Copy material indices if they exist. Otherwise the material indices are all zeroes.
		if (defMaterialIndices != null) {
			let materialIndex = defMaterialIndices[primitive.triangleIndex];
			materialIndices[index] = materialIndex;
		}
	}

	// Sort triangle in DFS order. Casts and volume queries will return sorted arrays.
	// This also sorts material indices, but not the materials.
	// This can fail if the BVH height is too large.
	let success = b3SortMeshTriangles(mesh);
	if (success == false) {
		return null;
	}

	if (def.identifyEdges) {
		b3IdentifyEdges(mesh);
	}

	mesh.hash = 0;
	mesh.hash = b3Hash64NonZero(b3SerializeMesh(mesh), mesh.byteCount);

	return mesh;
}

export function b3DestroyMesh(mesh: b3MeshData): void {}

// C's stack locals of the queries below. None of them nests in another, so they share one set.
const s_queryTriangle = new StaticArray<b3Vec3>(3);
const s_queryTriangleProxy = new b3ShapeProxy();
const s_querySegment = new StaticArray<b3Vec3>(2);
const s_querySegmentProxy = new b3ShapeProxy();
const s_queryLocalPoints = new StaticArray<b3Vec3>(B3_MAX_SHAPE_CAST_POINTS);
const s_queryLocalProxy = new b3ShapeProxy();
const s_queryDistanceInput = new b3DistanceInput();
const s_queryCastInput = new b3ShapeCastPairInput();

// src/shape.c, until lib/shape.as exists
function b3ComputeProxyAABB(proxy: b3ShapeProxy): b3AABB {
	let points = proxy.points;
	let aabb = b3AABB(points[0], points[0]);

	for (let i = 1; i < proxy.count; ++i) {
		aabb.lowerBound = b3Min(aabb.lowerBound, points[i]);
		aabb.upperBound = b3Max(aabb.upperBound, points[i]);
	}

	let r = b3Vec3(proxy.radius, proxy.radius, proxy.radius);
	aabb.lowerBound = b3Sub(aabb.lowerBound, r);
	aabb.upperBound = b3Add(aabb.upperBound, r);
	return aabb;
}

export function b3OverlapMesh(shape: b3Mesh, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool {
	if (B3_ASSERT_ENABLED) B3_ASSERT(proxy.count > 0);
	let cache = b3SimplexCache();

	let localProxy = b3SetLocalProxy(s_queryLocalProxy, proxy, shapeTransform, s_queryLocalPoints);
	let aabb = b3ComputeProxyAABB(localProxy);

	let meshScale = shape.scale;

	// Scale may have reflection so min/max may become invalid when unscaled
	let scale = b3LoadV(meshScale);
	let invScale = b3DivV(b3_oneV, scale);
	let temp1 = b3MulV(invScale, b3LoadV(aabb.lowerBound));
	let temp2 = b3MulV(invScale, b3LoadV(aabb.upperBound));
	let invScaledBoundsMin = b3MinV(temp1, temp2);
	let invScaledBoundsMax = b3MaxV(temp1, temp2);
	let invScaledBoundsCenter = b3MulV(b3_halfV, b3AddV(invScaledBoundsMin, invScaledBoundsMax));
	let invScaledBoundsExtent = b3SubV(invScaledBoundsMax, invScaledBoundsCenter);

	let triangleVertices = s_queryTriangle;
	let input = s_queryDistanceInput;
	input.proxyA = b3SetProxy(s_queryTriangleProxy, triangleVertices, 3, 0.0);
	input.proxyB = localProxy;
	input.transform = b3Transform_identity;
	input.useRadii = true;

	let count = 0;
	let stack = b3_meshStack;
	let node = 0;
	let nodes = shape.data.nodes;
	let triangles = shape.data.triangles;
	let vertices = shape.data.vertices;

	while (true) {
		// Test node overlap in unscaled space
		let nodeMin = b3LoadV(nodes[node].lowerBound);
		let nodeMax = b3LoadV(nodes[node].upperBound);
		if (b3TestBoundsOverlap(nodeMin, nodeMax, invScaledBoundsMin, invScaledBoundsMax)) {
			if (b3IsLeaf(nodes, node)) {
				let triangleCount = <i32>nodes[node].data.asLeaf.triangleCount;
				let triangleOffset = <i32>nodes[node].triangleOffset;

				for (let index = 0; index < triangleCount; ++index) {
					let triangleIndex = triangleOffset + index;
					let triangle = triangles[triangleIndex];

					let vertex1 = vertices[triangle.index1];
					let vertex2 = vertices[triangle.index2];
					let vertex3 = vertices[triangle.index3];
					let v1 = b3LoadV(vertex1);
					let v2 = b3LoadV(vertex2);
					let v3 = b3LoadV(vertex3);

					// Bounding box overlap test in unscaled space
					if (b3TestBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3)) {
						// Shape-triangle overlap test in scaled space. Winding order doesn't matter.
						triangleVertices[0] = b3Mul(meshScale, vertex1);
						triangleVertices[1] = b3Mul(meshScale, vertex2);
						triangleVertices[2] = b3Mul(meshScale, vertex3);

						// reset the cache
						cache.count = 0;

						// get distance between triangle and query shape
						let output = b3ShapeDistance(input, cache, null, 0);

						let tolerance: f32 = 0.1 * B3_LINEAR_SLOP();
						if (output.distance < tolerance) {
							// overlap detected
							return true;
						}
					}
				}
			} else {
				// Recurse
				if (B3_ASSERT_ENABLED) B3_ASSERT(count <= B3_MESH_STACK_SIZE - 1);
				stack[count++] = b3GetRightChild(nodes, node);
				node = b3GetLeftChild(nodes, node);

				continue;
			}
		}

		if (count == 0) {
			break;
		}
		node = stack[--count];
	}

	return false;
}

export function b3ComputeMeshAABB(shape: b3MeshData, transform: b3Transform, scale: b3Vec3): b3AABB {
	let scaledLower = b3Mul(scale, shape.bounds.lowerBound);
	let scaledUpper = b3Mul(scale, shape.bounds.upperBound);
	let bounds = b3AABB(b3Min(scaledLower, scaledUpper), b3Max(scaledLower, scaledUpper));
	return b3AABB_Transform(transform, bounds);
}

export function b3RayCastMesh(mesh: b3Mesh, input: b3RayCastInput): b3CastOutput {
	let data = mesh.data;
	let meshScale = mesh.scale;

	let bestOutput = b3CastOutput();
	bestOutput.fraction = input.maxFraction;
	bestOutput.triangleIndex = B3_NULL_INDEX;

	let lambda = b3SplatV(input.maxFraction);

	let rayStart = b3LoadV(input.origin);
	let rayDelta = b3LoadV(input.translation);

	let scale = b3LoadV(meshScale);
	let invScale = b3DivV(b3_oneV, scale);
	let ccw = meshScale.x * meshScale.y * meshScale.z > 0.0;

	// Use the inverse scaled ray for traversal of the BVH
	let invScaledRayStart = b3MulV(invScale, rayStart);
	let invScaledRayDelta = b3MulV(invScale, rayDelta);
	let invScaledRayEnd = b3AddV(invScaledRayStart, b3MulV(lambda, invScaledRayDelta));
	let invScaledRayMin = b3MinV(invScaledRayStart, invScaledRayEnd);
	let invScaledRayMax = b3MaxV(invScaledRayStart, invScaledRayEnd);

	let count = 0;
	let stack = b3_meshStack;
	let node = 0;
	let nodes = data.nodes;
	let triangles = data.triangles;
	let vertices = data.vertices;
	let materialIndices = data.materialIndices;

	while (true) {
		// Test node/ray overlap using SAT
		let nodeMin = b3LoadV(nodes[node].lowerBound);
		let nodeMax = b3LoadV(nodes[node].upperBound);
		if (
			b3TestBoundsOverlap(nodeMin, nodeMax, invScaledRayMin, invScaledRayMax) &&
			b3TestBoundsRayOverlap(nodeMin, nodeMax, invScaledRayStart, invScaledRayDelta)
		) {
			// SAT: The node and ray overlap - process leaf node or recurse
			if (b3IsLeaf(nodes, node)) {
				let triangleCount = <i32>nodes[node].data.asLeaf.triangleCount;
				let triangleOffset = <i32>nodes[node].triangleOffset;

				for (let index = 0; index < triangleCount; ++index) {
					let triangleIndex = triangleOffset + index;
					let triangle = triangles[triangleIndex];

					// Collide ray with triangle in scaled space
					let vertex1 = b3Mul(meshScale, vertices[triangle.index1]);
					let vertex2: b3Vec3;
					let vertex3: b3Vec3;

					// The CPU should predict this branch
					if (ccw) {
						vertex2 = b3Mul(meshScale, vertices[triangle.index2]);
						vertex3 = b3Mul(meshScale, vertices[triangle.index3]);
					} else {
						vertex2 = b3Mul(meshScale, vertices[triangle.index3]);
						vertex3 = b3Mul(meshScale, vertices[triangle.index2]);
					}

					// Collide ray with triangle in scaled space
					let v1 = b3LoadV(vertex1);
					let v2 = b3LoadV(vertex2);
					let v3 = b3LoadV(vertex3);

					let alpha = b3IntersectRayTriangle(rayStart, rayDelta, v1, v2, v3);
					if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= alpha && alpha <= 1.0);

					if (alpha < bestOutput.fraction) {
						let edge1 = b3Sub(vertex2, vertex1);
						let edge2 = b3Sub(vertex3, vertex1);
						bestOutput.normal = b3Normalize(b3Cross(edge1, edge2));
						bestOutput.point = b3Add(input.origin, b3MulSV(alpha, input.translation));
						bestOutput.fraction = alpha;
						bestOutput.triangleIndex = triangleIndex;
						bestOutput.materialIndex = <i32>materialIndices[triangleIndex];
						bestOutput.hit = true;

						// Update ray bounds in unscaled space
						lambda = b3SplatV(alpha);
						invScaledRayEnd = b3AddV(invScaledRayStart, b3MulV(lambda, invScaledRayDelta));
						invScaledRayMin = b3MinV(invScaledRayStart, invScaledRayEnd);
						invScaledRayMax = b3MaxV(invScaledRayStart, invScaledRayEnd);
					}
				}
			} else {
				// Determine traversal order (front -> back) and recurse
				let axis = <i32>nodes[node].data.asNode.axis;
				if (b3GetV(invScaledRayDelta, axis) > 0.0) {
					if (B3_ASSERT_ENABLED) B3_ASSERT(count <= B3_MESH_STACK_SIZE - 1);
					stack[count++] = b3GetRightChild(nodes, node);
					node = b3GetLeftChild(nodes, node);
				} else {
					if (B3_ASSERT_ENABLED) B3_ASSERT(count <= B3_MESH_STACK_SIZE - 1);
					stack[count++] = b3GetLeftChild(nodes, node);
					node = b3GetRightChild(nodes, node);
				}

				continue;
			}
		}

		if (count == 0) {
			break;
		}
		node = stack[--count];
	}

	return bestOutput;
}

export function b3ShapeCastMesh(mesh: b3Mesh, input: b3ShapeCastInput): b3CastOutput {
	let data = mesh.data;
	let meshScale = mesh.scale;

	let bestOutput = b3CastOutput();
	bestOutput.fraction = input.maxFraction;
	bestOutput.triangleIndex = B3_NULL_INDEX;

	let lambda = b3SplatV(input.maxFraction);

	let shapeBounds = b3MakeAABB(input.proxy.points, input.proxy.count, input.proxy.radius);
	let center = b3AABB_Center(shapeBounds);
	let extents = b3AABB_Extents(shapeBounds);
	let shapeExtent = b3LoadV(extents);

	let rayStart = b3LoadV(center);
	let rayDelta = b3LoadV(input.translation);
	let rayEnd = b3AddV(rayStart, b3MulV(lambda, rayDelta));
	let rayMin = b3MinV(rayStart, rayEnd);
	let rayMax = b3MaxV(rayStart, rayEnd);

	let scale = b3LoadV(meshScale);
	let invScale = b3DivV(b3_oneV, scale);
	let absInvScale = b3AbsV(invScale);
	let ccw = meshScale.x * meshScale.y * meshScale.z > 0.0;

	// Use the inverse scaled shape cast for traversal of the BVH
	let invScaledRayStart = b3MulV(invScale, rayStart);
	let invScaledRayDelta = b3MulV(invScale, rayDelta);
	let invScaledRayEnd = b3AddV(invScaledRayStart, b3MulV(lambda, invScaledRayDelta));
	let invScaledRayMin = b3MinV(invScaledRayStart, invScaledRayEnd);
	let invScaledRayMax = b3MaxV(invScaledRayStart, invScaledRayEnd);
	let invScaledShapeExtent = b3MulV(absInvScale, shapeExtent);

	let triangleVertices = s_queryTriangle;
	let pairInput = s_queryCastInput;
	pairInput.proxyA = b3SetProxy(s_queryTriangleProxy, triangleVertices, 3, 0.0);

	let count = 0;
	let stack = b3_meshStack;
	let node = 0;
	let nodes = data.nodes;
	let triangles = data.triangles;
	let vertices = data.vertices;
	let materialIndices = data.materialIndices;

	while (true) {
		// Test node/ray overlap using SAT in unscaled space
		let nodeMin = b3SubV(b3LoadV(nodes[node].lowerBound), invScaledShapeExtent);
		let nodeMax = b3AddV(b3LoadV(nodes[node].upperBound), invScaledShapeExtent);

		if (
			b3TestBoundsOverlap(nodeMin, nodeMax, invScaledRayMin, invScaledRayMax) &&
			b3TestBoundsRayOverlap(nodeMin, nodeMax, invScaledRayStart, invScaledRayDelta)
		) {
			// SAT: The node and ray overlap - process leaf node or recurse
			if (b3IsLeaf(nodes, node)) {
				let triangleCount = <i32>nodes[node].data.asLeaf.triangleCount;
				let triangleOffset = <i32>nodes[node].triangleOffset;

				for (let index = 0; index < triangleCount; ++index) {
					let triangleIndex = triangleOffset + index;
					let triangle = triangles[triangleIndex];

					// Collide shape with triangle in scaled space
					let vertex1 = b3Mul(meshScale, vertices[triangle.index1]);
					let vertex2: b3Vec3;
					let vertex3: b3Vec3;

					// The CPU should predict this branch
					if (ccw) {
						vertex2 = b3Mul(meshScale, vertices[triangle.index2]);
						vertex3 = b3Mul(meshScale, vertices[triangle.index3]);
					} else {
						vertex2 = b3Mul(meshScale, vertices[triangle.index3]);
						vertex3 = b3Mul(meshScale, vertices[triangle.index2]);
					}

					let v1 = b3LoadV(vertex1);
					let v2 = b3LoadV(vertex2);
					let v3 = b3LoadV(vertex3);

					let triangleMin = b3SubV(b3MinV(v1, b3MinV(v2, v3)), shapeExtent);
					let triangleMax = b3AddV(b3MaxV(v1, b3MaxV(v2, v3)), shapeExtent);

					// Test extended triangle vs ray bounds overlap in scaled space. Skip edge tests.
					let overlap = b3TestBoundsOverlap(triangleMin, triangleMax, rayMin, rayMax);
					if (overlap == false) {
						continue;
					}

					// This test is in scaled space. vertices and center are scaled.
					let signedVolume = b3SignedVolume(vertex1, vertex2, vertex3, center);
					if (signedVolume < 0.0) {
						// Backside
						continue;
					}

					// Collide shape with triangle in scaled space.
					let origin = vertex1;
					triangleVertices[0] = b3Vec3_zero;
					triangleVertices[1] = b3Sub(vertex2, origin);
					triangleVertices[2] = b3Sub(vertex3, origin);
					let shiftedOrigin = b3Transform(b3Neg(origin), b3Quat_identity);

					pairInput.proxyB = input.proxy;
					pairInput.transform = shiftedOrigin;
					pairInput.maxFraction = bestOutput.fraction;
					pairInput.translationB = input.translation;
					pairInput.canEncroach = input.canEncroach;

					let pairOutput = b3ShapeCast(pairInput);

					if (pairOutput.hit) {
						pairOutput.point = b3Add(pairOutput.point, origin);

						bestOutput = pairOutput;
						bestOutput.triangleIndex = triangleIndex;
						bestOutput.materialIndex = <i32>materialIndices[triangleIndex];

						// Update ray bounds in scaled space
						lambda = b3SplatV(pairOutput.fraction);
						rayEnd = b3AddV(rayStart, b3MulV(lambda, rayDelta));
						rayMin = b3MinV(rayStart, rayEnd);
						rayMax = b3MaxV(rayStart, rayEnd);

						// Ray bounds in unscaled space
						invScaledRayEnd = b3AddV(invScaledRayStart, b3MulV(lambda, invScaledRayDelta));
						invScaledRayMin = b3MinV(invScaledRayStart, invScaledRayEnd);
						invScaledRayMax = b3MaxV(invScaledRayStart, invScaledRayEnd);
					}
				}
			} else {
				// Determine traversal order (front -> back) and recurse
				let axis = <i32>nodes[node].data.asNode.axis;
				if (b3GetV(invScaledRayDelta, axis) > 0.0) {
					if (B3_ASSERT_ENABLED) B3_ASSERT(count <= B3_MESH_STACK_SIZE - 1);
					stack[count++] = b3GetRightChild(nodes, node);
					node = b3GetLeftChild(nodes, node);
				} else {
					if (B3_ASSERT_ENABLED) B3_ASSERT(count <= B3_MESH_STACK_SIZE - 1);
					stack[count++] = b3GetLeftChild(nodes, node);
					node = b3GetRightChild(nodes, node);
				}

				continue;
			}
		}

		if (count == 0) {
			break;
		}
		node = stack[--count];
	}

	return bestOutput;
}

// C's planes pointer is planes from index planeOffset; capacity counts from there.
export function b3CollideMoverAndMesh(
	planes: StaticArray<b3PlaneResult>,
	planeOffset: i32,
	capacity: i32,
	shape: b3Mesh,
	mover: b3Capsule,
): i32 {
	if (capacity == 0) {
		return 0;
	}

	let triangleVertices = s_queryTriangle;
	let distanceInput = s_queryDistanceInput;
	distanceInput.proxyA = b3SetProxy(s_queryTriangleProxy, triangleVertices, 3, 0.0);
	s_querySegment[0] = mover.center1;
	s_querySegment[1] = mover.center2;
	distanceInput.proxyB = b3SetProxy(s_querySegmentProxy, s_querySegment, 2, 0.0);
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	let cache = b3SimplexCache();
	let radius = mover.radius;
	let center = b3Lerp(mover.center1, mover.center2, 0.5);
	let center1 = b3LoadV(mover.center1);
	let center2 = b3LoadV(mover.center2);
	let r = b3SplatV(radius);
	let boundsMin = b3SubV(b3MinV(center1, center2), r);
	let boundsMax = b3AddV(b3MaxV(center1, center2), r);

	// Scale may have reflection so min/max may become invalid when unscaled
	let meshScale = shape.scale;
	let ccw = meshScale.x * meshScale.y * meshScale.z > 0.0;
	let scale = b3LoadV(meshScale);
	let invScale = b3DivV(b3_oneV, scale);
	let temp1 = b3MulV(invScale, boundsMin);
	let temp2 = b3MulV(invScale, boundsMax);
	let invScaledBoundsMin = b3MinV(temp1, temp2);
	let invScaledBoundsMax = b3MaxV(temp1, temp2);
	let invScaledBoundsCenter = b3MulV(b3_halfV, b3AddV(invScaledBoundsMin, invScaledBoundsMax));
	let invScaledBoundsExtent = b3SubV(invScaledBoundsMax, invScaledBoundsCenter);

	let count = 0;
	let stack = b3_meshStack;
	let node = 0;
	let nodes = shape.data.nodes;
	let triangles = shape.data.triangles;
	let vertices = shape.data.vertices;
	let materialIndices = shape.data.materialIndices;

	let planeCount = 0;
	while (planeCount < capacity) {
		// Test node overlap in unscaled space
		let nodeMin = b3LoadV(nodes[node].lowerBound);
		let nodeMax = b3LoadV(nodes[node].upperBound);
		if (b3TestBoundsOverlap(nodeMin, nodeMax, invScaledBoundsMin, invScaledBoundsMax)) {
			if (b3IsLeaf(nodes, node)) {
				let triangleCount = <i32>nodes[node].data.asLeaf.triangleCount;
				let triangleOffset = <i32>nodes[node].triangleOffset;

				for (let index = 0; index < triangleCount; ++index) {
					let triangleIndex = triangleOffset + index;
					let triangle = triangles[triangleIndex];

					let vertex1 = vertices[triangle.index1];
					let vertex2 = vertices[triangle.index2];
					let vertex3 = vertices[triangle.index3];

					if (ccw == false) {
						let t = vertex2;
						vertex2 = vertex3;
						vertex3 = t;
					}

					let v1 = b3LoadV(vertex1);
					let v2 = b3LoadV(vertex2);
					let v3 = b3LoadV(vertex3);

					// Test triangle bounds overlap in unscaled space
					if (b3TestBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3)) {
						// Compute shape distance in scaled space.
						triangleVertices[0] = b3Mul(meshScale, vertex1);
						triangleVertices[1] = b3Mul(meshScale, vertex2);
						triangleVertices[2] = b3Mul(meshScale, vertex3);

						// Vertices and center are in scaled space.
						let signedVolume = b3SignedVolume(triangleVertices[0], triangleVertices[1], triangleVertices[2], center);
						if (signedVolume < 0.0) {
							// Backside
							continue;
						}

						// reset the cache
						cache.count = 0;

						// get distance between triangle and query shape
						let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0);

						if (distanceOutput.distance == 0.0) {
							// deep overlap
						} else if (distanceOutput.distance <= mover.radius) {
							ref result = planes[planeOffset + planeCount];
							result.plane = b3Plane(distanceOutput.normal, mover.radius - distanceOutput.distance);
							result.point = distanceOutput.pointA;
							result.triangleIndex = triangleIndex;
							result.childIndex = 0;
							result.materialIndex = <i32>materialIndices[triangleIndex];
							planeCount += 1;

							if (planeCount == capacity) {
								return planeCount;
							}
						}
					}
				}
			} else {
				// Recurse
				if (B3_ASSERT_ENABLED) B3_ASSERT(count <= B3_MESH_STACK_SIZE - 1);
				stack[count++] = b3GetRightChild(nodes, node);
				node = b3GetLeftChild(nodes, node);

				continue;
			}
		}

		if (count == 0) {
			break;
		}
		node = stack[--count];
	}

	return planeCount;
}

export function b3QueryMesh(mesh: b3Mesh, bounds: b3AABB, fcn: b3MeshQueryFcn): void {
	// The callback may query again, so this stack is not shared
	b3QueryMeshVisitor<b3MeshQueryFcn>(mesh, bounds, fcn, new StaticArray<i32>(B3_MESH_STACK_SIZE));
}

/// b3QueryMesh with a visitor whose visit(a, b, c, triangleIndex) method is the callback, over the caller's stack of
/// B3_MESH_STACK_SIZE, which parallel parts can run.
export function b3QueryMeshVisitor<V>(mesh: b3Mesh, bounds: b3AABB, fcn: V, stack: StaticArray<i32>): void {
	let meshScale = mesh.scale;
	let ccw = meshScale.x * meshScale.y * meshScale.z > 0.0;

	// Scale may have reflection so min/max may become invalid when unscaled
	let scale = b3LoadV(meshScale);
	let invScale = b3DivV(b3_oneV, scale);
	let temp1 = b3MulV(invScale, b3LoadV(bounds.lowerBound));
	let temp2 = b3MulV(invScale, b3LoadV(bounds.upperBound));
	let invScaledBoundsMin = b3MinV(temp1, temp2);
	let invScaledBoundsMax = b3MaxV(temp1, temp2);
	let invScaledBoundsCenter = b3MulV(b3_halfV, b3AddV(invScaledBoundsMin, invScaledBoundsMax));
	let invScaledBoundsExtent = b3SubV(invScaledBoundsMax, invScaledBoundsCenter);

	let data = mesh.data;

	let count = 0;
	let node = 0;
	let nodes = data.nodes;
	let triangles = data.triangles;
	let vertices = data.vertices;

	// Test node overlap in unscaled space
	if (
		b3TestBoundsOverlap(b3LoadV(nodes[node].lowerBound), b3LoadV(nodes[node].upperBound), invScaledBoundsMin, invScaledBoundsMax) ==
		false
	) {
		return;
	}

	while (true) {
		if (b3IsLeaf(nodes, node)) {
			let triangleCount = <i32>nodes[node].data.asLeaf.triangleCount;
			let triangleOffset = <i32>nodes[node].triangleOffset;

			for (let index = 0; index < triangleCount; ++index) {
				let triangleIndex = triangleOffset + index;
				let triangle = triangles[triangleIndex];

				let vertex1 = vertices[triangle.index1];
				let vertex2 = vertices[triangle.index2];
				let vertex3 = vertices[triangle.index3];
				let v1 = b3LoadV(vertex1);
				let v2 = b3LoadV(vertex2);
				let v3 = b3LoadV(vertex3);

				// Perform triangle overlap test in unscaled space.
				// todo it is possible that some margins are getting scaled
				if (b3TestBoundsTriangleOverlap(invScaledBoundsCenter, invScaledBoundsExtent, v1, v2, v3)) {
					let a = b3Mul(meshScale, vertex1);
					let b: b3Vec3;
					let c: b3Vec3;
					if (ccw) {
						b = b3Mul(meshScale, vertex2);
						c = b3Mul(meshScale, vertex3);
					} else {
						b = b3Mul(meshScale, vertex3);
						c = b3Mul(meshScale, vertex2);
					}

					let result = b3VisitMeshTriangle<V>(fcn, a, b, c, triangleIndex);
					if (result == false) {
						return;
					}
				}
			}
		} else {
			let leftChild = b3GetLeftChild(nodes, node);
			let rightChild = b3GetRightChild(nodes, node);

			let overlapLeft = b3TestBoundsOverlap(
				b3LoadV(nodes[leftChild].lowerBound),
				b3LoadV(nodes[leftChild].upperBound),
				invScaledBoundsMin,
				invScaledBoundsMax,
			);
			let overlapRight = b3TestBoundsOverlap(
				b3LoadV(nodes[rightChild].lowerBound),
				b3LoadV(nodes[rightChild].upperBound),
				invScaledBoundsMin,
				invScaledBoundsMax,
			);

			if (overlapLeft) {
				if (overlapRight) {
					if (B3_ASSERT_ENABLED) B3_ASSERT(count <= B3_MESH_STACK_SIZE - 1);
					stack[count++] = rightChild;
				}

				node = leftChild;
				continue;
			}

			if (overlapRight) {
				node = rightChild;
				continue;
			}
		}

		if (count == 0) {
			break;
		}
		node = stack[--count];
	}
}
