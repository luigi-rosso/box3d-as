// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/mesh_contact.c

import { Parallel } from "rive/parallel";
import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MAX, b3Log } from "./core";
import {
	B3_LINEAR_SLOP,
	B3_MAX_AABB_MARGIN,
	B3_MAX_MANIFOLD_POINTS,
	B3_MAX_MESH_CONTACT_TRIANGLES,
	B3_MAX_WORKERS,
	B3_MESH_REST_OFFSET,
	B3_SPECULATIVE_DISTANCE,
} from "./constants";
import {
	b3AABB,
	b3AABB_Contains,
	b3AABB_Transform,
	b3AbsFloat,
	b3Add,
	b3ClampInt,
	b3Cross,
	b3Dot,
	b3InvMulWorldTransforms,
	b3InvertTransform,
	b3IsValidFloat,
	b3MakeMatrixFromQuat,
	b3MaxFloat,
	b3Matrix3,
	b3MaxInt,
	b3MinFloat,
	b3MinInt,
	b3Mul,
	b3MulMV,
	b3MulSV,
	b3Perp,
	b3Pos_zero,
	b3RotateVector,
	b3Sub,
	b3SubPos,
	b3ToRelativeTransform,
	b3Vec2,
	b3Vec3,
	b3Vec3_one,
	b3Vec3_zero,
	b3WorldTransform,
} from "./math_functions";
import { b3Cross2, b3DistanceSquared2, b3MakeNormalFromPoints, b3Sub2, b3Triangle } from "./math_internal";
import {
	b3GetHeightFieldMaterialIndices,
	b3GetMeshMaterialIndices,
	b3GetMeshTriangles,
	b3GetMeshVertices,
	b3LocalManifold,
	b3LocalManifoldPoint,
	b3Manifold,
	b3HullData,
	b3MeshTriangle,
	b3Point2D,
	b3SATCache,
	b3TriangleFeature,
	b3_allFlatEdges,
	b3_edgePairAxis,
	b3_featureEdge1,
	b3_featureEdge2,
	b3_featureEdge3,
	b3_featureHullFace,
	b3_featureNone,
	b3_featureTriangleFace,
	b3_featureVertex1,
	b3_featureVertex2,
	b3_featureVertex3,
	b3_flatEdge1,
	b3_flatEdge2,
	b3_flatEdge3,
} from "./collision";
import { b3SurfaceMaterial, b3_capsuleShape, b3_compoundShape, b3_heightShape, b3_hullShape, b3_meshShape, b3_sphereShape } from "./types";
import {
	B3_FORCE_GHOST_COLLISIONS,
	MP_PERSISTED,
	MP_TOTALNORMALIMPULSE,
	MP_TRIANGLEINDEX,
	b3Contact,
	b3MeshContact,
	b3ContactCache,
	b3ManifoldPointAddress,
	b3StorePointAnchors,
	b3TriangleCache,
	b3_enableSpeculativePoints,
} from "./contact";
import { b3MakeFeatureId } from "./manifold";
import { b3AllocateManifolds, b3MixFriction, b3MixRestitution, b3World } from "./physics_world";
import { b3Shape, b3GetMeshTriangle, b3GetShapeMaterials } from "./shape";
import {
	b3AddW4,
	b3FloatW4,
	b3GreaterThanW4,
	b3LessThanW4,
	b3MaxW4,
	b3MinW4,
	b3MoveMaskW4,
	b3MulW4,
	b3OrW4,
	b3SetW4,
	b3SplatW4,
	b3StoreW4,
	b3SubW4,
	b3AddL,
	b3MaxL,
	b3MinL,
	b3MulL,
	b3SetL,
	b3SplatL,
	b3StoreL,
	b3StoreRow,
	b3SubL,
} from "./simd";
import { b3CTZ32 } from "./ctz";
import { Q_LOOP, Q_STACK_SIZE } from "./qsort";
import { B3_MESH_STACK_SIZE, b3QueryMeshVisitor } from "./mesh";
import { b3GetHeightFieldTriangle, b3QueryHeightFieldVisitor } from "./height_field";
import { b3CollideTriangleAndCapsule, b3CollideTriangleAndHull, b3CollideTriangleAndSphere } from "./triangle_manifold";

const B3_MAX_POINTS_PER_TRIANGLE: i32 = 32;

class b3TriangleQueryContext {
	indices: StaticArray<i32> = new StaticArray<i32>(B3_MAX_MESH_CONTACT_TRIANGLES);
	capacity: i32 = 0;
	count: i32 = 0;

	// The callback of the triangle query, called directly
	visit(a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool {
		if (this.count == this.capacity) {
			return false;
		}
		this.indices[this.count] = triangleIndex;
		this.count += 1;
		return this.count < this.capacity;
	}
}

@inline function b3ScratchCapacity(current: i32, needed: i32): i32 {
	return b3MaxInt(needed, 2 * current);
}

let s_once = false;

// C's refresh in two halves. This one finds the triangles of the enlarged query bounds into the worker's scratch and
// changes nothing else, so a parallel part can see whether the refresh fits before it commits to the contact.
function b3QueryCacheTriangles(meshContact: b3MeshContact, shapeA: b3Shape, xfA: b3WorldTransform, bounds: b3AABB, scratch: b3MeshScratch): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(shapeA.type == b3_meshShape || shapeA.type == b3_heightShape);

	// Enlarge to the query bounds to absorb small movement
	let radius = B3_MAX_AABB_MARGIN() + B3_SPECULATIVE_DISTANCE();
	let extension = b3Vec3(radius, radius, radius);
	let queryBounds = b3AABB(b3Sub(bounds.lowerBound, extension), b3Add(bounds.upperBound, extension));
	scratch.refreshBounds = queryBounds;

	// Query triangles
	let query = scratch.triangleQuery;
	query.capacity = B3_MAX_MESH_CONTACT_TRIANGLES;
	query.count = 0;

	// Bounds are in world space. Convert to the local mesh frame. The broadphase bounds are float,
	// so the demoted mesh transform is the matching float world frame (exact in float mode).
	let meshTransform = b3ToRelativeTransform(xfA, b3Pos_zero);
	let localBounds = b3AABB_Transform(b3InvertTransform(meshTransform), queryBounds);
	if (shapeA.type == b3_meshShape) {
		b3QueryMeshVisitor<b3TriangleQueryContext>(shapeA.mesh!, localBounds, query, scratch.meshStack);
	} else {
		if (B3_ASSERT_ENABLED) B3_ASSERT(shapeA.type == b3_heightShape);
		b3QueryHeightFieldVisitor<b3TriangleQueryContext>(shapeA.heightField!, localBounds, query);
	}
	return query.count;
}

// The second half: the new query bounds, and the new cache matched with the old one
unsafe function b3SaveCacheTriangles(meshContact: b3MeshContact, shapeA: b3Shape, triangleCount: i32, scratch: b3MeshScratch): void {
	let triangleCache = meshContact.triangleCache;
	let triangleIndices = scratch.triangleQuery.indices;
	meshContact.queryBounds = scratch.refreshBounds;

	// Triangle indices must be sorted to match caches.

	// Create new contact cache and match with old one
	let contactCache = scratch.contactCaches;

	let index2 = 0;
	for (let index1 = 0; index1 < triangleCount; ++index1) {
		while (index2 < triangleCache.length && triangleCache[index2].triangleIndex < triangleIndices[index1]) {
			index2 += 1;
		}

		if (index2 < triangleCache.length && triangleCache[index2].triangleIndex == triangleIndices[index1]) {
			contactCache[index1] = triangleCache[index2].cache;
		} else {
			contactCache[index1] = b3ContactCache();
		}
	}

	// Save new cache. A parallel part only refreshes a cache with the capacity, b3MeshContactFitsPart.
	if (Parallel.inPart()) {
		b3SetTriangleCacheLength(triangleCache, triangleCount);
	} else {
		triangleCache.length = triangleCount;
		s_meshTriangleHighWater = b3MaxInt(s_meshTriangleHighWater, triangleCount);
	}
	for (let i = 0; i < triangleCount; ++i) {
		triangleCache[i] = b3TriangleCache(triangleIndices[i], contactCache[i]);

		if (B3_ASSERT_ENABLED && shapeA.type == b3_meshShape) {
			B3_ASSERT(0 <= triangleCache[i].triangleIndex && triangleCache[i].triangleIndex < shapeA.mesh!.data.triangleCount);
		}
	}
}

// Array#length's capacity and its setter without the growth, which parallel parts may not run, over std's Array fields
@inline unsafe function b3TriangleCacheCapacity(triangleCache: Array<b3TriangleCache>): i32 {
	let byteLength = load<i32>(changetype<usize>(triangleCache) + offsetof<Array<b3TriangleCache>>("byteLength"));
	return byteLength / <i32>sizeof<b3TriangleCache>();
}

@inline unsafe function b3SetTriangleCacheLength(triangleCache: Array<b3TriangleCache>, length: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(length <= b3TriangleCacheCapacity(triangleCache));
	store<i32>(changetype<usize>(triangleCache) + offsetof<Array<b3TriangleCache>>("length_"), length);
}

unsafe function b3RefreshCache(contact: b3Contact, shapeA: b3Shape, xfA: b3WorldTransform, bounds: b3AABB, scratch: b3MeshScratch): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(shapeA.type == b3_meshShape || shapeA.type == b3_heightShape);

	let meshContact = contact.meshContact;
	let triangleCache = meshContact.triangleCache;

	// If the dynamic body didn't move out of the cached query bounds we are done!
	if (b3AABB_Contains(meshContact.queryBounds, bounds)) {
		if (B3_ASSERT_ENABLED && shapeA.type == b3_meshShape) {
			for (let i = 0; i < triangleCache.length; ++i) {
				B3_ASSERT(0 <= triangleCache[i].triangleIndex && triangleCache[i].triangleIndex < shapeA.mesh!.data.triangleCount);
			}
		}

		return;
	}

	let triangleCount = b3QueryCacheTriangles(meshContact, shapeA, xfA, bounds, scratch);

	let triangleCapacity = B3_MAX_MESH_CONTACT_TRIANGLES;
	if (triangleCount == triangleCapacity) {
		if (s_once == false) {
			b3Log("WARNING: complex mesh detected, triangle buffer capacity of " + triangleCapacity.toString() + " reached");
			s_once = true;
		}
	}

	b3SaveCacheTriangles(meshContact, shapeA, triangleCount, scratch);
}

struct b3TentativeTriangle {
	squaredDistance: f32 = 0;
	index: i32 = 0;
}

const B3_MAX_EDGE_COUNT: i32 = 64;

struct b3FoundEdges {
	// B3_MAX_EDGE_COUNT
	keys: FixedArray<u64, 64>;
	count: i32 = 0;
}

@inline function b3AddEdge(edges: ref b3FoundEdges, vertex1: i32, vertex2: i32): bool {
	let i1 = <u64>b3MinInt(vertex1, vertex2);
	let i2 = <u64>b3MaxInt(vertex1, vertex2);
	let key = (i1 << 32) | i2;

	let count = edges.count;
	for (let i = 0; i < count; ++i) {
		if (edges.keys[i] == key) {
			return false;
		}
	}

	if (count == B3_MAX_EDGE_COUNT) {
		// This will lead to a potential ghost collision
		return true;
	}

	edges.keys[count] = key;
	edges.count += 1;

	return true;
}

@inline function b3FindEdge(edges: ref b3FoundEdges, vertex1: i32, vertex2: i32): bool {
	let i1 = <u64>b3MinInt(vertex1, vertex2);
	let i2 = <u64>b3MaxInt(vertex1, vertex2);
	let key = (i1 << 32) | i2;

	let count = edges.count;
	for (let i = 0; i < count; ++i) {
		if (edges.keys[i] == key) {
			return true;
		}
	}

	return false;
}

const B3_MAX_VERTEX_COUNT: i32 = 64;

struct b3FoundVertices {
	// B3_MAX_VERTEX_COUNT
	keys: FixedArray<i32, 64>;
	count: i32 = 0;
}


@inline function b3AddVertex(vertices: ref b3FoundVertices, vertex: i32): bool {
	let key = vertex;

	let count = vertices.count;
	for (let i = 0; i < count; ++i) {
		if (vertices.keys[i] == key) {
			return false;
		}
	}

	if (count == B3_MAX_VERTEX_COUNT) {
		// This will lead to a potential ghost collision
		return true;
	}

	vertices.keys[count] = key;
	vertices.count += 1;

	return true;
}

// B3_MAX_MANIFOLD_POINTS == 4

// Returns true if (score, separation) should replace (bestScore, bestSeparation).
@inline function b3IsBetterCullCandidate(
	score: f32,
	separation: f32,
	bestScore: f32,
	bestSeparation: f32,
	scoreTol: f32,
	separationTol: f32,
): bool {
	if (score > bestScore + scoreTol) {
		return true;
	}
	if (score < bestScore - scoreTol) {
		return false;
	}

	// Break the tie using separation
	return separation < bestSeparation - separationTol;
}

function b3CullPoints(points: StaticArray<b3Point2D>, count: i32): i32 {
	if (count <= 1) {
		return count;
	}

	let tol: f32 = 0.25 * B3_LINEAR_SLOP();
	let tolSqr = tol * tol;
	let separationTol = B3_LINEAR_SLOP();

	let finalPoints: FixedArray<b3Point2D, 4>;
	let count1 = count;

	// Step 1: the two points with the largest distance, ties broken by deepest combined separation
	let bestScore: f32 = 0.0;
	let bestSeparation = FLT_MAX;
	let bestIndex1 = B3_NULL_INDEX;
	let bestIndex2 = B3_NULL_INDEX;

	for (let i = 0; i < count1; ++i) {
		let p1 = points[i].p;
		for (let j = i + 1; j < count1; ++j) {
			let score = b3DistanceSquared2(p1, points[j].p);
			// Separation sum heuristic
			let separation = points[i].separation + points[j].separation;

			if (b3IsBetterCullCandidate(score, separation, bestScore, bestSeparation, tolSqr, separationTol)) {
				bestIndex1 = i;
				bestIndex2 = j;
				bestScore = score;
				bestSeparation = separation;
			}
		}
	}

	if (bestScore < tolSqr) {
		// Choose deepest point
		let deepestIndex = 0;
		for (let i = 1; i < count1; ++i) {
			if (points[i].separation < points[deepestIndex].separation) {
				deepestIndex = i;
			}
		}

		if (deepestIndex != 0) {
			points[0] = points[deepestIndex];
		}
		return 1;
	}

	finalPoints[0] = points[bestIndex1];
	finalPoints[1] = points[bestIndex2];

	// Cull
	points[bestIndex2] = points[count1 - 1];
	points[bestIndex1] = points[count1 - 2];
	count1 -= 2;

	if (count1 == 0) {
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		return 2;
	}

	// First anchor point
	let a = finalPoints[0].p;

	// Second anchor point
	let b = finalPoints[1].p;
	let ba = b3Sub2(b, a);
	// float length = b3Length2( ba );
	// float areaTol = tol * length;

	// Step 2: find the point with the maximum triangular area, ties broken by deepest separation
	bestScore = 0.0;
	bestSeparation = FLT_MAX;
	let bestIndex = B3_NULL_INDEX;
	let bestSignedArea: f32 = 0.0;
	for (let i = 0; i < count1; ++i) {
		let p = points[i].p;
		let signedArea = b3Cross2(ba, b3Sub2(p, a));
		let score = b3AbsFloat(signedArea);

		if (b3IsBetterCullCandidate(score, points[i].separation, bestScore, bestSeparation, tolSqr, separationTol)) {
			bestSignedArea = signedArea;
			bestScore = score;
			bestSeparation = points[i].separation;
			bestIndex = i;
		}
	}

	if (bestIndex == B3_NULL_INDEX) {
		// All points collinear
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		return 2;
	}

	// Store best point
	finalPoints[2] = points[bestIndex];

	if (count1 == 1) {
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		points[2] = finalPoints[2];
		return 3;
	}

	// Cull
	points[bestIndex] = points[count1 - 1];
	count1 -= 1;

	// Step 4: get the point that adds the most area outside the current triangle

	// Third anchor
	let c = finalPoints[2].p;

	// Ensure CCW ordering
	if (bestSignedArea < 0.0) {
		let t = b;
		b = c;
		c = t;
		ba = b3Sub2(b, a);
	}

	let cb = b3Sub2(c, b);
	let ac = b3Sub2(a, c);

	bestScore = 0.0;
	bestSeparation = FLT_MAX;
	bestIndex = B3_NULL_INDEX;
	for (let i = 0; i < count1; ++i) {
		let p = points[i].p;
		let u1 = b3Cross2(b3Sub2(p, a), ba);
		let u2 = b3Cross2(b3Sub2(p, b), cb);
		let u3 = b3Cross2(b3Sub2(p, c), ac);
		let score = b3MaxFloat(u1, b3MaxFloat(u2, u3));

		// Use the area tolerance for collinear points and hysteresis
		if (b3IsBetterCullCandidate(score, points[i].separation, bestScore, bestSeparation, tolSqr, separationTol)) {
			bestScore = score;
			bestSeparation = points[i].separation;
			bestIndex = i;
		}
	}

	if (bestIndex == B3_NULL_INDEX) {
		// No additional area
		points[0] = finalPoints[0];
		points[1] = finalPoints[1];
		points[2] = finalPoints[2];
		return 3;
	}

	// Store best point
	finalPoints[3] = points[bestIndex];

	// Full quad
	points[0] = finalPoints[0];
	points[1] = finalPoints[1];
	points[2] = finalPoints[2];
	points[3] = finalPoints[3];
	return 4;
}

// C's points pointer is points from pointOffset.
function b3ReduceCluster(
	points: StaticArray<b3LocalManifoldPoint>,
	pointOffset: i32,
	count1: i32,
	normal: b3Vec3,
	pts: StaticArray<b3Point2D>,
): i32 {
	let targetCount = 1;
	if (count1 <= targetCount) {
		return count1;
	}

	let u = b3Perp(normal);
	let v = b3Cross(normal, u);
	let origin = points[pointOffset].point;

	for (let i = 0; i < count1; ++i) {
		let d = b3Sub(points[pointOffset + i].point, origin);
		pts[i].p = b3Vec2(b3Dot(d, u), b3Dot(d, v));
		pts[i].separation = points[pointOffset + i].separation;
		pts[i].originalIndex = i;
	}

	// B3_MAX_MANIFOLD_POINTS == 4
	let hull = pts;
	let count2 = b3CullPoints(pts, count1);
	if (B3_ASSERT_ENABLED) B3_ASSERT(count2 <= B3_MAX_MANIFOLD_POINTS);

	// B3_MAX_MANIFOLD_POINTS
	let finalPoints: FixedArray<b3LocalManifoldPoint, 4>;
	for (let i = 0; i < count2; ++i) {
		let index = hull[i].originalIndex;
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index && index < count1);
		finalPoints[i] = points[pointOffset + index];
	}

	for (let i = 0; i < count2; ++i) {
		points[pointOffset + i] = finalPoints[i];
	}
	return count2;
}

// QSORT's LESS and SWAP over the tentative triangles, as direct calls
class b3TentativeSorter {
	triangles: StaticArray<b3TentativeTriangle> = new StaticArray<b3TentativeTriangle>(0);

	@inline less(i: i32, j: i32): bool {
		return this.triangles[i].squaredDistance < this.triangles[j].squaredDistance;
	}

	@inline swap(i: i32, j: i32): void {
		let tmp = this.triangles[i];
		this.triangles[i] = this.triangles[j];
		this.triangles[j] = tmp;
	}
}


// C's points pointer is the cluster point buffer from pointOffset.
struct b3Cluster {
	manifoldNormal: b3Vec3 = b3Vec3();
	triangleNormal: b3Vec3 = b3Vec3();
	pointOffset: i32 = 0;
	pointCapacity: i32 = 0;
	pointCount: i32 = 0;
}

// The scratch of b3ComputeMeshManifolds, which does not nest, one set per worker since parallel parts collide at once.
// Buffers sized by the triangle count grow on the main lane only.
class b3MeshScratch {
	acceptedManifolds: StaticArray<i32> = new StaticArray<i32>(0);
	tentativeManifolds: StaticArray<i32> = new StaticArray<i32>(0);
	tentativeTriangles: StaticArray<b3TentativeTriangle> = new StaticArray<b3TentativeTriangle>(0);
	survivorIndices: StaticArray<i32> = new StaticArray<i32>(0);
	survivorVertices: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(0);
	clusters: StaticArray<b3Cluster> = new StaticArray<b3Cluster>(0);
	clusterMemberships: StaticArray<i32> = new StaticArray<i32>(0);
	oldManifolds: StaticArray<b3Manifold> = new StaticArray<b3Manifold>(0);
	consumed: StaticArray<bool> = new StaticArray<bool>(0);
	pointBuffer: StaticArray<b3LocalManifoldPoint> = new StaticArray<b3LocalManifoldPoint>(0);
	clusterPoints: StaticArray<b3LocalManifoldPoint> = new StaticArray<b3LocalManifoldPoint>(0);
	cullPoints: StaticArray<b3Point2D> = new StaticArray<b3Point2D>(0);
	// C bumps triangleCount manifolds; each slot points at pointBuffer
	manifoldBuffer: Array<b3LocalManifold> = new Array<b3LocalManifold>();
	laneX: StaticArray<f32> = new StaticArray<f32>(12);
	laneY: StaticArray<f32> = new StaticArray<f32>(12);
	laneZ: StaticArray<f32> = new StaticArray<f32>(12);
	triangleVertices: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(3);
	tentativeSorter: b3TentativeSorter = new b3TentativeSorter();
	sortStack: StaticArray<i32> = new StaticArray<i32>(Q_STACK_SIZE);
	foundEdges: b3FoundEdges = b3FoundEdges();
	foundVertices: b3FoundVertices = b3FoundVertices();
	// A cache refresh: the triangle query, its stack, and the matched caches
	triangleQuery: b3TriangleQueryContext = new b3TriangleQueryContext();
	meshStack: StaticArray<i32> = new StaticArray<i32>(B3_MESH_STACK_SIZE);
	contactCaches: StaticArray<b3ContactCache> = new StaticArray<b3ContactCache>(B3_MAX_MESH_CONTACT_TRIANGLES);
	// A refresh b3MeshContactFitsPart queried for the contact the part updates next
	refreshReady: bool = false;
	refreshCount: i32 = 0;
	refreshBounds: b3AABB = b3AABB();
}

function b3MakeMeshScratch(): StaticArray<b3MeshScratch> {
	let scratch = new StaticArray<b3MeshScratch>(B3_MAX_WORKERS);
	for (let i = 0; i < B3_MAX_WORKERS; ++i) {
		scratch[i] = new b3MeshScratch();
	}
	return scratch;
}

// One per worker, and each part uses its worker's
@parallel.shared const s_meshScratch = b3MakeMeshScratch();

// The most triangles any mesh contact's cache has held, which sizes every worker's scratch before a parallel collide
let s_meshTriangleHighWater = 0;

// Grows the scratch for a contact of triangleCount cached triangles, or as many old manifolds
function b3ReserveMeshScratch(scratch: b3MeshScratch, triangleCount: i32): void {
	if (scratch.acceptedManifolds.length < triangleCount) {
		let capacity = b3ScratchCapacity(scratch.acceptedManifolds.length, triangleCount);
		scratch.acceptedManifolds = new StaticArray<i32>(capacity);
		scratch.tentativeManifolds = new StaticArray<i32>(capacity);
		scratch.tentativeTriangles = new StaticArray<b3TentativeTriangle>(capacity);
		scratch.survivorIndices = new StaticArray<i32>(capacity);
		scratch.survivorVertices = new StaticArray<b3Vec3>(3 * capacity);
		scratch.clusters = new StaticArray<b3Cluster>(capacity);
		scratch.clusterMemberships = new StaticArray<i32>(capacity);
		scratch.oldManifolds = new StaticArray<b3Manifold>(capacity);
		scratch.consumed = new StaticArray<bool>(capacity);
		scratch.tentativeSorter.triangles = scratch.tentativeTriangles;
	}

	let pointCapacity = B3_MAX_POINTS_PER_TRIANGLE * triangleCount;
	if (scratch.pointBuffer.length < pointCapacity) {
		let capacity = b3ScratchCapacity(scratch.pointBuffer.length, pointCapacity);
		scratch.pointBuffer = new StaticArray<b3LocalManifoldPoint>(capacity);
		scratch.clusterPoints = new StaticArray<b3LocalManifoldPoint>(capacity);
		scratch.cullPoints = new StaticArray<b3Point2D>(capacity);
		let manifoldBuffer = scratch.manifoldBuffer;
		for (let i = 0; i < manifoldBuffer.length; ++i) {
			manifoldBuffer[i].points = scratch.pointBuffer;
		}
	}

	while (scratch.manifoldBuffer.length < triangleCount) {
		let manifold = new b3LocalManifold();
		manifold.points = scratch.pointBuffer;
		scratch.manifoldBuffer.push(manifold);
	}
}

// Sizes the scratch of workers 0 to workerCount - 1 for any contact a parallel part may take
export function b3ReserveMeshScratchForWorkers(workerCount: i32): void {
	for (let i = 0; i < workerCount; ++i) {
		b3ReserveMeshScratch(s_meshScratch[i], s_meshTriangleHighWater);
	}
}

// Whether a parallel part can update this mesh contact, before it changes it: its triangle cache needs no refresh, or one
// that fits the cache's capacity and is not of a compound child, and its manifolds and the worker's scratch have room for
// every triangle. A refresh it queries is left in the worker's scratch for b3ComputeMeshManifolds.
export unsafe function b3MeshContactFitsPart(contact: b3Contact, shapeA: b3Shape, xfA: b3WorldTransform, boundsB: b3AABB, workerIndex: i32): bool {
	let meshContact = contact.meshContact;
	let scratch = s_meshScratch[workerIndex];
	scratch.refreshReady = false;

	let triangleCount = meshContact.triangleCache.length;
	let refresh = b3AABB_Contains(meshContact.queryBounds, boundsB) == false;
	if (refresh) {
		if (shapeA.type == b3_compoundShape) {
			return false;
		}

		// A full query logs a warning, which only the main lane can
		triangleCount = b3QueryCacheTriangles(meshContact, shapeA, xfA, boundsB, scratch);
		if (triangleCount == B3_MAX_MESH_CONTACT_TRIANGLES || triangleCount > b3TriangleCacheCapacity(meshContact.triangleCache)) {
			return false;
		}
	}

	let fits = contact.manifolds.length >= triangleCount &&
		scratch.acceptedManifolds.length >= b3MaxInt(triangleCount, contact.manifoldCount) &&
		scratch.manifoldBuffer.length >= triangleCount && scratch.pointBuffer.length >= B3_MAX_POINTS_PER_TRIANGLE * triangleCount;
	if (fits && refresh) {
		scratch.refreshReady = true;
		scratch.refreshCount = triangleCount;
	}
	return fits;
}

// Checked reads for b3ComputeMeshManifolds, which is otherwise unchecked: every scratch buffer,
// cache and manifold index there is bounded where the buffer is sized, while these index shape data
// through values the shape supplies.
function b3MeshTriangleAt(triangles: StaticArray<b3MeshTriangle>, index: i32): b3MeshTriangle {
	return triangles[index];
}

function b3MeshVertexAt(vertices: StaticArray<b3Vec3>, index: i32): b3Vec3 {
	return vertices[index];
}

function b3MaterialIndexAt(indices: StaticArray<u8>, index: i32): u8 {
	return indices[index];
}

function b3MaterialMapAt(map: Uint16Array, index: i32): u16 {
	return map[index];
}

function b3SurfaceMaterialAt(materials: StaticArray<b3SurfaceMaterial>, index: i32): b3SurfaceMaterial {
	return materials[index];
}

// C's memcpy and memset of manifold arrays; both counts are within the arrays
unsafe function b3CopyManifolds(dst: StaticArray<b3Manifold>, src: StaticArray<b3Manifold>, count: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(count <= dst.length && count <= src.length);
	memory.copy(changetype<usize>(dst), changetype<usize>(src), <usize>count * sizeof<b3Manifold>());
}

unsafe function b3ZeroManifolds(manifolds: StaticArray<b3Manifold>, count: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(count <= manifolds.length);
	memory.fill(changetype<usize>(manifolds), 0, <usize>count * sizeof<b3Manifold>());
}

// The four lane prefilter of b3ComputeMeshManifolds: transforms the cached triangles into the frame of shape B, culls
// those outside the hull bounds and writes the survivors. Out of line, so its v128 lanes stay out of the caller's frame.
@unchecked
@noinline
function b3PrefilterTriangles(
	shapeA: b3Shape,
	triangleCaches: Array<b3TriangleCache>,
	triangleCount: i32,
	meshTriangles: StaticArray<b3MeshTriangle>?,
	meshVertices: StaticArray<b3Vec3>?,
	meshScale: b3Vec3,
	meshFlipped: bool,
	relativeMatrix: b3Matrix3,
	translation: b3Vec3,
	hullB: b3HullData?,
	hullMarginValue: f32,
	survivorIndices: StaticArray<i32>,
	survivorVertices: StaticArray<b3Vec3>,
	scratch: b3MeshScratch,
): i32 {
	let survivorCount = 0;
	let hullMargin = b3Vec3(hullMarginValue, hullMarginValue, hullMarginValue);
	let cxx = b3SplatW4(relativeMatrix.cx.x);
	let cxy = b3SplatW4(relativeMatrix.cx.y);
	let cxz = b3SplatW4(relativeMatrix.cx.z);
	let cyx = b3SplatW4(relativeMatrix.cy.x);
	let cyy = b3SplatW4(relativeMatrix.cy.y);
	let cyz = b3SplatW4(relativeMatrix.cy.z);
	let czx = b3SplatW4(relativeMatrix.cz.x);
	let czy = b3SplatW4(relativeMatrix.cz.y);
	let czz = b3SplatW4(relativeMatrix.cz.z);
	let px = b3SplatW4(translation.x);
	let py = b3SplatW4(translation.y);
	let pz = b3SplatW4(translation.z);

	let marginW = b3SplatW4(hullMargin.x);
	let hullBounds = hullB != null ? hullB.aabb : b3AABB();
	let hullLowerX = b3SplatW4(hullBounds.lowerBound.x);
	let hullLowerY = b3SplatW4(hullBounds.lowerBound.y);
	let hullLowerZ = b3SplatW4(hullBounds.lowerBound.z);
	let hullUpperX = b3SplatW4(hullBounds.upperBound.x);
	let hullUpperY = b3SplatW4(hullBounds.upperBound.y);
	let hullUpperZ = b3SplatW4(hullBounds.upperBound.z);

	// C's x[3][4], y[3][4], z[3][4], row k at index 4 * k
	let x = scratch.laneX;
	let y = scratch.laneY;
	let z = scratch.laneZ;

	for (let baseIndex = 0; baseIndex < triangleCount; baseIndex += 4) {
		let laneCount = b3MinInt(4, triangleCount - baseIndex);

		let lanes: FixedArray<FixedArray<b3Vec3, 4>, 3>;
		for (let lane = 0; lane < 4; ++lane) {
			let index = baseIndex + b3MinInt(lane, laneCount - 1);
			let triangleIndex = triangleCaches[index].triangleIndex;

			if (meshVertices != null) {
				let meshTriangle = b3MeshTriangleAt(meshTriangles!, triangleIndex);
				lanes[0][lane] = b3Mul(meshScale, b3MeshVertexAt(meshVertices, meshTriangle.index1));
				lanes[1][lane] = b3Mul(meshScale, b3MeshVertexAt(meshVertices, meshFlipped ? meshTriangle.index3 : meshTriangle.index2));
				lanes[2][lane] = b3Mul(meshScale, b3MeshVertexAt(meshVertices, meshFlipped ? meshTriangle.index2 : meshTriangle.index3));
			} else {
				if (B3_ASSERT_ENABLED) B3_ASSERT(shapeA.type == b3_heightShape);
				let triangle = b3GetHeightFieldTriangle(shapeA.heightField!, triangleIndex);
				lanes[0][lane] = triangle.vertices[0];
				lanes[1][lane] = triangle.vertices[1];
				lanes[2][lane] = triangle.vertices[2];
			}
		}

		let keepMask: u32 = (<u32>1 << <u32>laneCount) - 1;
		if (ASC_FEATURE_SIMD) {
			// The lane math below on v128 lanes, which notes/containers.md shows exact
			let v0x = b3TransformLaneL<v128>(cxx.x, cyx.x, czx.x, px.x, lanes[0][0], lanes[0][1], lanes[0][2], lanes[0][3]);
			let v0y = b3TransformLaneL<v128>(cxy.x, cyy.x, czy.x, py.x, lanes[0][0], lanes[0][1], lanes[0][2], lanes[0][3]);
			let v0z = b3TransformLaneL<v128>(cxz.x, cyz.x, czz.x, pz.x, lanes[0][0], lanes[0][1], lanes[0][2], lanes[0][3]);
			let v1x = b3TransformLaneL<v128>(cxx.x, cyx.x, czx.x, px.x, lanes[1][0], lanes[1][1], lanes[1][2], lanes[1][3]);
			let v1y = b3TransformLaneL<v128>(cxy.x, cyy.x, czy.x, py.x, lanes[1][0], lanes[1][1], lanes[1][2], lanes[1][3]);
			let v1z = b3TransformLaneL<v128>(cxz.x, cyz.x, czz.x, pz.x, lanes[1][0], lanes[1][1], lanes[1][2], lanes[1][3]);
			let v2x = b3TransformLaneL<v128>(cxx.x, cyx.x, czx.x, px.x, lanes[2][0], lanes[2][1], lanes[2][2], lanes[2][3]);
			let v2y = b3TransformLaneL<v128>(cxy.x, cyy.x, czy.x, py.x, lanes[2][0], lanes[2][1], lanes[2][2], lanes[2][3]);
			let v2z = b3TransformLaneL<v128>(cxz.x, cyz.x, czz.x, pz.x, lanes[2][0], lanes[2][1], lanes[2][2], lanes[2][3]);

			if (hullB != null) {
				let margin = f32x4.splat(marginW.x);
				let lowerX = b3SubL(b3MinL(v0x, b3MinL(v1x, v2x)), margin);
				let lowerY = b3SubL(b3MinL(v0y, b3MinL(v1y, v2y)), margin);
				let lowerZ = b3SubL(b3MinL(v0z, b3MinL(v1z, v2z)), margin);
				let upperX = b3AddL(b3MaxL(v0x, b3MaxL(v1x, v2x)), margin);
				let upperY = b3AddL(b3MaxL(v0y, b3MaxL(v1y, v2y)), margin);
				let upperZ = b3AddL(b3MaxL(v0z, b3MaxL(v1z, v2z)), margin);

				let separated = v128.or(f32x4.lt(upperX, f32x4.splat(hullLowerX.x)), f32x4.gt(lowerX, f32x4.splat(hullUpperX.x)));
				separated = v128.or(separated, v128.or(f32x4.lt(upperY, f32x4.splat(hullLowerY.x)), f32x4.gt(lowerY, f32x4.splat(hullUpperY.x))));
				separated = v128.or(separated, v128.or(f32x4.lt(upperZ, f32x4.splat(hullLowerZ.x)), f32x4.gt(lowerZ, f32x4.splat(hullUpperZ.x))));

				let cullMask = keepMask & <u32>i32x4.bitmask(separated);
				keepMask &= ~cullMask;

				while (cullMask != 0) {
					let lane = b3CTZ32(cullMask);
					cullMask &= cullMask - 1;
					triangleCaches[baseIndex + <i32>lane].cache.satCache = b3SATCache();
				}
			}

			if (keepMask == 0) {
				continue;
			}

			b3StoreLanesL(x, 0, v0x);
			b3StoreLanesL(y, 0, v0y);
			b3StoreLanesL(z, 0, v0z);
			b3StoreLanesL(x, 4, v1x);
			b3StoreLanesL(y, 4, v1y);
			b3StoreLanesL(z, 4, v1z);
			b3StoreLanesL(x, 8, v2x);
			b3StoreLanesL(y, 8, v2y);
			b3StoreLanesL(z, 8, v2z);
		} else {
			// Transform triangle into the shape frame
			let vx: FixedArray<b3FloatW4, 3>;
			let vy: FixedArray<b3FloatW4, 3>;
			let vz: FixedArray<b3FloatW4, 3>;
			for (let k = 0; k < 3; ++k) {
				let ax = b3SetW4(lanes[k][0].x, lanes[k][1].x, lanes[k][2].x, lanes[k][3].x);
				let ay = b3SetW4(lanes[k][0].y, lanes[k][1].y, lanes[k][2].y, lanes[k][3].y);
				let az = b3SetW4(lanes[k][0].z, lanes[k][1].z, lanes[k][2].z, lanes[k][3].z);

				vx[k] = b3AddW4(b3AddW4(b3AddW4(b3MulW4(cxx, ax), b3MulW4(cyx, ay)), b3MulW4(czx, az)), px);
				vy[k] = b3AddW4(b3AddW4(b3AddW4(b3MulW4(cxy, ax), b3MulW4(cyy, ay)), b3MulW4(czy, az)), py);
				vz[k] = b3AddW4(b3AddW4(b3AddW4(b3MulW4(cxz, ax), b3MulW4(cyz, ay)), b3MulW4(czz, az)), pz);
			}

			if (hullB != null) {
				let lowerX = b3SubW4(b3MinW4(vx[0], b3MinW4(vx[1], vx[2])), marginW);
				let lowerY = b3SubW4(b3MinW4(vy[0], b3MinW4(vy[1], vy[2])), marginW);
				let lowerZ = b3SubW4(b3MinW4(vz[0], b3MinW4(vz[1], vz[2])), marginW);
				let upperX = b3AddW4(b3MaxW4(vx[0], b3MaxW4(vx[1], vx[2])), marginW);
				let upperY = b3AddW4(b3MaxW4(vy[0], b3MaxW4(vy[1], vy[2])), marginW);
				let upperZ = b3AddW4(b3MaxW4(vz[0], b3MaxW4(vz[1], vz[2])), marginW);

				let separated = b3OrW4(b3LessThanW4(upperX, hullLowerX), b3GreaterThanW4(lowerX, hullUpperX));
				separated = b3OrW4(separated, b3OrW4(b3LessThanW4(upperY, hullLowerY), b3GreaterThanW4(lowerY, hullUpperY)));
				separated = b3OrW4(separated, b3OrW4(b3LessThanW4(upperZ, hullLowerZ), b3GreaterThanW4(lowerZ, hullUpperZ)));

				let cullMask = keepMask & <u32>b3MoveMaskW4(separated);
				keepMask &= ~cullMask;

				while (cullMask != 0) {
					let lane = b3CTZ32(cullMask);
					cullMask &= cullMask - 1;
					triangleCaches[baseIndex + <i32>lane].cache.satCache = b3SATCache();
				}
			}

			if (keepMask == 0) {
				continue;
			}

			for (let k = 0; k < 3; ++k) {
				b3StoreW4(x, 4 * k, vx[k]);
				b3StoreW4(y, 4 * k, vy[k]);
				b3StoreW4(z, 4 * k, vz[k]);
			}
		}

		while (keepMask != 0) {
			let lane = <i32>b3CTZ32(keepMask);
			keepMask &= keepMask - 1;

			let survivor = 3 * survivorCount;
			survivorVertices[survivor + 0] = b3Vec3(x[lane], y[lane], z[lane]);
			survivorVertices[survivor + 1] = b3Vec3(x[4 + lane], y[4 + lane], z[4 + lane]);
			survivorVertices[survivor + 2] = b3Vec3(x[8 + lane], y[8 + lane], z[8 + lane]);
			survivorIndices[survivorCount] = baseIndex + lane;
			survivorCount += 1;
		}
	}

	return survivorCount;
}

// One row of the triangle transform over four lane vertices, ( ( m0 x + m1 y ) + m2 z ) + p as the b3FloatW4 code computes it
@inline function b3TransformLaneL<T>(m0: f32, m1: f32, m2: f32, p: f32, a: b3Vec3, b: b3Vec3, c: b3Vec3, d: b3Vec3): T {
	let ax = b3SetL<T>(a.x, b.x, c.x, d.x);
	let ay = b3SetL<T>(a.y, b.y, c.y, d.y);
	let az = b3SetL<T>(a.z, b.z, c.z, d.z);
	return b3AddL(b3AddL(b3AddL(b3MulL(b3SplatL<T>(m0), ax), b3MulL(b3SplatL<T>(m1), ay)), b3MulL(b3SplatL<T>(m2), az)), b3SplatL<T>(p));
}

// b3StoreW4 of a v128 row into the lane scratch, which has room for three rows
@inline unsafe function b3StoreLanesL<T>(data: StaticArray<f32>, index: i32, a: T): void {
	b3StoreL<T>(changetype<usize>(data) + <usize>index * 4, a);
}

export unsafe function b3ComputeMeshManifolds(
	world: b3World,
	workerIndex: i32,
	contact: b3Contact,
	shapeA: b3Shape,
	materialMap: Uint16Array?,
	xfA: b3WorldTransform,
	shapeB: b3Shape,
	xfB: b3WorldTransform,
	isFast: bool,
): bool {
	return b3ComputeMeshManifoldsAt(world, workerIndex, contact, shapeA, materialMap, xfA, shapeB, xfB, isFast);
}

// Unchecked: its scratch covers the counts its loops are
// bounded by (notes/perf_collide.md, change 7); reads of shape supplied data stay checked through helpers.
@unchecked
export unsafe function b3ComputeMeshManifoldsAt(
	world: b3World,
	workerIndex: i32,
	contact: b3Contact,
	shapeA: b3Shape,
	materialMap: Uint16Array?,
	xfA: b3WorldTransform,
	shapeB: b3Shape,
	xfB: b3WorldTransform,
	isFast: bool,
): bool {
	if (B3_ASSERT_ENABLED) B3_ASSERT(shapeA.type == b3_meshShape || shapeA.type == b3_heightShape);

	let context = world.taskContexts[workerIndex];
	let scratch = s_meshScratch[workerIndex];

	// A parallel part saves the refresh b3MeshContactFitsPart queried, if any
	if (Parallel.inPart()) {
		if (scratch.refreshReady) {
			scratch.refreshReady = false;
			b3SaveCacheTriangles(contact.meshContact, shapeA, scratch.refreshCount, scratch);
		}
	} else {
		b3RefreshCache(contact, shapeA, xfA, shapeB.aabb, scratch);
	}

	// Collide with triangles and build manifolds
	let meshContact = contact.meshContact;
	let triangleCount = meshContact.triangleCache.length;

	// C's pointer arrays become indices into the manifold buffer. The contact's manifold storage holds one per cached
	// triangle, at least as many as there can be clusters, so a parallel part never grows it.
	if (!Parallel.inPart()) {
		b3ReserveMeshScratch(scratch, b3MaxInt(triangleCount, contact.manifoldCount));
		if (contact.manifolds.length < triangleCount) {
			let manifolds = b3AllocateManifolds(world, triangleCount);
			b3CopyManifolds(manifolds, contact.manifolds, contact.manifoldCount);
			contact.manifolds = manifolds;
		}
	}
	let acceptedManifolds = scratch.acceptedManifolds;
	let acceptedManifoldCount = 0;
	let tentativeManifolds = scratch.tentativeManifolds;
	let tentativeManifoldCount = 0;
	let tentativeTriangles = scratch.tentativeTriangles;
	let tentativeTriangleCount = 0;

	// C's uninitialized stack arrays; a local struct would zero all 128 keys on every call
	scratch.foundEdges.count = 0;
	scratch.foundVertices.count = 0;

	// This transform converts from mesh frame into the shapeB frame
	let transformAtoB = b3InvMulWorldTransforms(xfB, xfA);
	let relativeMatrix = b3MakeMatrixFromQuat(transformAtoB.q);
	let linearSlop = B3_LINEAR_SLOP();

	// This should push apart shapes after a time of impact event.
	// In the past I've called this `polygon skin`, but PhysX and Unreal
	// call it `rest offset` which seems appropriate in this case.
	// It leads to a small visual gap but seems to improve the quality of mesh
	// collision, especially for hull versus mesh.
	let restOffset = B3_MESH_REST_OFFSET();
	let enableSpeculative = (contact.flags & b3_enableSpeculativePoints) != 0;

	// Make room for clip points
	let pointBufferCapacity = B3_MAX_POINTS_PER_TRIANGLE * triangleCount;
	let totalPointCount = 0;

	// C bumps triangleCount manifolds; a slot is reused while it holds no points
	let manifoldBuffer = scratch.manifoldBuffer;
	let manifoldCount = 0;

	let triangleCaches = meshContact.triangleCache;

	let hullB = shapeB.type == b3_hullShape ? shapeB.hull : null;
	let hullSpeculativeDistance: f32 = enableSpeculative ? B3_SPECULATIVE_DISTANCE() : 0.0;
	let hullMargin = b3Vec3(hullSpeculativeDistance, hullSpeculativeDistance, hullSpeculativeDistance);

	let survivorIndices = scratch.survivorIndices;
	let survivorVertices = scratch.survivorVertices;

	let meshTriangles: StaticArray<b3MeshTriangle>? = null;
	let meshVertices: StaticArray<b3Vec3>? = null;
	let meshScale = b3Vec3_one;
	let meshFlipped = false;
	if (shapeA.type == b3_meshShape) {
		meshTriangles = b3GetMeshTriangles(shapeA.mesh!.data);
		meshVertices = b3GetMeshVertices(shapeA.mesh!.data);
		meshScale = shapeA.mesh!.scale;
		meshFlipped = meshScale.x * meshScale.y * meshScale.z < 0.0;
	}

	let survivorCount = b3PrefilterTriangles(
		shapeA,
		triangleCaches,
		triangleCount,
		meshTriangles,
		meshVertices,
		meshScale,
		meshFlipped,
		relativeMatrix,
		transformAtoB.p,
		hullB,
		hullMargin.x,
		survivorIndices,
		survivorVertices,
		scratch,
	);

	let vertices = scratch.triangleVertices;
	for (let survivorIndex = 0; survivorIndex < survivorCount && totalPointCount + 3 < pointBufferCapacity; ++survivorIndex) {
		let index = survivorIndices[survivorIndex];
		let triangleIndex = triangleCaches[index].triangleIndex;
		vertices[0] = survivorVertices[3 * survivorIndex + 0];
		vertices[1] = survivorVertices[3 * survivorIndex + 1];
		vertices[2] = survivorVertices[3 * survivorIndex + 2];

		let triangle: b3Triangle;
		if (shapeA.type == b3_meshShape) {
			triangle = b3GetMeshTriangle(shapeA.mesh!, triangleIndex);
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(shapeA.type == b3_heightShape);
			triangle = b3GetHeightFieldTriangle(shapeA.heightField!, triangleIndex);
		}

		// C writes through a pointer into the growable cache array; copy out and store back
		let cache = triangleCaches[index].cache;
		let pointCapacity = pointBufferCapacity - totalPointCount;
		let manifold = manifoldBuffer[manifoldCount];
		manifold.pointOffset = totalPointCount;
		manifold.pointCount = 0;
		manifold.triangleFlags = triangle.flags;
		manifold.feature = b3_featureNone;

		switch (shapeB.type) {
			case b3_capsuleShape:
				b3CollideTriangleAndCapsule(manifold, pointCapacity, vertices, shapeB.capsule, cache.simplexCache, workerIndex);
				break;

			case b3_hullShape:
				// Cached edge contact is dangerous at high speed because the hull can rotate around the edge and tunnel
				// through the triangle.
				if (isFast && cache.satCache.type == b3_edgePairAxis) {
					cache.satCache = b3SATCache();
				}

				b3CollideTriangleAndHull(
					manifold,
					pointCapacity,
					vertices[0],
					vertices[1],
					vertices[2],
					triangle.flags,
					hullB!,
					cache.satCache,
					enableSpeculative,
					workerIndex,
				);
				context.satCallCount += 1;
				context.satCacheHitCount += cache.satCache.hit;
				break;

			case b3_sphereShape:
				b3CollideTriangleAndSphere(manifold, pointCapacity, vertices, shapeB.sphere);
				break;

			default:
				B3_ASSERT(false);
				triangleCaches[index].cache = cache;
				return false;
		}

		triangleCaches[index].cache = cache;

		let manifoldPointCount = manifold.pointCount;

		if (manifoldPointCount > 0) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(manifold.feature != b3_featureNone);

			manifoldCount += 1;
			totalPointCount += manifoldPointCount;
			manifold.triangleIndex = triangleIndex;
			manifold.triangleNormal = b3MakeNormalFromPoints(vertices[0], vertices[1], vertices[2]);
			manifold.i1 = triangle.i1;
			manifold.i2 = triangle.i2;
			manifold.i3 = triangle.i3;

			if (manifold.feature == b3_featureTriangleFace || B3_FORCE_GHOST_COLLISIONS != 0) {
				b3AddEdge(scratch.foundEdges, triangle.i1, triangle.i2);
				b3AddEdge(scratch.foundEdges, triangle.i2, triangle.i3);
				b3AddEdge(scratch.foundEdges, triangle.i3, triangle.i1);
				b3AddVertex(scratch.foundVertices, triangle.i1);
				b3AddVertex(scratch.foundVertices, triangle.i2);
				b3AddVertex(scratch.foundVertices, triangle.i3);

				acceptedManifolds[acceptedManifoldCount] = manifoldCount - 1;
				acceptedManifoldCount += 1;
			} else if (manifold.feature == b3_featureHullFace) {
				let cosNormalAngle = b3Dot(manifold.triangleNormal, manifold.normal);
				if (cosNormalAngle > 0.5) {
					b3AddEdge(scratch.foundEdges, triangle.i1, triangle.i2);
					b3AddEdge(scratch.foundEdges, triangle.i2, triangle.i3);
					b3AddEdge(scratch.foundEdges, triangle.i3, triangle.i1);
					b3AddVertex(scratch.foundVertices, triangle.i1);
					b3AddVertex(scratch.foundVertices, triangle.i2);
					b3AddVertex(scratch.foundVertices, triangle.i3);

					acceptedManifolds[acceptedManifoldCount] = manifoldCount - 1;
					acceptedManifoldCount += 1;
				} else {
					let minSeparation = manifold.points[manifold.pointOffset].separation;
					for (let i = 1; i < manifoldPointCount; ++i) {
						minSeparation = b3MinFloat(minSeparation, manifold.points[manifold.pointOffset + i].separation);
					}

					if (minSeparation < -2.0 * linearSlop) {
						// Deep overlap
						b3AddEdge(scratch.foundEdges, triangle.i1, triangle.i2);
						b3AddEdge(scratch.foundEdges, triangle.i2, triangle.i3);
						b3AddEdge(scratch.foundEdges, triangle.i3, triangle.i1);
						b3AddVertex(scratch.foundVertices, triangle.i1);
						b3AddVertex(scratch.foundVertices, triangle.i2);
						b3AddVertex(scratch.foundVertices, triangle.i3);
						acceptedManifolds[acceptedManifoldCount] = manifoldCount - 1;
						acceptedManifoldCount += 1;
					} else {
						tentativeTriangles[tentativeTriangleCount].squaredDistance = manifold.squaredDistance;
						tentativeTriangles[tentativeTriangleCount].index = tentativeManifoldCount;
						tentativeTriangleCount += 1;
						tentativeManifolds[tentativeManifoldCount] = manifoldCount - 1;
						tentativeManifoldCount += 1;
					}
				}
			} else {
				tentativeTriangles[tentativeTriangleCount].squaredDistance = manifold.squaredDistance;
				tentativeTriangles[tentativeTriangleCount].index = tentativeManifoldCount;
				tentativeTriangleCount += 1;
				tentativeManifolds[tentativeManifoldCount] = manifoldCount - 1;
				tentativeManifoldCount += 1;
			}
		}
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(acceptedManifoldCount <= triangleCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(tentativeManifoldCount <= triangleCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(tentativeTriangleCount <= triangleCount);

	if (shapeB.type == b3_sphereShape) {
		// Sort triangles so the closest triangles are processed first
		{
			if (tentativeTriangleCount > 1) {
				Q_LOOP<b3TentativeSorter>(tentativeTriangleCount, scratch.tentativeSorter, scratch.sortStack);
			}
		}

		// Add tentative manifolds in sorted order. Avoid adding manifolds that generate ghost collisions.
		for (let i = 0; i < tentativeTriangleCount; ++i) {
			let tentativeIndex = tentativeManifolds[tentativeTriangles[i].index];
			let m = manifoldBuffer[tentativeIndex];

			let addedEdge1 = b3AddEdge(scratch.foundEdges, m.i1, m.i2);
			let addedEdge2 = b3AddEdge(scratch.foundEdges, m.i2, m.i3);
			let addedEdge3 = b3AddEdge(scratch.foundEdges, m.i3, m.i1);
			let addedVertex1 = b3AddVertex(scratch.foundVertices, m.i1);
			let addedVertex2 = b3AddVertex(scratch.foundVertices, m.i2);
			let addedVertex3 = b3AddVertex(scratch.foundVertices, m.i3);

			let feature: b3TriangleFeature = m.feature;
			let shouldCollide = false;
			switch (feature) {
				case b3_featureNone:
				case b3_featureTriangleFace:
					B3_ASSERT(false);
					break;

				case b3_featureEdge1:
					shouldCollide = addedEdge1;
					break;

				case b3_featureEdge2:
					shouldCollide = addedEdge2;
					break;

				case b3_featureEdge3:
					shouldCollide = addedEdge3;
					break;

				case b3_featureVertex1:
					shouldCollide = addedVertex1;
					break;

				case b3_featureVertex2:
					shouldCollide = addedVertex2;
					break;

				case b3_featureVertex3:
					shouldCollide = addedVertex3;
					break;

				default:
					B3_ASSERT(false);
					break;
			}

			if (shouldCollide == true) {
				acceptedManifolds[acceptedManifoldCount] = tentativeIndex;
				acceptedManifoldCount += 1;
			}
		}
	} else {
		// Problem: hull can tunnel if time of impact is at concave edge
		// Example: flat box sliding down a ramp to a flat bottom
		// Solution: only ignore flat edges
		for (let i = 0; i < tentativeManifoldCount; ++i) {
			let m = manifoldBuffer[tentativeManifolds[i]];
			let triangleFlags = m.triangleFlags;

			if ((triangleFlags & b3_allFlatEdges) == b3_allFlatEdges) {
				continue;
			}

			if ((triangleFlags & b3_flatEdge1) == b3_flatEdge1) {
				if (b3FindEdge(scratch.foundEdges, m.i1, m.i2)) {
					continue;
				}
			}

			if ((triangleFlags & b3_flatEdge2) == b3_flatEdge2) {
				if (b3FindEdge(scratch.foundEdges, m.i2, m.i3)) {
					continue;
				}
			}

			if ((triangleFlags & b3_flatEdge3) == b3_flatEdge3) {
				if (b3FindEdge(scratch.foundEdges, m.i3, m.i1)) {
					continue;
				}
			}

			acceptedManifolds[acceptedManifoldCount] = tentativeManifolds[i];
			acceptedManifoldCount += 1;
		}
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(acceptedManifoldCount <= triangleCount);

	// The contact keeps its manifold storage
	if (acceptedManifoldCount == 0) {
		contact.manifoldCount = 0;
		return false;
	}

	let clusters = scratch.clusters;
	let clusterMemberships = scratch.clusterMemberships;

	// Cluster tolerance is tighter than the warm starting manifold matching tolerance. These
	// serve different purposes.
	const clusterThreshold: f32 = 0.996;
	let clusterCount = 0;
	let clusterPointCount = 0;
	for (let i = 0; i < acceptedManifoldCount; ++i) {
		clusterMemberships[i] = B3_NULL_INDEX;

		let manifold = manifoldBuffer[acceptedManifolds[i]];
		clusterPointCount += manifold.pointCount;

		// Cluster based on the triangle normal and contact normal.
		// The first cluster found is accepted because the tolerance is tight.
		// todo consider requiring the triangles to be connect by an edge.
		// todo consider looking for the best cluster instead of the first one within tolerance
		// This bool is here to allow quick testing with and without clustering.
		let allowClustering = true;
		let manifoldNormal = manifold.normal;
		let triangleNormal = manifold.triangleNormal;
		let clusterIndex = B3_NULL_INDEX;
		for (let j = 0; j < clusterCount && allowClustering; ++j) {
			let cosManifoldAngle = b3Dot(clusters[j].manifoldNormal, manifoldNormal);
			let cosTriangleAngle = b3Dot(clusters[j].triangleNormal, triangleNormal);
			if (cosManifoldAngle <= clusterThreshold || cosTriangleAngle <= clusterThreshold) {
				continue;
			}

			// Found a cluster
			clusterIndex = j;
			break;
		}

		if (clusterIndex != B3_NULL_INDEX) {
			clusterMemberships[i] = clusterIndex;
			clusters[clusterIndex].pointCapacity += manifold.pointCount;
		} else {
			clusters[clusterCount].manifoldNormal = manifoldNormal;
			clusters[clusterCount].triangleNormal = triangleNormal;
			clusters[clusterCount].pointCapacity = manifold.pointCount;
			clusterMemberships[i] = clusterCount;
			clusterCount += 1;
		}
	}

	if (clusterPointCount == 0) {
		return false;
	}

	// Setup clusters
	let clusterPoints = scratch.clusterPoints;
	let pointOffset = 0;

	for (let i = 0; i < clusterCount; ++i) {
		ref cluster = clusters[i];
		cluster.pointOffset = pointOffset;
		cluster.pointCount = 0;
		pointOffset += cluster.pointCapacity;
	}

	// Populate clusters
	for (let i = 0; i < acceptedManifoldCount; ++i) {
		let clusterIndex = clusterMemberships[i];
		if (clusterIndex == B3_NULL_INDEX) {
			continue;
		}

		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= clusterIndex && clusterIndex < clusterCount);

		let am = manifoldBuffer[acceptedManifolds[i]];
		ref cm = clusters[clusterIndex];
		for (let j = 0; j < am.pointCount; ++j) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(cm.pointCount < cm.pointCapacity);
			ref ap = am.points[am.pointOffset + j];
			ref cp = clusterPoints[cm.pointOffset + cm.pointCount];

			cp.triangleIndex = am.triangleIndex;
			cp.point = ap.point;
			cp.separation = ap.separation;
			cp.pair = ap.pair;
			cm.pointCount += 1;
		}
	}

	// Simplify clusters
	for (let i = 0; i < clusterCount; ++i) {
		ref cm = clusters[i];
		if (B3_ASSERT_ENABLED) B3_ASSERT(cm.pointCount == cm.pointCapacity);
		let reducedCount = b3ReduceCluster(clusterPoints, cm.pointOffset, cm.pointCount, cm.triangleNormal, scratch.cullPoints);
		cm.pointCount = reducedCount;
	}

	// Make a temporary copy of previous manifolds
	let oldManifoldCount = contact.manifoldCount;
	let oldManifolds = scratch.oldManifolds;
	b3CopyManifolds(oldManifolds, contact.manifolds, oldManifoldCount);

	// Resize manifolds; the storage has room, and C's fresh manifolds are zeroed ones
	contact.manifoldCount = <i32>(<u16>clusterCount);
	b3ZeroManifolds(contact.manifolds, clusterCount);

	let consumed = scratch.consumed;
	for (let i = 0; i < oldManifoldCount; ++i) {
		consumed[i] = false;
	}

	let matrixB = b3MakeMatrixFromQuat(xfB.q);
	let offsetA = b3SubPos(xfB.p, xfA.p);

	const normalMatchTolerance: f32 = 0.995;
	for (let i = 0; i < clusterCount; ++i) {
		ref cm = clusters[i];
		let pointCount = cm.pointCount;
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 < pointCount && pointCount <= B3_MAX_MANIFOLD_POINTS);

		ref manifold = contact.manifolds[i];
		manifold.pointCount = pointCount;
		manifold.normal = b3MulMV(matrixB, cm.manifoldNormal);

		let clusterNormal = b3MulMV(matrixB, cm.manifoldNormal);
		let bestDot = normalMatchTolerance;
		let bestIndex = B3_NULL_INDEX;

		for (let j = 0; j < oldManifoldCount; ++j) {
			if (consumed[j] == true) {
				continue;
			}

			let dot = b3Dot(oldManifolds[j].normal, clusterNormal);
			if (dot > bestDot) {
				bestIndex = j;
				bestDot = dot;
			}
		}

		// C's matchedManifold pointer into the old manifold copy is its index
		let matchedIndex = B3_NULL_INDEX;
		if (bestIndex != B3_NULL_INDEX) {
			matchedIndex = bestIndex;
			manifold.frictionImpulse = oldManifolds[matchedIndex].frictionImpulse;
			manifold.rollingImpulse = oldManifolds[matchedIndex].rollingImpulse;
			manifold.twistImpulse = oldManifolds[matchedIndex].twistImpulse;
			consumed[bestIndex] = true;
		}

		for (let j = 0; j < pointCount; ++j) {
			let source = clusterPoints[cm.pointOffset + j];

			// Contact points are computed in frame B
			let anchorB = b3MulMV(matrixB, source.point);
			let anchorA = b3Add(anchorB, offsetA);
			let featureId = b3MakeFeatureId(source.pair);
			let triangleIndex = source.triangleIndex;

			// Preserve normal impulse if possible. The manifold was zeroed, so the other fields hold zero.
			let normalImpulse: f32 = 0.0;
			let persisted = false;
			if (matchedIndex != B3_NULL_INDEX) {
				ref matchedManifold = oldManifolds[matchedIndex];
				let oldPointCount = matchedManifold.pointCount;
				for (let k = 0; k < oldPointCount; ++k) {
					ref oldPt = matchedManifold.points[k];

					if (featureId == oldPt.featureId && triangleIndex == oldPt.triangleIndex) {
						normalImpulse = oldPt.normalImpulse;
						persisted = true;

						// claimed
						oldPt.triangleIndex = B3_NULL_INDEX;
						break;
					}
				}
			}

			let mp = b3ManifoldPointAddress(contact.manifolds, i, j);
			b3StorePointAnchors(mp, anchorA, anchorB, source.separation - restOffset, normalImpulse);
			b3StoreRow(mp + MP_TOTALNORMALIMPULSE, 0.0, 0.0, 0.0, reinterpret<f32>(featureId));
			store<i32>(mp + MP_TRIANGLEINDEX, triangleIndex);
			store<bool>(mp + MP_PERSISTED, persisted);
		}
	}

	let materialsA = b3GetShapeMaterials(shapeA);
	let materialB = b3GetShapeMaterials(shapeB)[0];
	let tangentVelocityA = b3Vec3_zero;

	// Update friction and restitution if the mesh has per triangle material
	if (shapeA.materialCount > 0) {
		let friction: f32 = 0.0;
		let restitution: f32 = 0.0;
		let sampleCount: f32 = 0.0;

		let materialIndices: StaticArray<u8>;
		if (shapeA.type == b3_meshShape) {
			materialIndices = b3GetMeshMaterialIndices(shapeA.mesh!.data);
		} else {
			materialIndices = b3GetHeightFieldMaterialIndices(shapeA.heightField!);
		}

		for (let i = 0; i < clusterCount; ++i) {
			let pointCount = contact.manifolds[i].pointCount;
			for (let j = 0; j < pointCount; ++j) {
				let triangleIndex = contact.manifolds[i].points[j].triangleIndex;
				let materialIndex: i32;
				if (shapeA.type == b3_meshShape) {
					materialIndex = <i32>b3MaterialIndexAt(materialIndices, triangleIndex);

					if (materialMap != null) {
						materialIndex = <i32>b3MaterialMapAt(materialMap, materialIndex);
					}
				} else {
					materialIndex = <i32>b3MaterialIndexAt(materialIndices, triangleIndex >> 1);
				}

				materialIndex = b3ClampInt(materialIndex, 0, shapeA.materialCount - 1);
				let material = b3SurfaceMaterialAt(materialsA, materialIndex);
				friction += b3MixFriction(world, material.friction, material.userMaterialId, materialB.friction, materialB.userMaterialId);
				restitution += b3MixRestitution(
					world,
					material.restitution,
					material.userMaterialId,
					materialB.restitution,
					materialB.userMaterialId,
				);

				tangentVelocityA = b3Add(tangentVelocityA, material.tangentVelocity);

				sampleCount += 1.0;
			}
		}

		if (sampleCount > 0.0) {
			let invCount: f32 = 1.0 / sampleCount;
			contact.friction = invCount * friction;
			contact.restitution = invCount * restitution;
			tangentVelocityA = b3MulSV(invCount, tangentVelocityA);
		}

		if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(contact.friction) && contact.friction >= 0.0);
		if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(contact.restitution) && contact.restitution >= 0.0);
	} else {
		// Keep these updated in case the values on the shapes are modified
		contact.friction = b3MixFriction(
			world,
			b3SurfaceMaterialAt(materialsA, 0).friction,
			b3SurfaceMaterialAt(materialsA, 0).userMaterialId,
			materialB.friction,
			materialB.userMaterialId,
		);
		contact.restitution = b3MixRestitution(
			world,
			b3SurfaceMaterialAt(materialsA, 0).restitution,
			b3SurfaceMaterialAt(materialsA, 0).userMaterialId,
			materialB.restitution,
			materialB.userMaterialId,
		);
		tangentVelocityA = b3SurfaceMaterialAt(materialsA, 0).tangentVelocity;
	}

	tangentVelocityA = b3RotateVector(xfA.q, tangentVelocityA);

	let radiusB: f32 = 0.0;
	if (shapeB.type == b3_sphereShape) {
		radiusB = shapeB.sphere.radius;
	} else if (shapeB.type == b3_capsuleShape) {
		radiusB = shapeB.capsule.radius;
	} else if (shapeB.type == b3_hullShape) {
		radiusB = shapeB.hull!.innerRadius;
	}

	contact.rollingResistance = materialB.rollingResistance * radiusB;

	let tangentVelocityB = b3RotateVector(xfB.q, materialB.tangentVelocity);
	contact.tangentVelocity = b3Sub(tangentVelocityA, tangentVelocityB);
	return true;
}
