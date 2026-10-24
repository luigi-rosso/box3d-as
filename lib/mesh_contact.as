// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/mesh_contact.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_MAX, b3Log } from "./core";
import {
	B3_LINEAR_SLOP,
	B3_MAX_AABB_MARGIN,
	B3_MAX_MANIFOLD_POINTS,
	B3_MAX_MESH_CONTACT_TRIANGLES,
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
	b3HeightFieldData,
	b3LocalManifold,
	b3LocalManifoldPoint,
	b3Manifold,
	b3Mesh,
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
import { b3SurfaceMaterial, b3_capsuleShape, b3_heightShape, b3_hullShape, b3_meshShape, b3_sphereShape } from "./types";
import { B3_FORCE_GHOST_COLLISIONS, b3Contact, b3ContactCache, b3TriangleCache, b3_enableSpeculativePoints } from "./contact";
import { b3MakeFeatureId } from "./manifold";
import { b3AllocateManifolds, b3FreeManifolds, b3World } from "./physics_world";
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
	b3SubL,
} from "./simd";
import { b3CTZ32 } from "./ctz";
import { Q_LOOP } from "./qsort";
import { b3QueryMesh } from "./mesh";
import { b3GetHeightFieldTriangle, b3QueryHeightField } from "./height_field";
import { b3CollideTriangleAndCapsule, b3CollideTriangleAndHull, b3CollideTriangleAndSphere } from "./triangle_manifold";

const B3_MAX_POINTS_PER_TRIANGLE: i32 = 32;

class b3TriangleQueryContext {
	indices: StaticArray<i32> = new StaticArray<i32>(B3_MAX_MESH_CONTACT_TRIANGLES);
	capacity: i32 = 0;
	count: i32 = 0;
}

// C keeps the context and the index buffer on its stack. The callbacks below capture nothing, so
// no closure is allocated per query; a refresh never nests.
const s_triangleQuery = new b3TriangleQueryContext();

function b3CollectTriangleIndicesCallback(a: b3Vec3, b: b3Vec3, c: b3Vec3, triangleIndex: i32): bool {
	let triangleContext = s_triangleQuery;
	if (triangleContext.count == triangleContext.capacity) {
		return false;
	}
	triangleContext.indices[triangleContext.count] = triangleIndex;
	triangleContext.count += 1;
	return triangleContext.count < triangleContext.capacity;
}

function b3QueryMeshTriangles(capacity: i32, mesh: b3Mesh, bounds: b3AABB): i32 {
	s_triangleQuery.capacity = capacity;
	s_triangleQuery.count = 0;
	b3QueryMesh(mesh, bounds, b3CollectTriangleIndicesCallback);
	return s_triangleQuery.count;
}

function b3QueryHeightFieldTriangles(capacity: i32, heightField: b3HeightFieldData, bounds: b3AABB): i32 {
	s_triangleQuery.capacity = capacity;
	s_triangleQuery.count = 0;
	b3QueryHeightField(heightField, bounds, b3CollectTriangleIndicesCallback);
	return s_triangleQuery.count;
}

// Scratch for C's per contact arena bumps, grown as needed and reused by every call. Like the arena
// they are not cleared between calls; every element is written before it is read.
let s_contactCaches = new StaticArray<b3ContactCache>(0);
let s_cullPoints = new StaticArray<b3Point2D>(0);
let s_acceptedManifolds = new StaticArray<i32>(0);
let s_tentativeManifolds = new StaticArray<i32>(0);
let s_tentativeTriangles = new StaticArray<b3TentativeTriangle>(0);
let s_pointBuffer = new StaticArray<b3LocalManifoldPoint>(0);
const s_manifoldBuffer = new Array<b3LocalManifold>();
let s_survivorIndices = new StaticArray<i32>(0);
let s_survivorVertices = new StaticArray<b3Vec3>(0);
const s_laneX = new StaticArray<f32>(12);
const s_laneY = new StaticArray<f32>(12);
const s_laneZ = new StaticArray<f32>(12);
const s_triangleVertices = new StaticArray<b3Vec3>(3);
let s_clusters = new StaticArray<b3Cluster>(0);
let s_clusterMemberships = new StaticArray<i32>(0);
let s_clusterPoints = new StaticArray<b3LocalManifoldPoint>(0);
let s_oldManifolds = new StaticArray<b3Manifold>(0);
let s_consumed = new StaticArray<bool>(0);

@inline function b3ScratchCapacity(current: i32, needed: i32): i32 {
	return b3MaxInt(needed, 2 * current);
}

let s_once = false;

function b3RefreshCache(contact: b3Contact, shapeA: b3Shape, xfA: b3WorldTransform, bounds: b3AABB): void {
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

	// Enlarge to the query bounds to absorb small movement
	let radius = B3_MAX_AABB_MARGIN() + B3_SPECULATIVE_DISTANCE();
	let extension = b3Vec3(radius, radius, radius);
	meshContact.queryBounds.lowerBound = b3Sub(bounds.lowerBound, extension);
	meshContact.queryBounds.upperBound = b3Add(bounds.upperBound, extension);

	// Query triangles
	let triangleCapacity = B3_MAX_MESH_CONTACT_TRIANGLES;
	let triangleIndices = s_triangleQuery.indices;

	// Bounds are in world space. Convert to the local mesh frame. The broadphase bounds are float,
	// so the demoted mesh transform is the matching float world frame (exact in float mode).
	let meshTransform = b3ToRelativeTransform(xfA, b3Pos_zero);
	let localBounds = b3AABB_Transform(b3InvertTransform(meshTransform), meshContact.queryBounds);
	let triangleCount: i32;
	if (shapeA.type == b3_meshShape) {
		triangleCount = b3QueryMeshTriangles(triangleCapacity, shapeA.mesh!, localBounds);
	} else {
		if (B3_ASSERT_ENABLED) B3_ASSERT(shapeA.type == b3_heightShape);
		triangleCount = b3QueryHeightFieldTriangles(triangleCapacity, shapeA.heightField!, localBounds);
	}

	if (triangleCount == triangleCapacity) {
		if (s_once == false) {
			b3Log("WARNING: complex mesh detected, triangle buffer capacity of " + triangleCapacity.toString() + " reached");
			s_once = true;
		}
	}

	// Triangle indices must be sorted to match caches.

	// Create new contact cache and match with old one
	if (s_contactCaches.length < triangleCount) {
		s_contactCaches = new StaticArray<b3ContactCache>(b3ScratchCapacity(s_contactCaches.length, triangleCount));
	}
	let contactCache = s_contactCaches;

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

	// Save new cache
	triangleCache.length = triangleCount;
	for (let i = 0; i < triangleCount; ++i) {
		triangleCache[i] = b3TriangleCache(triangleIndices[i], contactCache[i]);

		if (B3_ASSERT_ENABLED && shapeA.type == b3_meshShape) {
			B3_ASSERT(0 <= triangleCache[i].triangleIndex && triangleCache[i].triangleIndex < shapeA.mesh!.data.triangleCount);
		}
	}
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
function b3ReduceCluster(points: StaticArray<b3LocalManifoldPoint>, pointOffset: i32, count1: i32, normal: b3Vec3): i32 {
	let targetCount = 1;
	if (count1 <= targetCount) {
		return count1;
	}

	if (s_cullPoints.length < count1) {
		s_cullPoints = new StaticArray<b3Point2D>(b3ScratchCapacity(s_cullPoints.length, count1));
	}
	let pts = s_cullPoints;
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

const s_tentativeSorter = new b3TentativeSorter();

// C's points pointer is the cluster point buffer from pointOffset.
struct b3Cluster {
	manifoldNormal: b3Vec3 = b3Vec3();
	triangleNormal: b3Vec3 = b3Vec3();
	pointOffset: i32 = 0;
	pointCapacity: i32 = 0;
	pointCount: i32 = 0;
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
// those outside the hull bounds and writes the survivors. Its own function keeps the v128 lanes out of the caller's frame.
@unchecked
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
	let x = s_laneX;
	let y = s_laneY;
	let z = s_laneZ;

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

@unchecked
export function b3ComputeMeshManifolds(
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

	b3RefreshCache(contact, shapeA, xfA, shapeB.aabb);

	// Collide with triangles and build manifolds
	let meshContact = contact.meshContact;
	let triangleCount = meshContact.triangleCache.length;

	// C's pointer arrays become indices into the manifold buffer
	if (s_acceptedManifolds.length < triangleCount) {
		let capacity = b3ScratchCapacity(s_acceptedManifolds.length, triangleCount);
		s_acceptedManifolds = new StaticArray<i32>(capacity);
		s_tentativeManifolds = new StaticArray<i32>(capacity);
		s_tentativeTriangles = new StaticArray<b3TentativeTriangle>(capacity);
		s_survivorIndices = new StaticArray<i32>(capacity);
		s_survivorVertices = new StaticArray<b3Vec3>(3 * capacity);
	}
	let acceptedManifolds = s_acceptedManifolds;
	let acceptedManifoldCount = 0;
	let tentativeManifolds = s_tentativeManifolds;
	let tentativeManifoldCount = 0;
	let tentativeTriangles = s_tentativeTriangles;
	let tentativeTriangleCount = 0;

	let foundEdges: b3FoundEdges;
	let foundVertices: b3FoundVertices;
	foundEdges.count = 0;
	foundVertices.count = 0;

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

	if (s_pointBuffer.length < pointBufferCapacity) {
		s_pointBuffer = new StaticArray<b3LocalManifoldPoint>(b3ScratchCapacity(s_pointBuffer.length, pointBufferCapacity));
	}
	let pointBuffer = s_pointBuffer;
	let totalPointCount = 0;

	// C bumps triangleCount manifolds; a slot is created when first used and reused while it holds no points
	let manifoldBuffer = s_manifoldBuffer;
	let manifoldCount = 0;

	let triangleCaches = meshContact.triangleCache;

	let hullB = shapeB.type == b3_hullShape ? shapeB.hull : null;
	let hullSpeculativeDistance: f32 = enableSpeculative ? B3_SPECULATIVE_DISTANCE() : 0.0;
	let hullMargin = b3Vec3(hullSpeculativeDistance, hullSpeculativeDistance, hullSpeculativeDistance);

	let survivorIndices = s_survivorIndices;
	let survivorVertices = s_survivorVertices;

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

	// Mesh and height field each call the prefilter, which keeps it a function of its own in the release bake
	let survivorCount = 0;
	if (meshVertices != null) {
		survivorCount = b3PrefilterTriangles(
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
		);
	} else {
		survivorCount = b3PrefilterTriangles(
			shapeA,
			triangleCaches,
			triangleCount,
			null,
			null,
			meshScale,
			meshFlipped,
			relativeMatrix,
			transformAtoB.p,
			hullB,
			hullMargin.x,
			survivorIndices,
			survivorVertices,
		);
	}

	let vertices = s_triangleVertices;
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
		if (manifoldCount == manifoldBuffer.length) {
			manifoldBuffer.push(new b3LocalManifold());
		}
		let manifold = manifoldBuffer[manifoldCount];
		manifold.points = pointBuffer;
		manifold.pointOffset = totalPointCount;
		manifold.pointCount = 0;
		manifold.triangleFlags = triangle.flags;
		manifold.feature = b3_featureNone;

		switch (shapeB.type) {
			case b3_capsuleShape:
				b3CollideTriangleAndCapsule(manifold, pointCapacity, vertices, shapeB.capsule, cache.simplexCache);
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
				b3AddEdge(foundEdges, triangle.i1, triangle.i2);
				b3AddEdge(foundEdges, triangle.i2, triangle.i3);
				b3AddEdge(foundEdges, triangle.i3, triangle.i1);
				b3AddVertex(foundVertices, triangle.i1);
				b3AddVertex(foundVertices, triangle.i2);
				b3AddVertex(foundVertices, triangle.i3);

				acceptedManifolds[acceptedManifoldCount] = manifoldCount - 1;
				acceptedManifoldCount += 1;
			} else if (manifold.feature == b3_featureHullFace) {
				let cosNormalAngle = b3Dot(manifold.triangleNormal, manifold.normal);
				if (cosNormalAngle > 0.5) {
					b3AddEdge(foundEdges, triangle.i1, triangle.i2);
					b3AddEdge(foundEdges, triangle.i2, triangle.i3);
					b3AddEdge(foundEdges, triangle.i3, triangle.i1);
					b3AddVertex(foundVertices, triangle.i1);
					b3AddVertex(foundVertices, triangle.i2);
					b3AddVertex(foundVertices, triangle.i3);

					acceptedManifolds[acceptedManifoldCount] = manifoldCount - 1;
					acceptedManifoldCount += 1;
				} else {
					let minSeparation = manifold.points[manifold.pointOffset].separation;
					for (let i = 1; i < manifoldPointCount; ++i) {
						minSeparation = b3MinFloat(minSeparation, manifold.points[manifold.pointOffset + i].separation);
					}

					if (minSeparation < -2.0 * linearSlop) {
						// Deep overlap
						b3AddEdge(foundEdges, triangle.i1, triangle.i2);
						b3AddEdge(foundEdges, triangle.i2, triangle.i3);
						b3AddEdge(foundEdges, triangle.i3, triangle.i1);
						b3AddVertex(foundVertices, triangle.i1);
						b3AddVertex(foundVertices, triangle.i2);
						b3AddVertex(foundVertices, triangle.i3);
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
				s_tentativeSorter.triangles = tentativeTriangles;
				Q_LOOP<b3TentativeSorter>(tentativeTriangleCount, s_tentativeSorter);
			}
		}

		// Add tentative manifolds in sorted order. Avoid adding manifolds that generate ghost collisions.
		for (let i = 0; i < tentativeTriangleCount; ++i) {
			let tentativeIndex = tentativeManifolds[tentativeTriangles[i].index];
			let m = manifoldBuffer[tentativeIndex];

			let addedEdge1 = b3AddEdge(foundEdges, m.i1, m.i2);
			let addedEdge2 = b3AddEdge(foundEdges, m.i2, m.i3);
			let addedEdge3 = b3AddEdge(foundEdges, m.i3, m.i1);
			let addedVertex1 = b3AddVertex(foundVertices, m.i1);
			let addedVertex2 = b3AddVertex(foundVertices, m.i2);
			let addedVertex3 = b3AddVertex(foundVertices, m.i3);

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
				if (b3FindEdge(foundEdges, m.i1, m.i2)) {
					continue;
				}
			}

			if ((triangleFlags & b3_flatEdge2) == b3_flatEdge2) {
				if (b3FindEdge(foundEdges, m.i2, m.i3)) {
					continue;
				}
			}

			if ((triangleFlags & b3_flatEdge3) == b3_flatEdge3) {
				if (b3FindEdge(foundEdges, m.i3, m.i1)) {
					continue;
				}
			}

			acceptedManifolds[acceptedManifoldCount] = tentativeManifolds[i];
			acceptedManifoldCount += 1;
		}
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(acceptedManifoldCount <= triangleCount);

	if (acceptedManifoldCount == 0) {
		if (contact.manifoldCount > 0) {
			b3FreeManifolds(world, contact.manifolds, contact.manifoldCount);
			contact.manifolds = b3AllocateManifolds(world, 0);
			contact.manifoldCount = 0;
		}
		return false;
	}

	if (s_clusters.length < acceptedManifoldCount) {
		let capacity = b3ScratchCapacity(s_clusters.length, acceptedManifoldCount);
		s_clusters = new StaticArray<b3Cluster>(capacity);
		s_clusterMemberships = new StaticArray<i32>(capacity);
	}
	let clusters = s_clusters;
	let clusterMemberships = s_clusterMemberships;

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
	if (s_clusterPoints.length < clusterPointCount) {
		s_clusterPoints = new StaticArray<b3LocalManifoldPoint>(b3ScratchCapacity(s_clusterPoints.length, clusterPointCount));
	}
	let clusterPoints = s_clusterPoints;
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
		let reducedCount = b3ReduceCluster(clusterPoints, cm.pointOffset, cm.pointCount, cm.triangleNormal);
		cm.pointCount = reducedCount;
	}

	// Make a temporary copy of previous manifolds
	let oldManifoldCount = contact.manifoldCount;
	if (s_oldManifolds.length < oldManifoldCount) {
		let capacity = b3ScratchCapacity(s_oldManifolds.length, oldManifoldCount);
		s_oldManifolds = new StaticArray<b3Manifold>(capacity);
		s_consumed = new StaticArray<bool>(capacity);
	}
	let oldManifolds = s_oldManifolds;
	b3CopyManifolds(oldManifolds, contact.manifolds, oldManifoldCount);

	// Resize manifolds if needed
	if (oldManifoldCount != clusterCount) {
		b3FreeManifolds(world, contact.manifolds, contact.manifoldCount);
		contact.manifolds = b3AllocateManifolds(world, clusterCount);
		contact.manifoldCount = <i32>(<u16>clusterCount);
	} else {
		// Mem zero manifolds
		b3ZeroManifolds(contact.manifolds, contact.manifoldCount);
	}

	let consumed = s_consumed;
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
			ref target = manifold.points[j];

			// Contact points are computed in frame B
			target.anchorB = b3MulMV(matrixB, source.point);
			target.anchorA = b3Add(target.anchorB, offsetA);
			target.separation = source.separation - restOffset;
			target.featureId = b3MakeFeatureId(source.pair);
			target.triangleIndex = source.triangleIndex;

			// Preserve normal impulse if possible
			if (matchedIndex != B3_NULL_INDEX) {
				ref matchedManifold = oldManifolds[matchedIndex];
				let oldPointCount = matchedManifold.pointCount;
				for (let k = 0; k < oldPointCount; ++k) {
					ref oldPt = matchedManifold.points[k];

					if (target.featureId == oldPt.featureId && target.triangleIndex == oldPt.triangleIndex) {
						target.normalImpulse = oldPt.normalImpulse;
						target.persisted = true;

						// claimed
						oldPt.triangleIndex = B3_NULL_INDEX;
						break;
					}
				}
			}
		}
	}

	let materialsA = b3GetShapeMaterials(shapeA);
	let materialB = b3GetShapeMaterials(shapeB)[0];
	let tangentVelocityA = b3Vec3_zero;
	let frictionCallback = world.frictionCallback!;
	let restitutionCallback = world.restitutionCallback!;

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
				friction += frictionCallback(material.friction, material.userMaterialId, materialB.friction, materialB.userMaterialId);
				restitution += restitutionCallback(
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
		contact.friction = frictionCallback(b3SurfaceMaterialAt(materialsA, 0).friction, b3SurfaceMaterialAt(materialsA, 0).userMaterialId, materialB.friction, materialB.userMaterialId);
		contact.restitution = restitutionCallback(
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
