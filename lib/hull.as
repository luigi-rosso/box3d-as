// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/hull.h and src/hull.c

// Dirk Gregorius contributed portions of this code

import {
	B3_ASSERT,
	B3_ASSERT_ENABLED,
	B3_DEBUG,
	B3_ENABLE_VALIDATION,
	B3_NULL_INDEX,
	FLT_EPSILON,
	FLT_MAX,
	FLT_MIN,
	UINT8_MAX,
	b3Hash64NonZero,
	b3Log,
} from "./core";
import { B3_LINEAR_SLOP, B3_OVERLAP_SLOP, B3_MAX_HULL_EDGES, B3_MAX_HULL_FACES, B3_MAX_HULL_VERTICES } from "./constants";
import {
	B3_PI,
	b3AABB,
	b3AABB_Transform,
	b3AABB_Union,
	b3Abs,
	b3AbsFloat,
	b3Add,
	b3BoxInertia,
	b3ClampInt,
	b3ComputeCosSin,
	b3CosSin,
	b3Cos,
	b3Cross,
	b3DistanceSquared,
	b3Dot,
	b3GetLengthAndNormalize,
	b3InvMulTransforms,
	b3InvRotateVector,
	b3IsValidFloat,
	b3IsValidRay,
	b3IsValidTransform,
	b3Length,
	b3MakeMatrixFromQuat,
	b3MakeQuatFromMatrix,
	b3Matrix3,
	b3Max,
	b3MaxFloat,
	b3MaxInt,
	b3Min,
	b3MinFloat,
	b3Mul,
	b3MulMV,
	b3MulSM,
	b3MulSV,
	b3Neg,
	b3Normalize,
	b3Plane,
	b3Quat_identity,
	b3RotateVector,
	b3SafeScale,
	b3Sin,
	b3Steiner,
	b3Sub,
	b3SubMM,
	b3TransformPoint,
	b3Transform,
	b3Transform_identity,
	b3Vec2,
	b3Vec3,
	b3Vec3_axisX,
	b3Vec3_axisY,
	b3Vec3_axisZ,
	b3Vec3_zero,
} from "./math_functions";
import {
	B3_BOUNDS3_EMPTY,
	B3_SQRT3,
	b3DistanceSquared2,
	b3MakePlaneFromNormalAndPoint,
	b3MakePlaneFromPoints,
	b3MaxElementIndex,
	b3PlaneSeparation,
	b3RotateInertia,
	b3ScalarTripleProduct,
	b3ShapeExtent,
	b3TransformPlane,
} from "./math_internal";
import {
	B3_HULL_VERSION,
	b3BoxHull,
	b3CastOutput,
	b3GetHullEdges,
	b3GetHullFaces,
	b3GetHullPlanes,
	b3GetHullPoints,
	b3GetHullSoaVertices,
	b3GetHullVertices,
	b3Capsule,
	b3DistanceInput,
	b3HullData,
	b3HullHalfEdge,
	b3MassData,
	b3PlaneResult,
	b3Point2D,
	b3RayCastInput,
	b3ShapeCastInput,
	b3ShapeCastPairInput,
	b3ShapeProxy,
	b3SimplexCache,
} from "./collision";
import { b3MakeSegmentProxy, b3ShapeCast, b3ShapeDistance } from "./distance";
import {
	b3AddW4,
	b3BlendW4,
	b3GreaterThanW4,
	b3LoadW4,
	b3MulW4,
	b3SetW4,
	b3SplatW4,
} from "./simd";

// hull.h

// Big enough to handle AVX2.
export const B3_HULL_SOA_PADDING: i32 = 8;

@inline
export function b3GetHullSoaStride(count: i32): i32 {
	return (count + B3_HULL_SOA_PADDING - 1) & ~(B3_HULL_SOA_PADDING - 1);
}

// hull.c

const B3_AXIS_X: i32 = 0;
const B3_AXIS_Y: i32 = 1;
const B3_AXIS_Z: i32 = 2;

const B3_MARK_VISIBLE: i32 = 0;
const B3_MARK_DELETE: i32 = 1;

// Final hull is index-encoded with uint8_t, so vertex/edge/face counts are capped at 256.
const B3_HULL_MAX_COUNT: i32 = UINT8_MAX + 1;

// C links vertices and faces through pointers. Here every list node lives in b3HullBuilder::links
// and every pointer is an index, B3_NULL_INDEX for NULL. See b3HullBuilder for the node layout.
struct b3QHListNode {
	prev: i32 = 0;
	next: i32 = 0;
}

// The list link is links[vertex index].
struct b3QHVertex {
	conflictFace: i32 = 0;
	position: b3Vec3 = b3Vec3();

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex: i32 = 0;
	reachable: bool = false;
}

struct b3QHHalfEdge {
	// Edge ring (CCW) around the owning face. Not an external list.
	prev: i32 = 0;
	next: i32 = 0;

	origin: i32 = 0;
	face: i32 = 0;
	twin: i32 = 0;

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex: i32 = 0;
}

// The list link is links[faceLinkBase + face] and the conflict list head is
// links[conflictHeadBase + face].
struct b3QHFace {
	edge: i32 = 0;

	mark: i32 = 0;
	area: f32 = 0;
	plane: b3Plane = b3Plane();
	centroid: b3Vec3 = b3Vec3();
	maxConflictDistance: f32 = 0;

	// Cached farthest conflict vertex (above b3HullBuilder::minOutside).
	// NULL when no conflict above threshold; maxConflictDistance is then minOutside.
	maxConflict: i32 = 0;

	// Index in the finalized hull, stamped during emit. B3_NULL_INDEX until then.
	finalIndex: i32 = 0;
	flipped: bool = false;
}

// One frame of the iterative horizon DFS. Replaces a recursive call to b3HullBuilder_BuildHorizon.
struct b3HorizonFrame {
	face: i32 = 0;
	startEdge: i32 = 0; // ring termination sentinel
	edge: i32 = 0; // next edge to process
	started: bool = false; // false until the first edge of this ring has been processed
}

struct b3QHIndexPair {
	index1: i32 = 0;
	index2: i32 = 0;
}

// All working memory for one hull build.
// links holds every list node: vertex links at [0, vertexCapacity), then the orphaned, vertex and
// face list sentinels, then one link per face slot, then one conflict list head per face slot.
class b3HullBuilder {
	tolerance: f32 = 0;
	minRadius: f32 = 0;
	minOutside: f32 = 0;

	interiorPoint: b3Vec3 = b3Vec3();

	// List sentinels. Node indices into links.
	orphanedList: i32 = 0;
	vertexList: i32 = 0;
	faceList: i32 = 0;

	links: StaticArray<b3QHListNode>;
	faceLinkBase: i32 = 0;
	conflictHeadBase: i32 = 0;

	// Bump-allocated pools with index-based free lists for faces and edges.
	vertexBase: StaticArray<b3QHVertex>;
	vertexCapacity: i32 = 0;
	vertexCount: i32 = 0;

	edgeBase: StaticArray<b3QHHalfEdge>;
	edgeCapacity: i32 = 0;
	edgeCount: i32 = 0;

	// LIFO free list. Built with edge.next.
	edgeFreeHead: i32 = 0;

	faceBase: StaticArray<b3QHFace>;
	faceCapacity: i32 = 0;
	faceCount: i32 = 0;

	// LIFO free list. Built with the face link next.
	faceFreeHead: i32 = 0;

	// Reusable scratch buffers.
	horizon: StaticArray<i32>;
	horizonCapacity: i32 = 0;
	horizonCount: i32 = 0;

	cone: StaticArray<i32>;
	coneCapacity: i32 = 0;
	coneCount: i32 = 0;

	mergedFaces: StaticArray<i32>;
	mergedFacesCapacity: i32 = 0;
	mergedFacesCount: i32 = 0;

	// DFS stack used by the iterative b3HullBuilder_BuildHorizon. Depth bounded by live faces.
	horizonStack: StaticArray<b3HorizonFrame>;
	horizonStackCapacity: i32 = 0;

	// Final counts of the constructed hull (vertexList / faceList / half-edges around faces).
	finalVertexCount: i32 = 0;
	finalHalfEdgeCount: i32 = 0;
	finalFaceCount: i32 = 0;

	// b3HullBuilder_Init
	constructor(s: b3HullWorkSizes) {
		this.vertexCapacity = s.vertexCapacity;
		this.edgeCapacity = s.edgeCapacity;
		this.faceCapacity = s.faceCapacity;
		this.horizonCapacity = s.horizonCapacity;
		this.coneCapacity = s.coneCapacity;
		this.mergedFacesCapacity = s.mergedFacesCapacity;
		this.horizonStackCapacity = s.horizonStackCapacity;

		this.orphanedList = s.vertexCapacity;
		this.vertexList = s.vertexCapacity + 1;
		this.faceList = s.vertexCapacity + 2;
		this.faceLinkBase = s.vertexCapacity + 3;
		this.conflictHeadBase = this.faceLinkBase + s.faceCapacity;
		this.links = new StaticArray<b3QHListNode>(this.conflictHeadBase + s.faceCapacity);

		this.vertexBase = new StaticArray<b3QHVertex>(s.vertexCapacity);
		this.edgeBase = new StaticArray<b3QHHalfEdge>(s.edgeCapacity);
		this.faceBase = new StaticArray<b3QHFace>(s.faceCapacity);
		this.horizon = new StaticArray<i32>(s.horizonCapacity);
		this.cone = new StaticArray<i32>(s.coneCapacity);
		this.mergedFaces = new StaticArray<i32>(s.mergedFacesCapacity);
		this.horizonStack = new StaticArray<b3HorizonFrame>(s.horizonStackCapacity);

		this.edgeFreeHead = B3_NULL_INDEX;
		this.faceFreeHead = B3_NULL_INDEX;

		b3QHList_Init(this.links, this.orphanedList);
		b3QHList_Init(this.links, this.vertexList);
		b3QHList_Init(this.links, this.faceList);
	}
}

@inline
function b3QHList_Init(links: StaticArray<b3QHListNode>, head: i32): void {
	links[head].prev = head;
	links[head].next = head;
}

@inline
function B3_LIST_EMPTY(links: StaticArray<b3QHListNode>, a: i32): bool {
	return links[a].next == a;
}

@inline
function b3QHList_Contains(links: StaticArray<b3QHListNode>, node: i32): bool {
	return links[node].prev != B3_NULL_INDEX && links[node].next != B3_NULL_INDEX;
}

// Insert node before `where`.
function b3QHList_Insert(links: StaticArray<b3QHListNode>, node: i32, where: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(!b3QHList_Contains(links, node) && b3QHList_Contains(links, where));

	let prev = links[where].prev;
	links[node].prev = prev;
	links[node].next = where;

	links[prev].next = node;
	links[where].prev = node;
}

function b3QHList_Remove(links: StaticArray<b3QHListNode>, node: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3QHList_Contains(links, node));

	let prev = links[node].prev;
	let next = links[node].next;
	links[prev].next = next;
	links[next].prev = prev;

	links[node].prev = B3_NULL_INDEX;
	links[node].next = B3_NULL_INDEX;
}

@inline
function b3QHList_PushBack(links: StaticArray<b3QHListNode>, head: i32, node: i32): void {
	b3QHList_Insert(links, node, links[head].prev);
}

function b3HullBuilder_NewVertex(b: b3HullBuilder, position: b3Vec3): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b.vertexCount < b.vertexCapacity);
	let vertex = b.vertexCount++;

	b.links[vertex].prev = B3_NULL_INDEX;
	b.links[vertex].next = B3_NULL_INDEX;

	ref v = b.vertexBase[vertex];
	v.conflictFace = B3_NULL_INDEX;
	v.position = position;
	v.finalIndex = B3_NULL_INDEX;
	v.reachable = false;

	return vertex;
}

function b3HullBuilder_NewEdge(b: b3HullBuilder): i32 {
	let edge: i32;
	if (b.edgeFreeHead != B3_NULL_INDEX) {
		edge = b.edgeFreeHead;
		b.edgeFreeHead = b.edgeBase[edge].next;
	} else {
		if (B3_ASSERT_ENABLED) B3_ASSERT(b.edgeCount < b.edgeCapacity);
		edge = b.edgeCount++;
	}
	// All other fields (prev/next/origin/face/twin) are written by NewFace immediately after.
	b.edgeBase[edge].finalIndex = B3_NULL_INDEX;
	return edge;
}

@inline
function b3HullBuilder_RetireEdge(b: b3HullBuilder, edge: i32): void {
	b.edgeBase[edge].next = b.edgeFreeHead;
	b.edgeFreeHead = edge;
}

function b3HullBuilder_NewFace(b: b3HullBuilder, v1: i32, v2: i32, v3: i32): i32 {
	let links = b.links;
	let face: i32;
	if (b.faceFreeHead != B3_NULL_INDEX) {
		face = b.faceFreeHead;
		// link.next was used as free-list pointer; recover next head before we clobber.
		b.faceFreeHead = links[b.faceLinkBase + face].next;
	} else {
		if (B3_ASSERT_ENABLED) B3_ASSERT(b.faceCount < b.faceCapacity);
		face = b.faceCount++;
	}

	// link.prev: NULL on retired faces (cleared by Remove); NULL here so PushBack's
	// !b3QHList_Contains assert holds for fresh bump slots too.
	// link.next: was the free-list pointer on reused slots, now stale; PushBack overwrites.
	let faceLink = b.faceLinkBase + face;
	links[faceLink].prev = B3_NULL_INDEX;
	links[faceLink].next = B3_NULL_INDEX;

	ref f = b.faceBase[face];
	f.maxConflict = B3_NULL_INDEX;
	f.maxConflictDistance = 0.0;
	f.finalIndex = B3_NULL_INDEX;

	let edge1 = b3HullBuilder_NewEdge(b);
	let edge2 = b3HullBuilder_NewEdge(b);
	let edge3 = b3HullBuilder_NewEdge(b);

	let p1 = b.vertexBase[v1].position;
	let p2 = b.vertexBase[v2].position;
	let p3 = b.vertexBase[v3].position;

	let plane = b3Plane();
	plane.normal = b3Cross(b3Sub(p2, p1), b3Sub(p3, p1));
	let length: FixedArray<f32, 1>;
	plane.normal = b3GetLengthAndNormalize(length[0], plane.normal);
	plane.offset = b3Dot(plane.normal, p1);

	let area: f32 = 0.5 * length[0];

	f.edge = edge1;
	f.mark = B3_MARK_VISIBLE;
	f.area = area;
	f.centroid = b3MulSV(<f32>1.0 / <f32>3.0, b3Add(p1, b3Add(p2, p3)));
	f.plane = plane;
	f.flipped = b3PlaneSeparation(plane, b.interiorPoint) > 0.0;
	b3QHList_Init(links, b.conflictHeadBase + face);

	let edges = b.edgeBase;
	ref e1 = edges[edge1];
	e1.prev = edge3;
	e1.next = edge2;
	e1.origin = v1;
	e1.face = face;
	e1.twin = B3_NULL_INDEX;

	ref e2 = edges[edge2];
	e2.prev = edge1;
	e2.next = edge3;
	e2.origin = v2;
	e2.face = face;
	e2.twin = B3_NULL_INDEX;

	ref e3 = edges[edge3];
	e3.prev = edge2;
	e3.next = edge1;
	e3.origin = v3;
	e3.face = face;
	e3.twin = B3_NULL_INDEX;

	return face;
}

// Remove face and add to free list.
function b3HullBuilder_RetireFace(b: b3HullBuilder, face: i32): void {
	let links = b.links;
	let faceLink = b.faceLinkBase + face;

	// Sometimes a cone face gets merged and never added to the list.
	if (b3QHList_Contains(links, faceLink)) {
		b3QHList_Remove(links, faceLink);
	}

	b.faceBase[face].edge = B3_NULL_INDEX;
	// link.prev is already NULL after Remove (or was never set).
	links[faceLink].next = b.faceFreeHead;
	b.faceFreeHead = face;
}

function b3BuildBounds(vertexCount: i32, vertices: StaticArray<b3Vec3>): b3AABB {
	let bounds = B3_BOUNDS3_EMPTY;
	for (let i = 0; i < vertexCount; ++i) {
		bounds.lowerBound = b3Min(bounds.lowerBound, vertices[i]);
		bounds.upperBound = b3Max(bounds.upperBound, vertices[i]);
	}
	return bounds;
}

// C writes the two indices through out pointers.
function b3FindFarthestPointsAlongCardinalAxes(
	tolerance: f32,
	vertexCount: i32,
	vertexBase: StaticArray<b3Vec3>,
): b3QHIndexPair {
	let out = b3QHIndexPair();
	out.index1 = B3_NULL_INDEX;
	out.index2 = B3_NULL_INDEX;

	let v0 = vertexBase[0];
	let minPt: FixedArray<b3Vec3, 3>;
	let maxPt: FixedArray<b3Vec3, 3>;
	minPt[0] = v0;
	minPt[1] = v0;
	minPt[2] = v0;
	maxPt[0] = v0;
	maxPt[1] = v0;
	maxPt[2] = v0;

	let minIndex: FixedArray<i32, 3>;
	let maxIndex: FixedArray<i32, 3>;

	for (let i = 1; i < vertexCount; ++i) {
		let v = vertexBase[i];

		if (v.x < minPt[B3_AXIS_X].x) {
			minPt[B3_AXIS_X] = v;
			minIndex[B3_AXIS_X] = i;
		} else if (v.x > maxPt[B3_AXIS_X].x) {
			maxPt[B3_AXIS_X] = v;
			maxIndex[B3_AXIS_X] = i;
		}

		if (v.y < minPt[B3_AXIS_Y].y) {
			minPt[B3_AXIS_Y] = v;
			minIndex[B3_AXIS_Y] = i;
		} else if (v.y > maxPt[B3_AXIS_Y].y) {
			maxPt[B3_AXIS_Y] = v;
			maxIndex[B3_AXIS_Y] = i;
		}

		if (v.z < minPt[B3_AXIS_Z].z) {
			minPt[B3_AXIS_Z] = v;
			minIndex[B3_AXIS_Z] = i;
		} else if (v.z > maxPt[B3_AXIS_Z].z) {
			maxPt[B3_AXIS_Z] = v;
			maxIndex[B3_AXIS_Z] = i;
		}
	}

	let distance = b3Vec3();
	distance.x = maxPt[B3_AXIS_X].x - minPt[B3_AXIS_X].x;
	distance.y = maxPt[B3_AXIS_Y].y - minPt[B3_AXIS_Y].y;
	distance.z = maxPt[B3_AXIS_Z].z - minPt[B3_AXIS_Z].z;

	let distanceArray: FixedArray<f32, 3>;
	distanceArray[0] = distance.x;
	distanceArray[1] = distance.y;
	distanceArray[2] = distance.z;
	let maxElement = b3MaxElementIndex(distance);

	if (distanceArray[maxElement] > 2.0 * tolerance) {
		out.index1 = minIndex[maxElement];
		out.index2 = maxIndex[maxElement];
	}

	return out;
}

function b3FindFarthestPointFromLine(
	index1: i32,
	index2: i32,
	tolerance: f32,
	vertexCount: i32,
	vertexBase: StaticArray<b3Vec3>,
): i32 {
	let a = vertexBase[index1];
	let b = vertexBase[index2];

	// |ap x ab|^2 / |ab|^2 is the squared perpendicular distance from p to the line.
	// Compares against (2 * tolerance)^2
	let ab = b3Sub(b, a);
	let abLengthSqr = b3Dot(ab, ab);
	if (B3_ASSERT_ENABLED) B3_ASSERT(abLengthSqr > 0.0);

	let invAbLengthSqr: f32 = 1.0 / abLengthSqr;
	let maxDistanceSqr: f32 = 4.0 * tolerance * tolerance;
	let maxIndex = B3_NULL_INDEX;

	for (let i = 0; i < vertexCount; ++i) {
		if (i == index1 || i == index2) {
			continue;
		}

		let ap = b3Sub(vertexBase[i], a);
		let cross = b3Cross(ap, ab);
		let distanceSqr = b3Dot(cross, cross) * invAbLengthSqr;
		if (distanceSqr > maxDistanceSqr) {
			maxDistanceSqr = distanceSqr;
			maxIndex = i;
		}
	}

	return maxIndex;
}

function b3FindFarthestPointFromPlane(
	index1: i32,
	index2: i32,
	index3: i32,
	tolerance: f32,
	vertexCount: i32,
	vertexBase: StaticArray<b3Vec3>,
): i32 {
	let a = vertexBase[index1];
	let b = vertexBase[index2];
	let c = vertexBase[index3];

	let plane = b3MakePlaneFromPoints(a, b, c);

	let maxDistance: f32 = 2.0 * tolerance;
	let maxIndex = B3_NULL_INDEX;

	for (let i = 0; i < vertexCount; ++i) {
		if (i == index1 || i == index2 || i == index3) {
			continue;
		}

		let distance = b3AbsFloat(b3PlaneSeparation(plane, vertexBase[i]));
		if (distance > maxDistance) {
			maxDistance = distance;
			maxIndex = i;
		}
	}

	return maxIndex;
}

function b3IsEdgeConvex(b: b3HullBuilder, edge: i32, tolerance: f32): bool {
	let faces = b.faceBase;
	let edges = b.edgeBase;
	let distance = b3PlaneSeparation(faces[edges[edge].face].plane, faces[edges[edges[edge].twin].face].centroid);
	return distance < -tolerance;
}

function b3IsEdgeConcave(b: b3HullBuilder, edge: i32, tolerance: f32): bool {
	let faces = b.faceBase;
	let edges = b.edgeBase;
	let distance = b3PlaneSeparation(faces[edges[edge].face].plane, faces[edges[edges[edge].twin].face].centroid);
	return distance > tolerance;
}

function b3VertexCountOfFace(b: b3HullBuilder, face: i32): i32 {
	let edges = b.edgeBase;
	let start = b.faceBase[face].edge;
	let count = 0;
	let edge = start;
	do {
		count++;
		edge = edges[edge].next;
	} while (edge != start);

	return count;
}

function b3LinkFace(b: b3HullBuilder, face: i32, index: i32, twin: i32): void {
	let edges = b.edgeBase;
	if (B3_ASSERT_ENABLED) B3_ASSERT(face != edges[twin].face);

	let edge = b.faceBase[face].edge;
	while (index-- > 0) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(edges[edge].face == face);
		edge = edges[edge].next;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(edge != twin);
	edges[edge].twin = twin;
	edges[twin].twin = edge;
}

function b3LinkFaces(b: b3HullBuilder, face1: i32, index1: i32, face2: i32, index2: i32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(face1 != face2);
	let edges = b.edgeBase;

	let edge1 = b.faceBase[face1].edge;
	while (index1-- > 0) {
		edge1 = edges[edge1].next;
	}

	let edge2 = b.faceBase[face2].edge;
	while (index2-- > 0) {
		edge2 = edges[edge2].next;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(edge1 != edge2);
	edges[edge1].twin = edge2;
	edges[edge2].twin = edge1;
}

function b3NewellPlane(b: b3HullBuilder, face: i32): void {
	let edges = b.edgeBase;
	let vertices = b.vertexBase;
	ref f = b.faceBase[face];

	let count = 0;
	let centroid = b3Vec3_zero;
	let normal = b3Vec3_zero;

	let start = f.edge;
	let edge = start;
	if (B3_ASSERT_ENABLED) B3_ASSERT(edges[edge].face == face);

	// Use the first vertex as the origin to reduce round-off
	let origin = vertices[edges[edge].origin].position;

	do {
		let twin = edges[edge].twin;
		if (B3_ASSERT_ENABLED) B3_ASSERT(edges[twin].twin == edge);

		let v1 = b3Sub(vertices[edges[edge].origin].position, origin);
		let v2 = b3Sub(vertices[edges[twin].origin].position, origin);

		count++;
		centroid = b3Add(centroid, v1);
		normal.x += (v1.y - v2.y) * (v1.z + v2.z);
		normal.y += (v1.z - v2.z) * (v1.x + v2.x);
		normal.z += (v1.x - v2.x) * (v1.y + v2.y);

		edge = edges[edge].next;
	} while (edge != start);

	if (B3_ASSERT_ENABLED) B3_ASSERT(count > 0);
	centroid = b3MulSV(<f32>1.0 / <f32>count, centroid);
	centroid = b3Add(centroid, origin);

	let length = b3Length(normal);
	normal = b3MulSV(<f32>1.0 / length, normal);

	f.centroid = centroid;
	f.plane = b3MakePlaneFromNormalAndPoint(normal, centroid);
	f.area = 0.5 * length;
}

// B3_DEBUG only
function b3CheckConsistency(b: b3HullBuilder, face: i32): bool {
	let faces = b.faceBase;
	let edges = b.edgeBase;

	if (faces[face].mark == B3_MARK_DELETE) {
		return false;
	}

	if (b3VertexCountOfFace(b, face) < 3) {
		return false;
	}

	let start = faces[face].edge;
	let edge = start;

	do {
		let twin = edges[edge].twin;

		if (twin == B3_NULL_INDEX) {
			return false;
		}
		let twinFace = edges[twin].face;
		if (twinFace == B3_NULL_INDEX) {
			return false;
		}
		if (twinFace == face) {
			return false;
		}
		if (faces[twinFace].mark == B3_MARK_DELETE) {
			return false;
		}
		if (edges[twin].twin != edge) {
			return false;
		}
		if (edges[edges[edge].next].origin != edges[twin].origin) {
			return false;
		}
		if (edges[edge].origin != edges[edges[twin].next].origin) {
			return false;
		}
		if (edges[edge].face != face) {
			return false;
		}

		edge = edges[edge].next;
	} while (edge != start);

	return true;
}

function b3HullBuilder_ComputeTolerance(b: b3HullBuilder, pointCount: i32, points: StaticArray<b3Vec3>): void {
	let bounds = b3BuildBounds(pointCount, points);
	let maxAbs = b3Max(b3Abs(bounds.lowerBound), b3Abs(bounds.upperBound));

	let maxSum = maxAbs.x + maxAbs.y + maxAbs.z;
	let maxCoord = b3MaxFloat(maxAbs.x, b3MaxFloat(maxAbs.y, maxAbs.z));
	let maxDistance = b3MinFloat(B3_SQRT3 * maxCoord, maxSum);

	let tolerance: f32 = (3.0 * maxDistance * 1.01 + maxCoord) * FLT_EPSILON;

	b.tolerance = tolerance;
	b.minRadius = 4.0 * b.tolerance;
	b.minOutside = 2.0 * b.minRadius;
	if (B3_ASSERT_ENABLED) B3_ASSERT(b.minRadius < b.minOutside + 3.0 * FLT_EPSILON);
}

function b3HullBuilder_BuildInitialHull(b: b3HullBuilder, pointCount: i32, points: StaticArray<b3Vec3>): bool {
	let pair = b3FindFarthestPointsAlongCardinalAxes(b.tolerance, pointCount, points);
	let index1 = pair.index1;
	let index2 = pair.index2;
	if (index1 < 0 || index2 < 0) {
		return false;
	}

	let index3 = b3FindFarthestPointFromLine(index1, index2, b.tolerance, pointCount, points);
	if (index3 < 0) {
		return false;
	}

	let index4 = b3FindFarthestPointFromPlane(index1, index2, index3, b.tolerance, pointCount, points);
	if (index4 < 0) {
		return false;
	}

	let v1 = b3Sub(points[index1], points[index4]);
	let v2 = b3Sub(points[index2], points[index4]);
	let v3 = b3Sub(points[index3], points[index4]);

	if (b3ScalarTripleProduct(v1, v2, v3) < 0.0) {
		let temp = index2;
		index2 = index3;
		index3 = temp;
	}

	b.interiorPoint = b3Vec3_zero;
	b.interiorPoint = b3Add(b.interiorPoint, points[index1]);
	b.interiorPoint = b3Add(b.interiorPoint, points[index2]);
	b.interiorPoint = b3Add(b.interiorPoint, points[index3]);
	b.interiorPoint = b3Add(b.interiorPoint, points[index4]);
	b.interiorPoint = b3MulSV(0.25, b.interiorPoint);

	let links = b.links;
	let vertex1 = b3HullBuilder_NewVertex(b, points[index1]);
	b3QHList_PushBack(links, b.vertexList, vertex1);
	let vertex2 = b3HullBuilder_NewVertex(b, points[index2]);
	b3QHList_PushBack(links, b.vertexList, vertex2);
	let vertex3 = b3HullBuilder_NewVertex(b, points[index3]);
	b3QHList_PushBack(links, b.vertexList, vertex3);
	let vertex4 = b3HullBuilder_NewVertex(b, points[index4]);
	b3QHList_PushBack(links, b.vertexList, vertex4);

	let face1 = b3HullBuilder_NewFace(b, vertex1, vertex2, vertex3);
	b3QHList_PushBack(links, b.faceList, b.faceLinkBase + face1);
	let face2 = b3HullBuilder_NewFace(b, vertex4, vertex2, vertex1);
	b3QHList_PushBack(links, b.faceList, b.faceLinkBase + face2);
	let face3 = b3HullBuilder_NewFace(b, vertex4, vertex3, vertex2);
	b3QHList_PushBack(links, b.faceList, b.faceLinkBase + face3);
	let face4 = b3HullBuilder_NewFace(b, vertex4, vertex1, vertex3);
	b3QHList_PushBack(links, b.faceList, b.faceLinkBase + face4);

	b3LinkFaces(b, face1, 0, face2, 1);
	b3LinkFaces(b, face1, 1, face3, 1);
	b3LinkFaces(b, face1, 2, face4, 1);

	b3LinkFaces(b, face2, 0, face3, 2);
	b3LinkFaces(b, face3, 0, face4, 2);
	b3LinkFaces(b, face4, 0, face2, 2);

	if (B3_DEBUG != 0) {
		B3_ASSERT(b3CheckConsistency(b, face1));
		B3_ASSERT(b3CheckConsistency(b, face2));
		B3_ASSERT(b3CheckConsistency(b, face3));
		B3_ASSERT(b3CheckConsistency(b, face4));
	}

	let faces = b.faceBase;
	let faceListHead = b.faceList;
	let faceLinkBase = b.faceLinkBase;
	for (let index = 0; index < pointCount; ++index) {
		if (index == index1 || index == index2 || index == index3 || index == index4) {
			continue;
		}

		let point = points[index];

		let maxDistance = b.minOutside;
		let maxFace = B3_NULL_INDEX;

		for (let node = links[faceListHead].next; node != faceListHead; node = links[node].next) {
			let face = node - faceLinkBase;
			let distance = b3PlaneSeparation(faces[face].plane, point);
			if (distance > maxDistance) {
				maxDistance = distance;
				maxFace = face;
			}
		}

		if (maxFace != B3_NULL_INDEX) {
			let vertex = b3HullBuilder_NewVertex(b, point);
			b.vertexBase[vertex].conflictFace = maxFace;
			b3QHList_PushBack(links, b.conflictHeadBase + maxFace, vertex);
			if (maxDistance > faces[maxFace].maxConflictDistance) {
				faces[maxFace].maxConflictDistance = maxDistance;
				faces[maxFace].maxConflict = vertex;
			}
		}
	}

	return true;
}

// Recompute the farthest-conflict cache after a face's plane changes.
// Walks the existing conflict list once; cost is bounded by that list, not the global pool.
function b3HullBuilder_RecacheConflicts(b: b3HullBuilder, face: i32, minOutside: f32): void {
	let links = b.links;
	let vertices = b.vertexBase;
	ref f = b.faceBase[face];
	let head = b.conflictHeadBase + face;

	let maxVertex = B3_NULL_INDEX;
	let maxDistance = minOutside;

	for (let node = links[head].next; node != head; node = links[node].next) {
		let distance = b3PlaneSeparation(f.plane, vertices[node].position);
		if (distance > maxDistance) {
			maxDistance = distance;
			maxVertex = node;
		}
	}

	f.maxConflict = maxVertex;
	f.maxConflictDistance = maxDistance;
}

function b3HullBuilder_NextConflictVertex(b: b3HullBuilder): i32 {
	let links = b.links;
	let faces = b.faceBase;
	let faceListHead = b.faceList;
	let faceLinkBase = b.faceLinkBase;

	let maxVertex = B3_NULL_INDEX;
	let maxDistance = b.minOutside;

	for (let faceNode = links[faceListHead].next; faceNode != faceListHead; faceNode = links[faceNode].next) {
		let face = faceNode - faceLinkBase;
		if (faces[face].maxConflict != B3_NULL_INDEX && faces[face].maxConflictDistance > maxDistance) {
			maxDistance = faces[face].maxConflictDistance;
			maxVertex = faces[face].maxConflict;
		}
	}

	return maxVertex;
}

// Move every conflict vertex of `face` onto the orphaned list and clear their conflictFace.
function b3HullBuilder_DrainConflictList(b: b3HullBuilder, face: i32): void {
	let links = b.links;
	let head = b.conflictHeadBase + face;
	let node = links[head].next;
	while (node != head) {
		let orphan = node;
		node = links[node].next;

		b.vertexBase[orphan].conflictFace = B3_NULL_INDEX;
		b3QHList_Remove(links, orphan);
		b3QHList_PushBack(links, b.orphanedList, orphan);
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(B3_LIST_EMPTY(links, head));
}

// Mark a face for deletion, drain its conflict list, and populate a fresh DFS frame for it.
// `entryEdge` is the half-edge in `face` whose twin lies in the just-deleted parent face, or
// NULL for the seed. The frame skips `entryEdge` on recursive entries (it would be ignored
// anyway since the parent's mark is now DELETE, but skipping saves one iteration).
function b3HullBuilder_EnterHorizonFace(b: b3HullBuilder, face: i32, entryEdge: i32, frameIndex: i32): void {
	b.faceBase[face].mark = B3_MARK_DELETE;
	b3HullBuilder_DrainConflictList(b, face);

	ref frameOut = b.horizonStack[frameIndex];
	frameOut.face = face;
	frameOut.started = false;
	if (entryEdge != B3_NULL_INDEX) {
		frameOut.startEdge = entryEdge;
		frameOut.edge = b.edgeBase[entryEdge].next;
	} else {
		frameOut.startEdge = b.faceBase[face].edge;
		frameOut.edge = b.faceBase[face].edge;
	}
}

function b3HullBuilder_BuildHorizon(b: b3HullBuilder, apex: i32, seed: i32): void {
	let stack = b.horizonStack;
	let edges = b.edgeBase;
	let faces = b.faceBase;
	let apexPosition = b.vertexBase[apex].position;
	let top = 0;

	if (B3_ASSERT_ENABLED) B3_ASSERT(top < b.horizonStackCapacity);
	b3HullBuilder_EnterHorizonFace(b, seed, B3_NULL_INDEX, top++);

	while (top > 0) {
		ref f = stack[top - 1];

		if (f.started && f.edge == f.startEdge) {
			top--;
			continue;
		}
		f.started = true;

		let edge = f.edge;
		let twin = edges[edge].twin;
		f.edge = edges[edge].next;

		let twinFace = edges[twin].face;
		if (faces[twinFace].mark != B3_MARK_VISIBLE) {
			continue;
		}

		let distance = b3PlaneSeparation(faces[twinFace].plane, apexPosition);
		if (distance > b.minRadius) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(top < b.horizonStackCapacity);
			b3HullBuilder_EnterHorizonFace(b, twinFace, twin, top++);
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(b.horizonCount < b.horizonCapacity);
			b.horizon[b.horizonCount++] = edge;
		}
	}
}

function b3HullBuilder_BuildCone(b: b3HullBuilder, apex: i32): void {
	let edges = b.edgeBase;
	for (let i = 0; i < b.horizonCount; ++i) {
		let edge = b.horizon[i];
		let twin = edges[edge].twin;
		if (B3_ASSERT_ENABLED) B3_ASSERT(edges[twin].twin == edge);

		let face = b3HullBuilder_NewFace(b, apex, edges[edge].origin, edges[twin].origin);
		if (B3_ASSERT_ENABLED) B3_ASSERT(b.coneCount < b.coneCapacity);
		b.cone[b.coneCount++] = face;

		b3LinkFace(b, face, 1, twin);
	}

	let face1 = b.cone[b.coneCount - 1];
	for (let i = 0; i < b.coneCount; ++i) {
		let face2 = b.cone[i];
		b3LinkFaces(b, face1, 2, face2, 0);
		face1 = face2;
	}
}

// Retire half-edges in the half-open ring range [begin, end) by pushing each onto edgeFreeHead.
// The caller has already detached these edges from the live face ring (rewire happens before
// destroy), so they are unreachable from the live hull.
function b3HullBuilder_DestroyEdges(b: b3HullBuilder, begin: i32, end: i32): void {
	let edge = begin;
	while (edge != end) {
		let next = b.edgeBase[edge].next;
		b3HullBuilder_RetireEdge(b, edge);
		edge = next;
	}
}

function b3HullBuilder_ConnectEdges(b: b3HullBuilder, prev: i32, next: i32): void {
	let edges = b.edgeBase;
	let faces = b.faceBase;
	if (B3_ASSERT_ENABLED) B3_ASSERT(prev != next);
	if (B3_ASSERT_ENABLED) B3_ASSERT(edges[prev].face == edges[next].face);

	// If both shared neighbors are the same face, prev and next together would orphan that face.
	if (edges[edges[prev].twin].face == edges[edges[next].twin].face) {
		// next is redundant.
		let nextFace = edges[next].face;
		if (faces[nextFace].edge == next) {
			faces[nextFace].edge = prev;
		}

		let twin: i32;
		if (b3VertexCountOfFace(b, edges[edges[prev].twin].face) == 3) {
			// Capture all 3 half-edges of the dead triangle before the rewire overwrites prev->twin.
			let deadEdge0 = edges[prev].twin; // prev->twin (will be rewired below)
			let deadEdge1 = edges[next].twin; // next->twin
			let deadEdge2 = edges[edges[next].twin].prev; // third edge of the dead triangle

			twin = edges[deadEdge2].twin;
			if (B3_ASSERT_ENABLED) B3_ASSERT(faces[edges[twin].face].mark != B3_MARK_DELETE);

			let opposingFace = edges[edges[prev].twin].face;
			faces[opposingFace].mark = B3_MARK_DELETE;
			if (B3_ASSERT_ENABLED) B3_ASSERT(b.mergedFacesCount < b.mergedFacesCapacity);
			b.mergedFaces[b.mergedFacesCount++] = opposingFace;

			edges[prev].next = edges[next].next;
			edges[edges[prev].next].prev = prev;

			edges[prev].twin = twin;
			edges[twin].twin = prev;

			// Drop the redundant vertex (slot abandoned in the bump allocator).
			b3QHList_Remove(b.links, edges[next].origin);

			// Retire the 3 half-edges of the dead triangle now that the rewire is complete.
			b3HullBuilder_RetireEdge(b, deadEdge0);
			b3HullBuilder_RetireEdge(b, deadEdge1);
			b3HullBuilder_RetireEdge(b, deadEdge2);
		} else {
			twin = edges[next].twin;

			let twinFace = edges[twin].face;
			if (faces[twinFace].edge == edges[prev].twin) {
				faces[twinFace].edge = twin;
			}

			edges[twin].next = edges[edges[prev].twin].next;
			edges[edges[twin].next].prev = twin;
			// prev->twin slot is retired to the edge free list.
			b3HullBuilder_RetireEdge(b, edges[prev].twin);

			edges[prev].next = edges[next].next;
			edges[edges[prev].next].prev = prev;

			edges[prev].twin = twin;
			edges[twin].twin = prev;

			// Drop the redundant vertex (slot abandoned in the bump allocator).
			b3QHList_Remove(b.links, edges[next].origin);
		}

		// Twin->face changed shape; recompute its plane and refresh its cached max conflict.
		b3NewellPlane(b, edges[twin].face);
		b3HullBuilder_RecacheConflicts(b, edges[twin].face, b.minOutside);
	} else {
		edges[prev].next = next;
		edges[next].prev = prev;
	}
}

function b3HullBuilder_AbsorbFaces(b: b3HullBuilder, face: i32): void {
	let links = b.links;
	let vertices = b.vertexBase;
	let faces = b.faceBase;
	let faceHead = b.conflictHeadBase + face;

	for (let i = 0; i < b.mergedFacesCount; ++i) {
		let mergedFace = b.mergedFaces[i];
		if (B3_ASSERT_ENABLED) B3_ASSERT(faces[mergedFace].mark == B3_MARK_DELETE);
		let head = b.conflictHeadBase + mergedFace;

		let node = links[head].next;
		while (node != head) {
			let vertex = node;
			node = links[node].next;

			b3QHList_Remove(links, vertex);

			let distance = b3PlaneSeparation(faces[face].plane, vertices[vertex].position);
			if (distance > b.minOutside) {
				b3QHList_PushBack(links, faceHead, vertex);
				vertices[vertex].conflictFace = face;
				if (distance > faces[face].maxConflictDistance) {
					faces[face].maxConflictDistance = distance;
					faces[face].maxConflict = vertex;
				}
			} else {
				b3QHList_PushBack(links, b.orphanedList, vertex);
				vertices[vertex].conflictFace = B3_NULL_INDEX;
			}
		}

		if (B3_ASSERT_ENABLED) B3_ASSERT(B3_LIST_EMPTY(links, head));

		// Conflict list is now drained. Retire this face to the free list.
		b3HullBuilder_RetireFace(b, mergedFace);
	}
}

function b3HullBuilder_ConnectFaces(b: b3HullBuilder, edge: i32): void {
	let edges = b.edgeBase;
	let faces = b.faceBase;
	let face = edges[edge].face;

	let twin = edges[edge].twin;
	let twinFace = edges[twin].face;

	let edgePrev = edges[edge].prev;
	let edgeNext = edges[edge].next;
	let twinPrev = edges[twin].prev;
	let twinNext = edges[twin].next;

	while (edges[edges[edgePrev].twin].face == twinFace) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(edges[edgePrev].twin == twinNext);
		if (B3_ASSERT_ENABLED) B3_ASSERT(edges[twinNext].twin == edgePrev);

		edgePrev = edges[edgePrev].prev;
		twinNext = edges[twinNext].next;
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(edges[edgePrev].face != edges[twinNext].face);

	while (edges[edges[edgeNext].twin].face == twinFace) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(edges[edgeNext].twin == twinPrev);
		if (B3_ASSERT_ENABLED) B3_ASSERT(edges[twinPrev].twin == edgeNext);

		edgeNext = edges[edgeNext].next;
		twinPrev = edges[twinPrev].prev;
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(edges[edgeNext].face != edges[twinPrev].face);

	faces[face].edge = edgePrev;

	// Discard opposing face. mergedFaces is single-buffered: ConnectFaces does not nest.
	b.mergedFacesCount = 0;
	if (B3_ASSERT_ENABLED) B3_ASSERT(b.mergedFacesCount < b.mergedFacesCapacity);
	b.mergedFaces[b.mergedFacesCount++] = twinFace;
	faces[twinFace].mark = B3_MARK_DELETE;
	faces[twinFace].edge = B3_NULL_INDEX;

	let absorbEnd = edges[twinPrev].next;
	for (let absorbed = twinNext; absorbed != absorbEnd; absorbed = edges[absorbed].next) {
		edges[absorbed].face = face;
	}

	b3HullBuilder_DestroyEdges(b, edges[edgePrev].next, edgeNext);
	b3HullBuilder_DestroyEdges(b, edges[twinPrev].next, twinNext);

	b3HullBuilder_ConnectEdges(b, edgePrev, twinNext);
	b3HullBuilder_ConnectEdges(b, twinPrev, edgeNext);

	b3NewellPlane(b, face);
	// Existing conflicts now have stale distances under the new plane; AbsorbFaces will then
	// add more incrementally and update the cache as it goes.
	b3HullBuilder_RecacheConflicts(b, face, b.minOutside);
	if (B3_DEBUG != 0) {
		B3_ASSERT(b3CheckConsistency(b, face));
	}

	b3HullBuilder_AbsorbFaces(b, face);
}

function b3HullBuilder_MergeConcave(b: b3HullBuilder, face: i32): bool {
	let edges = b.edgeBase;
	let start = b.faceBase[face].edge;
	let edge = start;

	do {
		let twin = edges[edge].twin;

		if (b3IsEdgeConcave(b, edge, b.minRadius) || b3IsEdgeConcave(b, twin, b.minRadius)) {
			b3HullBuilder_ConnectFaces(b, edge);
			return true;
		}

		edge = edges[edge].next;
	} while (edge != start);

	return false;
}

function b3HullBuilder_MergeCoplanar(b: b3HullBuilder, face: i32): bool {
	let edges = b.edgeBase;
	let start = b.faceBase[face].edge;
	let edge = start;

	do {
		let twin = edges[edge].twin;

		if (!b3IsEdgeConvex(b, edge, b.minRadius) || !b3IsEdgeConvex(b, twin, b.minRadius)) {
			b3HullBuilder_ConnectFaces(b, edge);
			return true;
		}

		edge = edges[edge].next;
	} while (edge != start);

	return false;
}

function b3HullBuilder_MergeFaces(b: b3HullBuilder): void {
	let edges = b.edgeBase;
	let faces = b.faceBase;

	for (let i = 0; i < b.coneCount; ++i) {
		let face = b.cone[i];
		if (faces[face].mark == B3_MARK_VISIBLE && faces[face].flipped) {
			faces[face].flipped = false;

			let bestArea: f32 = 0;
			let bestEdge = B3_NULL_INDEX;

			let start = faces[face].edge;
			let edge = start;
			do {
				let twin = edges[edge].twin;
				let area = faces[edges[twin].face].area;
				if (area > bestArea) {
					bestArea = area;
					bestEdge = edge;
				}

				edge = edges[edge].next;
			} while (edge != start);

			if (B3_ASSERT_ENABLED) B3_ASSERT(bestEdge != B3_NULL_INDEX);
			b3HullBuilder_ConnectFaces(b, bestEdge);
		}
	}

	for (let i = 0; i < b.coneCount; ++i) {
		let face = b.cone[i];
		if (faces[face].mark == B3_MARK_VISIBLE) {
			while (b3HullBuilder_MergeConcave(b, face)) {}
		}
	}

	for (let i = 0; i < b.coneCount; ++i) {
		let face = b.cone[i];
		if (faces[face].mark == B3_MARK_VISIBLE) {
			while (b3HullBuilder_MergeCoplanar(b, face)) {}
		}
	}
}

function b3HullBuilder_ResolveVertices(b: b3HullBuilder): void {
	let links = b.links;
	let vertices = b.vertexBase;
	let faces = b.faceBase;
	let head = b.orphanedList;

	let node = links[head].next;
	while (node != head) {
		let vertex = node;
		node = links[node].next;
		b3QHList_Remove(links, vertex);

		let position = vertices[vertex].position;
		let maxDistance = b.minOutside;
		let maxFace = B3_NULL_INDEX;

		for (let i = 0; i < b.coneCount; ++i) {
			let coneFace = b.cone[i];
			if (faces[coneFace].mark == B3_MARK_VISIBLE) {
				let distance = b3PlaneSeparation(faces[coneFace].plane, position);
				if (distance > maxDistance) {
					maxDistance = distance;
					maxFace = coneFace;
				}
			}
		}

		if (maxFace != B3_NULL_INDEX) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(faces[maxFace].mark == B3_MARK_VISIBLE);
			b3QHList_PushBack(links, b.conflictHeadBase + maxFace, vertex);
			vertices[vertex].conflictFace = maxFace;
			if (maxDistance > faces[maxFace].maxConflictDistance) {
				faces[maxFace].maxConflictDistance = maxDistance;
				faces[maxFace].maxConflict = vertex;
			}
		}
		// Otherwise: vertex is interior to the hull. Its slot in the bump pool is abandoned.
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(B3_LIST_EMPTY(links, head));
}

function b3HullBuilder_ResolveFaces(b: b3HullBuilder): void {
	let links = b.links;
	let edges = b.edgeBase;
	let faces = b.faceBase;
	let faceListHead = b.faceList;
	let faceLinkBase = b.faceLinkBase;

	// Splice deleted faces out of the face list. Faces already retired by AbsorbFaces are no
	// longer on faceList, so guard with b3QHList_Contains before removing.
	let node = links[faceListHead].next;
	while (node != faceListHead) {
		let face = node - faceLinkBase;
		node = links[node].next;

		if (faces[face].mark == B3_MARK_DELETE && b3QHList_Contains(links, faceLinkBase + face)) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(B3_LIST_EMPTY(links, b.conflictHeadBase + face));

			// Each half-edge is owned by exactly one face, so ring walks over the
			// dead region retire every interior edge exactly once. Merge deleted
			// faces are already off the face list.
			let start = faces[face].edge;
			let edge = start;
			do {
				let next = edges[edge].next;
				b3HullBuilder_RetireEdge(b, edge);
				edge = next;
			} while (edge != start);

			b3HullBuilder_RetireFace(b, face);
		}
	}

	for (let i = 0; i < b.coneCount; ++i) {
		let face = b.cone[i];
		if (faces[face].mark == B3_MARK_DELETE) {
			continue;
		}
		b3QHList_PushBack(links, faceListHead, faceLinkBase + face);
	}
}

function b3HullBuilder_AddVertexToHull(b: b3HullBuilder, vertex: i32): void {
	let face = b.vertexBase[vertex].conflictFace;
	b.vertexBase[vertex].conflictFace = B3_NULL_INDEX;
	b3QHList_Remove(b.links, vertex);
	b3QHList_PushBack(b.links, b.vertexList, vertex);

	b.horizonCount = 0;
	b3HullBuilder_BuildHorizon(b, vertex, face);
	if (B3_ASSERT_ENABLED) B3_ASSERT(b.horizonCount >= 3);

	b.coneCount = 0;
	b3HullBuilder_BuildCone(b, vertex);
	if (B3_ASSERT_ENABLED) B3_ASSERT(b.coneCount >= 3);

	b3HullBuilder_MergeFaces(b);
	b3HullBuilder_ResolveVertices(b);
	b3HullBuilder_ResolveFaces(b);
}

function b3HullBuilder_CleanHull(b: b3HullBuilder, origin: b3Vec3): void {
	let links = b.links;
	let edges = b.edgeBase;
	let faces = b.faceBase;
	let vertices = b.vertexBase;
	let faceListHead = b.faceList;
	let faceLinkBase = b.faceLinkBase;

	let faceCount = 0;
	let halfEdgeCount = 0;

	for (let faceNode = links[faceListHead].next; faceNode != faceListHead; faceNode = links[faceNode].next) {
		let face = faceNode - faceLinkBase;
		ref f = faces[face];
		let start = f.edge;
		let edge = start;

		do {
			vertices[edges[edge].origin].reachable = true;
			edge = edges[edge].next;
			halfEdgeCount++;
		} while (edge != start);

		f.plane.offset += b3Dot(f.plane.normal, origin);
		f.centroid = b3Add(f.centroid, origin);
		faceCount++;
	}

	let vertexCount = 0;
	let head = b.vertexList;
	let node = links[head].next;
	while (node != head) {
		let vertex = node;
		node = links[node].next;

		if (!vertices[vertex].reachable) {
			b3QHList_Remove(links, vertex);
		} else {
			vertices[vertex].position = b3Add(vertices[vertex].position, origin);
			vertexCount++;
		}
	}

	b.interiorPoint = b3Add(b.interiorPoint, origin);

	b.finalVertexCount = vertexCount;
	b.finalHalfEdgeCount = halfEdgeCount;
	b.finalFaceCount = faceCount;
}

// B3_DEBUG only
function b3HullBuilder_IsConsistent(b: b3HullBuilder): bool {
	let v = b.finalVertexCount;
	let e = b.finalHalfEdgeCount / 2;
	let f = b.finalFaceCount;

	if (v - e + f != 2) {
		return false;
	}

	let links = b.links;
	let edges = b.edgeBase;
	let faces = b.faceBase;
	let vertices = b.vertexBase;
	let faceListHead = b.faceList;

	for (let faceNode = links[faceListHead].next; faceNode != faceListHead; faceNode = links[faceNode].next) {
		let face = faceNode - b.faceLinkBase;
		let start = faces[face].edge;
		if (edges[start].face != face) {
			return false;
		}

		if (!b3CheckConsistency(b, face)) {
			return false;
		}

		if (b3PlaneSeparation(faces[face].plane, b.interiorPoint) > 0) {
			return false;
		}

		if (faces[face].mark != B3_MARK_VISIBLE) {
			return false;
		}

		let edge = start;

		do {
			let twin = edges[edge].twin;
			if (edges[edges[edge].next].origin != edges[twin].origin) {
				return false;
			}
			if (edges[edges[edge].prev].next != edge) {
				return false;
			}
			if (edges[edges[edge].next].prev != edge) {
				return false;
			}
			if (edges[twin].twin != edge) {
				return false;
			}
			if (edges[edge].face != face) {
				return false;
			}
			let p1 = vertices[edges[edge].origin].position;
			let p2 = vertices[edges[twin].origin].position;
			if (b3DistanceSquared(p1, p2) < 1000.0 * FLT_MIN) {
				return false;
			}

			edge = edges[edge].next;
		} while (edge != start);
	}

	return true;
}

function b3HullBuilder_HasHull(b: b3HullBuilder): bool {
	let v = b.finalVertexCount;
	let e = b.finalHalfEdgeCount / 2;
	let f = b.finalFaceCount;
	return v - e + f == 2 && f >= 4;
}

// Build the entire hull. Returns true iff the result satisfies Euler's identity.
function b3HullBuilder_Construct(
	b: b3HullBuilder,
	points: StaticArray<b3Vec3>,
	pointCount: i32,
	maxVertexCount: i32,
	origin: b3Vec3,
	shiftedPoints: StaticArray<b3Vec3>,
): bool {
	if (pointCount < 4) {
		return false;
	}

	for (let i = 0; i < pointCount; ++i) {
		shiftedPoints[i] = b3Sub(points[i], origin);
	}

	b3HullBuilder_ComputeTolerance(b, pointCount, shiftedPoints);
	let haveInitialHull = b3HullBuilder_BuildInitialHull(b, pointCount, shiftedPoints);
	if (haveInitialHull == false) {
		return false;
	}

	let budget = b3ClampInt(maxVertexCount - 4, 0, B3_HULL_MAX_COUNT - 4);

	let vertex = b3HullBuilder_NextConflictVertex(b);
	while (vertex != B3_NULL_INDEX && budget > 0) {
		b3HullBuilder_AddVertexToHull(b, vertex);
		vertex = b3HullBuilder_NextConflictVertex(b);
		budget -= 1;
	}

	b3HullBuilder_CleanHull(b, origin);

	if (B3_DEBUG != 0) {
		B3_ASSERT(b3HullBuilder_IsConsistent(b));
	}

	return b3HullBuilder_HasHull(b);
}

// The byte offsets of the C work block are dropped: each pool is its own array.
class b3HullWorkSizes {
	// Input point count
	N: i32 = 0;
	// Output point limit
	M: i32 = 0;
	vertexCapacity: i32 = 0;
	edgeCapacity: i32 = 0;
	faceCapacity: i32 = 0;
	horizonCapacity: i32 = 0;
	coneCapacity: i32 = 0;
	mergedFacesCapacity: i32 = 0;
	horizonStackCapacity: i32 = 0;
}

function b3ComputeHullWorkSizes(pointCount: i32, clampedMaxCount: i32): b3HullWorkSizes {
	let s = new b3HullWorkSizes();
	s.N = pointCount;
	s.M = clampedMaxCount;

	// Vertices: 4 initial hull vertices + at most one per remaining input point. No free list.
	s.vertexCapacity = pointCount + 4;

	// Edges and faces use free-list recycling; capacity is proportional to live hull size.
	// edgeCapacity: peak is ~twice live edges plus cone edges. Minimum 48.
	s.edgeCapacity = b3MaxInt(48, 24 * s.M - 48);

	// faceCapacity: peak intermediate state live faces (<=2*M-4) plus full cone (<=3*M-6). Minimum 16.
	s.faceCapacity = b3MaxInt(16, 5 * s.M - 10);

	// Horizon/cone bounded by current half-edge count. Merged faces by face count.
	s.horizonCapacity = b3MaxInt(6, 3 * s.M - 6);
	s.coneCapacity = s.horizonCapacity;
	s.mergedFacesCapacity = b3MaxInt(4, 2 * s.M - 4);

	// Horizon DFS depth is bounded by the number of live faces (Euler: <=2*M-4).
	s.horizonStackCapacity = b3MaxInt(4, 2 * s.M - 4);

	return s;
}

function b3UpdateHullEdgeCosines(hull: b3HullData): void {
	let edges = b3GetHullEdges(hull);
	let planes = b3GetHullPlanes(hull);
	let cosines = hull.edgeCosines;
	let count = hull.edgeCount / 2;

	for (let i = 0; i < count; ++i) {
		cosines[i] = b3Dot(planes[edges[2 * i].face].normal, planes[edges[2 * i + 1].face].normal);
	}
}

export function b3FindHullSupportVertex(hull: b3HullData, direction: b3Vec3): i32 {
	let bestIndex = B3_NULL_INDEX;
	let bestDot = -FLT_MAX;

	let vertexCount = hull.vertexCount;
	let soa = b3GetHullSoaVertices(hull);

	let soaVertexCount = b3GetHullSoaStride(vertexCount);
	let vy = soaVertexCount;
	let vz = vy + soaVertexCount;

	let dx = b3SplatW4(direction.x);
	let dy = b3SplatW4(direction.y);
	let dz = b3SplatW4(direction.z);
	let four = b3SplatW4(4.0);
	let index = b3SetW4(0.0, 1.0, 2.0, 3.0);
	let bestDotW = b3SplatW4(-FLT_MAX);
	let bestIndexW = b3SplatW4(-1.0);

	// Keep this at 4 wide to avoid regressing 4 wide platforms.
	let loopCount = (vertexCount + 3) & ~3;
	for (let i = 0; i < loopCount; i += 4) {
		let dot = b3AddW4(
			b3AddW4(b3MulW4(dx, b3LoadW4(soa, i)), b3MulW4(dy, b3LoadW4(soa, vy + i))),
			b3MulW4(dz, b3LoadW4(soa, vz + i)),
		);
		let mask = b3GreaterThanW4(dot, bestDotW);
		bestDotW = b3BlendW4(bestDotW, dot, mask);
		bestIndexW = b3BlendW4(bestIndexW, index, mask);
		index = b3AddW4(index, four);
	}

	let dots: FixedArray<f32, 4>;
	let indices: FixedArray<f32, 4>;
	dots[0] = bestDotW.x;
	dots[1] = bestDotW.y;
	dots[2] = bestDotW.z;
	dots[3] = bestDotW.w;
	indices[0] = bestIndexW.x;
	indices[1] = bestIndexW.y;
	indices[2] = bestIndexW.z;
	indices[3] = bestIndexW.w;

	for (let lane = 0; lane < 4; ++lane) {
		let laneIndex = <i32>indices[lane];
		if (laneIndex >= 0 && (dots[lane] > bestDot || (dots[lane] == bestDot && laneIndex < bestIndex))) {
			bestIndex = laneIndex;
			bestDot = dots[lane];
		}
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= bestIndex && bestIndex < vertexCount);
	return bestIndex;
}

export function b3FindHullSupportFace(hull: b3HullData, direction: b3Vec3): i32 {
	let bestIndex = B3_NULL_INDEX;
	let bestDot = -FLT_MAX;

	let faceCount = hull.faceCount;
	let planes = b3GetHullPlanes(hull);

	for (let index = 0; index < faceCount; ++index) {
		let dot = b3Dot(planes[index].normal, direction);
		if (dot > bestDot) {
			bestDot = dot;
			bestIndex = index;
		}
	}
	if (B3_ASSERT_ENABLED) B3_ASSERT(bestIndex >= 0);

	return bestIndex;
}

/// The B3_ENABLE_VALIDATION body of b3IsValidHull, callable in any build.
export function b3ValidateHull(hull: b3HullData): bool {
	if (hull.version != B3_HULL_VERSION) {
		return false;
	}

	let v = hull.vertexCount;
	let e = hull.edgeCount / 2;
	let f = hull.faceCount;

	if (v - e + f != 2) {
		return false;
	}

	let vertices = b3GetHullVertices(hull);
	let edges = b3GetHullEdges(hull);
	for (let index = 0; index < hull.vertexCount; ++index) {
		let edge = <i32>vertices[index].edge;

		if (<i32>edges[edge].origin != index) {
			return false;
		}
	}

	for (let index = 0; index < hull.edgeCount; index += 2) {
		if (<i32>edges[index + 0].twin != index + 1) {
			return false;
		}

		if (<i32>edges[index + 1].twin != index + 0) {
			return false;
		}
	}

	let faces = b3GetHullFaces(hull);
	let planes = b3GetHullPlanes(hull);
	for (let faceIndex = 0; faceIndex < hull.faceCount; ++faceIndex) {
		let baseEdgeIndex = <i32>faces[faceIndex].edge;

		let plane = planes[faceIndex];
		if (b3PlaneSeparation(plane, hull.center) >= 0.0) {
			return false;
		}

		let edgeIndex = baseEdgeIndex;
		do {
			let next = <i32>edges[edgeIndex].next;
			let twin = <i32>edges[edgeIndex].twin;

			if (<i32>edges[edgeIndex].face != faceIndex) {
				return false;
			}

			if (<i32>edges[twin].twin != edgeIndex) {
				return false;
			}

			if (edges[next].origin != edges[twin].origin) {
				return false;
			}

			edgeIndex = next;
		} while (edgeIndex != baseEdgeIndex);
	}

	if (hull.volume <= 0.0) {
		return false;
	}

	if (hull.surfaceArea <= 0.0) {
		return false;
	}

	if (hull.innerRadius <= 0.0) {
		return false;
	}

	return true;
}

export function b3IsValidHull(hull: b3HullData): bool {
	if (B3_ENABLE_VALIDATION) {
		return b3ValidateHull(hull);
	}
	return true;
}

/// Create a tessellated cylinder as a hull.
export function b3CreateCylinder(height: f32, radius: f32, yOffset: f32, sides: i32): b3HullData? {
	if (B3_ASSERT_ENABLED) B3_ASSERT(height > 0.0);
	if (B3_ASSERT_ENABLED) B3_ASSERT(radius > 0.0);
	if (B3_ASSERT_ENABLED) B3_ASSERT(3 <= sides && sides <= 32);

	let pointCount = 2 * sides;
	let points = new StaticArray<b3Vec3>(pointCount);

	let alpha: f32 = 0.0;
	let deltaAlpha: f32 = (2.0 * B3_PI) / <f32>sides;

	for (let index = 0; index < sides; ++index) {
		let sinAlpha = b3Sin(alpha);
		let cosAlpha = b3Cos(alpha);

		points[2 * index + 0] = b3Vec3(radius * cosAlpha, yOffset, radius * sinAlpha);
		points[2 * index + 1] = b3Vec3(radius * cosAlpha, yOffset + height, radius * sinAlpha);

		alpha += deltaAlpha;
	}

	let hull = b3CreateHull(points, pointCount, pointCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull!.vertexCount == pointCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull!.edgeCount == 6 * sides);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull!.faceCount == sides + 2);

	return hull;
}

/// Create a tessellated cone as a hull.
export function b3CreateCone(height: f32, radius1: f32, radius2: f32, slices: i32): b3HullData? {
	if (B3_ASSERT_ENABLED) B3_ASSERT(height > 0.0);
	if (B3_ASSERT_ENABLED) B3_ASSERT(radius1 > 0.0);
	if (B3_ASSERT_ENABLED) B3_ASSERT(radius2 > 0.0);
	if (B3_ASSERT_ENABLED) B3_ASSERT(4 <= slices && slices <= 32);

	let pointCount = 2 * slices;
	let points = new StaticArray<b3Vec3>(pointCount);

	let alpha: f32 = 0.0;
	let deltaAlpha: f32 = (2.0 * B3_PI) / <f32>slices;

	for (let index = 0; index < slices; ++index) {
		let sinAlpha = b3Sin(alpha);
		let cosAlpha = b3Cos(alpha);

		points[2 * index + 0] = b3Vec3(radius1 * cosAlpha, 0.0, radius1 * sinAlpha);
		points[2 * index + 1] = b3Vec3(radius2 * cosAlpha, height, radius2 * sinAlpha);

		alpha += deltaAlpha;
	}

	let hull = b3CreateHull(points, pointCount, pointCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull!.vertexCount == pointCount);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull!.edgeCount == 6 * slices);
	if (B3_ASSERT_ENABLED) B3_ASSERT(hull!.faceCount == slices + 2);

	return hull;
}

/// Create a rock shaped hull.
export function b3CreateRock(radius: f32): b3HullData? {
	let pointCount = 10;

	// Golden ratio
	const phi: f32 = (<f32>1.0 + Mathf.sqrt(<f32>5.0)) / <f32>2.0;

	// Fibonacci lattice
	let points = new StaticArray<b3Vec3>(10);

	// Azimuthal angle
	let theta: f32 = (2.0 * B3_PI) / phi;

	let cs = b3CosSin(1.0, 0.0);
	let deltaCS = b3ComputeCosSin(theta);

	for (let i = 0; i < pointCount; ++i) {
		// Z coordinate
		let z: f32 = <f32>1.0 - (<f32>2.0 * <f32>i + <f32>1.0) / <f32>pointCount;
		// Radius in xy-plane
		let radius_XY = Mathf.sqrt(<f32>1.0 - z * z);

		points[i].x = radius * radius_XY * cs.cosine;
		points[i].y = radius * radius_XY * cs.sine;
		points[i].z = radius * z;

		let cs0 = cs;
		cs.cosine = deltaCS.cosine * cs0.cosine - deltaCS.sine * cs0.sine;
		cs.sine = deltaCS.sine * cs0.cosine + deltaCS.cosine * cs0.sine;
	}

	return b3CreateHull(points, pointCount, pointCount);
}

// PEEL's BasicRandom. This is just for testing. C advances the state through a pointer.
class b3ComplexHullRandom {
	state: u32 = 0;
}

function b3NextComplexHullRandom(state: b3ComplexHullRandom): u32 {
	state.state = state.state * 2147001325 + 715136305;
	return state.state;
}

function b3ComplexHullRandomFloat(state: b3ComplexHullRandom): f32 {
	return <f32>(b3NextComplexHullRandom(state) & 0xffff) / <f32>65535.0 - <f32>0.5;
}

function b3ComplexHullRandomDirection(state: b3ComplexHullRandom): b3Vec3 {
	let point = b3Vec3();
	let lengthSquared: f32;
	do {
		point.x = b3ComplexHullRandomFloat(state);
		point.y = b3ComplexHullRandomFloat(state);
		point.z = b3ComplexHullRandomFloat(state);
		lengthSquared = b3Dot(point, point);
	} while (lengthSquared > 0.25);

	return b3Normalize(point);
}

/// Hull of 32 pseudo random points on a sphere. From the convex pile hull in PEEL.
export function b3CreateComplexHull(radius: f32): b3HullData? {
	const pointCount = 32;

	let points = new StaticArray<b3Vec3>(pointCount);
	let state = new b3ComplexHullRandom();
	state.state = 42;
	for (let i = 0; i < pointCount; ++i) {
		points[i] = b3MulSV(radius, b3ComplexHullRandomDirection(state));
	}

	return b3CreateHull(points, pointCount, pointCount);
}

function b3UpdateHullBounds(hull: b3HullData): void {
	let points = b3GetHullPoints(hull);
	let vertexCount = hull.vertexCount;

	if (B3_ASSERT_ENABLED) B3_ASSERT(vertexCount > 0);
	let bounds = b3AABB();
	bounds.lowerBound = points[0];
	bounds.upperBound = points[0];

	for (let i = 1; i < vertexCount; ++i) {
		let p = points[i];
		bounds.lowerBound = b3Min(bounds.lowerBound, p);
		bounds.upperBound = b3Max(bounds.upperBound, p);
	}

	hull.aabb = bounds;
}

// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
function b3UpdateHullBulkProperties(hull: b3HullData): bool {
	let points = b3GetHullPoints(hull);
	let faces = b3GetHullFaces(hull);
	let edges = b3GetHullEdges(hull);
	let planes = b3GetHullPlanes(hull);

	let area: f32 = 0.0;
	let volume: f32 = 0.0;
	let center = b3Vec3_zero;

	// Use the first vertex to reduce round-off errors.
	let origin = points[0];

	let xx: f32 = 0.0;
	let xy: f32 = 0.0;
	let yy: f32 = 0.0;
	let xz: f32 = 0.0;
	let zz: f32 = 0.0;
	let yz: f32 = 0.0;

	let faceCount = hull.faceCount;

	for (let faceIndex = 0; faceIndex < faceCount; ++faceIndex) {
		let edge1 = <i32>faces[faceIndex].edge;
		let edge2 = <i32>edges[edge1].next;
		let edge3 = <i32>edges[edge2].next;

		if (B3_ASSERT_ENABLED) B3_ASSERT(edge1 != edge3);
		if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[edge1].origin < hull.vertexCount);

		let v1 = b3Sub(points[edges[edge1].origin], origin);

		do {
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[edge2].origin < hull.vertexCount);
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[edge3].origin < hull.vertexCount);

			let v2 = b3Sub(points[edges[edge2].origin], origin);
			let v3 = b3Sub(points[edges[edge3].origin], origin);

			area += b3Length(b3Cross(b3Sub(v2, v1), b3Sub(v3, v1)));

			let det = b3ScalarTripleProduct(v1, v2, v3);

			volume += det;

			let v4 = b3Add(v1, b3Add(v2, v3));
			center = b3Add(center, b3MulSV(det, v4));

			xx += det * (v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x);
			yy += det * (v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y);
			zz += det * (v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z);
			xy += det * (v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y);
			xz += det * (v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z);
			yz += det * (v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z);

			edge2 = edge3;
			edge3 = <i32>edges[edge3].next;
		} while (edge1 != edge3);
	}

	let localCenter = volume > 0.0 ? b3MulSV(<f32>0.25 / volume, center) : b3Vec3_zero;
	center = b3Add(localCenter, origin);

	let radius = FLT_MAX;
	for (let faceIndex = 0; faceIndex < faceCount; ++faceIndex) {
		let plane = planes[faceIndex];
		let distance = b3PlaneSeparation(plane, center);

		radius = b3MinFloat(radius, -distance);
	}

	let inertia = b3Matrix3();
	inertia.cx.x = yy + zz;
	inertia.cy.x = -xy;
	inertia.cz.x = -xz;
	inertia.cx.y = -xy;
	inertia.cy.y = xx + zz;
	inertia.cz.y = -yz;
	inertia.cx.z = -xz;
	inertia.cy.z = -yz;
	inertia.cz.z = xx + yy;

	let mass: f32 = volume / 6.0;

	let centralInertia = b3MulSM(<f32>1.0 / <f32>120.0, inertia);
	centralInertia = b3SubMM(centralInertia, b3Steiner(mass, localCenter));

	hull.center = center;
	hull.centralInertia = centralInertia;
	hull.volume = mass;
	hull.surfaceArea = 0.5 * area;
	hull.innerRadius = radius;

	if (mass <= 0.0) {
		return false;
	}

	if (volume <= 0.0) {
		return false;
	}

	if (area <= 0.0) {
		return false;
	}

	if (radius <= 0.0) {
		return false;
	}

	return true;
}

/// Create a generic convex hull. This can fail if B3_MAX_HULL_VERTICES, B3_MAX_HULL_FACES,
/// or B3_MAX_HULL_EDGES is exceeded.
/// Only the first pointCount entries of points are read.
export function b3CreateHull(points: StaticArray<b3Vec3>, pointCount: i32, maxVertexCount: i32): b3HullData? {
	if (pointCount < 4) {
		return null;
	}

	let origin = points[0];
	let clampedMaxCount = b3ClampInt(maxVertexCount, 4, B3_MAX_HULL_VERTICES);

	let sizes = b3ComputeHullWorkSizes(pointCount, clampedMaxCount);
	let builder = new b3HullBuilder(sizes);

	let shiftedPoints = new StaticArray<b3Vec3>(pointCount);

	let ok = b3HullBuilder_Construct(builder, points, pointCount, clampedMaxCount, origin, shiftedPoints);
	if (ok == false) {
		return null;
	}

	if (builder.finalVertexCount > B3_MAX_HULL_VERTICES) {
		b3Log(
			"hull final vertex count of " +
				builder.finalVertexCount.toString() +
				" exceeds limit of " +
				B3_MAX_HULL_VERTICES.toString(),
		);
		return null;
	}

	if (builder.finalFaceCount > B3_MAX_HULL_FACES) {
		b3Log(
			"hull final face count of " +
				builder.finalFaceCount.toString() +
				" exceeds limit of " +
				B3_MAX_HULL_FACES.toString(),
		);
		return null;
	}

	let maxHalfEdgeCount = 2 * B3_MAX_HULL_EDGES;
	if (builder.finalHalfEdgeCount > maxHalfEdgeCount) {
		b3Log(
			"hull final half edge count of " +
				builder.finalHalfEdgeCount.toString() +
				" exceeds limit of " +
				maxHalfEdgeCount.toString(),
		);
		return null;
	}

	let links = builder.links;
	let qhVertices = builder.vertexBase;
	let qhEdges = builder.edgeBase;
	let qhFaces = builder.faceBase;

	// Walk lists into temp arrays bounded by B3_HULL_MAX_COUNT, stamping finalIndex on each node so
	// the resolution pass below is O(E + F) instead of O(E^2 + F^2).
	let tempVertices = new StaticArray<i32>(B3_HULL_MAX_COUNT);
	let vertexCount = 0;
	let vertexListHead = builder.vertexList;
	for (let node = links[vertexListHead].next; node != vertexListHead; node = links[node].next) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(vertexCount < B3_HULL_MAX_COUNT);

		qhVertices[node].finalIndex = vertexCount;
		tempVertices[vertexCount++] = node;
	}

	// Collect edges in twin-paired order (i, i+1) by stamping each pair as we discover it.
	let tempFaces = new StaticArray<i32>(B3_HULL_MAX_COUNT);
	let tempEdges = new StaticArray<i32>(B3_HULL_MAX_COUNT);
	let faceCount = 0;
	let edgeCount = 0;

	let faceListHead = builder.faceList;
	for (let faceNode = links[faceListHead].next; faceNode != faceListHead; faceNode = links[faceNode].next) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(faceCount < B3_HULL_MAX_COUNT);

		let face = faceNode - builder.faceLinkBase;
		qhFaces[face].finalIndex = faceCount;
		tempFaces[faceCount++] = face;

		let start = qhFaces[face].edge;
		let edge = start;
		do {
			if (qhEdges[edge].finalIndex < 0) {
				if (B3_ASSERT_ENABLED) B3_ASSERT(edgeCount + 1 < B3_HULL_MAX_COUNT);

				qhEdges[edge].finalIndex = edgeCount;
				tempEdges[edgeCount++] = edge;
				let twin = qhEdges[edge].twin;
				qhEdges[twin].finalIndex = edgeCount;
				tempEdges[edgeCount++] = twin;
			}
			edge = qhEdges[edge].next;
		} while (edge != start);
	}

	let soaVertexCount = b3GetHullSoaStride(vertexCount);
	let soaNormalCount = b3GetHullSoaStride(faceCount);

	// Allocate the hull. The constructor sets the offsets and byteCount as C lays out the blob.
	let hull = new b3HullData(vertexCount, edgeCount, faceCount);
	hull.version = B3_HULL_VERSION;

	let vertices = hull.vertices;
	let edges = hull.edges;
	let faces = hull.faces;
	let finalPoints = hull.points;
	let planes = hull.planes;
	let soaVertices = hull.soaVertices;
	let soaNormals = hull.soaNormals;

	let vy = soaVertexCount;
	let vz = vy + soaVertexCount;

	for (let index = 0; index < vertexCount; ++index) {
		vertices[index].edge = 0;
		let p = qhVertices[tempVertices[index]].position;
		finalPoints[index] = p;

		soaVertices[index] = p.x;
		soaVertices[vy + index] = p.y;
		soaVertices[vz + index] = p.z;
	}

	for (let index = vertexCount; index < soaVertexCount; ++index) {
		soaVertices[index] = soaVertices[0];
		soaVertices[vy + index] = soaVertices[vy];
		soaVertices[vz + index] = soaVertices[vz];
	}

	for (let index = 0; index < edgeCount; ++index) {
		ref edge = qhEdges[tempEdges[index]];
		let nextIndex = qhEdges[edge.next].finalIndex;
		let twinIndex = qhEdges[edge.twin].finalIndex;
		let faceIndex = qhFaces[edge.face].finalIndex;
		let originIndex = qhVertices[edge.origin].finalIndex;
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= nextIndex && nextIndex <= UINT8_MAX);
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= twinIndex && twinIndex <= UINT8_MAX);
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= faceIndex && faceIndex <= UINT8_MAX);
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= originIndex && originIndex <= UINT8_MAX);

		edges[index].next = <u8>nextIndex;
		edges[index].twin = <u8>twinIndex;
		edges[index].face = <u8>faceIndex;
		edges[index].origin = <u8>originIndex;

		vertices[originIndex].edge = <u8>index;
	}

	let ny = soaNormalCount;
	let nz = ny + soaNormalCount;

	for (let index = 0; index < faceCount; ++index) {
		ref face = qhFaces[tempFaces[index]];
		let faceEdgeIndex = qhEdges[face.edge].finalIndex;
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= faceEdgeIndex && faceEdgeIndex <= UINT8_MAX);

		faces[index].edge = <u8>faceEdgeIndex;
		planes[index] = face.plane;

		let n = face.plane.normal;
		soaNormals[index] = n.x;
		soaNormals[ny + index] = n.y;
		soaNormals[nz + index] = n.z;
	}

	for (let index = faceCount; index < soaNormalCount; ++index) {
		soaNormals[index] = 0.0;
		soaNormals[ny + index] = 0.0;
		soaNormals[nz + index] = 0.0;
	}

	b3UpdateHullEdgeCosines(hull);

	b3UpdateHullBounds(hull);
	let success = b3UpdateHullBulkProperties(hull);
	if (success == false) {
		return null;
	}

	if (b3IsValidHull(hull) == false) {
		return null;
	}

	hull.hash = 0;
	hull.hash = b3Hash64NonZero(b3SerializeHullData(hull), hull.byteCount);

	return hull;
}

// C memcpy's the blob. Every section is copied, so the clone shares no array with hull.
function b3CopyHullData(hull: b3HullData): b3HullData {
	let clone = new b3HullData(hull.vertexCount, hull.edgeCount, hull.faceCount);
	clone.version = hull.version;
	clone.hash = hull.hash;
	clone.aabb = hull.aabb;
	clone.surfaceArea = hull.surfaceArea;
	clone.volume = hull.volume;
	clone.innerRadius = hull.innerRadius;
	clone.center = hull.center;
	clone.centralInertia = hull.centralInertia;
	clone.vertexOffset = hull.vertexOffset;
	clone.pointOffset = hull.pointOffset;
	clone.edgeOffset = hull.edgeOffset;
	clone.planeOffset = hull.planeOffset;
	clone.faceOffset = hull.faceOffset;
	clone.soaVertexOffset = hull.soaVertexOffset;
	clone.soaNormalOffset = hull.soaNormalOffset;
	clone.edgeCosineOffset = hull.edgeCosineOffset;
	clone.byteCount = hull.byteCount;
	clone.padding = hull.padding;

	for (let i = 0; i < hull.vertices.length; ++i) clone.vertices[i] = hull.vertices[i];
	for (let i = 0; i < hull.points.length; ++i) clone.points[i] = hull.points[i];
	for (let i = 0; i < hull.edges.length; ++i) clone.edges[i] = hull.edges[i];
	for (let i = 0; i < hull.planes.length; ++i) clone.planes[i] = hull.planes[i];
	for (let i = 0; i < hull.faces.length; ++i) clone.faces[i] = hull.faces[i];
	for (let i = 0; i < hull.soaVertices.length; ++i) clone.soaVertices[i] = hull.soaVertices[i];
	for (let i = 0; i < hull.soaNormals.length; ++i) clone.soaNormals[i] = hull.soaNormals[i];
	for (let i = 0; i < hull.edgeCosines.length; ++i) clone.edgeCosines[i] = hull.edgeCosines[i];
	return clone;
}

/// Deep clone a hull.
export function b3CloneHull(hull: b3HullData?): b3HullData? {
	if (hull == null || b3IsValidHull(hull) == false) {
		return null;
	}

	return b3CopyHullData(hull);
}

export function b3HashHullData(hull: b3HullData): u64 {
	return hull.hash;
}

/// C memcmp's the blobs, so this compares the serialized bytes.
export function b3CompareHullData(hull1: b3HullData, hull2: b3HullData): bool {
	if (hull1 == hull2) {
		return true;
	}

	if (hull1.byteCount != hull2.byteCount) {
		return false;
	}

	let bytes1 = b3SerializeHullData(hull1);
	let bytes2 = b3SerializeHullData(hull2);
	let n = hull1.byteCount;
	for (let i = 0; i < n; ++i) {
		if (bytes1[i] != bytes2[i]) {
			return false;
		}
	}
	return true;
}

@inline
function b3WriteU32(bytes: Uint8Array, offset: i32, value: u32): void {
	bytes[offset] = <u8>value;
	bytes[offset + 1] = <u8>(value >> 8);
	bytes[offset + 2] = <u8>(value >> 16);
	bytes[offset + 3] = <u8>(value >> 24);
}

@inline
function b3WriteF32(bytes: Uint8Array, offset: i32, value: f32): void {
	b3WriteU32(bytes, offset, reinterpret<u32>(value));
}

@inline
function b3WriteVec3(bytes: Uint8Array, offset: i32, v: b3Vec3): void {
	b3WriteF32(bytes, offset, v.x);
	b3WriteF32(bytes, offset + 4, v.y);
	b3WriteF32(bytes, offset + 8, v.z);
}

/// The C blob of a hull, byte for byte: the b3HullData header, then each section at its offset,
/// gaps zero. A b3BoxHull base serializes to the b3BoxHull struct. C hashes and compares these bytes.
export function b3SerializeHullData(hull: b3HullData): Uint8Array {
	let bytes = new Uint8Array(hull.byteCount);

	b3WriteU32(bytes, 0, <u32>hull.version);
	b3WriteU32(bytes, 4, <u32>(hull.version >> 32));
	b3WriteU32(bytes, 8, <u32>hull.hash);
	b3WriteU32(bytes, 12, <u32>(hull.hash >> 32));
	b3WriteVec3(bytes, 16, hull.aabb.lowerBound);
	b3WriteVec3(bytes, 28, hull.aabb.upperBound);
	b3WriteF32(bytes, 40, hull.surfaceArea);
	b3WriteF32(bytes, 44, hull.volume);
	b3WriteF32(bytes, 48, hull.innerRadius);
	b3WriteVec3(bytes, 52, hull.center);
	b3WriteVec3(bytes, 64, hull.centralInertia.cx);
	b3WriteVec3(bytes, 76, hull.centralInertia.cy);
	b3WriteVec3(bytes, 88, hull.centralInertia.cz);
	b3WriteU32(bytes, 100, <u32>hull.vertexCount);
	b3WriteU32(bytes, 104, <u32>hull.vertexOffset);
	b3WriteU32(bytes, 108, <u32>hull.pointOffset);
	b3WriteU32(bytes, 112, <u32>hull.edgeCount);
	b3WriteU32(bytes, 116, <u32>hull.edgeOffset);
	b3WriteU32(bytes, 120, <u32>hull.faceCount);
	b3WriteU32(bytes, 124, <u32>hull.planeOffset);
	b3WriteU32(bytes, 128, <u32>hull.faceOffset);
	b3WriteU32(bytes, 132, <u32>hull.soaVertexOffset);
	b3WriteU32(bytes, 136, <u32>hull.soaNormalOffset);
	b3WriteU32(bytes, 140, <u32>hull.edgeCosineOffset);
	b3WriteU32(bytes, 144, <u32>hull.byteCount);
	b3WriteU32(bytes, 148, <u32>hull.padding);

	let vertexCount = hull.vertexCount;
	let edgeCount = hull.edgeCount;
	let faceCount = hull.faceCount;

	let offset = hull.vertexOffset;
	for (let i = 0; i < vertexCount; ++i) {
		bytes[offset + i] = hull.vertices[i].edge;
	}

	offset = hull.pointOffset;
	for (let i = 0; i < vertexCount; ++i) {
		b3WriteVec3(bytes, offset + 12 * i, hull.points[i]);
	}

	offset = hull.edgeOffset;
	for (let i = 0; i < edgeCount; ++i) {
		let edge = hull.edges[i];
		bytes[offset + 4 * i] = edge.next;
		bytes[offset + 4 * i + 1] = edge.twin;
		bytes[offset + 4 * i + 2] = edge.origin;
		bytes[offset + 4 * i + 3] = edge.face;
	}

	offset = hull.planeOffset;
	for (let i = 0; i < faceCount; ++i) {
		let plane = hull.planes[i];
		b3WriteVec3(bytes, offset + 16 * i, plane.normal);
		b3WriteF32(bytes, offset + 16 * i + 12, plane.offset);
	}

	offset = hull.faceOffset;
	for (let i = 0; i < faceCount; ++i) {
		bytes[offset + i] = hull.faces[i].edge;
	}

	let soaVertexCount = 3 * b3GetHullSoaStride(vertexCount);
	offset = hull.soaVertexOffset;
	for (let i = 0; i < soaVertexCount; ++i) {
		b3WriteF32(bytes, offset + 4 * i, hull.soaVertices[i]);
	}

	let soaNormalCount = 3 * b3GetHullSoaStride(faceCount);
	offset = hull.soaNormalOffset;
	for (let i = 0; i < soaNormalCount; ++i) {
		b3WriteF32(bytes, offset + 4 * i, hull.soaNormals[i]);
	}

	let cosineCount = edgeCount / 2;
	offset = hull.edgeCosineOffset;
	for (let i = 0; i < cosineCount; ++i) {
		b3WriteF32(bytes, offset + 4 * i, hull.edgeCosines[i]);
	}

	return bytes;
}

// b3HullMap: C instantiates verstable keyed by hull content. Same contract, lookup by hash then
// content, but iteration runs in slot order, not verstable's bucket order.

/// Iterator: a slot index, B3_NULL_INDEX at the end.
export type b3HullMap_itr = i32;

export class b3HullMap {
	keys: Array<b3HullData?> = new Array<b3HullData?>();
	vals: Array<i32> = new Array<i32>();
	freeSlots: Array<i32> = new Array<i32>();
	buckets: Map<u64, Array<i32>> = new Map<u64, Array<i32>>();
	count: i32 = 0;
}

export function b3HullMap_init(map: b3HullMap): void {
	map.keys = new Array<b3HullData?>();
	map.vals = new Array<i32>();
	map.freeSlots = new Array<i32>();
	map.buckets = new Map<u64, Array<i32>>();
	map.count = 0;
}

export function b3HullMap_reserve(map: b3HullMap, count: i32): bool {
	return true;
}

export function b3HullMap_cleanup(map: b3HullMap): void {
	b3HullMap_init(map);
}

export function b3HullMap_size(map: b3HullMap): i32 {
	return map.count;
}

/// Slot count, not verstable's bucket count.
export function b3HullMap_bucket_count(map: b3HullMap): i32 {
	return map.keys.length;
}

@inline
export function b3HullMap_is_end(itr: b3HullMap_itr): bool {
	return itr == B3_NULL_INDEX;
}

export function b3HullMap_get(map: b3HullMap, key: b3HullData): b3HullMap_itr {
	let hash = b3HashHullData(key);
	if (!map.buckets.has(hash)) {
		return B3_NULL_INDEX;
	}

	let bucket = map.buckets.get(hash);
	for (let i = 0; i < bucket.length; ++i) {
		let slot = bucket[i];
		if (b3CompareHullData(map.keys[slot]!, key)) {
			return slot;
		}
	}
	return B3_NULL_INDEX;
}

function b3HullMap_add(map: b3HullMap, key: b3HullData, val: i32): b3HullMap_itr {
	let slot: i32;
	if (map.freeSlots.length > 0) {
		slot = map.freeSlots.pop();
		map.keys[slot] = key;
		map.vals[slot] = val;
	} else {
		slot = map.keys.length;
		map.keys.push(key);
		map.vals.push(val);
	}

	let hash = b3HashHullData(key);
	if (!map.buckets.has(hash)) {
		map.buckets.set(hash, new Array<i32>());
	}
	map.buckets.get(hash).push(slot);
	map.count += 1;
	return slot;
}

/// Inserts the key, or replaces the key and value of a matching entry, as verstable does.
export function b3HullMap_insert(map: b3HullMap, key: b3HullData, val: i32): b3HullMap_itr {
	let itr = b3HullMap_get(map, key);
	if (itr != B3_NULL_INDEX) {
		map.keys[itr] = key;
		map.vals[itr] = val;
		return itr;
	}

	return b3HullMap_add(map, key, val);
}

/// Returns the matching entry, or inserts the key with val.
export function b3HullMap_get_or_insert(map: b3HullMap, key: b3HullData, val: i32): b3HullMap_itr {
	let itr = b3HullMap_get(map, key);
	if (itr != B3_NULL_INDEX) {
		return itr;
	}

	return b3HullMap_add(map, key, val);
}

export function b3HullMap_key(map: b3HullMap, itr: b3HullMap_itr): b3HullData {
	return map.keys[itr]!;
}

export function b3HullMap_val(map: b3HullMap, itr: b3HullMap_itr): i32 {
	return map.vals[itr];
}

/// C writes itr.data->val.
export function b3HullMap_setVal(map: b3HullMap, itr: b3HullMap_itr, val: i32): void {
	map.vals[itr] = val;
}

export function b3HullMap_first(map: b3HullMap): b3HullMap_itr {
	return b3HullMap_next(map, B3_NULL_INDEX);
}

/// C's b3HullMap_next takes only the iterator.
export function b3HullMap_next(map: b3HullMap, itr: b3HullMap_itr): b3HullMap_itr {
	for (let slot = itr + 1; slot < map.keys.length; ++slot) {
		if (map.keys[slot] != null) {
			return slot;
		}
	}
	return B3_NULL_INDEX;
}

/// Returns the next entry, as verstable does.
export function b3HullMap_erase_itr(map: b3HullMap, itr: b3HullMap_itr): b3HullMap_itr {
	let key = map.keys[itr]!;
	let bucket = map.buckets.get(b3HashHullData(key));
	let index = bucket.indexOf(itr);
	bucket[index] = bucket[bucket.length - 1];
	bucket.pop();
	if (bucket.length == 0) {
		map.buckets.delete(b3HashHullData(key));
	}

	map.keys[itr] = null;
	map.vals[itr] = 0;
	map.freeSlots.push(itr);
	map.count -= 1;
	return b3HullMap_next(map, itr);
}

export function b3HullMap_erase(map: b3HullMap, key: b3HullData): bool {
	let itr = b3HullMap_get(map, key);
	if (itr == B3_NULL_INDEX) {
		return false;
	}
	b3HullMap_erase_itr(map, itr);
	return true;
}

/// Total map allocation in bytes, excluding the stored hull data. Not tracked: the runtime owns memory.
export function b3HullMapByteCount(map: b3HullMap): i32 {
	return 0;
}

/// Clone and transform a hull. Supports non-uniform and mirroring scale.
export function b3CloneAndTransformHull(original: b3HullData?, transform: b3Transform, scale: b3Vec3): b3HullData? {
	if (original == null || b3IsValidHull(original) == false) {
		return null;
	}

	let hull = b3CopyHullData(original);

	let safeScale = b3SafeScale(scale);

	let edges = hull.edges;
	let faces = hull.faces;
	let faceCount = hull.faceCount;
	let vertexCount = hull.vertexCount;

	if (safeScale.x * safeScale.y * safeScale.z < 0.0) {
		// Reflected: reverse edge winding for each face.
		for (let i = 0; i < faceCount; ++i) {
			let startEdgeIndex = <i32>faces[i].edge;
			let currentEdgeIndex = startEdgeIndex;
			let prevEdgeIndex = B3_NULL_INDEX;

			do {
				if (<i32>edges[currentEdgeIndex].next == startEdgeIndex) {
					prevEdgeIndex = currentEdgeIndex;
					break;
				}

				currentEdgeIndex = <i32>edges[currentEdgeIndex].next;
			} while (currentEdgeIndex != startEdgeIndex);

			if (B3_ASSERT_ENABLED) B3_ASSERT(prevEdgeIndex != B3_NULL_INDEX);

			currentEdgeIndex = startEdgeIndex;

			do {
				let nextIndex = edges[currentEdgeIndex].next;
				edges[currentEdgeIndex].next = <u8>prevEdgeIndex;

				let twinIndex = <i32>edges[currentEdgeIndex].twin;
				if (currentEdgeIndex < twinIndex) {
					let t = edges[currentEdgeIndex].origin;
					edges[currentEdgeIndex].origin = edges[twinIndex].origin;
					edges[twinIndex].origin = t;
				}

				prevEdgeIndex = currentEdgeIndex;
				currentEdgeIndex = <i32>nextIndex;
			} while (currentEdgeIndex != startEdgeIndex);
		}

		let vertices = hull.vertices;

		for (let i = 0; i < vertexCount; ++i) {
			vertices[i].edge = edges[vertices[i].edge].twin;
		}
	}

	let matrix = b3MakeMatrixFromQuat(transform.q);
	let points = hull.points;

	let soaVertexCount = b3GetHullSoaStride(vertexCount);
	let soa = hull.soaVertices;
	let vy = soaVertexCount;
	let vz = vy + soaVertexCount;

	for (let i = 0; i < vertexCount; ++i) {
		let p = b3Add(b3MulMV(matrix, b3Mul(safeScale, points[i])), transform.p);
		points[i] = p;
		soa[i] = p.x;
		soa[vy + i] = p.y;
		soa[vz + i] = p.z;
	}

	for (let i = vertexCount; i < soaVertexCount; ++i) {
		soa[i] = soa[0];
		soa[vy + i] = soa[vy];
		soa[vz + i] = soa[vz];
	}

	let planes = hull.planes;
	let soaNormalCount = b3GetHullSoaStride(faceCount);
	let nsoa = hull.soaNormals;
	let ny = soaNormalCount;
	let nz = ny + soaNormalCount;

	for (let i = 0; i < faceCount; ++i) {
		let count = 0;
		let centroid = b3Vec3_zero;
		let normal = b3Vec3_zero;

		let startEdgeIndex = <i32>faces[i].edge;
		let currentEdgeIndex = startEdgeIndex;

		if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[currentEdgeIndex].face == i);
		if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[currentEdgeIndex].origin < vertexCount);

		let origin = points[edges[currentEdgeIndex].origin];

		do {
			let twinIndex = <i32>edges[currentEdgeIndex].twin;
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[twinIndex].twin == currentEdgeIndex);

			let v1 = b3Sub(points[edges[currentEdgeIndex].origin], origin);
			let v2 = b3Sub(points[edges[twinIndex].origin], origin);

			count++;
			centroid = b3Add(centroid, v1);
			normal.x += (v1.y - v2.y) * (v1.z + v2.z);
			normal.y += (v1.z - v2.z) * (v1.x + v2.x);
			normal.z += (v1.x - v2.x) * (v1.y + v2.y);

			currentEdgeIndex = <i32>edges[currentEdgeIndex].next;
		} while (currentEdgeIndex != startEdgeIndex);

		if (B3_ASSERT_ENABLED) B3_ASSERT(count > 0);
		centroid = b3MulSV(<f32>1.0 / <f32>count, centroid);
		centroid = b3Add(centroid, origin);

		let area = b3Length(normal);
		if (B3_ASSERT_ENABLED) B3_ASSERT(area > 0.0);
		normal = b3MulSV(<f32>1.0 / area, normal);

		planes[i] = b3MakePlaneFromNormalAndPoint(normal, centroid);

		nsoa[i] = normal.x;
		nsoa[ny + i] = normal.y;
		nsoa[nz + i] = normal.z;
	}

	for (let i = faceCount; i < soaNormalCount; ++i) {
		nsoa[i] = 0.0;
		nsoa[ny + i] = 0.0;
		nsoa[nz + i] = 0.0;
	}

	b3UpdateHullEdgeCosines(hull);

	b3UpdateHullBounds(hull);
	let success = b3UpdateHullBulkProperties(hull);
	if (success == false) {
		return null;
	}

	// Must ensure the hash is 0 so it doesn't contribute to itself.
	hull.hash = 0;
	hull.hash = b3Hash64NonZero(b3SerializeHullData(hull), hull.byteCount);

	return hull;
}

/// Destroy a hull. The runtime owns memory, so this does nothing.
export function b3DestroyHull(hull: b3HullData): void {}

/// Compute mass properties of a hull
export function b3ComputeHullMass(shape: b3HullData, density: f32): b3MassData {
	let out = b3MassData();
	out.mass = density * shape.volume;
	out.center = shape.center;

	// Inertia about the center of mass
	out.inertia = b3MulSM(density, shape.centralInertia);
	return out;
}

/// Compute the bounding box of a transformed hull
export function b3ComputeHullAABB(shape: b3HullData, transform: b3Transform): b3AABB {
	return b3AABB_Transform(transform, shape.aabb);
}

export function b3ComputeSweptHullAABB(shape: b3HullData, xf1: b3Transform, xf2: b3Transform): b3AABB {
	let aabb1 = b3AABB_Transform(xf1, shape.aabb);
	let aabb2 = b3AABB_Transform(xf2, shape.aabb);
	return b3AABB_Union(aabb1, aabb2);
}

/// Overlap shape versus hull
export function b3OverlapHull(shape: b3HullData, shapeTransform: b3Transform, proxy: b3ShapeProxy): bool {
	let points = b3GetHullPoints(shape);

	let input = new b3DistanceInput();
	input.proxyA = new b3ShapeProxy(points, shape.vertexCount, 0.0);
	input.proxyB = proxy;
	input.transform = b3InvMulTransforms(shapeTransform, b3Transform_identity);
	input.useRadii = true;

	let cache = b3SimplexCache();
	let output = b3ShapeDistance(input, cache, null, 0);
	return output.distance < B3_OVERLAP_SLOP();
}

/// Ray cast versus hull shape in local space. A zero length ray is a point query. Initial overlap
/// is a hit at the origin with a zero normal.
export function b3RayCastHull(shape: b3HullData, input: b3RayCastInput): b3CastOutput {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidRay(input));
	let output = b3CastOutput();

	let lower: f32 = 0.0;
	let upper = input.maxFraction;
	let bestFace = B3_NULL_INDEX;

	let planes = b3GetHullPlanes(shape);

	for (let faceIndex = 0; faceIndex < shape.faceCount; ++faceIndex) {
		let plane = planes[faceIndex];

		let distance = plane.offset - b3Dot(plane.normal, input.origin);
		let denominator = b3Dot(plane.normal, input.translation);

		if (denominator == 0.0) {
			if (distance < 0.0) {
				return output;
			}
		} else {
			let fraction = distance / denominator;

			if (denominator < 0.0) {
				if (fraction > lower) {
					bestFace = faceIndex;
					lower = fraction;
				}
			} else {
				if (fraction < upper) {
					upper = fraction;
				}
			}

			if (upper < lower) {
				return output;
			}
		}
	}

	if (bestFace >= 0) {
		output.point = b3Add(input.origin, b3MulSV(lower, input.translation));
		output.normal = planes[bestFace].normal;
		output.fraction = lower;
		output.hit = true;
	} else {
		output.point = input.origin;
		output.hit = true;
	}

	return output;
}

/// Shape cast versus a hull. Initial overlap is treated as a miss.
export function b3ShapeCastHull(shape: b3HullData, input: b3ShapeCastInput): b3CastOutput {
	let points = b3GetHullPoints(shape);

	let pairInput = new b3ShapeCastPairInput();
	pairInput.proxyA = new b3ShapeProxy(points, shape.vertexCount, 0.0);
	pairInput.proxyB = input.proxy;
	pairInput.transform = b3Transform_identity;
	pairInput.translationB = input.translation;
	pairInput.maxFraction = input.maxFraction;
	pairInput.canEncroach = input.canEncroach;

	let output = b3ShapeCast(pairInput);
	return output;
}

export function b3CollideMoverAndHull(result: ref b3PlaneResult, shape: b3HullData, mover: b3Capsule): i32 {
	let points = b3GetHullPoints(shape);
	let distanceInput = new b3DistanceInput();
	distanceInput.proxyA = new b3ShapeProxy(points, shape.vertexCount, 0.0);
	distanceInput.proxyB = b3MakeSegmentProxy(mover.center1, mover.center2, mover.radius);
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	let totalRadius = mover.radius;

	let cache = b3SimplexCache();
	let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0);

	if (distanceOutput.distance == 0.0) {
		// I could handle deep overlap on hulls, but there is no reasonable solution for
		// deep overlap on meshes. So if someone converted a hull to a mesh there would be
		// different behavior. So I think there is not a good reason to handle deep overlap
		// on hulls.
		return 0;
	}

	if (distanceOutput.distance <= totalRadius) {
		let plane = b3Plane(distanceOutput.normal, totalRadius - distanceOutput.distance);
		let planeResult = b3PlaneResult();
		planeResult.plane = plane;
		planeResult.point = distanceOutput.pointA;
		result = planeResult;
		return 1;
	}

	return 0;
}

export function b3ComputeHullExtent(hull: b3HullData, origin: b3Vec3): b3ShapeExtent {
	let points = b3GetHullPoints(hull);

	let extent = b3ShapeExtent();
	extent.minExtent = hull.innerRadius;
	extent.maxExtent = b3Vec3_zero;
	for (let index = 0; index < hull.vertexCount; ++index) {
		let point = points[index];
		extent.maxExtent = b3Max(extent.maxExtent, b3Abs(b3Sub(point, origin)));
	}

	return extent;
}

export function b3ComputeHullProjectedArea(hull: b3HullData, direction: b3Vec3): f32 {
	let area: f32 = 0.0;

	let faceCount = hull.faceCount;
	let hullFaces = b3GetHullFaces(hull);
	let hullEdges = b3GetHullEdges(hull);
	let hullPoints = b3GetHullPoints(hull);

	for (let i = 0; i < faceCount; ++i) {
		let baseEdge = <i32>hullFaces[i].edge;
		let edge = baseEdge;
		let p1 = hullPoints[hullEdges[edge].origin];

		let edgeIndex = <i32>hullEdges[edge].next;
		edge = edgeIndex;
		let p2 = hullPoints[hullEdges[edge].origin];

		edgeIndex = <i32>hullEdges[edge].next;

		do {
			edge = edgeIndex;
			let p3 = hullPoints[hullEdges[edge].origin];

			let e1 = b3Sub(p2, p1);
			let e2 = b3Sub(p3, p1);
			let n = b3Cross(e1, e2);
			let a = b3Dot(n, direction);
			area += b3MaxFloat(a, 0.0);

			p2 = p3;
			edgeIndex = <i32>hullEdges[edge].next;
		} while (edgeIndex != baseEdge);
	}

	return 0.5 * area;
}

// Constant template box (vertex/edge/face/topology). b3MakeTransformedBoxHull copies and
// fills in the runtime-dependent fields (boxPoints, boxPlanes, aabb, mass properties, hash).
const s_boxVertexEdges: StaticArray<u8> = [8, 1, 0, 9, 13, 3, 5, 11];

// next, twin, origin, face for each half-edge
const s_boxEdges: StaticArray<u8> = [
	2, 1, 2, 0, 17, 0, 1, 5, 4, 3, 1, 0, 20, 2, 5, 3,
	6, 5, 5, 0, 23, 4, 6, 4, 0, 7, 6, 0, 18, 6, 2, 2,
	10, 9, 0, 1, 21, 8, 3, 5, 12, 11, 3, 1, 16, 10, 7, 2,
	14, 13, 7, 1, 19, 12, 4, 4, 8, 15, 4, 1, 22, 14, 0, 3,
	7, 17, 3, 2, 9, 16, 2, 5, 11, 19, 6, 2, 5, 18, 7, 4,
	15, 21, 1, 3, 1, 20, 0, 5, 3, 23, 4, 3, 13, 22, 5, 4,
];

const s_boxFaceEdges: StaticArray<u8> = [0, 8, 16, 20, 19, 21];

/// Make a transformed box as a hull. Do not call b3DestroyHull on this.
export function b3MakeTransformedBoxHull(hx: f32, hy: f32, hz: f32, transform: b3Transform): b3BoxHull {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidTransform(transform));

	let boxHull = new b3BoxHull();
	let base = boxHull.base;
	base.version = B3_HULL_VERSION;
	base.hash = 0;
	for (let i = 0; i < 8; ++i) {
		boxHull.boxVertices[i].edge = s_boxVertexEdges[i];
	}
	for (let i = 0; i < 24; ++i) {
		boxHull.boxEdges[i] = b3HullHalfEdge(
			s_boxEdges[4 * i],
			s_boxEdges[4 * i + 1],
			s_boxEdges[4 * i + 2],
			s_boxEdges[4 * i + 3],
		);
	}
	for (let i = 0; i < 6; ++i) {
		boxHull.boxFaces[i].edge = s_boxFaceEdges[i];
	}

	let minH: f32 = 0.2 * B3_LINEAR_SLOP();
	let h = b3Max(b3Vec3(minH, minH, minH), b3Vec3(hx, hy, hz));

	base.aabb = b3AABB_Transform(transform, b3AABB(b3Neg(h), h));
	base.surfaceArea = 8.0 * (h.x * h.y + h.x * h.z + h.y * h.z);
	base.volume = 8.0 * h.x * h.y * h.z;
	base.innerRadius = b3MinFloat(h.x, b3MinFloat(h.y, h.z));
	base.center = transform.p;

	let boxInertia = b3BoxInertia(base.volume, b3Neg(h), h);
	base.centralInertia = b3RotateInertia(transform.q, boxInertia);

	let lower = b3Neg(h);
	let upper = h;

	let planes = boxHull.boxPlanes;
	planes[0] = b3TransformPlane(transform, b3MakePlaneFromNormalAndPoint(b3Neg(b3Vec3_axisX), lower));
	planes[1] = b3TransformPlane(transform, b3MakePlaneFromNormalAndPoint(b3Vec3_axisX, upper));
	planes[2] = b3TransformPlane(transform, b3MakePlaneFromNormalAndPoint(b3Neg(b3Vec3_axisY), lower));
	planes[3] = b3TransformPlane(transform, b3MakePlaneFromNormalAndPoint(b3Vec3_axisY, upper));
	planes[4] = b3TransformPlane(transform, b3MakePlaneFromNormalAndPoint(b3Neg(b3Vec3_axisZ), lower));
	planes[5] = b3TransformPlane(transform, b3MakePlaneFromNormalAndPoint(b3Vec3_axisZ, upper));

	let points = boxHull.boxPoints;
	points[0] = b3TransformPoint(transform, b3Vec3(h.x, h.y, h.z));
	points[1] = b3TransformPoint(transform, b3Vec3(-h.x, h.y, h.z));
	points[2] = b3TransformPoint(transform, b3Vec3(-h.x, -h.y, h.z));
	points[3] = b3TransformPoint(transform, b3Vec3(h.x, -h.y, h.z));
	points[4] = b3TransformPoint(transform, b3Vec3(h.x, h.y, -h.z));
	points[5] = b3TransformPoint(transform, b3Vec3(-h.x, h.y, -h.z));
	points[6] = b3TransformPoint(transform, b3Vec3(-h.x, -h.y, -h.z));
	points[7] = b3TransformPoint(transform, b3Vec3(h.x, -h.y, -h.z));

	// SOA: vx, vy, vz are [0, 8), [8, 16), [16, 24); nx, ny, nz likewise
	let v = boxHull.soaVertices;
	for (let i = 0; i < 8; ++i) {
		v[i] = points[i].x;
		v[8 + i] = points[i].y;
		v[16 + i] = points[i].z;
	}

	let n = boxHull.soaNormals;
	for (let i = 0; i < 6; ++i) {
		n[i] = planes[i].normal.x;
		n[8 + i] = planes[i].normal.y;
		n[16 + i] = planes[i].normal.z;
	}
	n[6] = 0.0;
	n[7] = 0.0;
	n[14] = 0.0;
	n[15] = 0.0;
	n[22] = 0.0;
	n[23] = 0.0;

	// Must ensure the hash is 0 so it doesn't contribute to itself.
	base.hash = 0;
	base.hash = b3Hash64NonZero(b3SerializeHullData(base), base.byteCount);

	return boxHull;
}

/// Make a cube as a hull. Do not call b3DestroyHull on this.
export function b3MakeCubeHull(halfWidth: f32): b3BoxHull {
	return b3MakeBoxHull(halfWidth, halfWidth, halfWidth);
}

/// Make an offset box as a hull. Do not call b3DestroyHull on this.
export function b3MakeOffsetBoxHull(hx: f32, hy: f32, hz: f32, offset: b3Vec3): b3BoxHull {
	let transform = b3Transform(offset, b3Quat_identity);
	return b3MakeTransformedBoxHull(hx, hy, hz, transform);
}

/// Make a box as a hull. Do not call b3DestroyHull on this.
export function b3MakeBoxHull(hx: f32, hy: f32, hz: f32): b3BoxHull {
	return b3MakeTransformedBoxHull(hx, hy, hz, b3Transform_identity);
}

export function b3ScaleBox(halfWidths: ref b3Vec3, transform: ref b3Transform, postScale: b3Vec3, minHalfWidth: f32): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidFloat(minHalfWidth) && minHalfWidth > 0.0);

	let q = transform.q;

	if (postScale.x < 0.0 || postScale.y < 0.0 || postScale.z < 0.0) {
		// todo this might be unnecessary if rotation is identity
		// todo compare with polar decomposition (much more expensive)
		let m = b3MakeMatrixFromQuat(q);
		m.cx.x *= postScale.x;
		m.cy.x *= postScale.x;
		m.cz.x *= postScale.x;
		m.cx.y *= postScale.y;
		m.cy.y *= postScale.y;
		m.cz.y *= postScale.y;
		m.cx.z *= postScale.z;
		m.cy.z *= postScale.z;
		m.cz.z *= postScale.z;
		m.cx = b3Normalize(m.cx);
		m.cy = b3Normalize(m.cy);
		m.cz = b3Normalize(m.cz);
		m.cx = postScale.x < 0.0 ? b3Neg(m.cx) : m.cx;
		m.cy = postScale.y < 0.0 ? b3Neg(m.cy) : m.cy;
		m.cz = postScale.z < 0.0 ? b3Neg(m.cz) : m.cz;
		q = b3MakeQuatFromMatrix(m);
	}

	let absScale = b3Abs(postScale);

	let h = halfWidths;
	let p1 = b3Mul(absScale, b3RotateVector(q, b3Neg(h)));
	let p2 = b3Mul(absScale, b3RotateVector(q, h));

	let localP1 = b3InvRotateVector(q, p1);
	let localP2 = b3InvRotateVector(q, p2);

	let lower = b3Min(localP1, localP2);
	let upper = b3Max(localP1, localP2);

	let scaledHalfWidth = b3MulSV(0.5, b3Sub(upper, lower));

	let mLimit = b3Vec3(minHalfWidth, minHalfWidth, minHalfWidth);
	halfWidths = b3Max(scaledHalfWidth, mLimit);
	transform.p = b3Mul(postScale, transform.p);
	transform.q = q;
}

// todo use new hull scaling technique
/// This makes a transformed box hull with post scaling. This is useful for boxes that are scaled in
/// a mesh asset. The post scale is applied after the rotation, so the resulting box
/// may be approximate. If you need to support shear consider using b3CreateHull.
/// Do not call b3DestroyHull on this.
export function b3MakeScaledBoxHull(halfWidths: b3Vec3, transform: b3Transform, postScale: b3Vec3): b3BoxHull {
	if (B3_ASSERT_ENABLED) B3_ASSERT(b3IsValidTransform(transform));

	let h = halfWidths;
	let xf = transform;
	b3ScaleBox(h, xf, postScale, 4.0 * B3_LINEAR_SLOP());
	return b3MakeTransformedBoxHull(h.x, h.y, h.z, xf);
}

// 2D hull (Andrew's monotone chain)

// cross(b - a, c - a)
@inline
function b3Cross2D(a: b3Vec2, b: b3Vec2, c: b3Vec2): f32 {
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/// Simplify a 2D hull. Used internally for contact manifold simplification. Here for testing.
// Iteratively remove the vertices contributing the least area.
export function b3SimplifyHull2D(hull: StaticArray<b3Point2D>, count1: i32, target: i32): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(target >= 3);

	if (count1 <= 3) {
		return count1;
	}

	let linearSlop = B3_LINEAR_SLOP();
	let areaTol: f32 = 0.25 * linearSlop * linearSlop;
	let count2 = count1;

	for (;;) {
		let minArea = FLT_MAX;
		let minIndex = 0;

		for (let i = 0; i < count2; i++) {
			let prev = (i + count2 - 1) % count2;
			let next = (i + 1) % count2;
			let area = b3Cross2D(hull[prev].p, hull[i].p, hull[next].p);

			if (area + areaTol < minArea) {
				minArea = area;
				minIndex = i;
			}
		}

		if (count2 <= 3 || (minArea > areaTol && count2 <= target)) {
			break;
		}

		count2 -= 1;
		if (minIndex < count2) {
			for (let i = minIndex; i < count2; ++i) {
				hull[i] = hull[i + 1];
			}
		}
	}

	return count2;
}

// Sort comparator: by x, then by y. Insertion since n is typically 20 or less.
function b3Sort2D(pts: StaticArray<b3Point2D>, count: i32): void {
	for (let i = 1; i < count; ++i) {
		let base = pts[i];
		let j = i - 1;
		while (j >= 0 && (pts[j].p.x > base.p.x || (pts[j].p.x == base.p.x && pts[j].p.y > base.p.y))) {
			pts[j + 1] = pts[j];
			j -= 1;
		}
		pts[j + 1] = base;
	}
}

// Welds points that have been sorted O(n) time
function b3Weld2D(pts: StaticArray<b3Point2D>, count1: i32): i32 {
	if (count1 <= 1) {
		return count1;
	}

	let linearSlop: f32 = 0.25 * B3_LINEAR_SLOP();
	let tolSqr = linearSlop * linearSlop;

	let count2 = 1;
	let baseIndex = 0;
	for (let i = 1; i < count1; i++) {
		let distanceSqr = b3DistanceSquared2(pts[i].p, pts[baseIndex].p);
		if (distanceSqr < tolSqr) {
			if (pts[i].originalIndex < pts[baseIndex].originalIndex) {
				pts[baseIndex] = pts[i];
			}
		} else {
			pts[count2] = pts[i];
			baseIndex = count2;
			count2 += 1;
		}
	}

	return count2;
}

/// Compute a 2D hull. Used internally for contact manifold simplification. Here for testing.
// Andrew's monotone chain convex hull algorithm
// hull must have space for 2 * count1 points
export function b3Hull2D(pts: StaticArray<b3Point2D>, count: i32, hull: StaticArray<b3Point2D>): i32 {
	let count1 = count;

	if (count1 <= 0) {
		return 0;
	}

	if (count1 == 1) {
		hull[0] = pts[0];
		return 1;
	}

	b3Sort2D(pts, count1);
	count1 = b3Weld2D(pts, count1);

	if (count1 == 1) {
		hull[0] = pts[0];
		return 1;
	}

	if (count1 == 2) {
		hull[0] = pts[0];
		hull[1] = pts[1];
		return 2;
	}

	let count2 = 0;

	for (let i = 0; i < count1; i++) {
		while (count2 >= 2) {
			let area = b3Cross2D(hull[count2 - 2].p, hull[count2 - 1].p, pts[i].p);
			if (area > 0.0) {
				break;
			}

			count2 -= 1;
		}

		hull[count2] = pts[i];
		count2 += 1;
	}

	let lowerCount = count2 + 1;

	for (let i = count1 - 2; i >= 0; i--) {
		while (count2 >= lowerCount) {
			let area = b3Cross2D(hull[count2 - 2].p, hull[count2 - 1].p, pts[i].p);
			if (area > 0.0) {
				break;
			}

			count2 -= 1;
		}

		hull[count2] = pts[i];
		count2 += 1;
	}

	return count2 - 1;
}
