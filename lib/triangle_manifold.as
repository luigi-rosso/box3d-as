// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/triangle_manifold.c

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_EPSILON, FLT_MAX, FLT_MIN, b3AtomicInt } from "./core";
import { B3_LINEAR_SLOP, B3_SPECULATIVE_DISTANCE } from "./constants";
import {
	b3AbsFloat,
	b3Add,
	b3ClosestPointOnTriangle,
	b3Cross,
	b3DistanceSquared,
	b3Dot,
	b3LengthSquared,
	b3Lerp,
	b3LineDistance,
	b3MaxFloat,
	b3MinFloat,
	b3MinInt,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Normalize,
	b3Plane,
	b3Sub,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3MakePlaneFromNormalAndPoint, b3MakePlaneFromPoints, b3PlaneSeparation } from "./math_internal";
import {
	b3Capsule,
	b3DistanceInput,
	b3GetHullEdges,
	b3GetHullFaces,
	b3GetHullPlanes,
	b3GetHullPoints,
	b3HullData,
	b3LocalManifold,
	b3SATCache,
	b3SimplexCache,
	b3Sphere,
	b3TriangleFeature,
	b3_backsideAxis,
	b3_edgePairAxis,
	b3_faceAxisA,
	b3_faceAxisB,
	b3_featureEdge1,
	b3_featureEdge2,
	b3_featureEdge3,
	b3_featureHullFace,
	b3_featureNone,
	b3_featureTriangleFace,
	b3_featureVertex1,
	b3_featureVertex2,
	b3_featureVertex3,
	b3_invalidAxis,
	b3_manualEdgePairAxis,
	b3_manualFaceAxisA,
	b3_manualFaceAxisB,
} from "./collision";
import { b3ShapeDistance } from "./distance";
import { b3FindHullSupportVertex } from "./hull";
import {
	B3_MAX_CLIP_POINTS,
	b3ClipPolygon,
	b3ClipVertex,
	b3FeaturePair_single,
	b3FindIncidentFace,
	b3FlipPair,
	b3MakeFeaturePair,
	b3SeparatingAxis,
	b3_featureShapeA,
	b3_featureShapeB,
} from "./manifold";

struct b3TriangleData {
	v1: b3Vec3 = b3Vec3();
	v2: b3Vec3 = b3Vec3();
	v3: b3Vec3 = b3Vec3();
	e1: b3Vec3 = b3Vec3();
	e2: b3Vec3 = b3Vec3();
	e3: b3Vec3 = b3Vec3();
	plane: b3Plane = b3Plane();
	flags: i32 = 0;
}

// Indexed by the 3-bit vertex mask
const s_triangleFeatures: StaticArray<b3TriangleFeature> = [
	b3_featureNone, // 000  (unreachable)
	b3_featureVertex1, // 001
	b3_featureVertex2, // 010
	b3_featureEdge1, // 011  v1,v2
	b3_featureVertex3, // 100
	b3_featureEdge3, // 101  v1,v3
	b3_featureEdge2, // 110  v2,v3
	b3_featureTriangleFace, // 111
];

const s_edgeFeatures: StaticArray<b3TriangleFeature> = [b3_featureEdge1, b3_featureEdge2, b3_featureEdge3];

// Scratch for the C stack buffers. b3CollideHullFace shares the larger pair, its 64 entries in C can overflow
const s_segment = new StaticArray<b3ClipVertex>(2);
const s_clipBuffer1 = new StaticArray<b3ClipVertex>(2 * B3_MAX_CLIP_POINTS);
const s_clipBuffer2 = new StaticArray<b3ClipVertex>(2 * B3_MAX_CLIP_POINTS);
const s_distanceInput = new b3DistanceInput();
const s_capsulePoints = new StaticArray<b3Vec3>(2);
const s_trianglePoints = new StaticArray<b3Vec3>(3);

function b3GetTriangleFeature(cache: b3SimplexCache): b3TriangleFeature {
	let count = <i32>cache.count;
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 < count && count < 4);

	// Bit i set means triangle vertex i participates in the simplex.
	let mask = 0;
	for (let i = 0; i < count; ++i) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(cache.indexA[i] < 3);
		mask |= 1 << cache.indexA[i];
	}

	return s_triangleFeatures[mask];
}

// triangleA holds the three vertices
export function b3CollideTriangleAndSphere(
	manifold: b3LocalManifold,
	capacity: i32,
	triangleA: StaticArray<b3Vec3>,
	sphereB: b3Sphere,
): void {
	manifold.pointCount = 0;

	if (capacity == 0) {
		return;
	}

	let center = sphereB.center;
	let v1 = triangleA[0],
		v2 = triangleA[1],
		v3 = triangleA[2];
	let plane = b3MakePlaneFromPoints(v1, v2, v3);

	let offset = b3PlaneSeparation(plane, center);
	if (offset < 0.0) {
		// Cull back side collision
		return;
	}

	let radius = sphereB.radius;

	// Closest point on triangle to sphere center
	let closest = b3ClosestPointOnTriangle(v1, v2, v3, center);

	// Test separating axis
	let squaredDistance = b3DistanceSquared(closest.point, center);
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();
	let maxDistance = radius + speculativeDistance;
	if (squaredDistance > maxDistance * maxDistance) {
		return;
	}

	let distance = Mathf.sqrt(squaredDistance);

	// Normal points from triangle to sphere
	let normal: b3Vec3;
	if (distance * distance > 1000.0 * FLT_MIN) {
		normal = b3MulSV(1.0 / distance, b3Sub(center, closest.point));
	} else {
		normal = b3Normalize(b3Cross(b3Sub(v2, v1), b3Sub(v3, v1)));
	}

	// contact point mid-way
	// p = 0.5 * (c + q) - 0.5 * r * n
	let contactPoint = b3MulSV(0.5, b3Add(b3Sub(center, b3MulSV(radius, normal)), closest.point));

	manifold.normal = normal;
	manifold.pointCount = 1;
	manifold.feature = closest.feature;
	manifold.squaredDistance = squaredDistance;

	ref mp = manifold.points[manifold.pointOffset + 0];
	mp.point = contactPoint;
	mp.separation = distance - radius;
	mp.pair = b3FeaturePair_single;
}

function b3ClipSegmentToTriangleFace(segment: StaticArray<b3ClipVertex>, points: StaticArray<b3Vec3>, plane: b3Plane): bool {
	let vertex1 = points[2];
	for (let i = 0; i < 3; ++i) {
		let vertex2 = points[i];
		let tangent = b3Normalize(b3Sub(vertex2, vertex1));
		let binormal = b3Cross(tangent, plane.normal);

		let clipPlane = b3MakePlaneFromNormalAndPoint(binormal, vertex1);

		let vertexCount = 0;
		let p1 = segment[0];
		let p2 = segment[1];

		let distance1 = b3PlaneSeparation(clipPlane, p1.position);
		let distance2 = b3PlaneSeparation(clipPlane, p2.position);

		// If the points are behind the plane
		if (distance1 <= 0.0) {
			segment[vertexCount++] = p1;
		}
		if (distance2 <= 0.0) {
			segment[vertexCount++] = p2;
		}

		// If the points are on different sides of the plane
		if ((distance1 > 0.0) != (distance2 > 0.0)) {
			// Find intersection point of edge and plane
			let t = distance1 / (distance1 - distance2);
			segment[vertexCount].position = b3Lerp(p1.position, p2.position, t);
			segment[vertexCount].pair = distance1 > 0.0 ? p1.pair : p2.pair;
			vertexCount++;
		}

		if (vertexCount != 2) {
			return false;
		}

		vertex1 = vertex2;
	}

	return true;
}

function b3QueryTriangleFaceAndCapsule(plane: b3Plane, capsule: b3Capsule): b3SeparatingAxis {
	let separation1 = b3PlaneSeparation(plane, capsule.center1);
	let separation2 = b3PlaneSeparation(plane, capsule.center2);

	let axis: b3SeparatingAxis;
	axis.normal = plane.normal;
	axis.indexA = 0;

	if (separation1 < separation2) {
		axis.separation = separation1;
		axis.indexB = 0;
		return axis;
	}

	axis.separation = separation2;
	axis.indexB = 1;
	return axis;
}

function b3QueryTriangleAndCapsuleEdges(vertices: StaticArray<b3Vec3>, plane: b3Plane, capsule: b3Capsule): b3SeparatingAxis {
	// Work in the local space of the capsule
	let p1 = capsule.center1;
	let p2 = capsule.center2;
	let capsuleEdge = b3Sub(p2, p1);

	// Find axis of minimum penetration
	let maxNormal = b3Vec3_zero;
	let maxSeparation = -FLT_MAX;
	let maxIndex1 = B3_NULL_INDEX;
	let maxIndex2 = B3_NULL_INDEX;
	let squaredTolerance: f32 = 0.005 * 0.005;

	let edgeIndex = 2;
	let a = b3Dot(capsuleEdge, plane.normal);
	let v1 = vertices[2];
	for (let index = 0; index < 3; ++index) {
		let v2 = vertices[index];
		let triangleEdge = b3Sub(v2, v1);
		let sideNormal = b3Normalize(b3Cross(triangleEdge, plane.normal));

		// Pretend the triangle edge embeds a zero area face with a side normal. This
		// provides a way to find an edge-edge normal that points outward from
		// the triangle.
		let b = b3Dot(capsuleEdge, sideNormal);

		// Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
		if (a * a + b * b < squaredTolerance * b3LengthSquared(capsuleEdge)) {
			v1 = v2;
			edgeIndex = index;
			continue;
		}

		// Similar to hull vs hull (b3QueryEdgeDirections)
		let axis: b3Vec3;
		if (a * b <= 0.0) {
			let t = b / (b - a);
			axis = b3Lerp(sideNormal, plane.normal, t);
		} else {
			let t = b / (a + b);
			axis = b3Lerp(sideNormal, b3Neg(plane.normal), t);
		}

		axis = b3Normalize(axis);
		let separation = b3Dot(axis, b3Sub(p1, v1));

		if (separation > maxSeparation) {
			// Note: We don't exit early if we find a separating axis here since we want to
			// find the best one for caching and account for the convex radius later.
			maxNormal = axis;
			maxSeparation = separation;
			maxIndex1 = edgeIndex;
			maxIndex2 = 0;
		}

		v1 = v2;
		edgeIndex = index;
	}

	// Save result
	let result: b3SeparatingAxis;
	result.normal = maxNormal;
	result.separation = maxSeparation;
	result.indexA = maxIndex1;
	result.indexB = maxIndex2;
	return result;
}

function b3BuildTriangleAndCapsuleFaceContact(
	manifold: b3LocalManifold,
	triangle: StaticArray<b3Vec3>,
	plane: b3Plane,
	capsule: b3Capsule,
): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(manifold.pointCount == 0);

	let segment = s_segment;
	segment[0].position = capsule.center1;
	segment[0].separation = 0.0;
	segment[0].pair = b3MakeFeaturePair(b3_featureShapeA, 0, b3_featureShapeA, 0);
	segment[1].position = capsule.center2;
	segment[1].separation = 0.0;
	segment[1].pair = b3MakeFeaturePair(b3_featureShapeA, 1, b3_featureShapeA, 1);

	let havePoints = b3ClipSegmentToTriangleFace(segment, triangle, plane);
	if (havePoints == false) {
		return;
	}

	let radius = capsule.radius;
	let distance1 = b3PlaneSeparation(plane, segment[0].position);
	let distance2 = b3PlaneSeparation(plane, segment[1].position);

	let speculativeDistance = B3_SPECULATIVE_DISTANCE();
	if (distance1 > speculativeDistance + radius && distance2 > speculativeDistance + radius) {
		return;
	}

	// Average points. Half-way between capsule bottom and triangle plane.
	let point1 = b3MulSub(segment[0].position, 0.5 * (distance1 + capsule.radius), plane.normal);
	let point2 = b3MulSub(segment[1].position, 0.5 * (distance2 + capsule.radius), plane.normal);

	manifold.normal = plane.normal;
	manifold.feature = b3_featureTriangleFace;
	manifold.pointCount = 2;

	let base = manifold.pointOffset;
	ref pt1 = manifold.points[base + 0];
	pt1.point = point1;
	pt1.separation = distance1 - capsule.radius;
	pt1.pair = segment[0].pair;

	ref pt2 = manifold.points[base + 1];
	pt2.point = point2;
	pt2.separation = distance2 - capsule.radius;
	pt2.pair = segment[1].pair;
}

function b3BuildTriangleAndCapsuleEdgeContact(
	manifold: b3LocalManifold,
	triangle: StaticArray<b3Vec3>,
	plane: b3Plane,
	capsule: b3Capsule,
	query: b3SeparatingAxis,
): void {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= query.indexA && query.indexA < 3);

	let p1 = capsule.center1;
	let p2 = capsule.center2;
	let capsuleEdge = b3Sub(p2, p1);

	let vs = triangle;

	let v1 = vs[query.indexA];
	let v2 = vs[(query.indexA + 1) % 3];
	let triangleEdge = b3Sub(v2, v1);

	let sideNormal = b3Normalize(b3Cross(triangleEdge, plane.normal));

	// Pretend the triangle edge embeds a zero area face with a side normal. This
	// provides a way to find an edge-edge normal that points outward from
	// the triangle.
	let a = b3Dot(capsuleEdge, plane.normal);
	let b = b3Dot(capsuleEdge, sideNormal);

	// Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
	let squaredTolerance: f32 = 0.005 * 0.005;
	if (a * a + b * b < squaredTolerance * b3LengthSquared(capsuleEdge)) {
		return;
	}

	// Similar to hull vs hull (b3QueryEdgeDirections)
	let normal = query.normal;
	let result = b3LineDistance(v1, triangleEdge, p1, capsuleEdge);

	if (result.fraction1 < 0.0 || 1.0 < result.fraction1 || result.fraction2 < 0.0 || 1.0 < result.fraction2) {
		// closest point beyond end points
		return;
	}

	let point = b3Lerp(result.point1, b3MulSub(result.point2, capsule.radius, normal), 0.5);
	let separation = b3Dot(normal, b3Sub(p1, v1));

	manifold.normal = normal;
	manifold.pointCount = 1;

	manifold.feature = s_edgeFeatures[query.indexA];

	ref pt = manifold.points[manifold.pointOffset + 0];
	pt.point = point;
	pt.separation = separation - capsule.radius;
	pt.pair = b3MakeFeaturePair(b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB);
}

// triangleA holds the three vertices
export function b3CollideTriangleAndCapsule(
	manifold: b3LocalManifold,
	capacity: i32,
	triangleA: StaticArray<b3Vec3>,
	capsuleB: b3Capsule,
	cache: ref b3SimplexCache,
): void {
	manifold.pointCount = 0;

	if (capacity < 2) {
		return;
	}

	let v1 = triangleA[0],
		v2 = triangleA[1],
		v3 = triangleA[2];
	let plane = b3MakePlaneFromPoints(v1, v2, v3);
	let capsuleCenter = b3Lerp(capsuleB.center1, capsuleB.center2, 0.5);

	let offset = b3PlaneSeparation(plane, capsuleCenter);
	if (offset < 0.0) {
		// Cull back side collision
		return;
	}

	let distanceInput = s_distanceInput;
	distanceInput.proxyA.points = triangleA;
	distanceInput.proxyA.count = 3;
	distanceInput.proxyA.radius = 0.0;
	s_capsulePoints[0] = capsuleB.center1;
	s_capsulePoints[1] = capsuleB.center2;
	distanceInput.proxyB.points = s_capsulePoints;
	distanceInput.proxyB.count = 2;
	distanceInput.proxyB.radius = 0.0;
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0);
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();
	let radius = capsuleB.radius;
	if (distanceOutput.distance > radius + speculativeDistance) {
		// Shapes are separated, persist the cache
		return;
	}

	if (distanceOutput.distance > 100.0 * FLT_EPSILON) {
		// Shallow penetration
		let delta = b3Normalize(b3Sub(distanceOutput.pointB, distanceOutput.pointA));

		// Try to create two contact points if closest points difference is nearly parallel to face normal
		let kTolerance: f32 = 0.2;
		let cosAngle = b3AbsFloat(b3Dot(plane.normal, delta));
		if (cosAngle > kTolerance) {
			// Clip capsule segment against side planes of reference face
			let segment = s_segment;
			segment[0].position = capsuleB.center1;
			segment[0].separation = 0.0;
			segment[0].pair = b3MakeFeaturePair(b3_featureShapeA, 0, b3_featureShapeA, 0);
			segment[1].position = capsuleB.center2;
			segment[1].separation = 0.0;
			segment[1].pair = b3MakeFeaturePair(b3_featureShapeA, 1, b3_featureShapeA, 1);

			let havePoints = b3ClipSegmentToTriangleFace(segment, triangleA, plane);

			if (havePoints == true) {
				let distance1 = b3PlaneSeparation(plane, segment[0].position);
				let distance2 = b3PlaneSeparation(plane, segment[1].position);

				let normal = plane.normal;
				let point1 = b3MulSub(segment[0].position, 0.5 * (radius + distance1), normal);
				let point2 = b3MulSub(segment[1].position, 0.5 * (radius + distance2), normal);

				// Normal points from triangle to capsule
				manifold.normal = normal;
				manifold.feature = b3_featureTriangleFace;
				manifold.pointCount = 2;

				let base = manifold.pointOffset;
				ref mp1 = manifold.points[base + 0];
				mp1.point = point1;
				mp1.separation = distance1 - radius;
				mp1.pair = segment[0].pair;

				ref mp2 = manifold.points[base + 1];
				mp2.point = point2;
				mp2.separation = distance2 - radius;
				mp2.pair = segment[1].pair;

				return;
			}
		}

		// Create contact from closest points.
		let point = b3Lerp(distanceOutput.pointA, b3MulSub(distanceOutput.pointB, radius, delta), 0.5);

		// Normal points from triangle to capsule.
		manifold.normal = delta;
		manifold.pointCount = 1;
		manifold.feature = b3GetTriangleFeature(cache);

		ref mp = manifold.points[manifold.pointOffset + 0];
		mp.point = point;
		mp.separation = distanceOutput.distance - radius;
		mp.pair = b3FeaturePair_single;

		return;
	}

	// Deep penetration

	let faceQuery = b3QueryTriangleFaceAndCapsule(plane, capsuleB);
	if (faceQuery.separation > radius) {
		// Shapes are separated. Should be impossible for a reasonable capsule radius.
		return;
	}

	let edgeQuery = b3QueryTriangleAndCapsuleEdges(triangleA, plane, capsuleB);
	if (edgeQuery.separation > radius) {
		// Shapes are separated. Should be impossible for a reasonable capsule radius.
		return;
	}

	// Create face contact
	let faceSeparation = faceQuery.separation - radius;
	b3BuildTriangleAndCapsuleFaceContact(manifold, triangleA, plane, capsuleB);
	if (manifold.pointCount == 2) {
		// This becomes the clipped separation.
		let base = manifold.pointOffset;
		faceSeparation = b3MinFloat(manifold.points[base + 0].separation, manifold.points[base + 1].separation);
	}

	if (edgeQuery.indexA == B3_NULL_INDEX) {
		// No valid edge contact.
		return;
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	let linearSlop = B3_LINEAR_SLOP();
	let edgeSeparation = edgeQuery.separation - radius;
	if (manifold.pointCount == 0 || edgeSeparation > faceSeparation + linearSlop) {
		// Edge contact
		b3BuildTriangleAndCapsuleEdgeContact(manifold, triangleA, plane, capsuleB, edgeQuery);
	}
}

@inline
function b3GetTriangleSupport(points: FixedArray<b3Vec3, 3>, direction: b3Vec3): i32 {
	let index = 0;
	let distance = b3Dot(points[0], direction);

	let d = b3Dot(points[1], direction);
	if (d > distance) {
		distance = d;
		index = 1;
	}

	d = b3Dot(points[2], direction);
	if (d > distance) {
		return 2;
	}

	return index;
}

function b3QueryTriangleFace(triangle: b3TriangleData, hull: b3HullData): b3SeparatingAxis {
	let hullPoints = b3GetHullPoints(hull);
	let plane = triangle.plane;
	let normal = b3Neg(plane.normal);
	let vertexIndex = b3FindHullSupportVertex(hull, normal);
	let support = hullPoints[vertexIndex];
	let separation = b3PlaneSeparation(plane, support);

	let axis: b3SeparatingAxis;
	axis.normal = plane.normal;
	axis.separation = separation;
	axis.indexA = 0;
	axis.indexB = vertexIndex;
	axis.type = b3_faceAxisA;
	return axis;
}

function b3QueryHullFace(triangle: b3TriangleData, hull: b3HullData): b3SeparatingAxis {
	let hullPlanes = b3GetHullPlanes(hull);
	let faceCount = hull.faceCount;

	let trianglePoints: FixedArray<b3Vec3, 3>;
	trianglePoints[0] = triangle.v1;
	trianglePoints[1] = triangle.v2;
	trianglePoints[2] = triangle.v3;

	let maxNormal = b3Vec3_zero;
	let maxFaceSeparation = -FLT_INF;
	let maxFaceIndex = B3_NULL_INDEX;
	let maxVertexIndex = B3_NULL_INDEX;

	for (let faceIndex = 0; faceIndex < faceCount; ++faceIndex) {
		let plane = hullPlanes[faceIndex];

		let vertexIndex = b3GetTriangleSupport(trianglePoints, b3Neg(plane.normal));
		let support = trianglePoints[vertexIndex];
		let separation = b3PlaneSeparation(plane, support);
		if (separation > maxFaceSeparation) {
			maxNormal = plane.normal;
			maxFaceSeparation = separation;
			maxFaceIndex = faceIndex;
			maxVertexIndex = vertexIndex;
		}
	}

	// Normal points from triangle to hull
	let axis: b3SeparatingAxis;
	axis.normal = b3Neg(maxNormal);
	axis.separation = maxFaceSeparation;
	axis.indexA = maxVertexIndex;
	axis.indexB = maxFaceIndex;
	axis.type = b3_faceAxisB;
	return axis;
}

const FLT_INF: f32 = reinterpret<f32>(0x7f800000);

// A: hull, B: triangle
function b3QueryTriangleAndHullEdges(triangle: b3TriangleData, hull: b3HullData): b3SeparatingAxis {
	let result: b3SeparatingAxis;
	result.normal = b3Vec3_zero;
	result.separation = -FLT_INF;
	result.indexA = B3_NULL_INDEX;
	result.indexB = B3_NULL_INDEX;
	result.type = b3_edgePairAxis;

	let trianglePoints: FixedArray<b3Vec3, 3>;
	trianglePoints[0] = triangle.v1;
	trianglePoints[1] = triangle.v2;
	trianglePoints[2] = triangle.v3;
	let triangleEdges: FixedArray<b3Vec3, 3>;
	triangleEdges[0] = triangle.e1;
	triangleEdges[1] = triangle.e2;
	triangleEdges[2] = triangle.e3;
	// int edgeFlags[] = { b3_concaveEdge1, b3_concaveEdge1, b3_concaveEdge3 };

	let triNormal = triangle.plane.normal;

	let hullEdges = b3GetHullEdges(hull);
	let hullPoints = b3GetHullPoints(hull);
	let hullPlanes = b3GetHullPlanes(hull);
	let edgeCount = hull.edgeCount;
	let squaredTolerance: f32 = 0.005 * 0.005;

	for (let i = 0; i < edgeCount; i += 2) {
		let edge = hullEdges[i];
		let twin = hullEdges[i + 1];
		if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edge.twin == i + 1 && <i32>twin.twin == i);

		let hullPoint = hullPoints[edge.origin];
		let hullEdge = b3Sub(hullPoints[twin.origin], hullPoint);

		let hullNormal1 = hullPlanes[edge.face].normal;
		let hullNormal2 = hullPlanes[twin.face].normal;

		for (let j = 0; j < 3; ++j) {
			let triEdge = triangleEdges[j];

			let cab = b3Dot(hullNormal1, triEdge);
			let dab = b3Dot(hullNormal2, triEdge);
			let bcd = b3Dot(triNormal, hullEdge);
			if (cab * dab >= 0.0 || cab * bcd <= 0.0) {
				continue;
			}

			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if (b3MaxFloat(cab * cab, dab * dab) < squaredTolerance * b3LengthSquared(triEdge)) {
				continue;
			}

			// Similar to hull vs hull (b3QueryEdgeDirections)
			// dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
			// Normal points out of hull by construction.
			let t = cab / (cab - dab);
			let axis = b3Lerp(hullNormal1, hullNormal2, t);
			axis = b3Normalize(axis);
			let separation = b3Dot(axis, b3Sub(trianglePoints[j], hullPoint));

			// if ( separation > result.separation && ( edgeFlags[j] & triangleFlags ) == 0 )
			if (separation > result.separation) {
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching.
				// Flip normal to point from triangle to hull.
				result.normal = b3Neg(axis);
				result.separation = separation;
				result.indexA = j;
				result.indexB = i;
			}
		}
	}

	return result;
}

function b3CollideHullFace(
	manifold: b3LocalManifold,
	pointCapacity: i32,
	triangle: b3TriangleData,
	hull: b3HullData,
	query: b3SeparatingAxis,
	cache: ref b3SATCache,
	enableSpeculative: bool,
): f32 {
	manifold.pointCount = 0;

	let hullFaces = b3GetHullFaces(hull);
	let hullEdges = b3GetHullEdges(hull);
	let hullPlanes = b3GetHullPlanes(hull);
	let hullPoints = b3GetHullPoints(hull);

	// Reference hull face
	let refPlane = hullPlanes[query.indexB];

	// Build clip polygon from triangle face (the incident face)
	let buffer1 = s_clipBuffer1;
	let buffer2 = s_clipBuffer2;

	let v1 = triangle.v1;
	let v2 = triangle.v2;
	let v3 = triangle.v3;
	buffer1[0].position = v1;
	buffer1[0].separation = b3PlaneSeparation(refPlane, v1);
	buffer1[0].pair = b3MakeFeaturePair(b3_featureShapeB, 2, b3_featureShapeB, 0);
	buffer1[1].position = v2;
	buffer1[1].separation = b3PlaneSeparation(refPlane, v2);
	buffer1[1].pair = b3MakeFeaturePair(b3_featureShapeB, 0, b3_featureShapeB, 1);
	buffer1[2].position = v3;
	buffer1[2].separation = b3PlaneSeparation(refPlane, v3);
	buffer1[2].pair = b3MakeFeaturePair(b3_featureShapeB, 1, b3_featureShapeB, 2);
	let pointCount = 3;

	// Clip triangle face against side planes of reference face
	let input = buffer1;
	let output = buffer2;

	let faceEdge = <i32>hullFaces[query.indexB].edge;
	let edgeIndex = faceEdge;

	do {
		let edge = hullEdges[edgeIndex];
		let nextEdgeIndex = <i32>edge.next;
		let next = hullEdges[nextEdgeIndex];
		let vertex1 = hullPoints[edge.origin];
		let vertex2 = hullPoints[next.origin];
		let tangent = b3Normalize(b3Sub(vertex2, vertex1));
		let binormal = b3Cross(tangent, refPlane.normal);

		let clipPlane = b3MakePlaneFromNormalAndPoint(binormal, vertex1);

		pointCount = b3ClipPolygon(output, input, pointCount, clipPlane, edgeIndex, refPlane);
		if (B3_ASSERT_ENABLED) B3_ASSERT(pointCount <= B3_MAX_CLIP_POINTS);

		if (pointCount < 3) {
			// Using a stale cache
			cache = b3SATCache();
			return query.separation;
		}

		// Swap buffers, output becomes input for the next clipping plane
		let swap = output;
		output = input;
		input = swap;
		edgeIndex = nextEdgeIndex;
	} while (edgeIndex != faceEdge);

	pointCount = b3MinInt(pointCount, pointCapacity);
	let minSeparation = FLT_MAX;
	let finalPointCount = 0;

	let base = manifold.pointOffset;
	for (let i = 0; i < pointCount; ++i) {
		let clipPoint = input[i];
		minSeparation = b3MinFloat(minSeparation, clipPoint.separation);

		if (enableSpeculative == false && clipPoint.separation > 0.0) {
			continue;
		}

		// Move point onto hull face improved culling
		let point = b3MulSub(clipPoint.position, clipPoint.separation, refPlane.normal);

		ref pt = manifold.points[base + finalPointCount];
		pt.point = point;
		pt.separation = clipPoint.separation;
		pt.pair = b3FlipPair(clipPoint.pair);

		finalPointCount += 1;
	}

	let speculativeDistance: f32 = enableSpeculative ? B3_SPECULATIVE_DISTANCE() : 0.0;
	if (minSeparation > speculativeDistance) {
		// This can occur with a stale SAT cache
		manifold.pointCount = 0;
		cache = b3SATCache();
		return minSeparation;
	}

	manifold.pointCount = finalPointCount;
	manifold.normal = b3Neg(refPlane.normal);
	manifold.feature = b3_featureHullFace;

	// Save cache
	cache.separation = minSeparation;
	cache.type = <u8>b3_faceAxisB;
	cache.indexA = <u8>query.indexA;
	cache.indexB = <u8>query.indexB;
	return minSeparation;
}

function b3CollideTriangleFace(
	manifold: b3LocalManifold,
	pointCapacity: i32,
	triangle: b3TriangleData,
	hull: b3HullData,
	query: b3SeparatingAxis,
	cache: ref b3SATCache,
	enableSpeculative: bool,
): f32 {
	let hullFaces = b3GetHullFaces(hull);
	let hullEdges = b3GetHullEdges(hull);
	let hullPoints = b3GetHullPoints(hull);

	// Find incident face
	let refPlane = triangle.plane;

	let incFace = b3FindIncidentFace(hull, refPlane.normal, query.indexB);

	// Build clip polygon from incident face
	let buffer1 = s_clipBuffer1;
	let buffer2 = s_clipBuffer2;
	let pointCount = 0;
	let faceEdge = <i32>hullFaces[incFace].edge;
	let hullEdgeIndex = faceEdge;

	do {
		let edge = hullEdges[hullEdgeIndex];

		let nextEdgeIndex = <i32>edge.next;
		let next = hullEdges[nextEdgeIndex];

		let hullPoint = hullPoints[next.origin];
		buffer1[pointCount].position = hullPoint;
		buffer1[pointCount].separation = b3PlaneSeparation(refPlane, hullPoint);
		buffer1[pointCount].pair = b3MakeFeaturePair(b3_featureShapeB, hullEdgeIndex, b3_featureShapeB, nextEdgeIndex);

		pointCount += 1;
		hullEdgeIndex = nextEdgeIndex;
	} while (hullEdgeIndex != faceEdge && pointCount < 2 * B3_MAX_CLIP_POINTS);

	if (B3_ASSERT_ENABLED) B3_ASSERT(pointCount >= 3);

	// Clip incident face against side planes of reference face (triangle)
	let input = buffer1;
	let output = buffer2;

	let trianglePoints: FixedArray<b3Vec3, 3>;
	trianglePoints[0] = triangle.v1;
	trianglePoints[1] = triangle.v2;
	trianglePoints[2] = triangle.v3;
	let triangleEdges: FixedArray<b3Vec3, 3>;
	triangleEdges[0] = triangle.e1;
	triangleEdges[1] = triangle.e2;
	triangleEdges[2] = triangle.e3;

	for (let i = 0; i < 3 && pointCount > 0; ++i) {
		let sideNormal = b3Cross(triangleEdges[i], refPlane.normal);
		sideNormal = b3Normalize(sideNormal);

		let clipPlane = b3MakePlaneFromNormalAndPoint(sideNormal, trianglePoints[i]);

		pointCount = b3ClipPolygon(output, input, pointCount, clipPlane, i, refPlane);
		if (B3_ASSERT_ENABLED) B3_ASSERT(pointCount <= 2 * B3_MAX_CLIP_POINTS);

		let swap = output;
		output = input;
		input = swap;
	}

	if (pointCount == 0) {
		// Triangle face clipped away. Invalidate cache.
		cache = b3SATCache();
		return FLT_MAX;
	}

	pointCount = b3MinInt(pointCount, pointCapacity);

	let minSeparation = FLT_MAX;

	let finalPointCount = 0;
	let base = manifold.pointOffset;
	for (let i = 0; i < pointCount; ++i) {
		let clipPoint = input[i];
		minSeparation = b3MinFloat(minSeparation, clipPoint.separation);

		if (enableSpeculative == false && clipPoint.separation > 0.0) {
			continue;
		}

		// Move point onto triangle surface for improved culling
		// b3Vec3 point = b3MulSub( clipPoint->position, clipPoint->separation, refPlane.normal );
		let point = clipPoint.position;

		ref pt = manifold.points[base + finalPointCount];
		pt.point = point;
		pt.separation = clipPoint.separation;
		pt.pair = clipPoint.pair;

		finalPointCount += 1;
	}

	let speculativeDistance: f32 = enableSpeculative ? B3_SPECULATIVE_DISTANCE() : 0.0;
	if (minSeparation >= speculativeDistance) {
		// This can happens if the objects move a part while re-using a cached axis
		cache = b3SATCache();
		return minSeparation;
	}

	manifold.pointCount = finalPointCount;
	manifold.normal = refPlane.normal;
	manifold.feature = b3_featureTriangleFace;

	// Save cache
	cache.separation = minSeparation;
	cache.type = <u8>b3_faceAxisA;
	cache.indexA = <u8>query.indexA;
	cache.indexB = <u8>query.indexB;
	return minSeparation;
}

function b3CollideTriangleAndHullEdges(
	manifold: b3LocalManifold,
	capacity: i32,
	trianglePoint: b3Vec3,
	triangleEdge: b3Vec3,
	hull: b3HullData,
	query: b3SeparatingAxis,
	cache: ref b3SATCache,
): void {
	let pA = trianglePoint;
	let eA = triangleEdge;

	let edgesB = b3GetHullEdges(hull);
	let pointsB = b3GetHullPoints(hull);
	let edgeB = edgesB[query.indexB];
	let twinB = edgesB[edgeB.twin];
	let pB = pointsB[edgeB.origin];
	let qB = pointsB[twinB.origin];
	let eB = b3Sub(qB, pB);

	// Get the closest points between the infinite edge lines
	let result = b3LineDistance(pA, eA, pB, eB);

	// Is one of the closest points outside of the associated edge segment?
	if (
		capacity == 0 ||
		result.fraction1 < 0.0 ||
		1.0 < result.fraction1 ||
		result.fraction2 < 0.0 ||
		1.0 < result.fraction2
	) {
		// Invalid edge pair, no points generated
		if (B3_ASSERT_ENABLED) B3_ASSERT(manifold.pointCount == 0);
		cache = b3SATCache();
		return;
	}

	let separation = b3Dot(query.normal, b3Sub(pB, pA));
	let point = b3MulSV(0.5, b3Add(result.point1, result.point2));

	ref pt = manifold.points[manifold.pointOffset + 0];
	pt.point = point;
	pt.separation = separation;
	pt.pair = b3MakeFeaturePair(b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB);

	// Save cache
	cache.separation = separation;
	cache.type = <u8>b3_edgePairAxis;
	cache.indexA = <u8>query.indexA;
	cache.indexB = <u8>query.indexB;

	manifold.normal = query.normal;
	manifold.pointCount = 1;

	manifold.feature = s_edgeFeatures[query.indexA];
}

export let b3_triangleConvexCalls = b3AtomicInt();
export let b3_triangleCacheHits = b3AtomicInt();

// Triangle is in the local space of the hull for efficiency.
export function b3CollideTriangleAndHull(
	manifold: b3LocalManifold,
	capacity: i32,
	v1: b3Vec3,
	v2: b3Vec3,
	v3: b3Vec3,
	triangleFlags: i32,
	hullB: b3HullData,
	cache: ref b3SATCache,
	enableSpeculative: bool,
): void {
	manifold.pointCount = 0;
	manifold.feature = b3_featureNone;

	if (capacity < 4) {
		return;
	}

	let trianglePlane = b3MakePlaneFromPoints(v1, v2, v3);
	let linearSlop = B3_LINEAR_SLOP();

	let offset = b3PlaneSeparation(trianglePlane, hullB.center);
	if (<i32>cache.type == b3_backsideAxis) {
		// Use hysteresis to avoid jitter on wavy meshes
		if (b3AbsFloat(cache.separation - offset) < linearSlop) {
			return;
		}

		cache.type = <u8>b3_invalidAxis;
	}

	if (offset < -linearSlop) {
		// Cull back side collision. Cache offset to add hysteresis.
		cache.type = <u8>b3_backsideAxis;
		cache.separation = offset;
		return;
	}

	let trianglePoints: FixedArray<b3Vec3, 3>;
	trianglePoints[0] = v1;
	trianglePoints[1] = v2;
	trianglePoints[2] = v3;
	let triangleEdges: FixedArray<b3Vec3, 3>;
	triangleEdges[0] = b3Sub(v2, v1);
	triangleEdges[1] = b3Sub(v3, v2);
	triangleEdges[2] = b3Sub(v1, v3);

	let triangle: b3TriangleData;
	triangle.v1 = v1;
	triangle.v2 = v2;
	triangle.v3 = v3;
	triangle.e1 = triangleEdges[0];
	triangle.e2 = triangleEdges[1];
	triangle.e3 = triangleEdges[2];
	triangle.plane = trianglePlane;
	triangle.flags = triangleFlags;

	let edges = b3GetHullEdges(hullB);
	let hullPlanes = b3GetHullPlanes(hullB);
	let hullPoints = b3GetHullPoints(hullB);

	let speculativeDistance: f32 = enableSpeculative ? B3_SPECULATIVE_DISTANCE() : 0.0;
	cache.hit = 1;

	// Attempt to use the cache to speed up collision
	switch (<i32>cache.type) {
		case b3_faceAxisA: {
			if (B3_ASSERT_ENABLED) B3_ASSERT(cache.indexA == 0);

			let vertexIndex = b3FindHullSupportVertex(hullB, b3Neg(trianglePlane.normal));
			let support = hullPoints[vertexIndex];
			let separation = b3PlaneSeparation(trianglePlane, support);
			if (separation > speculativeDistance) {
				// Cache hit, shapes are separated
				return;
			}

			let faceQuery: b3SeparatingAxis;
			faceQuery.normal = trianglePlane.normal;
			faceQuery.separation = separation;
			faceQuery.indexA = cache.indexA;
			faceQuery.indexB = vertexIndex;
			faceQuery.type = b3_faceAxisA;

			// Read cache but don't modify it
			let localCache = cache;
			let clippedSeparation = b3CollideTriangleFace(
				manifold,
				capacity,
				triangle,
				hullB,
				faceQuery,
				localCache,
				enableSpeculative,
			);

			if (manifold.pointCount > 0 && b3AbsFloat(cache.separation - clippedSeparation) < linearSlop) {
				// Cache hit, contact points generated
				return;
			}

			// Invalidate cache and fall through
			manifold.pointCount = 0;
			cache = b3SATCache();
			break;
		}

		case b3_faceAxisB: {
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>cache.indexB < hullB.faceCount);

			let plane = hullPlanes[cache.indexB];

			// Get triangle support point
			let vertexIndex = 0;
			let distance = -b3Dot(v1, plane.normal);
			for (let i = 1; i < 3; ++i) {
				let d = -b3Dot(trianglePoints[i], plane.normal);
				if (d > distance) {
					distance = d;
					vertexIndex = i;
				}
			}

			let support = trianglePoints[vertexIndex];

			// Separation of triangle support point with hull plane
			let separation = b3PlaneSeparation(plane, support);
			if (separation > speculativeDistance) {
				// Cache hit, shapes are separated
				return;
			}

			// Deep overlap may lead to an invalid cache
			// todo confirm
			let isDeep = separation < -2.0 * linearSlop;

			// Don't persist deep cache or allow separation to change too much
			if (isDeep == false) {
				//  Try to rebuild contact from last features
				let faceQuery: b3SeparatingAxis;
				faceQuery.normal = b3Neg(plane.normal);
				faceQuery.separation = separation;
				faceQuery.indexA = vertexIndex;
				faceQuery.indexB = cache.indexB;
				faceQuery.type = b3_faceAxisB;

				// Read cache but don't modify it
				let localCache = cache;
				let clippedSeparation = b3CollideHullFace(
					manifold,
					capacity,
					triangle,
					hullB,
					faceQuery,
					localCache,
					enableSpeculative,
				);

				// Cache reuse is only successful if it creates contact points and the clipped separation didn't change much.
				if (manifold.pointCount > 0 && b3AbsFloat(cache.separation - clippedSeparation) < linearSlop) {
					// Cache hit, contact points generated
					return;
				}
			}

			// Invalidate cache and fall through
			manifold.pointCount = 0;
			cache = b3SATCache();
			break;
		}

		case b3_edgePairAxis: {
			if (B3_ASSERT_ENABLED) B3_ASSERT(cache.indexA < 3);
			let indexA = <i32>cache.indexA;

			let triPoint = trianglePoints[indexA];
			let triEdge = triangleEdges[indexA];

			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>cache.indexB < hullB.edgeCount - 1);
			let indexB = <i32>cache.indexB;

			let edge2 = edges[indexB];
			let twin2 = edges[indexB + 1];
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edge2.twin == indexB + 1 && <i32>twin2.twin == indexB);

			let hullPoint = hullPoints[edge2.origin];
			let hullEdge = b3Sub(hullPoints[twin2.origin], hullPoint);
			let hullNormal1 = hullPlanes[edge2.face].normal;
			let hullNormal2 = hullPlanes[twin2.face].normal;

			// Confirm the edge pair is still a Minkowski face.
			// See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
			// Simplified for triangle versus hull.
			let cab = b3Dot(hullNormal1, triEdge);
			let dab = b3Dot(hullNormal2, triEdge);
			let bcd = b3Dot(trianglePlane.normal, hullEdge);

			if (cab * dab < 0.0 && cab * bcd > 0.0) {
				let squaredTolerance: f32 = 0.005 * 0.005;

				// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
				if (b3MaxFloat(cab * cab, dab * dab) >= squaredTolerance * b3LengthSquared(triEdge)) {
					// Similar to hull vs hull (b3QueryEdgeDirections)
					// dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
					// Normal points out of hull by construction.
					let t = cab / (cab - dab);
					let axis = b3Lerp(hullNormal1, hullNormal2, t);
					axis = b3Normalize(axis);
					let separation = b3Dot(axis, b3Sub(triPoint, hullPoint));
					if (separation > speculativeDistance) {
						// Cache hit, shapes are separated
						return;
					}

					if (b3AbsFloat(cache.separation - separation) < linearSlop) {
						// Try to rebuild contact from last features
						// Flip normal to point from triangle to hull
						let edgeQuery: b3SeparatingAxis;
						edgeQuery.normal = b3Neg(axis);
						edgeQuery.indexA = indexA;
						edgeQuery.indexB = indexB;
						edgeQuery.separation = separation;
						edgeQuery.type = b3_edgePairAxis;

						// Read cache but don't modify it
						let localCache = cache;
						b3CollideTriangleAndHullEdges(manifold, capacity, triPoint, triEdge, hullB, edgeQuery, localCache);

						if (manifold.pointCount > 0) {
							// Cache hit, contact point generated
							return;
						}
					}
				}
			}

			// Invalidate cache and fall through
			cache = b3SATCache();
			break;
		}

		// This case is for testing
		case b3_manualFaceAxisA: {
			let query = b3QueryTriangleFace(triangle, hullB);
			b3CollideTriangleFace(manifold, capacity, triangle, hullB, query, cache, enableSpeculative);
			return;
		}

		// This case is for testing
		case b3_manualFaceAxisB: {
			let query = b3QueryHullFace(triangle, hullB);
			b3CollideHullFace(manifold, capacity, triangle, hullB, query, cache, enableSpeculative);
			return;
		}

		// This case is for testing
		case b3_manualEdgePairAxis: {
			let query = b3QueryTriangleAndHullEdges(triangle, hullB);
			if (query.indexA != B3_NULL_INDEX) {
				let trianglePoint = trianglePoints[query.indexA];
				let triangleEdge = triangleEdges[query.indexA];
				b3CollideTriangleAndHullEdges(manifold, capacity, trianglePoint, triangleEdge, hullB, query, cache);
			}
			return;
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>cache.type == b3_invalidAxis);
			break;
	}

	// Cache miss
	cache.hit = 0;

	// Find axis of minimum penetration
	let faceQueryA = b3QueryTriangleFace(triangle, hullB);
	if (faceQueryA.separation > speculativeDistance) {
		// Separating axis found
		cache.separation = faceQueryA.separation;
		cache.type = <u8>b3_faceAxisA;
		cache.indexA = <u8>faceQueryA.indexA;
		cache.indexB = <u8>faceQueryA.indexB;
		return;
	}

	let faceQueryB = b3QueryHullFace(triangle, hullB);
	if (faceQueryB.separation > speculativeDistance) {
		// Separating axis found
		cache.separation = faceQueryB.separation;
		cache.type = <u8>b3_faceAxisB;
		cache.indexA = <u8>faceQueryB.indexA;
		cache.indexB = <u8>faceQueryB.indexB;
		return;
	}

	let edgeQuery = b3QueryTriangleAndHullEdges(triangle, hullB);
	if (edgeQuery.separation > speculativeDistance) {
		// Separating axis found
		cache.separation = edgeQuery.separation;
		cache.type = <u8>b3_edgePairAxis;
		cache.indexA = <u8>edgeQuery.indexA;
		cache.indexB = <u8>edgeQuery.indexB;
		return;
	}

	let clipSeparation: f32;

	// Don't admit a hull face significantly opposed to the triangle face.
	// Need a tolerance to avoid ghost collisions.
	let pushingDown = b3Dot(faceQueryB.normal, trianglePlane.normal) < -0.25;
	if (faceQueryB.separation >= faceQueryA.separation && pushingDown == false) {
		clipSeparation = b3CollideHullFace(manifold, capacity, triangle, hullB, faceQueryB, cache, enableSpeculative);
	} else {
		clipSeparation = b3CollideTriangleFace(manifold, capacity, triangle, hullB, faceQueryA, cache, enableSpeculative);
	}

	// Does an edge axis exist?
	if (edgeQuery.indexA != B3_NULL_INDEX) {
		// When axes are aligned the edge separation can be garbage.
		// If a face axis has positive separation there may be no points.
		let maxFaceSeparation = b3MaxFloat(faceQueryA.separation, faceQueryB.separation);

		if (
			(manifold.pointCount == 0 && edgeQuery.separation > maxFaceSeparation) ||
			(manifold.pointCount == 1 && edgeQuery.separation > clipSeparation + linearSlop)
		) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= edgeQuery.indexA && edgeQuery.indexA < 3);
			let trianglePoint = trianglePoints[edgeQuery.indexA];
			let triangleEdge = triangleEdges[edgeQuery.indexA];
			manifold.pointCount = 0;
			b3CollideTriangleAndHullEdges(manifold, capacity, trianglePoint, triangleEdge, hullB, edgeQuery, cache);
		}
	}

	// Using the speculative distance means that sometimes there are no valid contact points from SAT.
	// In this fall back to GJK. This is important to prevent tunneling in rare cases.
	if (manifold.pointCount == 0) {
		let triangleB = s_trianglePoints;
		triangleB[0] = v1;
		triangleB[1] = v2;
		triangleB[2] = v3;
		let input = s_distanceInput;
		input.proxyA.points = triangleB;
		input.proxyA.count = 3;
		input.proxyA.radius = 0.0;
		input.proxyB.points = hullPoints;
		input.proxyB.count = hullB.vertexCount;
		input.proxyB.radius = 0.0;
		input.transform = b3Transform_identity;
		input.useRadii = false;

		let simplexCache = b3SimplexCache();
		let output = b3ShapeDistance(input, simplexCache, null, 0);

		if (output.distance > 0.0) {
			if (B3_ASSERT_ENABLED) B3_ASSERT(0 < simplexCache.count && simplexCache.count <= 3);

			manifold.pointCount = 1;
			manifold.feature = b3GetTriangleFeature(simplexCache);
			manifold.normal = output.normal;
			ref pt = manifold.points[manifold.pointOffset + 0];
			pt.point = output.pointB;
			pt.separation = output.distance;

			// This feature pair not accurate but maybe it doesn't matter
			pt.pair = b3FeaturePair_single;
		}

		// No way to cache this scenario
		cache = b3SATCache();
	}
}
