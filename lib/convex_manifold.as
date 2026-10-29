// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/convex_manifold.c and src/convex_manifold_wide.inl (width 4)

import { B3_ASSERT, B3_ASSERT_ENABLED, B3_NULL_INDEX, FLT_EPSILON, FLT_MAX, FLT_MIN } from "./core";
import {
	B3_LINEAR_SLOP,
	B3_MAX_HULL_EDGES,
	B3_MAX_HULL_FACES,
	B3_MAX_HULL_VERTICES,
	B3_MAX_WORKERS,
	B3_MIN_CAPSULE_LENGTH,
	B3_PARALLEL_EDGE_TOL,
	B3_SPECULATIVE_DISTANCE,
} from "./constants";
import {
	b3AABB_Center,
	b3AABB_Extents,
	b3Abs,
	b3AbsFloat,
	b3Add,
	b3Cross,
	b3Distance,
	b3Dot,
	b3GetLengthAndNormalize,
	b3InvRotateVector,
	b3InvTransformPoint,
	b3InvertTransform,
	b3Length,
	b3LengthSquared,
	b3Lerp,
	b3LineDistance,
	b3MakeMatrixFromQuat,
	b3Matrix3,
	b3MaxFloat,
	b3MinFloat,
	b3MinInt,
	b3MulAdd,
	b3MulMV,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Normalize,
	b3Plane,
	b3PointToSegmentDistance,
	b3RotateVector,
	b3SegmentDistance,
	b3Sub,
	b3Transform,
	b3TransformPoint,
	b3Transform_identity,
	b3Transpose,
	b3Vec3,
	b3Vec3_zero,
} from "./math_functions";
import { b3ArbitraryPerp, b3IsWithinSegments, b3MakePlaneFromNormalAndPoint, b3PlaneSeparation } from "./math_internal";
import {
	b3Capsule,
	b3DistanceInput,
	b3GetHullEdgeCosines,
	b3GetHullEdges,
	b3GetHullFaces,
	b3GetHullPlanes,
	b3GetHullPoints,
	b3GetHullSoaNormals,
	b3GetHullSoaVertices,
	b3HullData,
	b3LocalManifold,
	b3LocalManifoldPoint,
	b3SATCache,
	b3SimplexCache,
	b3Sphere,
	b3_edgePairAxis,
	b3_faceAxisA,
	b3_faceAxisB,
	b3_invalidAxis,
	b3_manualEdgePairAxis,
	b3_manualFaceAxisA,
	b3_manualFaceAxisB,
} from "./collision";
import { b3GetPointSupport, b3ShapeDistance } from "./distance";
import { b3FindHullSupportFace, b3FindHullSupportVertex, b3GetHullSoaStride } from "./hull";
import {
	B3_MAX_CLIP_POINTS,
	b3AxisQuery,
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
import {
	B3_SIMD_WIDTH,
	b3AddVW,
	b3AndW,
	b3AnyTrueW,
	b3BlendW,
	b3DivW,
	b3DotW,
	b3EmbedIndexW,
	b3GetSIMDWidth,
	b3GreaterThanW,
	b3LessThanW,
	b3LoadVW,
	b3LoadW,
	b3MaxW,
	b3MinIndexW,
	b3MinW,
	b3MulAddSVW,
	b3MulSVW,
	b3MulW,
	b3NegVW,
	b3NegW,
	b3SplatVW,
	b3SplatW,
	b3SqrtW,
	b3StoreVW,
	b3StoreW,
	b3SubVW,
	b3SubW,
	b3Vec3W,
	b3ZeroW,
} from "./simd";

const FLT_INF: f32 = reinterpret<f32>(0x7f800000);

// Temporary abbreviations for convenience.
const NE: i32 = B3_MAX_HULL_EDGES + B3_SIMD_WIDTH;
const NF: i32 = B3_MAX_HULL_FACES + B3_SIMD_WIDTH;
const NV: i32 = B3_MAX_HULL_VERTICES + B3_SIMD_WIDTH;

// Scratch for the C stack buffers, one set per worker since parallel parts collide at once. No function here holds
// one across a call that uses it again.
class b3ConvexScratch {
	segment: StaticArray<b3ClipVertex> = new StaticArray<b3ClipVertex>(2);
	capsulePoints: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(2);
	// Twice the C size, since clipping a 64 vertex incident face can overflow the C buffer
	clipBuffer1: StaticArray<b3ClipVertex> = new StaticArray<b3ClipVertex>(2 * B3_MAX_CLIP_POINTS);
	clipBuffer2: StaticArray<b3ClipVertex> = new StaticArray<b3ClipVertex>(2 * B3_MAX_CLIP_POINTS);
	clipPoints: StaticArray<b3LocalManifoldPoint> = new StaticArray<b3LocalManifoldPoint>(B3_MAX_CLIP_POINTS);
	edgeManifold: b3LocalManifold = new b3LocalManifold();
	edgePoint: StaticArray<b3LocalManifoldPoint> = new StaticArray<b3LocalManifoldPoint>(1);
	distanceInput: b3DistanceInput = new b3DistanceInput();
	// The proxies own their points, since parallel code may not point a proxy at a hull's
	distancePointsA: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(B3_MAX_HULL_VERTICES);
	distancePointsB: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(2);

	// The C stack arrays of the separating axis test
	dotA: StaticArray<f32> = new StaticArray<f32>(NF);
	dotB: StaticArray<f32> = new StaticArray<f32>(NF);
	planeDotA: StaticArray<f32> = new StaticArray<f32>(NF);
	planeDotB: StaticArray<f32> = new StaticArray<f32>(NF);
	edgeIndicesA: StaticArray<i32> = new StaticArray<i32>(NE);
	edgeIndicesB: StaticArray<i32> = new StaticArray<i32>(B3_MAX_HULL_EDGES);
	bFNx: StaticArray<f32> = new StaticArray<f32>(NF);
	bFNy: StaticArray<f32> = new StaticArray<f32>(NF);
	bFNz: StaticArray<f32> = new StaticArray<f32>(NF);
	bWx: StaticArray<f32> = new StaticArray<f32>(NV);
	bWy: StaticArray<f32> = new StaticArray<f32>(NV);
	bWz: StaticArray<f32> = new StaticArray<f32>(NV);
	aN0x: StaticArray<f32> = new StaticArray<f32>(NE);
	aN0y: StaticArray<f32> = new StaticArray<f32>(NE);
	aN0z: StaticArray<f32> = new StaticArray<f32>(NE);
	aN1x: StaticArray<f32> = new StaticArray<f32>(NE);
	aN1y: StaticArray<f32> = new StaticArray<f32>(NE);
	aN1z: StaticArray<f32> = new StaticArray<f32>(NE);
	aDx: StaticArray<f32> = new StaticArray<f32>(NE);
	aDy: StaticArray<f32> = new StaticArray<f32>(NE);
	aDz: StaticArray<f32> = new StaticArray<f32>(NE);
	aV0x: StaticArray<f32> = new StaticArray<f32>(NE);
	aV0y: StaticArray<f32> = new StaticArray<f32>(NE);
	aV0z: StaticArray<f32> = new StaticArray<f32>(NE);
	aTol: StaticArray<f32> = new StaticArray<f32>(NE);
	sA: StaticArray<f32> = new StaticArray<f32>(B3_SIMD_WIDTH);
	nxA: StaticArray<f32> = new StaticArray<f32>(B3_SIMD_WIDTH);
	nyA: StaticArray<f32> = new StaticArray<f32>(B3_SIMD_WIDTH);
	nzA: StaticArray<f32> = new StaticArray<f32>(B3_SIMD_WIDTH);

	constructor() {
		this.edgeManifold.points = this.edgePoint;
		this.distanceInput.proxyA.points = this.distancePointsA;
		this.distanceInput.proxyB.points = this.distancePointsB;
	}
}

function b3MakeConvexScratch(): StaticArray<b3ConvexScratch> {
	let scratch = new StaticArray<b3ConvexScratch>(B3_MAX_WORKERS);
	for (let i = 0; i < B3_MAX_WORKERS; ++i) {
		scratch[i] = new b3ConvexScratch();
	}
	return scratch;
}

// One per worker, and each part uses its worker's
@parallel.shared const s_convexScratch = b3MakeConvexScratch();

function b3CopyHullPoints(dst: StaticArray<b3Vec3>, hull: b3HullData): void {
	let points = b3GetHullPoints(hull);
	let count = hull.vertexCount;
	for (let i = 0; i < count; ++i) {
		dst[i] = points[i];
	}
}

function b3ClipSegment(segment: StaticArray<b3ClipVertex>, plane: b3Plane): i32 {
	let vertexCount = 0;
	let vertex1 = segment[0];
	let vertex2 = segment[1];

	let distance1 = b3PlaneSeparation(plane, vertex1.position);
	let distance2 = b3PlaneSeparation(plane, vertex2.position);

	// If the points are behind the plane
	if (distance1 <= 0.0) {
		segment[vertexCount++] = vertex1;
	}
	if (distance2 <= 0.0) {
		segment[vertexCount++] = vertex2;
	}

	// If the points are on different sides of the plane
	if (distance1 * distance2 < 0.0) {
		// Find intersection point of edge and plane
		let t = distance1 / (distance1 - distance2);
		segment[vertexCount].position = b3Add(b3MulSV(1.0 - t, vertex1.position), b3MulSV(t, vertex2.position));
		segment[vertexCount].pair = distance1 > 0.0 ? vertex1.pair : vertex2.pair;
		vertexCount++;
	}

	return vertexCount;
}

function b3ClipSegmentToHullFace(segment: StaticArray<b3ClipVertex>, hull: b3HullData, refFace: i32): i32 {
	let faces = b3GetHullFaces(hull);
	let planes = b3GetHullPlanes(hull);
	let edges = b3GetHullEdges(hull);
	let points = b3GetHullPoints(hull);

	let refPlane = planes[refFace];

	let faceEdge = <i32>faces[refFace].edge;

	let edgeIndex = faceEdge;

	do {
		let edge = edges[edgeIndex];
		let nextEdgeIndex = <i32>edge.next;
		let next = edges[nextEdgeIndex];

		let vertex1 = points[edge.origin];
		let vertex2 = points[next.origin];
		let tangent = b3Normalize(b3Sub(vertex2, vertex1));
		let binormal = b3Cross(tangent, refPlane.normal);

		let pointCount = b3ClipSegment(segment, b3MakePlaneFromNormalAndPoint(binormal, vertex1));
		if (pointCount < 2) {
			return 0;
		}

		edgeIndex = nextEdgeIndex;
	} while (edgeIndex != faceEdge);

	return 2;
}

function b3QueryFaceDirectionHullAndCapsule(
	hull: b3HullData,
	capsule: b3Capsule,
	capsuleTransform: b3Transform,
	workerIndex: i32,
): b3SeparatingAxis {
	let scratch = s_convexScratch[workerIndex];
	let maxFaceIndex = -1;
	let maxVertexIndex = -1;
	let maxFaceSeparation = -FLT_MAX;
	let planes = b3GetHullPlanes(hull);

	let capsulePoints = scratch.capsulePoints;
	capsulePoints[0] = b3TransformPoint(capsuleTransform, capsule.center1);
	capsulePoints[1] = b3TransformPoint(capsuleTransform, capsule.center2);

	for (let faceIndex = 0; faceIndex < hull.faceCount; ++faceIndex) {
		let plane = planes[faceIndex];

		let vertexIndex = b3GetPointSupport(capsulePoints, 2, b3Neg(plane.normal));
		let support = capsulePoints[vertexIndex];
		let separation = b3PlaneSeparation(plane, support);
		if (separation > maxFaceSeparation) {
			maxVertexIndex = vertexIndex;
			maxFaceIndex = faceIndex;
			maxFaceSeparation = separation;
		}
	}

	let axis: b3SeparatingAxis;
	axis.normal = planes[maxFaceIndex].normal;
	axis.separation = maxFaceSeparation;
	axis.indexA = <i32>(<u8>maxFaceIndex);
	axis.indexB = <i32>(<u8>maxVertexIndex);
	return axis;
}

function b3QueryEdgeDirectionHullAndCapsule(
	hull: b3HullData,
	capsule: b3Capsule,
	capsuleTransform: b3Transform,
): b3SeparatingAxis {
	// Find axis of minimum penetration
	let maxNormal = b3Vec3_zero;
	let maxSeparation = -FLT_MAX;
	let maxIndexA = B3_NULL_INDEX;
	let maxIndexB = B3_NULL_INDEX;

	// We perform all computations in local space of the hull
	let pA = b3TransformPoint(capsuleTransform, capsule.center1);
	let qA = b3TransformPoint(capsuleTransform, capsule.center2);
	let eA = b3Sub(qA, pA);

	let edges = b3GetHullEdges(hull);
	let points = b3GetHullPoints(hull);
	let planes = b3GetHullPlanes(hull);
	let squaredTolerance = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;

	for (let index = 0; index < hull.edgeCount; index += 2) {
		let edge = edges[index];
		let twin = edges[index + 1];
		if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edge.twin == index + 1 && <i32>twin.twin == index);

		let qB = points[twin.origin];
		let uB = planes[edge.face].normal;
		let vB = planes[twin.face].normal;

		// An isolated edge (e.g. like in a capsule) defines a circle through the
		// origin on the Gauss map. So testing for overlap between this circle and
		// the arc AB simplifies to a plane test.
		let cba = b3Dot(uB, eA);
		let dba = b3Dot(vB, eA);

		if (cba * dba < 0.0) {
			// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
			if (b3MaxFloat(cba * cba, dba * dba) < squaredTolerance * b3LengthSquared(eA)) {
				continue;
			}

			// The intersection of the arcs on the Gauss map is the edge pair axis. Cast the
			// arc of hull B (from uB to vB) against the plane containing the arc of hull A:
			// dot(uB + t * (vB - uB), eA) == 0
			// then
			// t = cba / (cba - dba)
			//
			// The signs of cba and dba differ (Minkowski test), so the division is safe.
			//
			// The axis generated points from B to A by construction since it lands between
			// two face normals on B. This removes the need to orient the separation axis
			// using the hull centers.
			//
			// The axis is perpendicular to both edges so I can use qA and qB as arbitrary
			// points on edgeA and edgeB to measure the separation.

			let t = cba / (cba - dba);
			let axis = b3Lerp(uB, vB, t);
			axis = b3Normalize(axis);
			let separation = b3Dot(axis, b3Sub(qA, qB));

			if (separation > maxSeparation) {
				// Note: We don't exit early if we find a separating axis here since we want to
				// find the best one for caching and account for the convex radius later.
				maxNormal = axis;
				maxSeparation = separation;
				maxIndexA = 0;
				maxIndexB = index;
			}
		}
	}

	// Save result
	let result: b3SeparatingAxis;
	result.normal = maxNormal;
	result.separation = maxSeparation;
	result.indexA = maxIndexA;
	result.indexB = maxIndexB;
	return result;
}

export function b3CollideSpheres(
	manifold: b3LocalManifold,
	capacity: i32,
	sphereA: b3Sphere,
	sphereB: b3Sphere,
	transformBtoA: b3Transform,
): void {
	if (capacity == 0) {
		return;
	}

	// Work in shapeB coordinates
	let center1 = sphereA.center;
	let center2 = b3TransformPoint(transformBtoA, sphereB.center);

	let totalRadius = sphereA.radius + sphereB.radius;
	let maxDistance = totalRadius + B3_SPECULATIVE_DISTANCE();
	let offset = b3Sub(center2, center1);
	let distanceSq = b3LengthSquared(offset);

	if (distanceSq > maxDistance * maxDistance) {
		// We found a separating axis
		return;
	}

	let normal = b3Vec3(0.0, 1.0, 0.0);
	let distance = Mathf.sqrt(distanceSq);
	if (distance * distance > 1000.0 * FLT_MIN) {
		normal = b3MulSV(1.0 / distance, offset);
	}

	// Contact at the midpoint
	// 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
	let point = b3MulSV(
		0.5,
		b3MulSub(b3Add(b3MulAdd(center1, sphereA.radius, normal), center2), sphereB.radius, normal),
	);

	// Manifold in frame B
	manifold.normal = normal;
	manifold.pointCount = 1;

	ref pt = manifold.points[manifold.pointOffset + 0];
	pt.point = point;
	pt.separation = distance - totalRadius;
	pt.pair = b3FeaturePair_single;
}

export function b3CollideCapsuleAndSphere(
	manifold: b3LocalManifold,
	capacity: i32,
	capsuleA: b3Capsule,
	sphereB: b3Sphere,
	transformBtoA: b3Transform,
): void {
	manifold.pointCount = 0;

	if (capacity < 1) {
		return;
	}

	// Work in shape B coordinates
	let center = b3TransformPoint(transformBtoA, sphereB.center);
	let center1 = capsuleA.center1;
	let center2 = capsuleA.center2;

	let totalRadius = sphereB.radius + capsuleA.radius;
	let maxDistance = totalRadius + B3_SPECULATIVE_DISTANCE();

	let closestPoint = b3PointToSegmentDistance(center1, center2, center);
	let offset = b3Sub(center, closestPoint);
	let distanceSq = b3LengthSquared(offset);

	if (distanceSq > maxDistance * maxDistance) {
		// We found a separating axis
		return;
	}

	let normal = b3Vec3(0.0, 1.0, 0.0);
	let distance = Mathf.sqrt(distanceSq);
	if (distance * distance > 1000.0 * FLT_MIN) {
		normal = b3MulSV(1.0 / distance, offset);
	}

	// Contact at the midpoint
	// 0.5 * (((center - sB*n) + closestPoint) + cA*n)
	let point = b3MulSV(
		0.5,
		b3MulAdd(b3Add(b3MulSub(center, sphereB.radius, normal), closestPoint), capsuleA.radius, normal),
	);

	// Manifold in frame B
	manifold.normal = normal;
	manifold.pointCount = 1;

	ref pt = manifold.points[manifold.pointOffset + 0];
	pt.point = point;
	pt.separation = distance - totalRadius;
	pt.pair = b3FeaturePair_single;
}

export function b3CollideHullAndSphere(
	manifold: b3LocalManifold,
	capacity: i32,
	hullA: b3HullData,
	sphereB: b3Sphere,
	transformBtoA: b3Transform,
	cache: ref b3SimplexCache,
	workerIndex: i32 = 0,
): void {
	let scratch = s_convexScratch[workerIndex];
	manifold.pointCount = 0;

	if (capacity == 0) {
		return;
	}

	let center = b3TransformPoint(transformBtoA, sphereB.center);

	let speculativeDistance = B3_SPECULATIVE_DISTANCE();

	// Work in shapeA coordinates

	let distanceInput = scratch.distanceInput;
	b3CopyHullPoints(scratch.distancePointsA, hullA);
	distanceInput.proxyA.count = hullA.vertexCount;
	distanceInput.proxyA.radius = 0.0;
	scratch.distancePointsB[0] = center;
	distanceInput.proxyB.count = 1;
	distanceInput.proxyB.radius = 0.0;
	distanceInput.transform = b3Transform_identity;
	distanceInput.useRadii = false;

	let radiusA: f32 = 0.0;
	let radiusB = sphereB.radius;
	let radius = radiusA + radiusB;

	let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0, workerIndex);

	if (distanceOutput.distance > radius + speculativeDistance) {
		// We found a separating axis
		cache = b3SimplexCache();
		return;
	}

	if (distanceOutput.distance > 100.0 * FLT_EPSILON) {
		// Shallow penetration
		let normal = b3Normalize(b3Sub(distanceOutput.pointB, distanceOutput.pointA));

		// cA is the projection of the sphere center onto to the hull (pointA if radiusA == 0).
		let cA = b3MulAdd(center, radiusA - b3Dot(b3Sub(center, distanceOutput.pointA), normal), normal);

		// cB is the deepest point on the sphere with respect to the reference f
		let cB = b3MulSub(center, radiusB, normal);

		let point = b3Lerp(cA, cB, 0.5);

		// Manifold in frame A
		manifold.normal = normal;
		manifold.pointCount = 1;

		ref pt = manifold.points[manifold.pointOffset + 0];
		pt.point = point;
		pt.separation = distanceOutput.distance - radius;
		pt.pair = b3FeaturePair_single;
	} else {
		// Deep penetration
		let bestIndex = -1;
		let bestDistance = -FLT_MAX;
		let planes = b3GetHullPlanes(hullA);

		for (let index = 0; index < hullA.faceCount; ++index) {
			let plane = planes[index];

			let distance = b3PlaneSeparation(plane, center);
			if (distance > bestDistance) {
				bestIndex = index;
				bestDistance = distance;
			}
		}
		if (B3_ASSERT_ENABLED) B3_ASSERT(bestIndex >= 0);

		let normal = planes[bestIndex].normal;

		// cA is the projection of the sphere center onto to the hull
		let cA = b3MulAdd(center, radiusA - b3Dot(b3Sub(center, distanceOutput.pointA), normal), normal);

		// cB is the deepest point on the sphere with respect to the reference f
		let cB = b3MulSub(center, radiusB, normal);

		let point = b3Lerp(cA, cB, 0.5);

		// Manifold in frame A
		manifold.normal = normal;
		manifold.pointCount = 1;

		ref pt = manifold.points[manifold.pointOffset + 0];
		pt.point = point;
		pt.separation = bestDistance - radius;
		pt.pair = b3FeaturePair_single;
	}
}

export function b3CollideCapsules(
	manifold: b3LocalManifold,
	capacity: i32,
	capsuleA: b3Capsule,
	capsuleB: b3Capsule,
	transformBtoA: b3Transform,
	workerIndex: i32 = 0,
): void {
	let scratch = s_convexScratch[workerIndex];
	manifold.pointCount = 0;

	if (capacity < 2) {
		return;
	}

	// Work in shapeA coordinates
	let centerA1 = capsuleA.center1;
	let centerA2 = capsuleA.center2;
	let centerB1 = b3TransformPoint(transformBtoA, capsuleB.center1);
	let centerB2 = b3TransformPoint(transformBtoA, capsuleB.center2);

	let radius = capsuleA.radius + capsuleB.radius;
	let maxDistance = radius + B3_SPECULATIVE_DISTANCE();

	let result = b3SegmentDistance(centerA1, centerA2, centerB1, centerB2);
	let offset = b3Sub(result.point2, result.point1);
	let distanceSquared = b3LengthSquared(offset);
	let linearSlop = B3_LINEAR_SLOP();
	let minDistance = 0.01 * linearSlop;

	if (distanceSquared > maxDistance * maxDistance || distanceSquared < minDistance * minDistance) {
		// We found a separating axis
		return;
	}

	// lengthA, lengthB, distance
	let lengths: FixedArray<f32, 3>;

	let segmentA = b3Sub(centerA2, centerA1);
	let edgeA = b3GetLengthAndNormalize(lengths[0], segmentA);
	if (lengths[0] < B3_MIN_CAPSULE_LENGTH()) {
		return;
	}

	let segmentB = b3Sub(centerB2, centerB1);
	let edgeB = b3GetLengthAndNormalize(lengths[1], segmentB);
	if (lengths[1] < B3_MIN_CAPSULE_LENGTH()) {
		return;
	}

	// Parallel edges: |eA x eB| = sin(alpha)
	let alphaTol: f32 = 0.05;
	let alphaTolSqr = alphaTol * alphaTol;
	let axis = b3Cross(edgeA, edgeB);

	// Try to create two contact points if the capsules are nearly parallel
	if (b3LengthSquared(axis) < alphaTolSqr) {
		// Clip segment B against side planes of segment A

		// Sides planes of A
		let planeA0: b3Plane;
		planeA0.normal = b3Neg(edgeA);
		planeA0.offset = -b3Dot(edgeA, capsuleA.center1);
		let planeA1: b3Plane;
		planeA1.normal = edgeA;
		planeA1.offset = b3Dot(edgeA, capsuleA.center2);

		// Clip points for B
		let verticesB = scratch.segment;
		verticesB[0].position = centerB1;
		verticesB[0].separation = 0.0;
		verticesB[0].pair = b3MakeFeaturePair(b3_featureShapeA, 0, b3_featureShapeA, 0);
		verticesB[1].position = centerB2;
		verticesB[1].separation = 0.0;
		verticesB[1].pair = b3MakeFeaturePair(b3_featureShapeA, 1, b3_featureShapeA, 1);

		let pointCount = b3ClipSegment(verticesB, planeA0);
		if (pointCount == 2) {
			pointCount = b3ClipSegment(verticesB, planeA1);
		}

		if (pointCount == 2) {
			// Closest points on A to the clipped points on B.
			let closestPoint1 = b3PointToSegmentDistance(centerA1, centerA2, verticesB[0].position);
			let closestPoint2 = b3PointToSegmentDistance(centerA1, centerA2, verticesB[1].position);

			let distance1 = b3Distance(closestPoint1, verticesB[0].position);
			let distance2 = b3Distance(closestPoint2, verticesB[1].position);
			if (distance1 <= radius && distance2 <= radius) {
				if (distance1 < minDistance || distance2 < minDistance) {
					// Avoid divide by zero
					return;
				}

				let normal1 = b3MulSV(1.0 / distance1, b3Sub(verticesB[0].position, closestPoint1));
				let normal2 = b3MulSV(1.0 / distance2, b3Sub(verticesB[1].position, closestPoint2));
				let normal = b3Normalize(b3Add(normal1, normal2));
				let radiusA = capsuleA.radius;
				let radiusB = capsuleB.radius;

				// Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
				let point1 = b3MulSV(
					0.5,
					b3MulSub(b3Add(b3MulAdd(verticesB[0].position, radiusA, normal1), closestPoint1), radiusB, normal),
				);
				let point2 = b3MulSV(
					0.5,
					b3MulSub(b3Add(b3MulAdd(verticesB[1].position, radiusA, normal2), closestPoint2), radiusB, normal),
				);

				// Manifold in frame A
				manifold.normal = normal;
				manifold.pointCount = 2;

				let base = manifold.pointOffset;
				ref pt1 = manifold.points[base + 0];
				pt1.point = point1;
				pt1.separation = distance1 - radius;
				pt1.pair = verticesB[0].pair;

				ref pt2 = manifold.points[base + 1];
				pt2.point = point2;
				pt2.separation = distance2 - radius;
				pt2.pair = verticesB[1].pair;

				return;
			}
		}
	}

	let normal = b3GetLengthAndNormalize(lengths[2], offset);
	let distance = lengths[2];
	// Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
	let point = b3MulSV(
		0.5,
		b3MulSub(b3Add(b3MulAdd(result.point1, capsuleA.radius, normal), result.point2), capsuleB.radius, normal),
	);

	// Manifold in frame A
	manifold.normal = normal;
	manifold.pointCount = 1;

	ref pt = manifold.points[manifold.pointOffset + 0];
	pt.point = point;
	pt.separation = distance - radius;
	pt.pair = b3FeaturePair_single;
}

function b3BuildHullFaceAndCapsuleContact(
	manifold: b3LocalManifold,
	hullA: b3HullData,
	capsuleB: b3Capsule,
	transformBtoA: b3Transform,
	query: b3SeparatingAxis,
	workerIndex: i32,
): bool {
	let scratch = s_convexScratch[workerIndex];
	// Work in shapeA coordinates
	let planes = b3GetHullPlanes(hullA);

	// Clip the capsule edge against the side planes of the reference face
	let refFace = query.indexA;
	let refPlane = planes[refFace];

	let segmentB = scratch.segment;
	segmentB[0].position = b3TransformPoint(transformBtoA, capsuleB.center1);
	segmentB[0].separation = 0.0;
	segmentB[0].pair = b3MakeFeaturePair(b3_featureShapeA, 0, b3_featureShapeA, 0);
	segmentB[1].position = b3TransformPoint(transformBtoA, capsuleB.center2);
	segmentB[1].separation = 0.0;
	segmentB[1].pair = b3MakeFeaturePair(b3_featureShapeA, 1, b3_featureShapeA, 1);

	let pointCount = b3ClipSegmentToHullFace(segmentB, hullA, refFace);
	if (pointCount < 2) {
		return false;
	}

	let distance1 = b3PlaneSeparation(refPlane, segmentB[0].position);
	let distance2 = b3PlaneSeparation(refPlane, segmentB[1].position);
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();

	if (distance1 <= speculativeDistance || distance2 <= speculativeDistance) {
		let normal = refPlane.normal;
		let point1 = b3MulSub(segmentB[0].position, 0.5 * (distance1 + capsuleB.radius), normal);
		let point2 = b3MulSub(segmentB[1].position, 0.5 * (distance2 + capsuleB.radius), normal);

		// Manifold in frame A
		manifold.normal = normal;
		manifold.pointCount = 2;

		let base = manifold.pointOffset;
		ref pt1 = manifold.points[base + 0];
		pt1.point = point1;
		pt1.separation = distance1 - capsuleB.radius;
		pt1.pair = segmentB[0].pair;

		ref pt2 = manifold.points[base + 1];
		pt2.point = point2;
		pt2.separation = distance2 - capsuleB.radius;
		pt2.pair = segmentB[1].pair;

		return true;
	}

	return false;
}

function b3BuildHullAndCapsuleEdgeContact(
	manifold: b3LocalManifold,
	capacity: i32,
	hullA: b3HullData,
	capsuleB: b3Capsule,
	transformBtoA: b3Transform,
	query: b3SeparatingAxis,
): bool {
	if (capacity < 1) {
		return false;
	}

	// Work in shapeA coordinates

	let pc = b3TransformPoint(transformBtoA, capsuleB.center1);
	let qc = b3TransformPoint(transformBtoA, capsuleB.center2);
	let ec = b3Sub(qc, pc);

	let edges = b3GetHullEdges(hullA);
	let points = b3GetHullPoints(hullA);

	let edge2 = edges[query.indexB];
	let twin2 = edges[edge2.twin];
	let ph = points[edge2.origin];
	let qh = points[twin2.origin];
	let eh = b3Sub(qh, ph);

	let normal = query.normal;

	let result = b3LineDistance(ph, eh, pc, ec);

	if (b3IsWithinSegments(result) == false) {
		// closest point beyond end points
		return false;
	}

	let point = b3MulSV(0.5, b3Add(b3MulSub(result.point1, capsuleB.radius, normal), result.point2));
	let separation = b3Dot(normal, b3Sub(result.point2, result.point1));

	// Manifold in frame A
	manifold.normal = normal;
	manifold.pointCount = 1;

	ref pt = manifold.points[manifold.pointOffset + 0];
	pt.point = point;
	pt.separation = separation - capsuleB.radius;
	pt.pair = b3MakeFeaturePair(b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB);
	return true;
}

export function b3CollideHullAndCapsule(
	manifold: b3LocalManifold,
	capacity: i32,
	hullA: b3HullData,
	capsuleB: b3Capsule,
	transformBtoA: b3Transform,
	cache: ref b3SimplexCache,
	workerIndex: i32 = 0,
): void {
	let scratch = s_convexScratch[workerIndex];
	manifold.pointCount = 0;

	if (capacity < 2) {
		return;
	}

	// Work in shapeA coordinates
	let distanceInput = scratch.distanceInput;
	b3CopyHullPoints(scratch.distancePointsA, hullA);
	distanceInput.proxyA.count = hullA.vertexCount;
	distanceInput.proxyA.radius = 0.0;
	scratch.distancePointsB[0] = capsuleB.center1;
	scratch.distancePointsB[1] = capsuleB.center2;
	distanceInput.proxyB.count = 2;
	distanceInput.proxyB.radius = 0.0;
	distanceInput.transform = transformBtoA;
	distanceInput.useRadii = false;

	let distanceOutput = b3ShapeDistance(distanceInput, cache, null, 0, workerIndex);
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();

	if (distanceOutput.distance > capsuleB.radius + speculativeDistance) {
		// We found a separating axis
		cache = b3SimplexCache();
		return;
	}

	if (distanceOutput.distance > 100.0 * FLT_EPSILON) {
		let planes = b3GetHullPlanes(hullA);

		// Shallow penetration
		let delta = distanceOutput.normal;
		let refFace = b3FindHullSupportFace(hullA, delta);
		let refPlane = planes[refFace];

		// Try to create two contact points if closest
		// points difference is nearly parallel to face normal
		let kTolerance: f32 = 0.998;
		if (b3AbsFloat(b3Dot(refPlane.normal, delta)) > kTolerance) {
			// Clip capsule segment against side planes of reference face
			let verticesB = scratch.segment;
			verticesB[0].position = b3TransformPoint(transformBtoA, capsuleB.center1);
			verticesB[0].separation = 0.0;
			verticesB[0].pair = b3MakeFeaturePair(b3_featureShapeA, 0, b3_featureShapeA, 0);
			verticesB[1].position = b3TransformPoint(transformBtoA, capsuleB.center2);
			verticesB[1].separation = 0.0;
			verticesB[1].pair = b3MakeFeaturePair(b3_featureShapeA, 1, b3_featureShapeA, 1);

			let pointCount = b3ClipSegmentToHullFace(verticesB, hullA, refFace);

			if (pointCount == 2) {
				let distance1 = b3PlaneSeparation(refPlane, verticesB[0].position);
				let distance2 = b3PlaneSeparation(refPlane, verticesB[1].position);
				if (
					distance1 <= capsuleB.radius + speculativeDistance ||
					distance2 <= capsuleB.radius + speculativeDistance
				) {
					let normal = refPlane.normal;
					let point1 = b3MulSub(verticesB[0].position, 0.5 * (capsuleB.radius + distance1), normal);
					let point2 = b3MulSub(verticesB[1].position, 0.5 * (capsuleB.radius + distance2), normal);

					// Manifold in frame A
					manifold.normal = normal;
					manifold.pointCount = 2;

					let base = manifold.pointOffset;
					ref pt1 = manifold.points[base + 0];
					pt1.point = point1;
					pt1.separation = distance1 - capsuleB.radius;
					pt1.pair = verticesB[0].pair;

					ref pt2 = manifold.points[base + 1];
					pt2.point = point2;
					pt2.separation = distance2 - capsuleB.radius;
					pt2.pair = verticesB[1].pair;

					return;
				}
			}
		}

		// Create contact from closest points
		let point = b3MulSV(0.5, b3Add(b3MulSub(distanceOutput.pointA, capsuleB.radius, delta), distanceOutput.pointB));

		// Manifold in frame A
		manifold.normal = delta;
		manifold.pointCount = 1;

		ref pt = manifold.points[manifold.pointOffset + 0];
		pt.point = point;
		pt.separation = distanceOutput.distance - capsuleB.radius;
		pt.pair = b3FeaturePair_single;
		return;
	}

	// Deep penetration

	let faceQuery = b3QueryFaceDirectionHullAndCapsule(hullA, capsuleB, transformBtoA, workerIndex);
	if (faceQuery.separation > capsuleB.radius) {
		// We found a separating axis
		return;
	}

	let edgeQuery = b3QueryEdgeDirectionHullAndCapsule(hullA, capsuleB, transformBtoA);
	if (edgeQuery.separation > capsuleB.radius) {
		// We found a separating axis
		return;
	}

	// Create face contact
	let faceSeparation = faceQuery.separation - capsuleB.radius;
	b3BuildHullFaceAndCapsuleContact(manifold, hullA, capsuleB, transformBtoA, faceQuery, workerIndex);
	if (manifold.pointCount == 2) {
		// This becomes the clipped separation.
		let base = manifold.pointOffset;
		faceSeparation = b3MinFloat(manifold.points[base + 0].separation, manifold.points[base + 1].separation);
	}

	// Is there a valid edge-edge axis?
	if (edgeQuery.indexA == B3_NULL_INDEX) {
		return;
	}

	// Face contact can be empty if it does not realize the axis of minimum penetration.
	// Create edge contact if face contact fails or edge contact is significantly better!
	let linearSlop = B3_LINEAR_SLOP();
	let edgeSeparation = edgeQuery.separation - capsuleB.radius;
	if (manifold.pointCount == 0 || edgeSeparation > faceSeparation + linearSlop) {
		// Edge contact
		b3BuildHullAndCapsuleEdgeContact(manifold, capacity, hullA, capsuleB, transformBtoA, edgeQuery);
	}
}

// B3_MAX_MANIFOLD_POINTS is 4, so only the 4 point reduction is compiled, as in C.

// Reduce the manifold points to a maximum of 4 points.
// Note: this modifies the input point array to improve performance
function b3ReduceManifoldPoints(
	manifold: b3LocalManifold,
	capacity: i32,
	points: StaticArray<b3LocalManifoldPoint>,
	count: i32,
): void {
	if (capacity < 4) {
		return;
	}

	let out = manifold.points;
	let base = manifold.pointOffset;

	if (count <= 4) {
		for (let i = 0; i < count; ++i) {
			out[base + i] = points[i];
		}

		manifold.pointCount = count;
		return;
	}

	let normal = manifold.normal;
	// float linearSlop = B3_LINEAR_SLOP;
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();
	let tolSqr = speculativeDistance * speculativeDistance;

	// This bias is very important for contact point consistency across time steps.
	// It creates a pecking order to avoid flickering between candidates with similar scores.
	let bias: f32 = 0.95;

	// Step 1: find extreme point that is touching
	let bestIndex = B3_NULL_INDEX;
	let bestScore = -FLT_MAX;

	// Arbitrary tangent direction
	// b3Vec3 perp1 = b3Perp( normal );
	// b3Vec3 perp2 = b3Cross( perp1, normal );
	// b3Vec3 searchDirection = -0.4535961214255773f * perp1 + 0.8912073600614354f * perp2;
	let searchDirection = b3ArbitraryPerp(normal);
	for (let index = 0; index < count; ++index) {
		let pt = points[index];

		if (pt.separation > speculativeDistance) {
			continue;
		}

		// The deeper the better
		let score = -pt.separation + b3Dot(searchDirection, pt.point);
		if (bias * score > bestScore) {
			bestIndex = index;
			bestScore = score;
		}
	}

	if (bestIndex == B3_NULL_INDEX) {
		manifold.pointCount = 0;
		return;
	}

	out[base + 0] = points[bestIndex];
	manifold.pointCount = 1;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	let a = out[base + 0].point;

	// Step 2: Find farthest point in 2D
	bestScore = 0.0;
	bestIndex = B3_NULL_INDEX;

	for (let index = 0; index < count; ++index) {
		let p = points[index].point;
		let d = b3Sub(p, a);
		let v = b3MulSub(d, b3Dot(d, normal), normal);
		let distanceSquared = b3LengthSquared(v);
		let separation = b3MaxFloat(0.0, -points[index].separation);
		let score = distanceSquared + 4.0 * separation * separation;
		if (bias * score > bestScore) {
			bestScore = score;
			bestIndex = index;
		}
	}

	if (bestScore < tolSqr) {
		return;
	}

	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= bestIndex && bestIndex < count);
	out[base + 1] = points[bestIndex];
	manifold.pointCount = 2;

	// Remove best point from array
	points[bestIndex] = points[count - 1];
	count -= 1;

	let b = out[base + 1].point;

	// Step 3: Find the point with the maximum triangular area
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	let bestSignedArea: f32 = 0.0;
	let ba = b3Sub(b, a);
	for (let index = 0; index < count; ++index) {
		let p = points[index].point;
		let signedArea = b3Dot(normal, b3Cross(ba, b3Sub(p, a)));
		let score = b3AbsFloat(signedArea);
		if (bias * score >= bestScore) {
			bestScore = score;
			bestIndex = index;
			bestSignedArea = signedArea;
		}
	}

	if (bestIndex == B3_NULL_INDEX) {
		return;
	}

	out[base + 2] = points[bestIndex];
	manifold.pointCount = 3;
	points[bestIndex] = points[count - 1];
	count -= 1;

	let c = out[base + 2].point;

	// Step 4: get the point that adds the most area outside the current triangle
	bestScore = tolSqr;
	bestIndex = B3_NULL_INDEX;
	let sign: f32 = bestSignedArea < 0.0 ? -1.0 : 1.0;
	for (let index = 0; index < count; ++index) {
		let p = points[index].point;
		let u1 = sign * b3Dot(normal, b3Cross(b3Sub(p, a), ba));
		let u2 = sign * b3Dot(normal, b3Cross(b3Sub(p, b), b3Sub(c, b)));
		let u3 = sign * b3Dot(normal, b3Cross(b3Sub(p, c), b3Sub(a, c)));
		let score = b3MaxFloat(u1, b3MaxFloat(u2, u3));

		if (bias * score > bestScore) {
			bestScore = score;
			bestIndex = index;
		}
	}

	if (bestIndex != B3_NULL_INDEX) {
		out[base + manifold.pointCount] = points[bestIndex];
		manifold.pointCount += 1;
	}
}

function b3BuildPolygon(
	out: StaticArray<b3ClipVertex>,
	transform: b3Transform,
	hull: b3HullData,
	incFace: i32,
	refPlane: b3Plane,
): i32 {
	let faces = b3GetHullFaces(hull);
	let edges = b3GetHullEdges(hull);
	let points = b3GetHullPoints(hull);

	let faceEdge = <i32>faces[incFace].edge;
	let edgeIndex = faceEdge;
	if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edges[edgeIndex].face == incFace);

	let outCount = 0;

	let matrix = b3MakeMatrixFromQuat(transform.q);

	do {
		let edge = edges[edgeIndex];

		let nextEdgeIndex = <i32>edge.next;
		let next = edges[nextEdgeIndex];

		let vertex: b3ClipVertex;
		vertex.position = b3Add(b3MulMV(matrix, points[next.origin]), transform.p);
		vertex.separation = b3PlaneSeparation(refPlane, vertex.position);
		vertex.pair = b3MakeFeaturePair(b3_featureShapeB, edgeIndex, b3_featureShapeB, nextEdgeIndex);

		out[outCount] = vertex;
		outCount += 1;

		edgeIndex = nextEdgeIndex;
	} while (edgeIndex != faceEdge && outCount < B3_MAX_CLIP_POINTS);

	return outCount;
}

function b3BuildFaceAContact(
	manifold: b3LocalManifold,
	capacity: i32,
	hullA: b3HullData,
	hullB: b3HullData,
	transformBtoA: b3Transform,
	query: b3SeparatingAxis,
	cache: ref b3SATCache,
	workerIndex: i32,
): bool {
	let scratch = s_convexScratch[workerIndex];
	if (B3_ASSERT_ENABLED) B3_ASSERT(capacity > 0);

	let facesA = b3GetHullFaces(hullA);
	let edgesA = b3GetHullEdges(hullA);
	let planesA = b3GetHullPlanes(hullA);
	let pointsA = b3GetHullPoints(hullA);

	// Reference face
	let refFace = query.indexA;
	let refPlane = planesA[refFace];

	// Find incident face
	let refNormalInB = b3InvRotateVector(transformBtoA.q, refPlane.normal);
	let incFace = b3FindIncidentFace(hullB, refNormalInB, query.indexB);

	// Build clip polygon from incident face in frame A
	let pointCount = b3BuildPolygon(scratch.clipBuffer1, transformBtoA, hullB, incFace, refPlane);

	// Clip incident face against side planes of reference face
	let input = scratch.clipBuffer1;
	let output = scratch.clipBuffer2;

	let faceEdge = <i32>facesA[refFace].edge;
	let edgeIndex = faceEdge;

	do {
		let edge = edgesA[edgeIndex];
		let nextEdgeIndex = <i32>edge.next;
		let next = edgesA[nextEdgeIndex];
		let vertex1 = pointsA[edge.origin];
		let vertex2 = pointsA[next.origin];
		let tangent = b3Normalize(b3Sub(vertex2, vertex1));
		let binormal = b3Cross(tangent, refPlane.normal);

		let clipPlane = b3MakePlaneFromNormalAndPoint(binormal, vertex1);

		pointCount = b3ClipPolygon(output, input, pointCount, clipPlane, edgeIndex, refPlane);
		if (B3_ASSERT_ENABLED) B3_ASSERT(pointCount <= B3_MAX_CLIP_POINTS);

		let swap = output;
		output = input;
		input = swap;

		if (pointCount < 3) {
			cache = b3SATCache();
			return false;
		}

		edgeIndex = nextEdgeIndex;
	} while (edgeIndex != faceEdge);

	pointCount = b3MinInt(pointCount, B3_MAX_CLIP_POINTS);

	let points = scratch.clipPoints;
	let minSeparation = FLT_MAX;

	manifold.normal = refPlane.normal;

	for (let i = 0; i < pointCount; ++i) {
		let clipPoint = input[i];
		ref pt = points[i];
		pt = b3LocalManifoldPoint();

		// Using the half-way point keeps the points in the same position when swapping reference face from A to B.
		let point = b3MulSub(clipPoint.position, 0.5 * clipPoint.separation, refPlane.normal);

		// Old way of pushing onto the reference face.
		// b3Vec3 point = clipPoint->position - clipPoint->separation * refPlane.normal;

		pt.point = point;
		pt.separation = clipPoint.separation;
		pt.pair = clipPoint.pair;

		minSeparation = b3MinFloat(minSeparation, clipPoint.separation);
	}

	if (minSeparation >= B3_SPECULATIVE_DISTANCE()) {
		cache = b3SATCache();
		return false;
	}

	b3ReduceManifoldPoints(manifold, capacity, points, pointCount);

	// Save cache
	cache.separation = minSeparation;
	cache.type = <u8>b3_faceAxisA;
	cache.indexA = <u8>query.indexA;
	cache.indexB = <u8>query.indexB;

	return true;
}

function b3BuildFaceBContact(
	manifold: b3LocalManifold,
	capacity: i32,
	hullA: b3HullData,
	hullB: b3HullData,
	transformBtoA: b3Transform,
	query: b3SeparatingAxis,
	cache: ref b3SATCache,
	workerIndex: i32,
): bool {
	let transformAtoB = b3InvertTransform(transformBtoA);
	let flippedQuery: b3SeparatingAxis;
	flippedQuery.normal = b3Neg(query.normal);
	flippedQuery.separation = query.separation;
	flippedQuery.indexA = query.indexB;
	flippedQuery.indexB = query.indexA;
	flippedQuery.type = b3_faceAxisA;

	let touching = b3BuildFaceAContact(manifold, capacity, hullB, hullA, transformAtoB, flippedQuery, cache, workerIndex);
	if (touching == false) {
		cache = b3SATCache();
		return false;
	}

	// Results are in frame B, need to transform them into frame A
	let matrix = b3MakeMatrixFromQuat(transformBtoA.q);

	// Transform and flip normal so it points from A to B, even though the B has the reference face.
	manifold.normal = b3Neg(b3MulMV(matrix, manifold.normal));

	// Transform points from frame B to frame A.
	// Also flip the pairs to ensure correct matches.
	let base = manifold.pointOffset;
	for (let i = 0; i < manifold.pointCount; ++i) {
		ref pt = manifold.points[base + i];
		pt.point = b3Add(b3MulMV(matrix, pt.point), transformBtoA.p);
		pt.pair = b3FlipPair(pt.pair);
	}

	cache.type = <u8>b3_faceAxisB;
	cache.indexA = <u8>query.indexA;
	cache.indexB = <u8>query.indexB;

	return true;
}

function b3BuildEdgeContact(
	manifold: b3LocalManifold,
	hullA: b3HullData,
	hullB: b3HullData,
	transformBtoA: b3Transform,
	query: b3SeparatingAxis,
	cache: ref b3SATCache,
): bool {
	// Work in shapeA coordinates
	let edgesA = b3GetHullEdges(hullA);
	let pointsA = b3GetHullPoints(hullA);

	let edgesB = b3GetHullEdges(hullB);
	let pointsB = b3GetHullPoints(hullB);

	// B3_VALIDATE( query.separation <= 2.0f * B3_SPECULATIVE_DISTANCE );

	let edgeA = edgesA[query.indexA];
	let twinA = edgesA[edgeA.twin];
	let pA = pointsA[edgeA.origin];
	let qA = pointsA[twinA.origin];
	let eA = b3Sub(qA, pA);

	let edgeB = edgesB[query.indexB];
	let twinB = edgesB[edgeB.twin];
	let pB = b3TransformPoint(transformBtoA, pointsB[edgeB.origin]);
	let qB = b3TransformPoint(transformBtoA, pointsB[twinB.origin]);
	let eB = b3Sub(qB, pB);

	let normal = query.normal;
	let result = b3LineDistance(pA, eA, pB, eB);

	if (b3IsWithinSegments(result) == false) {
		cache = b3SATCache();
		return false;
	}

	// This can slide off the end from caching
	let separation = b3Dot(normal, b3Sub(result.point2, result.point1));
	let point = b3MulSV(0.5, b3Add(result.point1, result.point2));

	// Result in frame A
	manifold.normal = normal;
	manifold.pointCount = 1;

	ref pt = manifold.points[manifold.pointOffset + 0];
	pt.point = point;
	pt.separation = separation;
	pt.pair = b3MakeFeaturePair(b3_featureShapeA, query.indexA, b3_featureShapeB, query.indexB);

	// Save cache
	cache.separation = separation;
	cache.type = <u8>b3_edgePairAxis;
	cache.indexA = <u8>query.indexA;
	cache.indexB = <u8>query.indexB;

	return true;
}

export function b3ComputeSeparatingAxisAtWidth(
	hullA: b3HullData,
	hullB: b3HullData,
	xfB: b3Transform,
	earlyReturn: bool,
	simdWidth: i32,
	workerIndex: i32 = 0,
): b3AxisQuery {
	// No width 8 path is ported
	return b3ComputeSeparatingAxisW4(hullA, hullB, xfB, earlyReturn, workerIndex);
}

export function b3ComputeSeparatingAxis(
	hullA: b3HullData,
	hullB: b3HullData,
	xfB: b3Transform,
	earlyReturn: bool,
): b3AxisQuery {
	return b3ComputeSeparatingAxisAtWidth(hullA, hullB, xfB, earlyReturn, b3GetSIMDWidth());
}

export function b3CollideHullsAtWidth(
	manifold: b3LocalManifold,
	capacity: i32,
	hullA: b3HullData,
	hullB: b3HullData,
	transformBtoA: b3Transform,
	cache: ref b3SATCache,
	simdWidth: i32,
	workerIndex: i32 = 0,
): void {
	let scratch = s_convexScratch[workerIndex];
	manifold.pointCount = 0;

	if (capacity < 4) {
		return;
	}

	// Work in shapeA coordinates
	let speculativeDistance = B3_SPECULATIVE_DISTANCE();

	let linearSlop = B3_LINEAR_SLOP();
	let edgesA = b3GetHullEdges(hullA);
	let planesA = b3GetHullPlanes(hullA);
	let pointsA = b3GetHullPoints(hullA);

	let edgesB = b3GetHullEdges(hullB);
	let planesB = b3GetHullPlanes(hullB);
	let pointsB = b3GetHullPoints(hullB);

	cache.hit = 0;

	// Attempt to use the cache to speed up collision
	switch (<i32>cache.type) {
		case b3_invalidAxis:
			break;

		case b3_faceAxisA: {
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>cache.indexA < hullA.faceCount);

			// Check for separation using cached face
			let plane = planesA[cache.indexA];
			let searchDirectionInB = b3Neg(b3InvRotateVector(transformBtoA.q, plane.normal));

			let vertexIndex = b3FindHullSupportVertex(hullB, searchDirectionInB);
			let support = b3TransformPoint(transformBtoA, pointsB[vertexIndex]);
			let separation = b3PlaneSeparation(plane, support);

			if (separation >= speculativeDistance) {
				// Cache hit, shapes are separated
				cache.hit = 1;
				return;
			}

			// Attempt face contact using cached feature
			let faceQuery: b3SeparatingAxis;
			faceQuery.normal = plane.normal;
			faceQuery.separation = 0.0;
			faceQuery.indexA = cache.indexA;
			faceQuery.indexB = vertexIndex;
			faceQuery.type = b3_faceAxisA;

			let localCache = b3SATCache();
			let touching = b3BuildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, localCache, workerIndex);
			if (touching == true && b3AbsFloat(cache.separation - localCache.separation) < linearSlop) {
				// Cache hit, contact points generated
				cache.hit = 1;
				return;
			}
			break;
		}

		case b3_faceAxisB: {
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>cache.indexB < hullB.faceCount);

			// Check for separation using cached face
			let plane = planesB[cache.indexB];
			let searchDirectionInA = b3Neg(b3RotateVector(transformBtoA.q, plane.normal));

			// todo use b3GetSupportWide
			let vertexIndex = b3FindHullSupportVertex(hullA, searchDirectionInA);
			let support = b3InvTransformPoint(transformBtoA, pointsA[vertexIndex]);
			let separation = b3PlaneSeparation(plane, support);

			if (separation >= speculativeDistance) {
				// Cache hit, shapes are separated
				cache.hit = 1;
				return;
			}

			// Attempt face contact using cached feature
			let faceQuery: b3SeparatingAxis;
			faceQuery.normal = b3Neg(plane.normal);
			faceQuery.separation = 0.0;
			faceQuery.indexA = vertexIndex;
			faceQuery.indexB = cache.indexB;
			faceQuery.type = b3_faceAxisB;

			let localCache = b3SATCache();
			let touching = b3BuildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, localCache, workerIndex);
			if (touching == true && b3AbsFloat(cache.separation - localCache.separation) < linearSlop) {
				// Cache hit, contact points generated
				cache.hit = 1;
				return;
			}
			break;
		}

		case b3_edgePairAxis: {
			let indexA = <i32>cache.indexA;
			let edge1 = edgesA[indexA];
			let twin1 = edgesA[indexA + 1];
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edge1.twin == indexA + 1 && <i32>twin1.twin == indexA);

			let pA = pointsA[edge1.origin];
			let qA = pointsA[twin1.origin];
			let eA = b3Sub(qA, pA);

			let uA = planesA[edge1.face].normal;
			let vA = planesA[twin1.face].normal;

			let indexB = <i32>cache.indexB;
			let edge2 = edgesB[indexB];
			let twin2 = edgesB[indexB + 1];
			if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edge2.twin == indexB + 1 && <i32>twin2.twin == indexB);

			let pB = b3TransformPoint(transformBtoA, pointsB[edge2.origin]);
			let qB = b3TransformPoint(transformBtoA, pointsB[twin2.origin]);
			let eB = b3Sub(qB, pB);

			let uB = b3RotateVector(transformBtoA.q, planesB[edge2.face].normal);
			let vB = b3RotateVector(transformBtoA.q, planesB[twin2.face].normal);

			// flipping the signs of u2 and v2
			// cross(v2, u2) == cross(-v2, -u2)
			// so we still use -e2
			// but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
			let cba = b3Dot(uB, eA);
			let dba = b3Dot(vB, eA);
			let adc = -b3Dot(uA, eB);
			let bdc = -b3Dot(vA, eB);

			if (cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0) {
				// Avoid nearly parallel edges that may lead to invalid separation values at the noise floor.
				let squaredTolerance = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;
				if (b3MaxFloat(cba * cba, dba * dba) >= squaredTolerance * b3LengthSquared(eA)) {
					// Transform reference center of the first hull into local space of the second hull
					let t = cba / (cba - dba);
					let axis = b3Lerp(uB, vB, t);
					axis = b3Normalize(axis);
					let separation = b3Dot(axis, b3Sub(qA, qB));

					if (separation > speculativeDistance) {
						// Cache hit, shapes are separated
						cache.hit = 1;
						return;
					}

					// Try to rebuild contact from last features
					let edgeQuery = b3SeparatingAxis();
					edgeQuery.normal = b3Neg(axis);
					edgeQuery.separation = 0.0;
					edgeQuery.indexA = cache.indexA;
					edgeQuery.indexB = cache.indexB;
					edgeQuery.type = b3_edgePairAxis;

					let localCache = b3SATCache();
					let touching = b3BuildEdgeContact(manifold, hullA, hullB, transformBtoA, edgeQuery, localCache);

					// This separation tolerance may have a big impact on performance in some benchmarks.
					if (touching && b3AbsFloat(cache.separation - localCache.separation) < linearSlop) {
						// Cache hit, contact point generated
						cache.hit = 1;
						return;
					}
				}
			}
			break;
		}

		// This case is for testing
		case b3_manualFaceAxisA: {
			let axisQuery = b3ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth, workerIndex);
			let faceQuery = axisQuery.faceA;
			b3BuildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache, workerIndex);
			return;
		}

		// This case is for testing
		case b3_manualFaceAxisB: {
			let axisQuery = b3ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth, workerIndex);
			let faceQuery = axisQuery.faceB;
			b3BuildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache, workerIndex);
			return;
		}

		// This case is for testing
		case b3_manualEdgePairAxis: {
			let axisQuery = b3ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, false, simdWidth, workerIndex);
			let edgeQuery = axisQuery.edge;
			if (edgeQuery.indexA != B3_NULL_INDEX) {
				b3BuildEdgeContact(manifold, hullA, hullB, transformBtoA, edgeQuery, cache);
			}
			return;
		}

		default:
			if (B3_ASSERT_ENABLED) B3_ASSERT(false);
			break;
	}

	manifold.pointCount = 0;
	cache = b3SATCache();

	let axisQuery = b3ComputeSeparatingAxisAtWidth(hullA, hullB, transformBtoA, true, simdWidth, workerIndex);

	if (axisQuery.separatedFeature != b3_invalidAxis) {
		// We found a separating axis
		cache.type = <u8>axisQuery.separatedFeature;

		if (axisQuery.separatedFeature == b3_faceAxisA) {
			cache.separation = axisQuery.faceA.separation;
			cache.indexA = <u8>axisQuery.faceA.indexA;
			cache.indexB = <u8>axisQuery.faceA.indexB;
		} else if (axisQuery.separatedFeature == b3_faceAxisB) {
			cache.separation = axisQuery.faceB.separation;
			cache.indexA = <u8>axisQuery.faceB.indexA;
			cache.indexB = <u8>axisQuery.faceB.indexB;
		} else {
			if (B3_ASSERT_ENABLED) B3_ASSERT(axisQuery.separatedFeature == b3_edgePairAxis);
			cache.separation = axisQuery.edge.separation;
			cache.indexA = <u8>axisQuery.edge.indexA;
			cache.indexB = <u8>axisQuery.edge.indexB;
		}
		return;
	}

	if (axisQuery.faceA.separation > axisQuery.faceB.separation) {
		let faceQuery = axisQuery.faceA;

		// Face contact A
		b3BuildFaceAContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache, workerIndex);
	} else {
		let faceQuery = axisQuery.faceB;

		// Face contact B
		b3BuildFaceBContact(manifold, capacity, hullA, hullB, transformBtoA, faceQuery, cache, workerIndex);
	}

	let edgeQuery = axisQuery.edge;

	if (edgeQuery.indexA == B3_NULL_INDEX) {
		// There are no valid edge pairs (all edges parallel)
		return;
	}

	let faceSeparation = b3MaxFloat(axisQuery.faceA.separation, axisQuery.faceB.separation);
	let clipSeparation = cache.separation;
	let edgeTol = linearSlop;

	// Face contact can be empty if it is not the axis of maximum separation. It can also
	// be empty in narrow cases in the speculative region. If that case was important then
	// a GJK fallback would be used. So far it doesn't seem important.
	// Create edge contact if face contact fails or edge contact is significantly better.
	if (
		(manifold.pointCount == 0 && edgeQuery.separation > faceSeparation) ||
		edgeQuery.separation > clipSeparation + edgeTol
	) {
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= edgeQuery.indexA && edgeQuery.indexA < hullA.edgeCount);
		if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= edgeQuery.indexB && edgeQuery.indexB < hullB.edgeCount);

		// Edge contact
		let edgeManifold = scratch.edgeManifold;
		b3ZeroLocalManifold(edgeManifold);
		scratch.edgePoint[0] = b3LocalManifoldPoint();

		let edgeCache = b3SATCache();
		b3BuildEdgeContact(edgeManifold, hullA, hullB, transformBtoA, edgeQuery, edgeCache);

		// It is possible with speculation to have vertex-vertex collision that is missed by SAT,
		// so edge contact yields no points. In that case perhaps the face contact has some points.
		if (edgeManifold.pointCount == 1) {
			// Copy edge manifold out, being careful to preserve manifold point buffer.
			b3CopyLocalManifoldFields(manifold, edgeManifold);
			manifold.points[manifold.pointOffset + 0] = scratch.edgePoint[0];
			cache = edgeCache;
		}
	}
}

// C's b3LocalManifold edgeManifold = { 0 }
export function b3ZeroLocalManifold(m: b3LocalManifold): void {
	m.normal = b3Vec3();
	m.triangleNormal = b3Vec3();
	m.pointOffset = 0;
	m.pointCount = 0;
	m.triangleIndex = 0;
	m.i1 = 0;
	m.i2 = 0;
	m.i3 = 0;
	m.squaredDistance = 0.0;
	m.feature = 0;
	m.triangleFlags = 0;
}

// C's *manifold = edgeManifold with the point buffer kept
function b3CopyLocalManifoldFields(dst: b3LocalManifold, src: b3LocalManifold): void {
	dst.normal = src.normal;
	dst.triangleNormal = src.triangleNormal;
	dst.pointCount = src.pointCount;
	dst.triangleIndex = src.triangleIndex;
	dst.i1 = src.i1;
	dst.i2 = src.i2;
	dst.i3 = src.i3;
	dst.squaredDistance = src.squaredDistance;
	dst.feature = src.feature;
	dst.triangleFlags = src.triangleFlags;
}

export function b3CollideHulls(
	manifold: b3LocalManifold,
	capacity: i32,
	hullA: b3HullData,
	hullB: b3HullData,
	transformBtoA: b3Transform,
	cache: ref b3SATCache,
): void {
	b3CollideHullsAtWidth(manifold, capacity, hullA, hullB, transformBtoA, cache, b3GetSIMDWidth());
}

// convex_manifold_wide.inl at B3_SIMD_WIDTH 4

// The scalar build takes the B3_SIMD_NONE edge pair loop; -D B3_SAT_WIDE_EDGES selects the lane loop.
const B3_SIMD_NONE: bool = !isDefined(B3_SAT_WIDE_EDGES);

@inline
function b3RoundUpToSIMDWidth(count: i32): i32 {
	return (count + B3_SIMD_WIDTH - 1) & ~(B3_SIMD_WIDTH - 1);
}

// Transform a SoA point/normal stream (already split into X/Y/Z) by out = -(R*v (+t)).
// The inputs come straight from the hull's stored SoA arrays, so there's no transpose here.
// C's three input pointers are offsets into one SoA array.
function b3NegativeTransformFromSoA(
	R: b3Matrix3,
	p: b3Vec3,
	soa: StaticArray<f32>,
	inX: i32,
	inY: i32,
	inZ: i32,
	n: i32,
	outX: StaticArray<f32>,
	outY: StaticArray<f32>,
	outZ: StaticArray<f32>,
	isPoint: bool,
): void {
	// row-column
	let r0 = b3Vec3W(b3SplatW(R.cx.x), b3SplatW(R.cy.x), b3SplatW(R.cz.x));
	let r1 = b3Vec3W(b3SplatW(R.cx.y), b3SplatW(R.cy.y), b3SplatW(R.cz.y));
	let r2 = b3Vec3W(b3SplatW(R.cx.z), b3SplatW(R.cy.z), b3SplatW(R.cz.z));

	let t = b3Vec3W(b3ZeroW(), b3ZeroW(), b3ZeroW());
	if (isPoint) {
		t = b3SplatVW(p);
	}

	for (let i = 0; i < n; i += B3_SIMD_WIDTH) {
		let v = b3LoadVW(soa, inX + i, soa, inY + i, soa, inZ + i);

		// Rotate one vector per lane
		let out = b3Vec3W(b3DotW(r0, v), b3DotW(r1, v), b3DotW(r2, v));

		if (isPoint) {
			out = b3AddVW(out, t);
		}

		b3StoreVW(outX, i, outY, i, outZ, i, b3NegVW(out));
	}
}

const B3_HULL_BIT_COUNT: i32 = 7;

// C's support and vertexIndex outputs
struct b3WideSupport {
	value: f32 = 0;
	vertexIndex: i32 = 0;
}

// SIMD support point calculation using a SoA vertex array padded with repeats of the first vertex
// to a multiple of 8.
//
// This minimizes (bias - dot), where the caller is expected to provide a bias that makes this always positive.
// It can be direction dependent. The bias should be just big enough to ensure the value is positive because
// an excessive bias causes a precision loss in the support calculation.
//
// The vertex index is embedded in the low B3_HULL_BIT_COUNT mantissa bits of the value. By minimizing a value that
// is always positive, the minimum carries the smallest index so that padded SoA values will never win. This is the
// purpose of using the bias instead of maximizing the dot directly.
//
// The support is then recomputed exactly as dot(normal, vertex), without the embedded index.
// todo consider using this for GJK
function b3GetSupportWide(
	normal: b3Vec3,
	soa: StaticArray<f32>,
	vx: i32,
	vy: i32,
	vz: i32,
	n: i32,
	bias: f32,
): b3WideSupport {
	let normalW = b3SplatVW(normal);
	let biasV = b3SplatW(bias);

	// Start the minimum at a large value.
	let minValue = b3SplatW(FLT_INF);

	// Tail lanes hold vertex 0 with index bits >= vertexCount, so they never become the min value.
	for (let i = 0; i < n; i += B3_SIMD_WIDTH) {
		let v = b3LoadVW(soa, vx + i, soa, vy + i, soa, vz + i);
		let d = b3DotW(normalW, v);

		// This is always positive.
		let value = b3SubW(biasV, d);
		let augmentedValue = b3EmbedIndexW(value, i, B3_HULL_BIT_COUNT);
		minValue = b3MinW(minValue, augmentedValue);
	}

	// One horizontal min, the winning lane's value and index bits ride through.
	let vi = b3MinIndexW(minValue, B3_HULL_BIT_COUNT);

	// Exact support for the chosen vertex.
	let result: b3WideSupport;
	result.vertexIndex = vi;

	// Dot product
	result.value = normal.x * soa[vx + vi] + normal.y * soa[vy + vi] + normal.z * soa[vz + vi];
	return result;
}

// value is the face separation
@inline
function b3GetFaceSeparation(
	direction: b3Vec3,
	planeSeparation: f32,
	soa: StaticArray<f32>,
	vx: i32,
	vy: i32,
	vz: i32,
	n: i32,
	center: b3Vec3,
	extents: b3Vec3,
): b3WideSupport {
	let bias = b3Dot(direction, center) + 1.0625 * b3Dot(b3Abs(direction), extents);
	let result = b3GetSupportWide(direction, soa, vx, vy, vz, n, bias);
	result.value = planeSeparation - result.value;
	return result;
}

// Wide dot(n, d) for all face normals n of the hull, padded to the SIMD width.
function b3GetFaceDots(hull: b3HullData, d: b3Vec3, dots: StaticArray<f32>): void {
	let faceStride = b3GetHullSoaStride(hull.faceCount);
	let soa = b3GetHullSoaNormals(hull);
	let nx = 0;
	let ny = nx + faceStride;
	let nz = ny + faceStride;

	let dW = b3SplatVW(d);

	let wideFaceCount = b3RoundUpToSIMDWidth(hull.faceCount);
	for (let i = 0; i < wideFaceCount; i += B3_SIMD_WIDTH) {
		// dot product per lane
		let m = b3DotW(b3LoadVW(soa, nx + i, soa, ny + i, soa, nz + i), dW);
		b3StoreW(dots, i, m);
	}
}

const B3_PARALLEL_TOL: f32 = 1e-4;

// Inscribed sphere edge test. https://box2d.org/posts/2026/09/inscribed-spheres/
// The implementation here is a mix of the versions from Cairn Overturf and Dirk Gregorius:
// https://gist.github.com/cairnc/dee7a2866da0709f2d9a77b6493b57b5
// https://gist.github.com/dgregorius/e6751b5c00937cd21af63cba3c53c861
// This benchmarks faster than the version from the blog post.
@inline
function b3TestEdgeCandidateSorted(a1: f32, a2: f32, c: f32, bound: f32): i32 {
	// We want to maximize
	//
	//   f(t) = dot(d, nlerp(n1, n2, t))
	//
	// over t in [0, 1], where
	//
	//   a1 = dot(n1, d)
	//   a2 = dot(n2, d)
	//   c  = dot(n1, n2).
	//
	// The maximum can occur either at an endpoint or at an interior
	// critical point where f'(t) = 0.

	// Since the endpoint values are a1 and a2, only the larger endpoint
	// needs to be tested against the bound.
	let hi = b3MaxFloat(a1, a2);
	let lo = b3MinFloat(a1, a2);
	let exterior = <i32>(hi >= bound);

	// Differentiating f(t) gives
	//
	//           a2 - c*a1 - (1 - c)*(a1 + a2)*t
	//   f'(t) = --------------------------------------
	//           ((1 - t)^2 + t^2 + 2*c*t*(1 - t))^(3/2)
	//
	// The denominator is positive and the numerator is linear in t.
	// An interior maximum therefore requires
	//
	//   f'(0) >= 0  ->  a2 >= c*a1
	//   f'(1) <= 0  ->  a1 >= c*a2.
	//
	// After sorting these become
	//
	//   hi >= c*lo
	//   lo >= c*hi.
	//
	// The second is always the stricter condition since
	//
	//   (hi - c*lo) - (lo - c*hi)
	//       = (1 + c)*(hi - lo) >= 0.
	//
	// Thus both conditions reduce to u >= 0.
	let u = lo - c * hi;
	let maxIsInterior = <i32>(u >= 0.0);

	// Solving f'(t) = 0 and evaluating f(t) at tmax gives
	//
	//   f(tmax)^2 =
	//       (a1^2 + a2^2 - 2*c*a1*a2) / (1 - c^2).
	//
	// After sorting,
	//
	//   hi^2 + lo^2 - 2*c*hi*lo
	//       = hi^2*(1 - c^2) + u^2.
	//
	// Let s = 1 - c^2 and rearrange f(tmax) >= bound:
	//
	//   u^2 >= (bound^2 - hi^2)*s.
	let s = 1.0 - c * c;
	let boundTerm = (bound - hi) * (bound + hi);
	let lhs = u * u;
	let rhs = boundTerm * s;
	let maxBeatsBound = <i32>(lhs >= rhs);

	// The interior expression contains 1 - c^2 in its denominator.
	// When this approaches zero the arc is degenerate or ill-conditioned,
	// so conservatively keep the edge as a candidate.
	let nearlyParallel = <i32>(s < B3_PARALLEL_TOL);

	let interior = maxIsInterior & (maxBeatsBound | nearlyParallel);

	return exterior | interior;
}

// dot(n, otherCenter) - offset for all faces of the hull, padded to the SIMD width.
function b3GetFacePlaneSeparations(hull: b3HullData, otherCenter: b3Vec3, separations: StaticArray<f32>): void {
	b3GetFaceDots(hull, otherCenter, separations);

	let planes = b3GetHullPlanes(hull);
	let faceCount = hull.faceCount;
	for (let i = 0; i < faceCount; ++i) {
		separations[i] -= planes[i].offset;
	}
}

// The number of edge pair tests needed to make the extra culling pass worthwhile.
const B3_EDGE_PROBE_MIN_TESTS: i32 = 10;

// Re-test edge candidates against the plane separations of a probe point on the other hull
function b3FilterEdgeCandidates(
	hull: b3HullData,
	planeSeparations: StaticArray<f32>,
	bound: f32,
	edgeIndices: StaticArray<i32>,
	count: i32,
): i32 {
	let halfEdges = b3GetHullEdges(hull);
	let cosines = b3GetHullEdgeCosines(hull);
	let keptCount = 0;

	for (let k = 0; k < count; ++k) {
		let i = edgeIndices[k];
		let i1 = <i32>halfEdges[i].face;
		let i2 = <i32>halfEdges[i + 1].face;
		let c = cosines[i >> 1];
		edgeIndices[keptCount] = i;
		keptCount += b3TestEdgeCandidateSorted(planeSeparations[i1], planeSeparations[i2], c, bound);
	}

	return keptCount;
}


function b3MakeAxis(type: i32): b3SeparatingAxis {
	let axis: b3SeparatingAxis;
	axis.normal = b3Vec3_zero;
	axis.separation = -FLT_INF;
	axis.indexA = B3_NULL_INDEX;
	axis.indexB = B3_NULL_INDEX;
	axis.type = type;
	return axis;
}

// SIMD separating axis test based on an implementation developed by Cairn Overturf.
// See his article: https://cairnc.github.io/posts/improvements-to-the-separating-axis/
export function b3ComputeSeparatingAxisW4(
	hullA: b3HullData,
	hullB: b3HullData,
	xfB: b3Transform,
	earlyReturn: bool,
	workerIndex: i32 = 0,
): b3AxisQuery {
	let scratch = s_convexScratch[workerIndex];
	let R = b3MakeMatrixFromQuat(xfB.q);
	let invR = b3Transpose(R);

	let speculativeDistance = B3_SPECULATIVE_DISTANCE();

	let res: b3AxisQuery;
	res.faceA = b3MakeAxis(b3_faceAxisA);
	res.faceB = b3MakeAxis(b3_faceAxisB);
	res.edge = b3MakeAxis(b3_edgePairAxis);
	res.separatedFeature = b3_invalidAxis;

	let faceCountA = hullA.faceCount;
	let planesA = b3GetHullPlanes(hullA);

	let vertexStrideB = b3GetHullSoaStride(hullB.vertexCount);
	let soaB = b3GetHullSoaVertices(hullB);
	let vxB = 0;
	let vyB = vxB + vertexStrideB;
	let vzB = vyB + vertexStrideB;
	let wideVertexCountB = b3RoundUpToSIMDWidth(hullB.vertexCount);

	let cB = b3AABB_Center(hullB.aabb);
	let hB = b3AABB_Extents(hullB.aabb);

	// The hulls have a precomputed inner radius and centroid.
	// A given axis cannot achieve a separation larger than:
	// dot(axis, centerB - centerA) - innerRadiusA - innerRadiusB
	// So this is the upper bound for the separation of a candidate axis.
	// An axis can be skipped if it has an upper bound that is less than the current
	// best separation. This lets me skip many of the candidates without computing
	// support points.

	let deltaCenter = b3Sub(b3Add(b3MulMV(R, hullB.center), xfB.p), hullA.center);
	let centerDistance = b3Length(deltaCenter);
	let radius = hullA.innerRadius + hullB.innerRadius;

	// Adjust the radius to ensure the best axis isn't skipped.
	let radiusBound = radius - (B3_LINEAR_SLOP() + 0.001 * (centerDistance + radius));

	// Compute dot(normalA, centerDelta) for all face normals of hullA.
	let dotA = scratch.dotA;
	b3GetFaceDots(hullA, deltaCenter, dotA);

	// Find the face of hullA that most aligns with centerDelta.
	let seedIndexA = 0;
	let maxDotA = dotA[0];
	for (let i = 1; i < faceCountA; ++i) {
		if (dotA[i] > maxDotA) {
			maxDotA = dotA[i];
			seedIndexA = i;
		}
	}

	// Use the seed to get a lower bound on the separation for the faces of hullA.
	let floorA = -FLT_INF;
	let seedSeparationA = -FLT_INF;
	let seedVertexB = 0;
	if (earlyReturn) {
		let plane = planesA[seedIndexA];
		let direction = b3Neg(b3MulMV(invR, plane.normal));
		let planeSeparation = b3Dot(plane.normal, xfB.p) - plane.offset;
		let seed = b3GetFaceSeparation(direction, planeSeparation, soaB, vxB, vyB, vzB, wideVertexCountB, cB, hB);
		seedSeparationA = seed.value;
		seedVertexB = seed.vertexIndex;
		floorA = b3MinFloat(seedSeparationA, speculativeDistance);
	}

	// Test A's face planes against B's vertices.
	for (let i = 0; i < faceCountA; ++i) {
		// The bound offset ensures the seed will be evaluated.
		if (dotA[i] - radiusBound < b3MaxFloat(floorA, res.faceA.separation)) {
			continue;
		}

		let plane = planesA[i];
		let vertexIndex = seedVertexB;
		let separation = seedSeparationA;

		// Avoid recomputing the seed face separation.
		if (earlyReturn == false || i != seedIndexA) {
			let direction = b3Neg(b3MulMV(invR, plane.normal));
			let planeSeparation = b3Dot(plane.normal, xfB.p) - plane.offset;
			let face = b3GetFaceSeparation(direction, planeSeparation, soaB, vxB, vyB, vzB, wideVertexCountB, cB, hB);
			separation = face.value;
			vertexIndex = face.vertexIndex;
		}

		if (separation > res.faceA.separation) {
			res.faceA.normal = plane.normal;
			res.faceA.separation = separation;
			res.faceA.indexA = i;
			res.faceA.indexB = vertexIndex;
			if (separation > speculativeDistance && earlyReturn) {
				res.separatedFeature = b3_faceAxisA;
				return res;
			}
		}
	}

	let faceCountB = hullB.faceCount;
	let planesB = b3GetHullPlanes(hullB);

	let vertexStrideA = b3GetHullSoaStride(hullA.vertexCount);
	let soaA = b3GetHullSoaVertices(hullA);
	let vxA = 0;
	let vyA = vxA + vertexStrideA;
	let vzA = vyA + vertexStrideA;
	let wideVertexCountA = b3RoundUpToSIMDWidth(hullA.vertexCount);

	let cA = b3AABB_Center(hullA.aabb);
	let hA = b3AABB_Extents(hullA.aabb);

	// Similarly, find the face of hullB that most aligns with the vector pointing from centerB to centerA.
	let dotB = scratch.dotB;
	b3GetFaceDots(hullB, b3Neg(b3MulMV(invR, deltaCenter)), dotB);

	let seedIndexB = 0;
	let maxDotB = dotB[0];
	for (let i = 1; i < faceCountB; ++i) {
		if (dotB[i] > maxDotB) {
			maxDotB = dotB[i];
			seedIndexB = i;
		}
	}

	// Get a lower bound on the separation for the faces of hullB.
	let floorB = -FLT_INF;
	let seedSeparationB = -FLT_INF;
	let seedVertexA = 0;
	if (earlyReturn) {
		let plane = planesB[seedIndexB];
		let direction = b3Neg(b3MulMV(R, plane.normal));
		let planeSeparation = b3Dot(direction, xfB.p) - plane.offset;
		let seed = b3GetFaceSeparation(direction, planeSeparation, soaA, vxA, vyA, vzA, wideVertexCountA, cA, hA);
		seedSeparationB = seed.value;
		seedVertexA = seed.vertexIndex;

		// Include the floor set by hull A faces.
		floorB = b3MaxFloat(seedSeparationB, res.faceA.separation);
		floorB = b3MinFloat(floorB, speculativeDistance);
	}

	// Test B's face planes against A's vertices.
	for (let i = 0; i < faceCountB; ++i) {
		if (dotB[i] - radiusBound < b3MaxFloat(floorB, res.faceB.separation)) {
			continue;
		}

		let plane = planesB[i];
		let direction = b3Neg(b3MulMV(R, plane.normal));
		let vertexIndex = seedVertexA;
		let separation = seedSeparationB;

		// Avoid recomputing the seed face separation.
		if (earlyReturn == false || i != seedIndexB) {
			let planeSeparation = b3Dot(direction, xfB.p) - plane.offset;
			let face = b3GetFaceSeparation(direction, planeSeparation, soaA, vxA, vyA, vzA, wideVertexCountA, cA, hA);
			separation = face.value;
			vertexIndex = face.vertexIndex;
		}

		if (separation > res.faceB.separation) {
			res.faceB.normal = direction;
			res.faceB.separation = separation;
			res.faceB.indexA = vertexIndex;
			res.faceB.indexB = i;
			if (separation > speculativeDistance && earlyReturn) {
				res.separatedFeature = b3_faceAxisB;
				return res;
			}
		}
	}

	// Gather edges of A that can feasibly create a separating axis that beats the maximum face separation.
	let halfEdgeCountA = hullA.edgeCount;
	let halfEdgesA = b3GetHullEdges(hullA);
	let edgeIndicesA = scratch.edgeIndicesA;
	let na = 0;

	let halfEdgeCountB = hullB.edgeCount;
	let halfEdgesB = b3GetHullEdges(hullB);
	let edgeIndicesB = scratch.edgeIndicesB;
	let nb = 0;

	let maxFaceSeparation = b3MaxFloat(res.faceA.separation, res.faceB.separation);
	let boundSlack = radius - radiusBound;
	let thresholdA = earlyReturn ? maxFaceSeparation + hullB.innerRadius - boundSlack : -FLT_INF;
	let thresholdB = earlyReturn ? maxFaceSeparation + hullA.innerRadius - boundSlack : -FLT_INF;

	let centerBinA = b3Add(b3MulMV(R, hullB.center), xfB.p);
	let centerAinB = b3MulMV(invR, b3Sub(hullA.center, xfB.p));

	let planeDotA = scratch.planeDotA;
	let planeDotB = scratch.planeDotB;
	b3GetFacePlaneSeparations(hullA, centerBinA, planeDotA);
	b3GetFacePlaneSeparations(hullB, centerAinB, planeDotB);

	let cosinesA = b3GetHullEdgeCosines(hullA);
	let cosinesB = b3GetHullEdgeCosines(hullB);

	for (let i = 0; i < halfEdgeCountA; i += 2) {
		let i1 = <i32>halfEdgesA[i].face;
		let i2 = <i32>halfEdgesA[i + 1].face;
		let c = cosinesA[i >> 1];
		edgeIndicesA[na] = i;
		na += b3TestEdgeCandidateSorted(planeDotA[i1], planeDotA[i2], c, thresholdA);
	}

	for (let i = 0; i < halfEdgeCountB; i += 2) {
		let i1 = <i32>halfEdgesB[i].face;
		let i2 = <i32>halfEdgesB[i + 1].face;
		let c = cosinesB[i >> 1];
		edgeIndicesB[nb] = i;
		nb += b3TestEdgeCandidateSorted(planeDotB[i1], planeDotB[i2], c, thresholdB);
	}

	// Apply additional culling use the support points for the best face axes.
	// The support vertices of the best faces are points on the other hull, so an edge pair axis cannot
	// have a larger separation than the plane separation of these points over the arc of the edge.
	// This can slow down boxes so skip this if there are not enough edge candidates.
	if (earlyReturn && nb * ((na + 3) >> 2) >= B3_EDGE_PROBE_MIN_TESTS) {
		let probeBound = maxFaceSeparation - (0.1 * B3_LINEAR_SLOP() + 0.001 * b3AbsFloat(centerDistance + radius));

		let vertexB = res.faceA.indexB != B3_NULL_INDEX ? res.faceA.indexB : seedVertexB;
		let probeB = b3Vec3(soaB[vxB + vertexB], soaB[vyB + vertexB], soaB[vzB + vertexB]);
		b3GetFacePlaneSeparations(hullA, b3Add(b3MulMV(R, probeB), xfB.p), planeDotA);
		na = b3FilterEdgeCandidates(hullA, planeDotA, probeBound, edgeIndicesA, na);

		let vertexA = res.faceB.indexA != B3_NULL_INDEX ? res.faceB.indexA : seedVertexA;
		let probeA = b3Vec3(soaA[vxA + vertexA], soaA[vyA + vertexA], soaA[vzA + vertexA]);
		b3GetFacePlaneSeparations(hullB, b3MulMV(invR, b3Sub(probeA, xfB.p)), planeDotB);
		nb = b3FilterEdgeCandidates(hullB, planeDotB, probeBound, edgeIndicesB, nb);
	}

	if (na == 0 || nb == 0) {
		// No edge candidates found.
		return res;
	}

	// B face normals in A space, negated.
	let bFNx = scratch.bFNx;
	let bFNy = scratch.bFNy;
	let bFNz = scratch.bFNz;

	// B vertices in A space, negated.
	let bWx = scratch.bWx;
	let bWy = scratch.bWy;
	let bWz = scratch.bWz;

	let faceStrideB = b3GetHullSoaStride(faceCountB);
	let soaNB = b3GetHullSoaNormals(hullB);
	let nxB = 0;
	let nyB = nxB + faceStrideB;
	let nzB = nyB + faceStrideB;

	let wideFaceCountB = b3RoundUpToSIMDWidth(faceCountB);
	b3NegativeTransformFromSoA(R, xfB.p, soaNB, nxB, nyB, nzB, wideFaceCountB, bFNx, bFNy, bFNz, false);
	b3NegativeTransformFromSoA(R, xfB.p, soaB, vxB, vyB, vzB, wideVertexCountB, bWx, bWy, bWz, true);

	// Per A edge data, already in A's space so just gathered. n0 and n1 are the two face
	// normals, d the edge vector av1-av0, v0 the first vertex. Tol is the
	// parallel edge tolerance, scaled by the edge length.
	let aN0x = scratch.aN0x;
	let aN0y = scratch.aN0y;
	let aN0z = scratch.aN0z;
	let aN1x = scratch.aN1x;
	let aN1y = scratch.aN1y;
	let aN1z = scratch.aN1z;
	// dir = av1 - av0
	let aDx = scratch.aDx;
	let aDy = scratch.aDy;
	let aDz = scratch.aDz;
	let aV0x = scratch.aV0x;
	let aV0y = scratch.aV0y;
	let aV0z = scratch.aV0z;
	let aTol = scratch.aTol;

	let squaredTol = B3_PARALLEL_EDGE_TOL * B3_PARALLEL_EDGE_TOL;
	for (let k = 0; k < na; ++k) {
		let edgeIndex = edgeIndicesA[k];
		let edge = halfEdgesA[edgeIndex];
		let twin = halfEdgesA[edgeIndex + 1];

		let A = planesA[edge.face].normal;
		let B = planesA[twin.face].normal;
		aN0x[k] = A.x;
		aN0y[k] = A.y;
		aN0z[k] = A.z;
		aN1x[k] = B.x;
		aN1y[k] = B.y;
		aN1z[k] = B.z;

		let v0 = <i32>edge.origin;
		let v1 = <i32>twin.origin;

		aDx[k] = soaA[vxA + v1] - soaA[vxA + v0];
		aDy[k] = soaA[vyA + v1] - soaA[vyA + v0];
		aDz[k] = soaA[vzA + v1] - soaA[vzA + v0];
		aV0x[k] = soaA[vxA + v0];
		aV0y[k] = soaA[vyA + v0];
		aV0z[k] = soaA[vzA + v0];

		aTol[k] = squaredTol * (aDx[k] * aDx[k] + aDy[k] * aDy[k] + aDz[k] * aDz[k]);
	}

	// Zero the tail lanes.
	let zero = b3ZeroW();
	let zeroV = b3Vec3W(zero, zero, zero);
	b3StoreVW(aN0x, na, aN0y, na, aN0z, na, zeroV);
	b3StoreVW(aN1x, na, aN1y, na, aN1z, na, zeroV);
	b3StoreVW(aDx, na, aDy, na, aDz, na, zeroV);
	b3StoreVW(aV0x, na, aV0y, na, aV0z, na, zeroV);

	b3StoreW(aTol, na, zero);

	let linearSlop = B3_LINEAR_SLOP();

	if (B3_SIMD_NONE) {
		// The SIMD emulated version of this code is very slow. This is a purely scalar version
		// for platforms that don't have SIMD capability. It is much faster than SIMD emulation.
		// WARNING: this math needs to match the SIMD version for cross platform determinism.

		let EPS = -linearSlop * linearSlop;

		for (let j = 0; j < nb; ++j) {
			let edgeIndex = edgeIndicesB[j];
			let edge = halfEdgesB[edgeIndex];
			let twin = halfEdgesB[edgeIndex + 1];
			let f0 = <i32>edge.face;
			let f1 = <i32>twin.face;
			let v0 = <i32>edge.origin;
			let v1 = <i32>twin.origin;

			let C = b3Vec3(bFNx[f0], bFNy[f0], bFNz[f0]);
			let D = b3Vec3(bFNx[f1], bFNy[f1], bFNz[f1]);
			let bv0 = b3Vec3(bWx[v0], bWy[v0], bWz[v0]);
			let bv1 = b3Vec3(bWx[v1], bWy[v1], bWz[v1]);
			let DC = b3Sub(bv1, bv0);

			for (let i = 0; i < na; ++i) {
				let d = b3Vec3(aDx[i], aDy[i], aDz[i]);

				// CBA = C.dir, DBA = D.dir, where dir = B_x_A
				let CBA = b3Dot(C, d);
				let DBA = b3Dot(D, d);
				if (CBA * DBA >= EPS) {
					continue;
				}

				let n0 = b3Vec3(aN0x[i], aN0y[i], aN0z[i]);
				let n1 = b3Vec3(aN1x[i], aN1y[i], aN1z[i]);

				// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
				let ADC = b3Dot(n0, DC);
				let BDC = b3Dot(n1, DC);
				if (ADC * BDC >= EPS || CBA * BDC >= EPS) {
					continue;
				}

				// Reject near parallel edges
				let maxCD = b3MaxFloat(CBA * CBA, DBA * DBA);
				if (maxCD <= aTol[i]) {
					continue;
				}

				// t = -CBA / (DBA - CBA)
				let t = -CBA / (DBA - CBA);

				// normal = lerp(t, C, D) = C + (D-C)*t
				let n = b3MulAdd(C, t, b3Sub(D, C));
				let len2 = b3Dot(n, n);

				let inv = 1.0 / Mathf.sqrt(len2);
				n = b3MulSV(inv, n);

				// separation = -dot(normal, av0 + bv0)
				let av0 = b3Vec3(aV0x[i], aV0y[i], aV0z[i]);
				let separation = -b3Dot(b3Add(av0, bv0), n);

				if (separation > res.edge.separation) {
					res.edge.normal = n;
					res.edge.separation = separation;

					// Half edge index
					res.edge.indexA = edgeIndicesA[i];
					res.edge.indexB = edgeIndicesB[j];

					if (separation > speculativeDistance && earlyReturn) {
						res.separatedFeature = b3_edgePairAxis;
						return res;
					}
				}
			}
		}
	} else {
		// This tolerance can skip edges shorter than 1cm.
		let EPS = b3SplatW(-linearSlop * linearSlop);
		let INF = b3SplatW(FLT_INF);

		let sA = scratch.sA;
		let nxA = scratch.nxA;
		let nyA = scratch.nyA;
		let nzA = scratch.nzA;

		for (let j = 0; j < nb; ++j) {
			let edgeIndex = edgeIndicesB[j];
			let edge = halfEdgesB[edgeIndex];
			let twin = halfEdgesB[edgeIndex + 1];
			let f0 = <i32>edge.face;
			let f1 = <i32>twin.face;
			let v0 = <i32>edge.origin;
			let v1 = <i32>twin.origin;

			let nC = b3Vec3(bFNx[f0], bFNy[f0], bFNz[f0]);
			let nD = b3Vec3(bFNx[f1], bFNy[f1], bFNz[f1]);
			let pB = b3Vec3(bWx[v0], bWy[v0], bWz[v0]);
			let qB = b3Vec3(bWx[v1], bWy[v1], bWz[v1]);

			let C = b3SplatVW(nC);
			let D = b3SplatVW(nD);
			let DC = b3SplatVW(b3Sub(qB, pB));
			let bv0 = b3SplatVW(pB);

			for (let i = 0; i < na; i += B3_SIMD_WIDTH) {
				let n0 = b3LoadVW(aN0x, i, aN0y, i, aN0z, i);
				let n1 = b3LoadVW(aN1x, i, aN1y, i, aN1z, i);
				let d = b3LoadVW(aDx, i, aDy, i, aDz, i);
				let av0 = b3LoadVW(aV0x, i, aV0y, i, aV0z, i);

				let tol = b3LoadW(aTol, i);

				// CBA = C.dir, DBA = D.dir, where dir = B_x_A
				let CBA = b3DotW(C, d);
				let DBA = b3DotW(D, d);
				// ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
				let ADC = b3DotW(n0, DC);
				let BDC = b3DotW(n1, DC);

				// Gauss map arc crossing test, CBA*DBA<eps and ADC*BDC<eps and CBA*BDC<eps
				let m1 = b3LessThanW(b3MulW(CBA, DBA), EPS);
				let m2 = b3LessThanW(b3MulW(ADC, BDC), EPS);
				let m3 = b3LessThanW(b3MulW(CBA, BDC), EPS);

				// Reject near parallel edges. The arc lerp is ill conditioned when both of B's normals are nearly
				// perpendicular to edge A, a scale invariant sine threshold relative to the edge length.
				let maxCD = b3MaxW(b3MulW(CBA, CBA), b3MulW(DBA, DBA));
				let notParallel = b3GreaterThanW(maxCD, tol);
				let mask = b3AndW(b3AndW(m1, m2), b3AndW(m3, notParallel));

				// Most A-edges fail the Gauss test, so skip the divide, sqrt and support work when no
				// lane passed.
				if (b3AnyTrueW(mask) == false) {
					continue;
				}

				// t = -CBA / (DBA - CBA)
				let t = b3DivW(b3NegW(CBA), b3SubW(DBA, CBA));

				// normal = lerp(t, C, D) = C + (D-C)*t
				let n = b3MulAddSVW(C, t, b3SubVW(D, C));

				// normalize
				let len2 = b3DotW(n, n);
				let inv = b3DivW(b3SplatW(1.0), b3SqrtW(len2));
				n = b3MulSVW(inv, n);

				// support = dot(normal, av0 + bv0)
				let support = b3DotW(b3AddVW(av0, bv0), n);

				// Lanes that fail the Gauss test can never win.
				support = b3BlendW(INF, support, mask);
				let separation = b3NegW(support);

				// Test all supports against the running best at once. If none beats it, skip the
				// store and scalar reduction.
				let improves = b3GreaterThanW(separation, b3SplatW(res.edge.separation));
				if (b3AnyTrueW(improves) == false) {
					continue;
				}

				b3StoreW(sA, 0, separation);
				b3StoreVW(nxA, 0, nyA, 0, nzA, 0, n);

				// Reduce in lane order so ties keep the first edge and the early out takes the first
				// improving support below zero. Padded tail lanes carry +INF support, so they never
				// update or index edges out of range.
				for (let lane = 0; lane < B3_SIMD_WIDTH; lane++) {
					let ei = i + lane;
					let s = sA[lane];
					if (s > res.edge.separation) {
						res.edge.normal = b3Vec3(nxA[lane], nyA[lane], nzA[lane]);
						res.edge.separation = s;

						// Half edge index
						res.edge.indexA = edgeIndicesA[ei];
						res.edge.indexB = edgeIndicesB[j];

						if (s > speculativeDistance && earlyReturn) {
							res.separatedFeature = b3_edgePairAxis;
							return res;
						}
					}
				}
			}
		}
	}

	return res;
}
