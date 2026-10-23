// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/manifold.c

import { B3_ASSERT, B3_ASSERT_ENABLED, FLT_MAX, UINT8_MAX } from "./core";
import { b3AbsFloat, b3Dot, b3MulAdd, b3Normalize, b3Plane, b3Sub, b3Vec3 } from "./math_functions";
import { b3PlaneSeparation } from "./math_internal";
import {
	b3FeaturePair,
	b3GetHullEdges,
	b3GetHullPlanes,
	b3GetHullPoints,
	b3GetHullVertices,
	b3HullData,
	b3SeparatingFeature,
} from "./collision";

export const B3_MAX_CLIP_POINTS: i32 = 64;

export struct b3SeparatingAxis {
	normal: b3Vec3 = b3Vec3();
	separation: f32 = 0;
	indexA: i32 = 0;
	indexB: i32 = 0;
	type: b3SeparatingFeature = 0;
}

export struct b3AxisQuery {
	faceA: b3SeparatingAxis = b3SeparatingAxis();
	faceB: b3SeparatingAxis = b3SeparatingAxis();
	edge: b3SeparatingAxis = b3SeparatingAxis();
	separatedFeature: b3SeparatingFeature = 0;
}

export struct b3ClipVertex {
	position: b3Vec3 = b3Vec3();
	separation: f32 = 0;
	pair: b3FeaturePair = b3FeaturePair();
}

export type b3FeatureOwner = i32;
export const b3_featureShapeA: b3FeatureOwner = 0;
export const b3_featureShapeB: b3FeatureOwner = 1;

// For single point contact, such as sphere-sphere, sphere-capsule, sphere-triangle
export const b3FeaturePair_single: b3FeaturePair = b3FeaturePair();

@inline export function b3MakeFeatureId(pair: b3FeaturePair): u32 {
	return (<u32>pair.owner1 << 24) | (<u32>pair.index1 << 16) | (<u32>pair.owner2 << 8) | <u32>pair.index2;
}

export function b3GetBestAxis(query: b3AxisQuery): b3SeparatingAxis {
	if (query.faceA.separation > query.faceB.separation) {
		if (query.edge.separation > query.faceA.separation) {
			return query.edge;
		}

		return query.faceA;
	}

	if (query.edge.separation > query.faceB.separation) {
		return query.edge;
	}

	return query.faceB;
}

// functions from manifold.c follow

// Dirk Gregorius contributed portions of this code

// This was extended to make the wedge shape get the correct incident face.
// Instead of looking directly for the most anti-parallel face, we first find the closest vertex (passed in).
// Then we look for all edges coming out of that vertex and look for the edge that is
// most perpendicular to the reference normal.
// Then from that edge, we select the adjacent face that is most anti-parallel to the reference normal.
export function b3FindIncidentFace(hull: b3HullData, refNormal: b3Vec3, vertexIndex: i32): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= vertexIndex && vertexIndex < hull.vertexCount);

	let vertices = b3GetHullVertices(hull);
	let edges = b3GetHullEdges(hull);
	let planes = b3GetHullPlanes(hull);
	let points = b3GetHullPoints(hull);

	let minEdgeIndex = -1;
	let minEdgeProjection = FLT_MAX;

	let vertexEdge = <i32>vertices[vertexIndex].edge;

	let edgeIndex = vertexEdge;
	let edge = edges[edgeIndex];
	let edgeOrigin = points[edge.origin];
	if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edge.origin == vertexIndex);

	do {
		let twin = edges[edge.twin];
		let twinOrigin = points[twin.origin];

		let axis = b3Normalize(b3Sub(twinOrigin, edgeOrigin));
		let edgeProjection = b3AbsFloat(b3Dot(axis, refNormal));
		if (edgeProjection < minEdgeProjection) {
			minEdgeIndex = edgeIndex;
			minEdgeProjection = edgeProjection;
		}

		edgeIndex = twin.next;
		edge = edges[edgeIndex];
		if (B3_ASSERT_ENABLED) B3_ASSERT(<i32>edge.origin == vertexIndex);
	} while (edgeIndex != vertexEdge);
	if (B3_ASSERT_ENABLED) B3_ASSERT(minEdgeIndex >= 0);

	let minEdge = edges[minEdgeIndex];
	let minFaceIndex1 = <i32>minEdge.face;
	let minPlane1 = planes[minFaceIndex1];

	let minTwin = edges[minEdge.twin];
	let minFaceIndex2 = <i32>minTwin.face;
	let minPlane2 = planes[minFaceIndex2];

	return b3Dot(minPlane1.normal, refNormal) < b3Dot(minPlane2.normal, refNormal) ? minFaceIndex1 : minFaceIndex2;
}

export function b3MakeFeaturePair(owner1: b3FeatureOwner, index1: i32, owner2: b3FeatureOwner, index2: i32): b3FeaturePair {
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index1 && index1 <= UINT8_MAX);
	if (B3_ASSERT_ENABLED) B3_ASSERT(0 <= index2 && index2 <= UINT8_MAX);

	let pair: b3FeaturePair;
	pair.index1 = <u8>index1;
	pair.owner1 = <u8>owner1;
	pair.index2 = <u8>index2;
	pair.owner2 = <u8>owner2;
	return pair;
}

// This logic seems wrong but it is designed so that choosing
// face A or B as the reference face does not change the resulting
// feature pair. This way the contact impulses are persisted even
// if there is reference face flip-flop. This is verified in the
// HullAndHull sample using the b3SATCache with a manual feature
// specified such as b3_manualFaceAxisA.
export function b3FlipPair(pair: b3FeaturePair): b3FeaturePair {
	if (B3_ASSERT_ENABLED) B3_ASSERT(pair.owner1 == 0 || pair.owner1 == 1);
	if (B3_ASSERT_ENABLED) B3_ASSERT(pair.owner2 == 0 || pair.owner2 == 1);
	let t1 = pair.owner1;
	pair.owner1 = pair.owner2;
	pair.owner2 = t1;
	pair.owner1 = 1 - pair.owner1;
	pair.owner2 = 1 - pair.owner2;
	let t2 = pair.index1;
	pair.index1 = pair.index2;
	pair.index2 = t2;
	return pair;
}

// C takes out and polygon pointers; both are clip buffers of at least the count it writes
export function b3ClipPolygon(
	out: StaticArray<b3ClipVertex>,
	polygon: StaticArray<b3ClipVertex>,
	count: i32,
	clipPlane: b3Plane,
	edge: i32,
	refPlane: b3Plane,
): i32 {
	if (B3_ASSERT_ENABLED) B3_ASSERT(count >= 3);

	let vertex1 = polygon[count - 1];
	let distance1 = b3PlaneSeparation(clipPlane, vertex1.position);
	let outCount = 0;

	for (let index = 0; index < count; ++index) {
		let vertex2 = polygon[index];
		let distance2 = b3PlaneSeparation(clipPlane, vertex2.position);

		// Clip edge against plane (Sutherland-Hodgman clipping)
		if (distance1 <= 0.0 && distance2 <= 0.0) {
			// Both vertices are behind the plane - keep vertex2
			out[outCount] = vertex2;
			outCount += 1;
		} else if (distance1 <= 0.0 && distance2 > 0.0) {
			// Vertex1 is behind of the plane, vertex2 is in front -> intersection point
			let fraction = distance1 / (distance1 - distance2);
			let position = b3MulAdd(vertex1.position, fraction, b3Sub(vertex2.position, vertex1.position));

			// Keep intersection point and adjust outgoing edge
			let vertex: b3ClipVertex;
			vertex.position = position;
			vertex.separation = b3PlaneSeparation(refPlane, position);
			vertex.pair = vertex2.pair;
			vertex.pair.owner2 = <u8>b3_featureShapeA;
			vertex.pair.index2 = <u8>edge;
			out[outCount] = vertex;
			outCount += 1;
		} else if (distance2 <= 0.0 && distance1 > 0.0) {
			// Vertex1 is in front, vertex2 is behind of the plane, -> intersection point
			let fraction = distance1 / (distance1 - distance2);
			let position = b3MulAdd(vertex1.position, fraction, b3Sub(vertex2.position, vertex1.position));

			// Keep intersection point and adjust incoming edge
			let vertex: b3ClipVertex;
			vertex.position = position;
			vertex.separation = b3PlaneSeparation(refPlane, position);
			vertex.pair = vertex1.pair;
			vertex.pair.owner1 = <u8>b3_featureShapeA;
			vertex.pair.index1 = <u8>edge;
			out[outCount] = vertex;
			outCount += 1;

			// And also keep vertex2
			out[outCount] = vertex2;
			outCount += 1;
		}

		// Keep vertex2 as starting vertex for next edge
		vertex1 = vertex2;
		distance1 = distance2;
	}

	return outCount;
}
