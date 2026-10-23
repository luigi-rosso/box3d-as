// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Printers, the hull library and pose generators shared by the manifold drivers,
// the twin of verify/cref/manifold_common.h.

import { vtag, vi32, vu32, vbool, vf32, vend, vseed, vrand32, vrandint, vrandf, vbitsf } from "./common";
import { pab, pm3, pv3, rotateBy, rquatOrIdentity, runit, rv } from "./distance_common";
import { hullPointBits, hullSets } from "./hull_data";
import { FLT_MAX } from "../../lib/core";
import { B3_MAX_HULL_VERTICES, B3_MAX_MANIFOLD_POINTS } from "../../lib/constants";
import {
	B3_PI,
	b3Add,
	b3Dot,
	b3MakeQuatFromAxisAngle,
	b3MulAdd,
	b3MulQuat,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Normalize,
	b3NormalizeQuat,
	b3Plane,
	b3Quat,
	b3Quat_identity,
	b3RotateVector,
	b3Transform,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_zero,
} from "../../lib/math_functions";
import {
	b3Capsule,
	b3FeaturePair,
	b3GetHullEdgeCosines,
	b3GetHullEdges,
	b3GetHullFaces,
	b3GetHullPlanes,
	b3GetHullPoints,
	b3GetHullSoaNormals,
	b3GetHullSoaVertices,
	b3GetHullVertices,
	b3HullData,
	b3LocalManifold,
	b3LocalManifoldPoint,
	b3SATCache,
	b3SimplexCache,
	b3Sphere,
	b3BoxHull,
} from "../../lib/collision";
import {
	b3CloneAndTransformHull,
	b3CreateCone,
	b3CreateComplexHull,
	b3CreateCylinder,
	b3CreateHull,
	b3CreateRock,
	b3GetHullSoaStride,
	b3MakeBoxHull,
	b3MakeCubeHull,
	b3MakeOffsetBoxHull,
	b3MakeScaledBoxHull,
	b3MakeTransformedBoxHull,
} from "../../lib/hull";
import { b3AxisQuery, b3MakeFeatureId, b3SeparatingAxis } from "../../lib/manifold";

export const POINT_BUFFER: i32 = 16;

export const g_pointBuffer = new StaticArray<b3LocalManifoldPoint>(POINT_BUFFER);
export const g_manifold = new b3LocalManifold();

// Sentinels everywhere, so a field or slot a collide function skips prints the same on both sides.
export function resetManifold(offset: i32): void {
	g_manifold.normal = b3Vec3(11.0, 12.0, 13.0);
	g_manifold.triangleNormal = b3Vec3(14.0, 15.0, 16.0);
	g_manifold.points = g_pointBuffer;
	g_manifold.pointOffset = offset;
	g_manifold.pointCount = 77;
	g_manifold.triangleIndex = 78;
	g_manifold.i1 = 79;
	g_manifold.i2 = 80;
	g_manifold.i3 = 81;
	g_manifold.squaredDistance = 82.0;
	g_manifold.feature = 83;
	g_manifold.triangleFlags = 84;

	for (let k = 0; k < POINT_BUFFER; ++k) {
		ref p = g_pointBuffer[k];
		p.point = b3Vec3(100.0 + <f32>k, 200.0 + <f32>k, 300.0 + <f32>k);
		p.separation = 400.0 + <f32>k;
		p.pair.owner1 = <u8>k;
		p.pair.index1 = <u8>(k + 1);
		p.pair.owner2 = <u8>(k + 2);
		p.pair.index2 = <u8>(k + 3);
		p.triangleIndex = 500 + k;
	}
}

export function ppair(pair: b3FeaturePair): void {
	vi32(<i32>pair.owner1);
	vi32(<i32>pair.index1);
	vi32(<i32>pair.owner2);
	vi32(<i32>pair.index2);
	vu32(b3MakeFeatureId(pair));
}

// The manifold fields, then every buffer slot below slotCount.
export function pmanifold(slotCount: i32): void {
	vtag("m");
	vi32(g_manifold.pointCount);
	pv3(g_manifold.normal);
	pv3(g_manifold.triangleNormal);
	vi32(g_manifold.triangleIndex);
	vi32(g_manifold.i1);
	vi32(g_manifold.i2);
	vi32(g_manifold.i3);
	vf32(g_manifold.squaredDistance);
	vi32(g_manifold.feature);
	vi32(g_manifold.triangleFlags);
	vend();

	for (let k = 0; k < slotCount; ++k) {
		let p = g_pointBuffer[k];
		vtag("p");
		vi32(k);
		pv3(p.point);
		vf32(p.separation);
		ppair(p.pair);
		vi32(p.triangleIndex);
		vend();
	}
}

export function psat(c: b3SATCache): void {
	vf32(c.separation);
	vi32(<i32>c.type);
	vi32(<i32>c.indexA);
	vi32(<i32>c.indexB);
	vi32(<i32>c.hit);
}

export function psimplexCache(c: b3SimplexCache): void {
	vf32(c.metric);
	vi32(<i32>c.count);
	for (let i = 0; i < 4; ++i) {
		vi32(<i32>c.indexA[i]);
	}
	for (let i = 0; i < 4; ++i) {
		vi32(<i32>c.indexB[i]);
	}
}

export function paxis(a: b3SeparatingAxis): void {
	pv3(a.normal);
	vf32(a.separation);
	vi32(a.indexA);
	vi32(a.indexB);
	vi32(a.type);
}

export function pquery(q: b3AxisQuery): void {
	paxis(q.faceA);
	paxis(q.faceB);
	paxis(q.edge);
	vi32(q.separatedFeature);
}

export function pplane(p: b3Plane): void {
	pv3(p.normal);
	vf32(p.offset);
}

export function phull(index: i32, hull: b3HullData): void {
	vtag("hull");
	vi32(index);
	vi32(hull.vertexCount);
	vi32(hull.edgeCount);
	vi32(hull.faceCount);
	pab(hull.aabb);
	vf32(hull.surfaceArea);
	vf32(hull.volume);
	vf32(hull.innerRadius);
	pv3(hull.center);
	pm3(hull.centralInertia);
	vend();

	let points = b3GetHullPoints(hull);
	let vertices = b3GetHullVertices(hull);
	for (let i = 0; i < hull.vertexCount; ++i) {
		vtag("hv");
		pv3(points[i]);
		vi32(<i32>vertices[i].edge);
		vend();
	}

	let edges = b3GetHullEdges(hull);
	let cosines = b3GetHullEdgeCosines(hull);
	for (let i = 0; i < hull.edgeCount; ++i) {
		vtag("he");
		vi32(<i32>edges[i].next);
		vi32(<i32>edges[i].twin);
		vi32(<i32>edges[i].origin);
		vi32(<i32>edges[i].face);
		if ((i & 1) == 0) {
			vf32(cosines[i >> 1]);
		}
		vend();
	}

	let planes = b3GetHullPlanes(hull);
	let faces = b3GetHullFaces(hull);
	for (let i = 0; i < hull.faceCount; ++i) {
		vtag("hf");
		pplane(planes[i]);
		vi32(<i32>faces[i].edge);
		vend();
	}

	let vertexStride = b3GetHullSoaStride(hull.vertexCount);
	let soaVertices = b3GetHullSoaVertices(hull);
	vtag("hsv");
	for (let i = 0; i < 3 * vertexStride; ++i) {
		vf32(soaVertices[i]);
	}
	vend();

	let faceStride = b3GetHullSoaStride(hull.faceCount);
	let soaNormals = b3GetHullSoaNormals(hull);
	vtag("hsn");
	for (let i = 0; i < 3 * faceStride; ++i) {
		vf32(soaNormals[i]);
	}
	vend();
}

export const g_hulls = new Array<b3HullData>();

function addBox(box: b3BoxHull): void {
	g_hulls.push(box.base);
}

function addHull(hull: b3HullData?): void {
	vtag("add");
	vi32(g_hulls.length);
	vbool(hull != null);
	vend();
	if (hull != null) {
		g_hulls.push(hull);
	}
}

const g_cloud = new StaticArray<b3Vec3>(B3_MAX_HULL_VERTICES * 16);

function hullFromSet(set: i32): b3HullData? {
	let offset = hullSets[3 * set + 0];
	let count = hullSets[3 * set + 1];
	let maxCount = hullSets[3 * set + 2];
	for (let i = 0; i < count; ++i) {
		let k = offset + i;
		g_cloud[i].x = vbitsf(hullPointBits[3 * k + 0]);
		g_cloud[i].y = vbitsf(hullPointBits[3 * k + 1]);
		g_cloud[i].z = vbitsf(hullPointBits[3 * k + 2]);
	}
	return b3CreateHull(g_cloud, count, maxCount);
}

function randomCloudHull(count: i32, s: f32): b3HullData? {
	for (let i = 0; i < count; ++i) {
		g_cloud[i] = rv(-s, s);
	}
	return b3CreateHull(g_cloud, count, B3_MAX_HULL_VERTICES);
}

export function buildHulls(): void {
	vseed(0x4d414e49464f4c44);
	g_hulls.length = 0;

	addBox(b3MakeCubeHull(0.5));
	addBox(b3MakeBoxHull(1.0, 0.1, 2.0));
	addBox(b3MakeBoxHull(0.05, 0.5, 0.05));
	addBox(b3MakeOffsetBoxHull(0.3, 0.4, 0.5, b3Vec3(0.2, -0.1, 0.3)));

	let boxTransform: b3Transform;
	boxTransform.p = b3Vec3(0.1, 0.2, -0.3);
	boxTransform.q = b3MakeQuatFromAxisAngle(b3Normalize(b3Vec3(1.0, 1.0, 0.0)), 0.7);
	addBox(b3MakeTransformedBoxHull(0.6, 0.2, 0.4, boxTransform));

	let scaleTransform: b3Transform;
	scaleTransform.p = b3Vec3(0.0, 0.1, 0.0);
	scaleTransform.q = b3MakeQuatFromAxisAngle(b3Vec3(0.0, 0.0, 1.0), 0.5);
	addBox(b3MakeScaledBoxHull(b3Vec3(0.5, 0.4, 0.3), scaleTransform, b3Vec3(1.0, 2.0, 0.5)));

	addHull(b3CreateCylinder(1.0, 0.4, 0.0, 12));
	addHull(b3CreateCylinder(0.2, 1.0, 0.1, 24));
	addHull(b3CreateCone(1.0, 0.5, 0.1, 10));
	addHull(b3CreateCone(0.6, 0.3, 0.3, 6));

	let rockIndex = g_hulls.length;
	addHull(b3CreateRock(0.5));
	addHull(b3CreateComplexHull(0.6));

	const sets: StaticArray<i32> = [0, 1, 2, 3, 4, 41, 42, 43, 44];
	let setBase = g_hulls.length;
	for (let i = 0; i < 9; ++i) {
		addHull(hullFromSet(sets[i]));
	}

	let cloneTransform: b3Transform;
	cloneTransform.p = b3Vec3(-0.2, 0.1, 0.05);
	cloneTransform.q = b3MakeQuatFromAxisAngle(b3Normalize(b3Vec3(1.0, -2.0, 0.5)), 1.1);
	addHull(b3CloneAndTransformHull(g_hulls[rockIndex], cloneTransform, b3Vec3(1.5, 0.5, 1.0)));
	addHull(b3CloneAndTransformHull(g_hulls[setBase + 4], b3Transform_identity, b3Vec3(0.3, 2.0, 1.0)));

	addHull(randomCloudHull(8, 0.6));
	addHull(randomCloudHull(20, 0.7));
	addHull(randomCloudHull(60, 1.2));

	for (let i = 0; i < g_hulls.length; ++i) {
		phull(i, g_hulls[i]);
	}
}

// Largest dot of the rotated hull points with d.
export function hullExtent(hull: b3HullData, q: b3Quat, d: b3Vec3): f32 {
	let points = b3GetHullPoints(hull);
	let best = -FLT_MAX;
	for (let i = 0; i < hull.vertexCount; ++i) {
		let e = b3Dot(b3RotateVector(q, points[i]), d);
		if (e > best) {
			best = e;
		}
	}
	return best;
}

export function axisDirection(k: i32): b3Vec3 {
	let v = b3Vec3_zero;
	let s: f32 = (k & 1) != 0 ? -1.0 : 1.0;
	if (k >> 1 == 0) {
		v.x = s;
	} else if (k >> 1 == 1) {
		v.y = s;
	} else {
		v.z = s;
	}
	return v;
}

// Identity or a quarter or half turn about a principal axis.
function rquatAligned(): b3Quat {
	let k = vrandint(0, 7);
	if (k == 0) {
		return b3Quat_identity;
	}
	let axis = vrandint(0, 3);
	let angle: f32 = k < 4 ? 0.5 * B3_PI : B3_PI;
	return b3MakeQuatFromAxisAngle(axisDirection(2 * axis), angle);
}

// Two 45 degree turns, so edges cross.
export function rquatEdge(): b3Quat {
	let a = vrandint(0, 3);
	let b = vrandint(0, 3);
	let jitter = vrandf(-0.05, 0.05);
	let qa = b3MakeQuatFromAxisAngle(axisDirection(2 * a), 0.25 * B3_PI);
	let qb = b3MakeQuatFromAxisAngle(axisDirection(2 * b), 0.25 * B3_PI + jitter);
	return b3NormalizeQuat(b3MulQuat(qa, qb));
}

export const POSE_MODE_COUNT: i32 = 8;

// Pose of a shape with extent eB along -u (already rotated by q) against a hull A extent along u.
// modes: 0 near touch, 1 deep, 2 separated, 3 aligned face, 4 crossing edges, 5 coincident, 6 exact touch, 7 far
export function placePose(eA: f32, eB: f32, u: b3Vec3, q: b3Quat, gap: f32, mode: i32): b3Transform {
	let xf: b3Transform;
	xf.q = q;
	xf.p = b3MulSV(eA + eB + gap, u);
	if (mode == 3) {
		let lateral = rv(-0.2, 0.2);
		lateral = b3MulSub(lateral, b3Dot(lateral, u), u);
		xf.p = b3Add(xf.p, lateral);
	}
	if (mode == 7) {
		xf.p = b3Add(xf.p, b3Vec3(900.0, -1300.0, 400.0));
	}
	return xf;
}

export function poseRotation(mode: i32): b3Quat {
	if (mode == 3) {
		return rquatAligned();
	}
	if (mode == 4) {
		return rquatEdge();
	}
	return rquatOrIdentity();
}

export function poseDirection(mode: i32): b3Vec3 {
	if (mode == 3) {
		let k = vrandint(0, 6);
		return axisDirection(k);
	}
	return runit();
}

export function poseGap(mode: i32): f32 {
	switch (mode) {
		case 0:
			return vrandf(-0.05, 0.05);
		case 1:
			return vrandf(-0.6, -0.05);
		case 2:
			return vrandf(0.03, 0.8);
		case 3:
		case 4:
			return vrandf(-0.03, 0.03);
		case 6:
			return 0.0;
		default:
			return vrandf(-0.1, 0.1);
	}
}

export function hullPose(hullA: b3HullData, hullB: b3HullData, mode: i32): b3Transform {
	if (mode == 5) {
		let xf: b3Transform;
		xf.q = rquatOrIdentity();
		xf.p = rv(-0.01, 0.01);
		return xf;
	}

	let q = poseRotation(mode);
	let u = poseDirection(mode);
	let gap = poseGap(mode);
	let eA = hullExtent(hullA, b3Quat_identity, u);
	let eB = hullExtent(hullB, q, b3Neg(u));
	return placePose(eA, eB, u, q, gap, mode);
}

// Small motion most steps, a repeat at step 1, a jump now and then.
export function advancePose(xf: b3Transform, step: i32): b3Transform {
	if (step == 0) {
		return xf;
	}
	let kind = vrandint(0, 8);
	if (kind == 0) {
		xf.p = b3Add(xf.p, rv(-0.1, 0.1));
		xf.q = rotateBy(xf.q, 0.3);
	} else {
		xf.p = b3Add(xf.p, rv(-0.005, 0.005));
		xf.q = rotateBy(xf.q, 0.01);
	}
	return xf;
}

// Mostly the contact capacity, sometimes the edges of each capacity test.
export function pickCapacity(): i32 {
	let k = vrandint(0, 16);
	if (k < 11) {
		return B3_MAX_MANIFOLD_POINTS;
	}
	if (k < 13) {
		return 8;
	}
	return vrandint(0, 4);
}

export function pickOffset(): i32 {
	if ((vrand32() & 1) != 0) {
		return 0;
	}
	return vrandint(1, 5);
}

export function randomCapsule(): b3Capsule {
	let c: b3Capsule;
	c.center1 = rv(-0.5, 0.5);
	let kind = vrandint(0, 8);
	let length: f32;
	if (kind == 0) {
		length = 0.0;
	} else if (kind == 1) {
		length = vrandf(0.0, 0.01);
	} else {
		length = vrandf(0.1, 2.0);
	}
	let axis = runit();
	c.center2 = b3MulAdd(c.center1, length, axis);
	c.radius = vrandf(0.02, 0.5);
	return c;
}

export function randomSphere(): b3Sphere {
	let s: b3Sphere;
	s.center = rv(-0.5, 0.5);
	let kind = vrandint(0, 8);
	if (kind == 0) {
		s.radius = 0.0;
	} else {
		s.radius = vrandf(0.01, 1.0);
	}
	return s;
}
