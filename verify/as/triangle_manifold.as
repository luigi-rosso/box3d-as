// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/triangle_manifold.as, mirrors verify/cref/triangle_manifold.c

import { vtag, vi32, vbool, vf32, vend, vseed, vrandint, vrandf } from "./common";
import { pv3, rquatOrIdentity, runit, rv } from "./distance_common";
import {
	POSE_MODE_COUNT,
	advancePose,
	buildHulls,
	g_hulls,
	g_manifold,
	hullExtent,
	pickCapacity,
	pickOffset,
	pmanifold,
	poseGap,
	poseRotation,
	psat,
	psimplexCache,
	resetManifold,
} from "./manifold_common";
import { b3SetLengthUnitsPerMeter } from "../../lib/core";
import {
	B3_PI,
	b3Add,
	b3ComputeCosSin,
	b3Cross,
	b3InvTransformPoint,
	b3Lerp,
	b3MulAdd,
	b3MulSub,
	b3Neg,
	b3Normalize,
	b3RotateVector,
	b3Sub,
	b3Transform,
	b3Vec3,
	b3Vec3_zero,
} from "../../lib/math_functions";
import {
	b3Capsule,
	b3SATCache,
	b3SimplexCache,
	b3Sphere,
	b3_concaveEdge1,
	b3_concaveEdge2,
	b3_concaveEdge3,
	b3_inverseConcaveEdge1,
	b3_inverseConcaveEdge2,
	b3_inverseConcaveEdge3,
	b3_manualFaceAxisA,
} from "../../lib/collision";
import {
	b3CollideTriangleAndCapsule,
	b3CollideTriangleAndHull,
	b3CollideTriangleAndSphere,
} from "../../lib/triangle_manifold";

export { frameBoundary } from "./common";

const STEP_COUNT: i32 = 6;

const g_triangle = new StaticArray<b3Vec3>(3);
const g_world = new StaticArray<b3Vec3>(3);

function ptri(t: StaticArray<b3Vec3>): void {
	pv3(t[0]);
	pv3(t[1]);
	pv3(t[2]);
}

// kinds: 0 random, 1 near equilateral, 2 sliver, 3 large, 4 tiny, 5 axis aligned right triangle
function randomTriangle(t: StaticArray<b3Vec3>): i32 {
	let kind = vrandint(0, 6);
	switch (kind) {
		case 0:
			t[0] = rv(-1.0, 1.0);
			t[1] = rv(-1.0, 1.0);
			t[2] = rv(-1.0, 1.0);
			break;

		case 1: {
			let center = rv(-0.5, 0.5);
			let axis = runit();
			let tangent = b3Normalize(b3Cross(axis, runit()));
			let bitangent = b3Cross(axis, tangent);
			let radius = vrandf(0.2, 2.0);
			let twoThirds: f32 = <f32>2.0 / <f32>3.0;
			for (let k = 0; k < 3; ++k) {
				let jitter = vrandf(-0.1, 0.1);
				let angle = twoThirds * B3_PI * <f32>k + jitter;
				let cs = b3ComputeCosSin(angle);
				let p = b3MulAdd(center, radius * cs.cosine, tangent);
				t[k] = b3MulAdd(p, radius * cs.sine, bitangent);
			}
			break;
		}

		case 2: {
			t[0] = rv(-1.0, 1.0);
			let d = runit();
			let length = vrandf(0.5, 2.0);
			t[1] = b3MulAdd(t[0], length, d);
			let s = vrandf(0.1, 0.9);
			let off = runit();
			t[2] = b3MulAdd(b3Lerp(t[0], t[1], s), 0.01, off);
			break;
		}

		case 3:
			t[0] = rv(-10.0, 10.0);
			t[1] = rv(-10.0, 10.0);
			t[2] = rv(-10.0, 10.0);
			break;

		case 4:
			t[0] = rv(-0.05, 0.05);
			t[1] = rv(-0.05, 0.05);
			t[2] = rv(-0.05, 0.05);
			break;

		default: {
			let s = vrandf(0.5, 3.0);
			t[0] = b3Vec3_zero;
			t[1] = b3Vec3(0.0, 0.0, s);
			t[2] = b3Vec3(s, 0.0, 0.0);
			break;
		}
	}
	return kind;
}

function triangleNormal(t: StaticArray<b3Vec3>): b3Vec3 {
	return b3Normalize(b3Cross(b3Sub(t[1], t[0]), b3Sub(t[2], t[0])));
}

// A point in the triangle plane, sometimes outside the triangle.
function trianglePoint(t: StaticArray<b3Vec3>): b3Vec3 {
	let b1 = vrandf(-0.3, 1.3);
	let b2 = vrandf(-0.3, 1.3);
	if (b1 + b2 > 1.6) {
		b1 = 1.0 - b1;
		b2 = 1.0 - b2;
	}
	let p = b3MulAdd(t[0], b1, b3Sub(t[1], t[0]));
	return b3MulAdd(p, b2, b3Sub(t[2], t[0]));
}

function triangleGap(mode: i32): f32 {
	if (mode == 5) {
		// Below the plane
		return vrandf(-1.5, -0.2);
	}
	return poseGap(mode);
}

function edgeFlags(combo: i32): i32 {
	let flags = 0;
	if ((combo & 1) != 0) {
		flags |= b3_concaveEdge1;
	}
	if ((combo & 2) != 0) {
		flags |= b3_concaveEdge2;
	}
	if ((combo & 4) != 0) {
		flags |= b3_concaveEdge3;
	}
	if ((combo & 8) != 0) {
		flags |= b3_inverseConcaveEdge1;
	}
	if ((combo & 16) != 0) {
		flags |= b3_inverseConcaveEdge2;
	}
	if ((combo & 32) != 0) {
		flags |= b3_inverseConcaveEdge3;
	}
	return flags;
}

export function begin(): void {
	buildHulls();
	vseed(0x747269616e676c65);
}

export function setLengthUnits(units: f32): void {
	b3SetLengthUnitsPerMeter(units);
}

export function runSpheres(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let kind = randomTriangle(g_triangle);
		let n = triangleNormal(g_triangle);
		let p = trianglePoint(g_triangle);
		let mode = vrandint(0, POSE_MODE_COUNT);
		let sphere: b3Sphere;
		sphere.radius = vrandf(0.0, 0.6);
		let gap = triangleGap(mode);
		sphere.center = b3MulAdd(p, sphere.radius + gap, n);
		if (mode == 7) {
			// Center on the triangle
			sphere.center = p;
		}
		let capacity = vrandint(0, 3);
		let offset = pickOffset();

		resetManifold(offset);
		vtag("ts");
		vi32(i);
		vi32(kind);
		vi32(mode);
		vi32(capacity);
		vi32(offset);
		ptri(g_triangle);
		pv3(sphere.center);
		vf32(sphere.radius);
		vend();
		b3CollideTriangleAndSphere(g_manifold, capacity, g_triangle, sphere);
		pmanifold(offset + 2);
	}
}

export function runCapsules(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let kind = randomTriangle(g_triangle);
		let n = triangleNormal(g_triangle);
		let p = trianglePoint(g_triangle);
		let mode = vrandint(0, POSE_MODE_COUNT);
		let pose = vrandint(0, 5);
		let capsule: b3Capsule;
		capsule.radius = vrandf(0.02, 0.5);
		let length = vrandf(0.0, 2.0);
		if (vrandint(0, 16) == 0) {
			// A point capsule leaves every edge axis undefined
			length = 0.0;
		}
		let gap = triangleGap(mode);
		let axis: b3Vec3;
		if (pose == 0) {
			// Lying on the plane
			axis = b3Normalize(b3Cross(n, runit()));
		} else if (pose == 1) {
			// Standing
			axis = n;
		} else if (pose == 2) {
			// Across a triangle edge
			let e = vrandint(0, 3);
			let edge = b3Sub(g_triangle[(e + 1) % 3], g_triangle[e]);
			let side = b3Normalize(b3Cross(edge, n));
			axis = b3Normalize(b3MulAdd(side, vrandf(-0.5, 0.5), n));
			p = b3Lerp(g_triangle[e], g_triangle[(e + 1) % 3], vrandf(0.0, 1.0));
			p = b3MulSub(p, 0.5 * length, axis);
		} else if (pose == 3) {
			axis = runit();
		} else {
			// Parallel to a triangle edge
			let e = vrandint(0, 3);
			axis = b3Normalize(b3Sub(g_triangle[(e + 1) % 3], g_triangle[e]));
		}
		capsule.center1 = b3MulAdd(p, capsule.radius + gap, n);
		capsule.center2 = b3MulAdd(capsule.center1, length, axis);
		if (mode == 7) {
			capsule.center1 = b3MulSub(p, 0.5 * length, axis);
			capsule.center2 = b3MulAdd(p, 0.5 * length, axis);
		}

		vtag("tc");
		vi32(i);
		vi32(kind);
		vi32(mode);
		vi32(pose);
		ptri(g_triangle);
		vend();

		let cache = b3SimplexCache();
		for (let step = 0; step < STEP_COUNT; ++step) {
			if (step > 0) {
				let jump = vrandint(0, 8);
				let scale: f32 = jump == 0 ? 0.1 : 0.005;
				let delta = rv(-scale, scale);
				capsule.center1 = b3Add(capsule.center1, delta);
				let delta2 = rv(-scale, scale);
				capsule.center2 = b3Add(capsule.center2, delta2);
			}
			let capacity = vrandint(0, 5);
			let offset = pickOffset();
			resetManifold(offset);
			vtag("tcc");
			vi32(step);
			vi32(capacity);
			vi32(offset);
			pv3(capsule.center1);
			pv3(capsule.center2);
			vf32(capsule.radius);
			psimplexCache(cache);
			vend();
			b3CollideTriangleAndCapsule(g_manifold, capacity, g_triangle, capsule, cache);
			pmanifold(offset + 4);
			vtag("out");
			psimplexCache(cache);
			vend();
		}
	}
}

export function runHulls(phase: i32, count: i32): void {
	for (let i = 0; i < count; ++i) {
		let kind = randomTriangle(g_world);
		let n = triangleNormal(g_world);
		let p = trianglePoint(g_world);
		let ih = vrandint(0, g_hulls.length);
		let hull = g_hulls[ih];
		let mode = vrandint(0, POSE_MODE_COUNT + 1);
		let xf: b3Transform;
		if (mode == POSE_MODE_COUNT) {
			// Hull center near the plane, for the back side hysteresis
			xf.q = rquatOrIdentity();
			let height = vrandf(-0.02, 0.005);
			let center = b3MulAdd(p, height, n);
			xf.p = b3Sub(center, b3RotateVector(xf.q, hull.center));
		} else {
			let q = poseRotation(mode);
			let gap = triangleGap(mode);
			let eB = hullExtent(hull, q, b3Neg(n));
			xf.q = q;
			xf.p = b3MulAdd(p, eB + gap, n);
			if (mode == 7) {
				xf.p = b3Sub(p, b3RotateVector(q, hull.center));
			}
		}

		vtag("th");
		vi32(phase);
		vi32(i);
		vi32(kind);
		vi32(ih);
		vi32(mode);
		ptri(g_world);
		vend();

		let cache = b3SATCache();
		for (let step = 0; step < STEP_COUNT; ++step) {
			xf = advancePose(xf, step);
			for (let k = 0; k < 3; ++k) {
				g_triangle[k] = b3InvTransformPoint(xf, g_world[k]);
			}

			let capacity = pickCapacity();
			let offset = pickOffset();
			let combo = (7 * i + step) & 63;
			let flags = edgeFlags(combo);
			let enableSpeculative = vrandint(0, 4) != 0;
			let manual = vrandint(0, 24);
			if (manual < 3) {
				cache.type = <u8>(b3_manualFaceAxisA + manual);
			}

			resetManifold(offset);
			vtag("thc");
			vi32(step);
			vi32(capacity);
			vi32(offset);
			vi32(flags);
			vbool(enableSpeculative);
			ptri(g_triangle);
			psat(cache);
			vend();
			b3CollideTriangleAndHull(
				g_manifold,
				capacity,
				g_triangle[0],
				g_triangle[1],
				g_triangle[2],
				flags,
				hull,
				cache,
				enableSpeculative,
			);
			pmanifold(offset + capacity);
			vtag("out");
			psat(cache);
			vend();
		}
	}
}
