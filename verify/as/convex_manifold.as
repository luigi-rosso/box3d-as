// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/convex_manifold.as, mirrors verify/cref/convex_manifold.c

import { vtag, vi32, vf32, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import { pv3, pxf, runit, rv, rquatOrIdentity } from "./distance_common";
import {
	POSE_MODE_COUNT,
	advancePose,
	axisDirection,
	buildHulls,
	g_hulls,
	g_manifold,
	hullExtent,
	hullPose,
	pickCapacity,
	pickOffset,
	placePose,
	pmanifold,
	poseDirection,
	poseGap,
	poseRotation,
	pquery,
	psat,
	psimplexCache,
	randomCapsule,
	randomSphere,
	rquatEdge,
	resetManifold,
} from "./manifold_common";
import { b3SetLengthUnitsPerMeter } from "../../lib/core";
import { B3_MAX_MANIFOLD_POINTS } from "../../lib/constants";
import {
	b3Add,
	b3Cross,
	b3Dot,
	b3InvRotateVector,
	b3Length,
	b3Lerp,
	b3MaxFloat,
	b3MulAdd,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Normalize,
	b3Quat_identity,
	b3RotateVector,
	b3Sub,
	b3Transform,
	b3Transform_identity,
	b3Vec3,
	b3Vec3_zero,
} from "../../lib/math_functions";
import { b3SATCache, b3SimplexCache, b3Sphere, b3_manualFaceAxisA } from "../../lib/collision";
import {
	b3CollideCapsuleAndSphere,
	b3CollideCapsules,
	b3CollideHullAndCapsule,
	b3CollideHullAndSphere,
	b3CollideHulls,
	b3CollideHullsAtWidth,
	b3CollideSpheres,
	b3ComputeSeparatingAxis,
} from "../../lib/convex_manifold";

export { frameBoundary } from "./common";

const STEP_COUNT: i32 = 6;

function pcall(tag: string, step: i32, capacity: i32, offset: i32, xf: b3Transform): void {
	vtag(tag);
	vi32(step);
	vi32(capacity);
	vi32(offset);
	pxf(xf);
	vend();
}

export function begin(): void {
	buildHulls();
	vseed(0x636f6e766578);
}

export function setLengthUnits(units: f32): void {
	b3SetLengthUnitsPerMeter(units);
}

export function runHulls(phase: i32, count: i32): void {
	for (let i = 0; i < count; ++i) {
		let ia = vrandint(0, g_hulls.length);
		let ib = vrandint(0, g_hulls.length);
		let mode = vrandint(0, POSE_MODE_COUNT);
		let hullA = g_hulls[ia];
		let hullB = g_hulls[ib];
		let xf = hullPose(hullA, hullB, mode);

		vtag("hh");
		vi32(phase);
		vi32(i);
		vi32(ia);
		vi32(ib);
		vi32(mode);
		vend();

		let early = b3ComputeSeparatingAxis(hullA, hullB, xf, true);
		vtag("sat1");
		pquery(early);
		vend();
		let full = b3ComputeSeparatingAxis(hullA, hullB, xf, false);
		vtag("sat0");
		pquery(full);
		vend();

		let cache = b3SATCache();
		for (let step = 0; step < STEP_COUNT; ++step) {
			xf = advancePose(xf, step);
			let capacity = pickCapacity();
			let offset = pickOffset();
			let manual = vrandint(0, 24);
			if (manual < 3) {
				cache.type = <u8>(b3_manualFaceAxisA + manual);
			}
			let width = (vrand32() & 1) != 0 ? 4 : 8;

			resetManifold(offset);
			pcall("hhc", step, capacity, offset, xf);
			vtag("in");
			vi32(width);
			psat(cache);
			vend();

			if (width == 4) {
				b3CollideHulls(g_manifold, capacity, hullA, hullB, xf, cache);
			} else {
				b3CollideHullsAtWidth(g_manifold, capacity, hullA, hullB, xf, cache, width);
			}

			pmanifold(offset + capacity);
			vtag("out");
			psat(cache);
			vend();
		}
	}
}

// The test_manifold.c style slide: two boxes moved along x through touch, cache fed back.
export function runSlide(phase: i32): void {
	for (let k = 0; k < 6; ++k) {
		let hullA = g_hulls[k];
		let hullB = g_hulls[(k + 1) % 6];
		let q = k < 3 ? b3Quat_identity : rquatEdge();
		let u = axisDirection(2 * (k % 3));
		let eA = hullExtent(hullA, b3Quat_identity, u);
		let eB = hullExtent(hullB, q, b3Neg(u));
		let cache = b3SATCache();
		for (let s = 0; s < 80; ++s) {
			let gap: f32 = -0.2 + 0.005 * <f32>s;
			let xf: b3Transform;
			xf.q = q;
			xf.p = b3MulSV(eA + eB + gap, u);
			resetManifold(0);
			vtag("slide");
			vi32(phase);
			vi32(k);
			vi32(s);
			pxf(xf);
			psat(cache);
			vend();
			b3CollideHulls(g_manifold, B3_MAX_MANIFOLD_POINTS, hullA, hullB, xf, cache);
			pmanifold(B3_MAX_MANIFOLD_POINTS);
			vtag("out");
			psat(cache);
			vend();
		}
	}
}

// A lerp that may leave [0, 1]
function extend(a: b3Vec3, b: b3Vec3, t: f32): b3Vec3 {
	return b3MulAdd(a, t, b3Sub(b, a));
}

function spherePose(centerA: b3Vec3, extentA: f32, sphereB: b3Sphere, mode: i32): b3Transform {
	let xf: b3Transform;
	xf.q = rquatOrIdentity();
	if (mode == 5) {
		// The rotated center of B lands near center A
		let rotated = b3RotateVector(xf.q, sphereB.center);
		xf.p = b3Sub(centerA, rotated);
		return xf;
	}
	let u = runit();
	let gap = poseGap(mode);
	let rotated = b3RotateVector(xf.q, sphereB.center);
	let target = b3MulAdd(centerA, extentA + sphereB.radius + gap, u);
	xf.p = b3Sub(target, rotated);
	if (mode == 7) {
		xf.p = b3Add(xf.p, b3Vec3(900.0, -1300.0, 400.0));
	}
	return xf;
}

export function runSpheres(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let sphereA = randomSphere();
		let sphereB = randomSphere();
		let mode = vrandint(0, POSE_MODE_COUNT);
		let xf = spherePose(sphereA.center, sphereA.radius, sphereB, mode);
		let capacity = vrandint(0, 3);
		let offset = pickOffset();

		resetManifold(offset);
		vtag("ss");
		vi32(i);
		vi32(mode);
		pv3(sphereA.center);
		vf32(sphereA.radius);
		pv3(sphereB.center);
		vf32(sphereB.radius);
		vend();
		pcall("ssc", 0, capacity, offset, xf);
		b3CollideSpheres(g_manifold, capacity, sphereA, sphereB, xf);
		pmanifold(offset + 2);
	}
}

export function runCapsuleSphere(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let capsuleA = randomCapsule();
		let sphereB = randomSphere();
		let mode = vrandint(0, POSE_MODE_COUNT);
		let t = vrandf(-0.3, 1.3);
		let onAxis = extend(capsuleA.center1, capsuleA.center2, t);
		let xf = spherePose(onAxis, capsuleA.radius, sphereB, mode);
		let capacity = vrandint(0, 3);
		let offset = pickOffset();

		resetManifold(offset);
		vtag("cs");
		vi32(i);
		vi32(mode);
		pv3(capsuleA.center1);
		pv3(capsuleA.center2);
		vf32(capsuleA.radius);
		pv3(sphereB.center);
		vf32(sphereB.radius);
		vend();
		pcall("csc", 0, capacity, offset, xf);
		b3CollideCapsuleAndSphere(g_manifold, capacity, capsuleA, sphereB, xf);
		pmanifold(offset + 2);
	}
}

export function runCapsules(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let capsuleA = randomCapsule();
		let capsuleB = randomCapsule();
		let kind = vrandint(0, 6);
		let xf: b3Transform;

		if (kind == 0) {
			// Parallel: B along A's axis, offset sideways by the radii plus a gap
			let axisA = b3Normalize(b3Sub(capsuleA.center2, capsuleA.center1));
			let axisB = b3Sub(capsuleB.center2, capsuleB.center1);
			let lengthB = b3Length(axisB);
			let side = b3Normalize(b3Cross(axisA, runit()));
			let gap = vrandf(-0.05, 0.05);
			let slide = vrandf(-0.5, 1.5);
			let tilt = vrandf(-0.04, 0.04);
			let dirB = b3Normalize(b3MulAdd(axisA, tilt, side));
			let start = extend(capsuleA.center1, capsuleA.center2, slide);
			start = b3MulAdd(start, capsuleA.radius + capsuleB.radius + gap, side);
			capsuleB.center1 = b3Vec3_zero;
			capsuleB.center2 = b3MulSV(lengthB, dirB);
			xf.q = b3Quat_identity;
			xf.p = start;
		} else if (kind == 1) {
			// Coincident
			capsuleB = capsuleA;
			xf = b3Transform_identity;
		} else {
			let mode = vrandint(0, POSE_MODE_COUNT);
			let t = vrandf(0.0, 1.0);
			let probe = b3Sphere(b3Lerp(capsuleB.center1, capsuleB.center2, t), capsuleB.radius);
			let s = vrandf(-0.2, 1.2);
			let onAxis = extend(capsuleA.center1, capsuleA.center2, s);
			xf = spherePose(onAxis, capsuleA.radius, probe, mode);
		}

		let capacity = vrandint(0, 5);
		let offset = pickOffset();

		resetManifold(offset);
		vtag("cc");
		vi32(i);
		vi32(kind);
		pv3(capsuleA.center1);
		pv3(capsuleA.center2);
		vf32(capsuleA.radius);
		pv3(capsuleB.center1);
		pv3(capsuleB.center2);
		vf32(capsuleB.radius);
		vend();
		pcall("ccc", 0, capacity, offset, xf);
		b3CollideCapsules(g_manifold, capacity, capsuleA, capsuleB, xf);
		pmanifold(offset + 4);
	}
}

export function runHullSphere(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let ia = vrandint(0, g_hulls.length);
		let hullA = g_hulls[ia];
		let sphereB = randomSphere();
		let mode = vrandint(0, POSE_MODE_COUNT);
		let xf: b3Transform;
		if (mode == 5) {
			// Deep inside
			xf.q = rquatOrIdentity();
			let inside = b3MulAdd(hullA.center, 0.1, rv(-1.0, 1.0));
			xf.p = b3Sub(inside, b3RotateVector(xf.q, sphereB.center));
		} else {
			let u = poseDirection(mode);
			let eA = hullExtent(hullA, b3Quat_identity, u);
			xf = spherePose(b3MulSV(eA, u), 0.0, sphereB, mode);
		}

		vtag("hs");
		vi32(i);
		vi32(ia);
		vi32(mode);
		pv3(sphereB.center);
		vf32(sphereB.radius);
		vend();

		let cache = b3SimplexCache();
		for (let step = 0; step < STEP_COUNT; ++step) {
			xf = advancePose(xf, step);
			let capacity = vrandint(0, 3);
			let offset = pickOffset();
			resetManifold(offset);
			pcall("hsc", step, capacity, offset, xf);
			vtag("in");
			psimplexCache(cache);
			vend();
			b3CollideHullAndSphere(g_manifold, capacity, hullA, sphereB, xf, cache);
			pmanifold(offset + 2);
			vtag("out");
			psimplexCache(cache);
			vend();
		}
	}
}

export function runHullCapsule(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let ia = vrandint(0, g_hulls.length);
		let hullA = g_hulls[ia];
		let capsuleB = randomCapsule();
		let mode = vrandint(0, POSE_MODE_COUNT);
		let xf: b3Transform;
		if (mode == 5) {
			xf.q = rquatOrIdentity();
			xf.p = b3Sub(hullA.center, b3RotateVector(xf.q, b3Lerp(capsuleB.center1, capsuleB.center2, 0.5)));
		} else {
			let q = poseRotation(mode);
			let u = poseDirection(mode);
			let gap = poseGap(mode);
			let lie = vrandint(0, 3);
			if (lie == 0) {
				// Lay the capsule flat on the support plane
				let axisB = b3RotateVector(q, b3Sub(capsuleB.center2, capsuleB.center1));
				let flat = b3MulSub(axisB, b3Dot(axisB, u), u);
				let local = b3InvRotateVector(q, flat);
				capsuleB.center2 = b3Add(capsuleB.center1, local);
			}
			let eA = hullExtent(hullA, b3Quat_identity, u);
			let down = b3Neg(u);
			let e1 = b3Dot(b3RotateVector(q, capsuleB.center1), down);
			let e2 = b3Dot(b3RotateVector(q, capsuleB.center2), down);
			let eB = b3MaxFloat(e1, e2) + capsuleB.radius;
			xf = placePose(eA, eB, u, q, gap, mode);
		}

		vtag("hc");
		vi32(i);
		vi32(ia);
		vi32(mode);
		pv3(capsuleB.center1);
		pv3(capsuleB.center2);
		vf32(capsuleB.radius);
		vend();

		let cache = b3SimplexCache();
		for (let step = 0; step < STEP_COUNT; ++step) {
			xf = advancePose(xf, step);
			let capacity = vrandint(0, 5);
			let offset = pickOffset();
			resetManifold(offset);
			pcall("hcc", step, capacity, offset, xf);
			vtag("in");
			psimplexCache(cache);
			vend();
			b3CollideHullAndCapsule(g_manifold, capacity, hullA, capsuleB, xf, cache);
			pmanifold(offset + 4);
			vtag("out");
			psimplexCache(cache);
			vend();
		}
	}
}
