// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/sphere.as, mirrors verify/cref/sphere.c

import { vtag, vi32, vbool, vf32, vend, vseed, vrand32, vrandint, vrandf } from "./common";
import {
	PROXY_KIND_COUNT,
	TestProxy,
	makeTestProxy,
	pab,
	pcast,
	pmass,
	pplaneResult,
	pv3,
	rquatOrIdentity,
	runit,
	rv,
	testProxy,
} from "./distance_common";
import { b3Add, b3Lerp, b3MulAdd, b3MulSV, b3Sub, b3Transform, b3Vec3, b3Vec3_zero } from "../../lib/math_functions";
import { b3Capsule, b3PlaneResult, b3RayCastInput, b3ShapeCastInput, b3Sphere } from "../../lib/collision";
import {
	b3CollideMoverAndSphere,
	b3ComputeSphereAABB,
	b3ComputeSphereMass,
	b3ComputeSweptSphereAABB,
	b3OverlapSphere,
	b3RayCastHollowSphere,
	b3RayCastSphere,
	b3ShapeCastSphere,
} from "../../lib/sphere";

export { frameBoundary } from "./common";

let g_proxy = new TestProxy();

function rsphere(): b3Sphere {
	let s: b3Sphere;
	s.center = rv(-2.0, 2.0);
	s.radius = (vrand32() & 7) == 0 ? <f32>0.0 : vrandf(0.01, 2.0);
	return s;
}

function rxf(extent: f32): b3Transform {
	let xf: b3Transform;
	xf.p = rv(-extent, extent);
	xf.q = rquatOrIdentity();
	return xf;
}

function rray(center: b3Vec3, radius: f32): b3RayCastInput {
	let input: b3RayCastInput;
	let mode = vrandint(0, 6);
	if (mode == 0) {
		// Inside
		let d = runit();
		let t = vrandf(0.0, 0.99);
		input.origin = b3MulAdd(center, t * radius, d);
	} else if (mode == 1) {
		// Near the surface
		let d = runit();
		let t = vrandf(0.98, 1.02);
		input.origin = b3MulAdd(center, t * radius, d);
	} else {
		let offset = rv(-6.0, 6.0);
		input.origin = b3Add(center, offset);
	}

	let aim = vrandint(0, 5);
	if (aim == 0) {
		input.translation = rv(-8.0, 8.0);
	} else if (aim == 1) {
		input.translation = b3Vec3_zero;
	} else {
		let target = rv(-radius, radius);
		target = b3Add(center, target);
		let scale = vrandf(0.3, 3.0);
		input.translation = b3MulSV(scale, b3Sub(target, input.origin));
	}

	let fraction = vrandint(0, 4);
	input.maxFraction = fraction < 2 ? <f32>1.0 : fraction == 2 ? vrandf(0.0, 1.0) : <f32>0.0;
	return input;
}

function rmover(): b3Capsule {
	let mover: b3Capsule;
	mover.center1 = rv(-2.0, 2.0);
	let mode = vrandint(0, 4);
	if (mode == 0) {
		mover.center2 = mover.center1;
	} else {
		let d = rv(-1.5, 1.5);
		mover.center2 = b3Add(mover.center1, d);
	}
	mover.radius = vrandf(0.05, 1.0);
	return mover;
}

export function begin(): void {
	vseed(0x5be7e5);
}

export function runBasics(): void {
	for (let i = 0; i < 1000; ++i) {
		let s = rsphere();
		let density = vrandf(0.0, 10.0);
		vtag("m");
		vi32(i);
		pmass(b3ComputeSphereMass(s, density));
		vend();
	}

	for (let i = 0; i < 2000; ++i) {
		let s = rsphere();
		let xf1 = rxf(10.0);
		let xf2 = rxf(10.0);
		vtag("ab");
		vi32(i);
		pab(b3ComputeSphereAABB(s, xf1));
		pab(b3ComputeSweptSphereAABB(s, xf1, xf2));
		vend();
	}
}

export function runOverlap(): void {
	for (let i = 0; i < 5000; ++i) {
		let s = rsphere();
		let xf = rxf(2.5);
		let kind = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_proxy, kind);
		let proxy = testProxy(g_proxy);
		vtag("o");
		vi32(i);
		vbool(b3OverlapSphere(s, xf, proxy));
		vend();
	}
}

export function runRayCast(): void {
	for (let i = 0; i < 10000; ++i) {
		let s = rsphere();
		let input = rray(s.center, s.radius);
		vtag("r");
		vi32(i);
		pv3(input.origin);
		pv3(input.translation);
		vf32(input.maxFraction);
		pcast(b3RayCastSphere(s, input));
		pcast(b3RayCastHollowSphere(s, input));
		vend();
	}
}

export function runShapeCast(): void {
	for (let i = 0; i < 5000; ++i) {
		let s = rsphere();
		let kind = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_proxy, kind);
		let offset = rv(-4.0, 4.0);
		for (let k = 0; k < g_proxy.count; ++k) {
			g_proxy.points[k] = b3Add(g_proxy.points[k], offset);
		}

		let input = new b3ShapeCastInput();
		input.proxy = testProxy(g_proxy);
		let aim = rv(-1.0, 1.0);
		let scale = vrandf(0.2, 2.5);
		input.translation = b3MulSV(scale, b3Sub(b3Add(s.center, aim), offset));
		if ((vrand32() & 7) == 0) {
			input.translation = rv(-6.0, 6.0);
		}
		input.maxFraction = (vrand32() & 1) != 0 ? <f32>1.0 : vrandf(0.0, 1.0);
		input.canEncroach = (vrand32() & 1) != 0;

		vtag("sc");
		vi32(i);
		pcast(b3ShapeCastSphere(s, input));
		vend();
	}
}

export function runMover(): void {
	for (let i = 0; i < 5000; ++i) {
		let s = rsphere();
		let mover = rmover();
		let mode = vrandint(0, 4);
		if (mode == 0) {
			// Sphere center on the mover axis
			let t = vrandf(0.0, 1.0);
			s.center = b3Lerp(mover.center1, mover.center2, t);
		} else if (mode == 1) {
			let d = rv(-0.003, 0.003);
			s.center = b3Add(mover.center1, d);
		}

		let result = b3PlaneResult();
		let count = b3CollideMoverAndSphere(result, s, mover);
		vtag("mv");
		vi32(i);
		vi32(count);
		if (count > 0) {
			pplaneResult(result);
		}
		vend();
	}
}
