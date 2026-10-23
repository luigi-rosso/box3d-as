// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/capsule.as, mirrors verify/cref/capsule.c

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
import { b3Add, b3Lerp, b3MulAdd, b3MulSV, b3Sub, b3Transform, b3Vec3_zero } from "../../lib/math_functions";
import { b3Capsule, b3PlaneResult, b3RayCastInput, b3ShapeCastInput } from "../../lib/collision";
import {
	b3CollideMoverAndCapsule,
	b3ComputeCapsuleAABB,
	b3ComputeCapsuleMass,
	b3ComputeSweptCapsuleAABB,
	b3OverlapCapsule,
	b3RayCastCapsule,
	b3ShapeCastCapsule,
} from "../../lib/capsule";

export { frameBoundary } from "./common";

let g_proxy = new TestProxy();

function rcapsule(): b3Capsule {
	let c: b3Capsule;
	c.center1 = rv(-2.0, 2.0);
	let mode = vrandint(0, 8);
	if (mode == 0) {
		c.center2 = c.center1;
	} else if (mode == 1) {
		// Shorter than the ray cast sphere fallback
		let d = rv(-1.0e-5, 1.0e-5);
		c.center2 = b3Add(c.center1, d);
	} else if (mode == 2) {
		let d = rv(-1.0e-20, 1.0e-20);
		c.center2 = b3Add(c.center1, d);
	} else {
		let d = rv(-2.0, 2.0);
		c.center2 = b3Add(c.center1, d);
	}
	c.radius = (vrand32() & 7) == 0 ? <f32>0.0 : vrandf(0.01, 1.5);
	return c;
}

function rxf(extent: f32): b3Transform {
	let xf: b3Transform;
	xf.p = rv(-extent, extent);
	xf.q = rquatOrIdentity();
	return xf;
}

function rray(c: b3Capsule): b3RayCastInput {
	let axis = b3Sub(c.center2, c.center1);
	let input: b3RayCastInput;
	let mode = vrandint(0, 8);
	if (mode == 0) {
		// Inside the capsule
		let t = vrandf(-0.2, 1.2);
		let d = runit();
		let s = vrandf(0.0, 0.99);
		input.origin = b3MulAdd(b3MulAdd(c.center1, t, axis), s * c.radius, d);
	} else if (mode == 1) {
		// Inside the infinite cylinder beyond the ends
		let t = (vrand32() & 1) != 0 ? vrandf(-3.0, -0.1) : vrandf(1.1, 4.0);
		let d = runit();
		let s = vrandf(0.0, 0.9);
		input.origin = b3MulAdd(b3MulAdd(c.center1, t, axis), s * c.radius, d);
	} else {
		let offset = rv(-5.0, 5.0);
		input.origin = b3Add(c.center1, offset);
	}

	let aim = vrandint(0, 7);
	if (aim == 0) {
		input.translation = rv(-8.0, 8.0);
	} else if (aim == 1) {
		input.translation = b3Vec3_zero;
	} else if (aim == 2) {
		// Parallel to the axis, maybe slightly off
		let scale = vrandf(-3.0, 3.0);
		let noise = rv(-1.0e-4, 1.0e-4);
		input.translation = b3Add(b3MulSV(scale, axis), noise);
	} else {
		let t = vrandf(-0.3, 1.3);
		let target = b3MulAdd(c.center1, t, axis);
		let jitter = rv(-c.radius, c.radius);
		target = b3Add(target, jitter);
		let scale = vrandf(0.3, 3.0);
		input.translation = b3MulSV(scale, b3Sub(target, input.origin));
	}

	let fraction = vrandint(0, 4);
	input.maxFraction = fraction < 2 ? <f32>1.0 : fraction == 2 ? vrandf(0.0, 1.0) : <f32>0.0;
	return input;
}

export function begin(): void {
	vseed(0xca95e1);
}

export function runBasics(): void {
	for (let i = 0; i < 2000; ++i) {
		let c = rcapsule();
		let density = vrandf(0.0, 10.0);
		vtag("m");
		vi32(i);
		pmass(b3ComputeCapsuleMass(c, density));
		vend();
	}

	for (let i = 0; i < 2000; ++i) {
		let c = rcapsule();
		let xf1 = rxf(10.0);
		let xf2 = rxf(10.0);
		vtag("ab");
		vi32(i);
		pab(b3ComputeCapsuleAABB(c, xf1));
		pab(b3ComputeSweptCapsuleAABB(c, xf1, xf2));
		vend();
	}
}

export function runOverlap(): void {
	for (let i = 0; i < 5000; ++i) {
		let c = rcapsule();
		let xf = rxf(2.5);
		let kind = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_proxy, kind);
		let proxy = testProxy(g_proxy);
		vtag("o");
		vi32(i);
		vbool(b3OverlapCapsule(c, xf, proxy));
		vend();
	}
}

export function runRayCast(): void {
	for (let i = 0; i < 20000; ++i) {
		let c = rcapsule();
		let input = rray(c);
		vtag("r");
		vi32(i);
		pv3(input.origin);
		pv3(input.translation);
		vf32(input.maxFraction);
		pcast(b3RayCastCapsule(c, input));
		vend();
	}
}

export function runShapeCast(): void {
	for (let i = 0; i < 5000; ++i) {
		let c = rcapsule();
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
		input.translation = b3MulSV(scale, b3Sub(b3Add(c.center1, aim), offset));
		if ((vrand32() & 7) == 0) {
			input.translation = rv(-6.0, 6.0);
		}
		input.maxFraction = (vrand32() & 1) != 0 ? <f32>1.0 : vrandf(0.0, 1.0);
		input.canEncroach = (vrand32() & 1) != 0;

		vtag("sc");
		vi32(i);
		pcast(b3ShapeCastCapsule(c, input));
		vend();
	}
}

export function runMover(): void {
	for (let i = 0; i < 5000; ++i) {
		let c = rcapsule();
		let mover = rcapsule();
		mover.radius = vrandf(0.05, 1.0);
		let mode = vrandint(0, 5);
		if (mode == 0) {
			// Core segments cross
			let t = vrandf(0.0, 1.0);
			let p = b3Lerp(c.center1, c.center2, t);
			let d = rv(-1.0, 1.0);
			mover.center1 = b3Sub(p, d);
			mover.center2 = b3Add(p, d);
		} else if (mode == 1) {
			// Parallel and close
			let d = rv(-0.5, 0.5);
			mover.center1 = b3Add(c.center1, d);
			mover.center2 = b3Add(c.center2, d);
		} else if (mode == 2) {
			mover.center1 = c.center1;
			mover.center2 = c.center1;
		}

		let result = b3PlaneResult();
		let count = b3CollideMoverAndCapsule(result, c, mover);
		vtag("mv");
		vi32(i);
		vi32(count);
		if (count > 0) {
			pplaneResult(result);
		}
		vend();
	}
}
