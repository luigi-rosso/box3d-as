// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Driver for lib/distance.as, mirrors verify/cref/distance.c

import { vtag, vi32, vbool, vf32, vend, vseed, vrand32, vrandint, vrandf, vspecialf } from "./common";
import {
	PROXY_KIND_COUNT,
	TestProxy,
	makeTestProxy,
	pcast,
	pproxy,
	pq,
	pv3,
	pxf,
	rotateBy,
	rquat,
	rquatOrIdentity,
	runit,
	rv,
	testProxy,
} from "./distance_common";
import { b3SetLengthUnitsPerMeter } from "../../lib/core";
import { B3_LINEAR_SLOP } from "../../lib/constants";
import {
	B3_PI,
	b3Add,
	b3Dot,
	b3MakeQuatFromAxisAngle,
	b3Quat_identity,
	b3Transform_identity,
	b3Vec3_axisX,
	b3MulAdd,
	b3MulSV,
	b3MulSub,
	b3Neg,
	b3Sub,
	b3Transform,
	b3Vec3,
	b3Vec3_zero,
} from "../../lib/math_functions";
import {
	b3DistanceInput,
	b3DistanceOutput,
	b3ShapeCastPairInput,
	b3ShapeProxy,
	b3Simplex,
	b3SimplexCache,
	b3Sweep,
	b3TOIInput,
	b3TOIOutput,
} from "../../lib/collision";
import {
	b3GetPointSupport,
	b3GetProxySupport,
	b3GetSweepTransform,
	b3ShapeCast,
	b3ShapeDistance,
	b3TimeOfImpact,
} from "../../lib/distance";

export { frameBoundary } from "./common";

const SIMPLEX_CAPACITY: i32 = 40;

function pcache(c: b3SimplexCache): void {
	vf32(c.metric);
	vi32(<i32>c.count);
	for (let i = 0; i < 4; ++i) {
		vi32(<i32>c.indexA[i]);
	}
	for (let i = 0; i < 4; ++i) {
		vi32(<i32>c.indexB[i]);
	}
}

function pdist(o: b3DistanceOutput): void {
	pv3(o.pointA);
	pv3(o.pointB);
	pv3(o.normal);
	vf32(o.distance);
	vi32(o.iterations);
	vi32(o.simplexCount);
}

function psimplex(s: b3Simplex): void {
	vi32(s.count);
	for (let i = 0; i < 4; ++i) {
		let v = s.vertices[i];
		pv3(v.wA);
		pv3(v.wB);
		pv3(v.w);
		vf32(v.a);
		vi32(v.indexA);
		vi32(v.indexB);
	}
}

function ptoi(o: b3TOIOutput): void {
	vi32(o.state);
	pv3(o.point);
	pv3(o.normal);
	vf32(o.fraction);
	vf32(o.distance);
	vi32(o.distanceIterations);
	vi32(o.pushBackIterations);
	vi32(o.rootIterations);
	vbool(o.usedFallback);
}

function rposition(): b3Vec3 {
	let mode = vrandint(0, 5);
	if (mode == 0) {
		return rv(-1.0, 1.0);
	}
	if (mode == 1) {
		return rv(-0.05, 0.05);
	}
	if (mode == 2) {
		let p = rv(-3.0, 3.0);
		return b3Add(p, b3Vec3(900.0, -1300.0, 400.0));
	}
	return rv(-3.0, 3.0);
}

function rxf(): b3Transform {
	let xf: b3Transform;
	xf.p = rposition();
	xf.q = rquatOrIdentity();
	return xf;
}

let g_a = new TestProxy();
let g_b = new TestProxy();
let g_simplexes = new StaticArray<b3Simplex>(SIMPLEX_CAPACITY);

export function begin(): void {
	vseed(0x5eed0d15);
}

function fixedDistance(
	id: i32,
	pa: StaticArray<b3Vec3>,
	ca: i32,
	ra: f32,
	pb: StaticArray<b3Vec3>,
	cb: i32,
	rb: f32,
	xf: b3Transform,
): void {
	for (let useRadii = 0; useRadii < 2; ++useRadii) {
		let input = new b3DistanceInput();
		input.proxyA = new b3ShapeProxy(pa, ca, ra);
		input.proxyB = new b3ShapeProxy(pb, cb, rb);
		input.transform = xf;
		input.useRadii = useRadii == 1;

		let cache = b3SimplexCache();
		for (let w = 0; w < 2; ++w) {
			let out = b3ShapeDistance(input, cache, g_simplexes, SIMPLEX_CAPACITY);
			vtag("fd");
			vi32(id);
			vi32(useRadii);
			vi32(w);
			pdist(out);
			pcache(cache);
			vend();
			for (let k = 0; k < out.simplexCount; ++k) {
				vtag("s");
				vi32(k);
				psimplex(g_simplexes[k]);
				vend();
			}
		}
	}
}

function points1(p: b3Vec3): StaticArray<b3Vec3> {
	let a = new StaticArray<b3Vec3>(1);
	a[0] = p;
	return a;
}

function points2(p: b3Vec3, q: b3Vec3): StaticArray<b3Vec3> {
	let a = new StaticArray<b3Vec3>(2);
	a[0] = p;
	a[1] = q;
	return a;
}

export function runFixed(): void {
	let origin = points1(b3Vec3(0.0, 0.0, 0.0));
	let corner = points1(b3Vec3(1.0, 1.0, 1.0));
	let segX = points2(b3Vec3(-1.0, 0.0, 0.0), b3Vec3(1.0, 0.0, 0.0));
	let segY = points2(b3Vec3(0.0, -1.0, 0.0), b3Vec3(0.0, 1.0, 0.0));
	let tri = new StaticArray<b3Vec3>(3);
	tri[0] = b3Vec3(-1.0, -1.0, 0.0);
	tri[1] = b3Vec3(1.0, -1.0, 0.0);
	tri[2] = b3Vec3(0.0, 1.0, 0.0);
	let box = new StaticArray<b3Vec3>(8);
	for (let i = 0; i < 8; ++i) {
		box[i].x = (i & 1) != 0 ? <f32>1.0 : <f32>-1.0;
		box[i].y = (i & 2) != 0 ? <f32>1.0 : <f32>-1.0;
		box[i].z = (i & 4) != 0 ? <f32>1.0 : <f32>-1.0;
	}

	let id = b3Transform_identity;
	let faceTouch = b3Transform(b3Vec3(2.0, 0.0, 0.0), b3Quat_identity);
	let faceGap = b3Transform(b3Vec3(2.5, 0.0, 0.0), b3Quat_identity);
	let deep = b3Transform(b3Vec3(0.5, 0.25, 0.0), b3Quat_identity);
	let edgeTouch = b3Transform(b3Vec3(2.0, 2.0, 0.0), b3Quat_identity);
	let cornerTouch = b3Transform(b3Vec3(2.0, 2.0, 2.0), b3Quat_identity);
	let turned = b3Transform(b3Vec3(0.0, 0.0, 1.0), b3MakeQuatFromAxisAngle(b3Vec3_axisX, 0.5 * B3_PI));

	fixedDistance(0, origin, 1, 0.0, origin, 1, 0.0, id);
	fixedDistance(1, origin, 1, 0.5, origin, 1, 0.25, faceTouch);
	fixedDistance(2, segX, 2, 0.0, origin, 1, 0.0, id);
	fixedDistance(3, segX, 2, 0.1, segY, 2, 0.1, id);
	fixedDistance(4, segX, 2, 0.0, segX, 2, 0.0, turned);
	fixedDistance(5, tri, 3, 0.0, origin, 1, 0.0, id);
	fixedDistance(6, tri, 3, 0.0, segY, 2, 0.0, turned);
	fixedDistance(7, box, 8, 0.0, box, 8, 0.0, faceTouch);
	fixedDistance(8, box, 8, 0.0, box, 8, 0.0, faceGap);
	fixedDistance(9, box, 8, 0.0, box, 8, 0.0, deep);
	fixedDistance(10, box, 8, 0.0, box, 8, 0.0, edgeTouch);
	fixedDistance(11, box, 8, 0.0, box, 8, 0.0, cornerTouch);
	fixedDistance(12, box, 8, 0.0, corner, 1, 0.0, id);
	fixedDistance(13, box, 8, 0.0, origin, 1, 0.0, id);
	fixedDistance(14, box, 8, 0.0, tri, 3, 0.0, turned);
	fixedDistance(15, tri, 3, 0.0, tri, 3, 0.0, id);
}

export function runSupport(): void {
	for (let i = 0; i < 3000; ++i) {
		let kind = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_a, kind);
		let proxy = testProxy(g_a);

		let axis: b3Vec3;
		if ((vrand32() & 1) != 0) {
			axis = rv(-1.0, 1.0);
		} else {
			axis.x = vspecialf();
			axis.y = vspecialf();
			axis.z = vspecialf();
		}

		vtag("sup");
		vi32(i);
		pproxy(g_a);
		pv3(axis);
		vi32(b3GetProxySupport(proxy, axis));
		vi32(b3GetPointSupport(g_a.points, g_a.count, axis));
		vend();
	}
}

export function runDistance(count: i32): void {
	for (let i = 0; i < count; ++i) {
		let kindA = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_a, kindA);
		let kindB = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_b, kindB);

		let input = new b3DistanceInput();
		input.proxyA = testProxy(g_a);
		input.proxyB = testProxy(g_b);
		input.transform = rxf();
		input.useRadii = (vrand32() & 1) != 0;

		let simplexMode = vrandint(0, 3);
		let simplexes: StaticArray<b3Simplex>? = simplexMode == 0 ? null : g_simplexes;
		let capacity = simplexMode == 0 ? 0 : simplexMode == 1 ? SIMPLEX_CAPACITY : 3;

		vtag("din");
		vi32(i);
		pproxy(g_a);
		pproxy(g_b);
		pxf(input.transform);
		vbool(input.useRadii);
		vi32(capacity);
		vend();

		let cache = b3SimplexCache();
		for (let w = 0; w < 5; ++w) {
			let out = b3ShapeDistance(input, cache, simplexes, capacity);

			vtag("d");
			vi32(i);
			vi32(w);
			pdist(out);
			pcache(cache);
			vend();

			if (simplexes != null) {
				for (let k = 0; k < out.simplexCount; ++k) {
					vtag("s");
					vi32(k);
					psimplex(simplexes[k]);
					vend();
				}
			}

			// Warm start: same pose, then small steps, then a jump
			if (w == 1 || w == 2) {
				let dp = rv(-0.05, 0.05);
				input.transform.p = b3Add(input.transform.p, dp);
				input.transform.q = rotateBy(input.transform.q, 0.1);
			} else if (w == 3) {
				input.transform = rxf();
			}
		}
	}
}

const gaps: StaticArray<f32> = [0.25, 0.75, 1.0, 1.2, 1.5, 2.5, 5.0];

export function runCast(count: i32): void {
	let linearSlop = B3_LINEAR_SLOP();

	for (let i = 0; i < count; ++i) {
		let kindA = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_a, kindA);
		let kindB = vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_b, kindB);

		let input = new b3ShapeCastPairInput();
		input.proxyA = testProxy(g_a);
		input.proxyB = testProxy(g_b);
		input.transform.p = rv(-4.0, 4.0);
		input.transform.q = rquatOrIdentity();

		let aim = rv(-1.0, 1.0);
		let scale = vrandf(0.2, 2.5);
		input.translationB = b3MulSV(scale, b3Sub(aim, input.transform.p));

		let mode = vrandint(0, 6);
		if (mode == 1) {
			input.translationB = rv(-6.0, 6.0);
		} else if (mode == 2) {
			input.transform.p = rv(-0.3, 0.3);
		} else if (mode == 3 || mode == 4) {
			let di = new b3DistanceInput();
			di.proxyA = input.proxyA;
			di.proxyB = input.proxyB;
			di.transform = input.transform;
			di.useRadii = false;
			let cache = b3SimplexCache();
			let d = b3ShapeDistance(di, cache, null, 0);

			if (mode == 3) {
				// Start near touching
				let pick = vrandint(0, 7);
				let desired = input.proxyA.radius + input.proxyB.radius + gaps[pick] * linearSlop;
				input.transform.p = b3MulAdd(input.transform.p, desired - d.distance, d.normal);
			} else {
				// Graze: keep only the tangential part
				let along = b3Dot(input.translationB, d.normal);
				input.translationB = b3MulSub(input.translationB, along, d.normal);
			}
		} else if (mode == 5) {
			input.translationB = b3Vec3_zero;
		}

		let fractionMode = vrandint(0, 4);
		input.maxFraction = fractionMode < 2 ? <f32>1.0 : fractionMode == 2 ? vrandf(0.0, 1.0) : vrandf(0.0, 0.01);
		input.canEncroach = (vrand32() & 1) != 0;

		let out = b3ShapeCast(input);

		vtag("c");
		vi32(i);
		vi32(mode);
		pproxy(g_a);
		pproxy(g_b);
		pxf(input.transform);
		pv3(input.translationB);
		vf32(input.maxFraction);
		vbool(input.canEncroach);
		pcast(out);
		vend();
	}
}

export function runSweep(): void {
	for (let i = 0; i < 2000; ++i) {
		let sweep: b3Sweep;
		sweep.localCenter = rv(-1.0, 1.0);
		sweep.c1 = rv(-10.0, 10.0);
		sweep.c2 = rv(-10.0, 10.0);
		sweep.q1 = rquat();
		sweep.q2 = rotateBy(sweep.q1, 3.0);
		let mode = vrandint(0, 4);
		let time: f32 = mode == 0 ? <f32>0.0 : mode == 1 ? <f32>1.0 : vrandf(0.0, 1.0);

		vtag("sw");
		vi32(i);
		vf32(time);
		pxf(b3GetSweepTransform(sweep, time));
		vend();
	}
}

function rsweepA(): b3Sweep {
	let s: b3Sweep;
	s.localCenter = rv(-0.2, 0.2);
	s.c1 = rv(-0.5, 0.5);
	s.c2 = s.c1;
	if ((vrand32() & 1) != 0) {
		let d = rv(-1.0, 1.0);
		s.c2 = b3Add(s.c1, d);
	}
	s.q1 = rquatOrIdentity();
	s.q2 = rotateBy(s.q1, 0.5);
	return s;
}

const angles: StaticArray<f32> = [0.0, 0.2, 1.0, 3.0];

export function runToi(count: i32, forcedMode: i32): void {
	for (let i = 0; i < count; ++i) {
		let mode = forcedMode >= 0 ? forcedMode : vrandint(0, 6);

		// Mode 5 rotates a segment fast around a triangle vertex, where progress can stall
		let kindA = mode == 5 ? 2 : vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_a, kindA);
		let kindB = mode == 5 ? 1 : vrandint(0, PROXY_KIND_COUNT);
		makeTestProxy(g_b, kindB);

		let input = new b3TOIInput();
		input.proxyA = testProxy(g_a);
		input.proxyB = testProxy(g_b);
		input.sweepA = rsweepA();

		let sb: b3Sweep;
		sb.localCenter = rv(-0.2, 0.2);
		sb.c1 = rv(-5.0, 5.0);
		let aim = rv(-1.0, 1.0);
		let scale = vrandf(1.0, 2.0);
		sb.c2 = b3Add(sb.c1, b3MulSV(scale, b3Sub(aim, sb.c1)));
		sb.q1 = rquatOrIdentity();
		let pick = vrandint(0, 4);
		sb.q2 = rotateBy(sb.q1, angles[pick]);

		if (mode == 1) {
			// Mostly misses
			sb.c2 = rv(-5.0, 5.0);
		} else if (mode == 2) {
			// Start overlapped
			sb.c1 = rv(-0.2, 0.2);
		} else if (mode == 3) {
			// Fast rotation in place near A
			let dir = runit();
			let dist = vrandf(0.5, 2.5);
			sb.c1 = b3MulSV(dist, dir);
			sb.c2 = sb.c1;
			sb.q2 = rotateBy(sb.q1, 3.0);
		} else if (mode == 4) {
			// Translate far in one step, tunneling candidates
			sb.c1 = rv(-20.0, -10.0);
			sb.c2 = rv(10.0, 20.0);
			let offset = rv(-0.5, 0.5);
			sb.c2 = b3Add(b3Neg(sb.c1), offset);
		} else if (mode == 5) {
			input.sweepA = b3Sweep();
			input.sweepA.q1 = b3Quat_identity;
			input.sweepA.q2 = b3Quat_identity;
			sb.localCenter = b3Vec3_zero;
			let v = vrandint(0, g_a.count);
			let offset = rv(-0.3, 0.3);
			sb.c1 = b3Add(g_a.points[v], offset);
			let move = rv(-0.5, 0.5);
			sb.c2 = b3Add(sb.c1, move);
			sb.q1 = rquat();
			sb.q2 = rotateBy(sb.q1, 3.1);
		}
		input.sweepB = sb;
		input.maxFraction = (vrand32() & 1) != 0 ? 1.0 : vrandf(0.1, 1.0);

		let out = b3TimeOfImpact(input);

		vtag("t");
		vi32(i);
		vi32(mode);
		pproxy(g_a);
		pproxy(g_b);
		pv3(input.sweepB.c1);
		pv3(input.sweepB.c2);
		pq(input.sweepB.q2);
		vf32(input.maxFraction);
		ptoi(out);
		vend();
	}
}

export function setLengthUnits(units: f32): void {
	b3SetLengthUnitsPerMeter(units);
}
