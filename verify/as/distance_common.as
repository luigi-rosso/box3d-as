// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// Printers and seeded proxies shared by the distance, sphere and capsule drivers,
// the twin of verify/cref/distance_common.h.

import { vi32, vbool, vf32, vrand32, vrandint, vrandf } from "./common";
import {
	b3AABB,
	b3LengthSquared,
	b3MakeQuatFromAxisAngle,
	b3Matrix3,
	b3MulAdd,
	b3MulQuat,
	b3Normalize,
	b3NormalizeQuat,
	b3Quat,
	b3Quat_identity,
	b3Transform,
	b3Vec3,
	b3Vec3_axisX,
} from "../../lib/math_functions";
import { b3CastOutput, b3MassData, b3PlaneResult, b3ShapeProxy } from "../../lib/collision";

export const PROXY_CAPACITY: i32 = 40;
export const PROXY_KIND_COUNT: i32 = 8;

export function pv3(v: b3Vec3): void {
	vf32(v.x);
	vf32(v.y);
	vf32(v.z);
}

export function pq(q: b3Quat): void {
	pv3(q.v);
	vf32(q.s);
}

export function pxf(t: b3Transform): void {
	pv3(t.p);
	pq(t.q);
}

export function pab(a: b3AABB): void {
	pv3(a.lowerBound);
	pv3(a.upperBound);
}

export function pm3(m: b3Matrix3): void {
	pv3(m.cx);
	pv3(m.cy);
	pv3(m.cz);
}

export function pcast(o: b3CastOutput): void {
	pv3(o.normal);
	pv3(o.point);
	vf32(o.fraction);
	vi32(o.iterations);
	vi32(o.triangleIndex);
	vi32(o.childIndex);
	vi32(o.materialIndex);
	vbool(o.hit);
}

export function pmass(m: b3MassData): void {
	vf32(m.mass);
	pv3(m.center);
	pm3(m.inertia);
}

export function pplaneResult(r: b3PlaneResult): void {
	pv3(r.plane.normal);
	vf32(r.plane.offset);
	pv3(r.point);
	vi32(r.triangleIndex);
	vi32(r.childIndex);
	vi32(r.materialIndex);
}

export function rv(lo: f32, hi: f32): b3Vec3 {
	let v: b3Vec3;
	v.x = vrandf(lo, hi);
	v.y = vrandf(lo, hi);
	v.z = vrandf(lo, hi);
	return v;
}

export function runit(): b3Vec3 {
	let v = rv(-1.0, 1.0);
	v = b3Normalize(v);
	if (b3LengthSquared(v) == 0.0) {
		v = b3Vec3_axisX;
	}
	return v;
}

export function rquat(): b3Quat {
	let q: b3Quat;
	q.v = rv(-1.0, 1.0);
	q.s = vrandf(-1.0, 1.0);
	return b3NormalizeQuat(q);
}

// A quarter identity rotations so axis aligned degeneracies show up.
export function rquatOrIdentity(): b3Quat {
	if ((vrand32() & 3) == 0) {
		return b3Quat_identity;
	}
	return rquat();
}

export function rotateBy(q: b3Quat, maxAngle: f32): b3Quat {
	let axis = runit();
	let angle = vrandf(-maxAngle, maxAngle);
	let r = b3MakeQuatFromAxisAngle(axis, angle);
	return b3NormalizeQuat(b3MulQuat(r, q));
}

export class TestProxy {
	points: StaticArray<b3Vec3> = new StaticArray<b3Vec3>(PROXY_CAPACITY);
	count: i32 = 0;
	radius: f32 = 0;
	kind: i32 = 0;
}

// kinds: 0 point, 1 segment, 2 triangle, 3 box, 4 cloud, 5 duplicates, 6 quad, 7 collinear
export function makeTestProxy(tp: TestProxy, kind: i32): void {
	tp.kind = kind;
	let s = vrandf(0.1, 1.5);

	switch (kind) {
		case 0:
			tp.count = 1;
			tp.points[0] = rv(-s, s);
			break;

		case 1:
			tp.count = 2;
			tp.points[0] = rv(-s, s);
			tp.points[1] = rv(-s, s);
			break;

		case 2:
			tp.count = 3;
			for (let i = 0; i < 3; ++i) {
				tp.points[i] = rv(-s, s);
			}
			break;

		case 3: {
			let h = rv(0.05, s);
			let c = rv(-0.5 * s, 0.5 * s);
			tp.count = 8;
			for (let i = 0; i < 8; ++i) {
				let p: b3Vec3;
				p.x = (i & 1) != 0 ? c.x + h.x : c.x - h.x;
				p.y = (i & 2) != 0 ? c.y + h.y : c.y - h.y;
				p.z = (i & 4) != 0 ? c.z + h.z : c.z - h.z;
				tp.points[i] = p;
			}
			break;
		}

		case 4: {
			let count = vrandint(4, 33);
			tp.count = count;
			for (let i = 0; i < count; ++i) {
				tp.points[i] = rv(-s, s);
			}
			break;
		}

		case 5: {
			let base: FixedArray<b3Vec3, 3>;
			for (let i = 0; i < 3; ++i) {
				base[i] = rv(-s, s);
			}
			let count = vrandint(2, 9);
			tp.count = count;
			for (let i = 0; i < count; ++i) {
				let pick = vrandint(0, 3);
				tp.points[i] = base[pick];
			}
			break;
		}

		case 6: {
			let hx = vrandf(0.05, s);
			let hy = vrandf(0.05, s);
			tp.count = 4;
			tp.points[0] = b3Vec3(-hx, -hy, 0.0);
			tp.points[1] = b3Vec3(hx, -hy, 0.0);
			tp.points[2] = b3Vec3(hx, hy, 0.0);
			tp.points[3] = b3Vec3(-hx, hy, 0.0);
			break;
		}

		default: {
			let p0 = rv(-s, s);
			let d = runit();
			let count = vrandint(3, 6);
			tp.count = count;
			for (let i = 0; i < count; ++i) {
				let t = vrandf(-s, s);
				tp.points[i] = b3MulAdd(p0, t, d);
			}
			break;
		}
	}

	if ((vrand32() & 1) != 0) {
		tp.radius = 0.0;
	} else {
		tp.radius = s * vrandf(0.01, 0.5);
	}
}

export function testProxy(tp: TestProxy): b3ShapeProxy {
	return new b3ShapeProxy(tp.points, tp.count, tp.radius);
}

export function pproxy(tp: TestProxy): void {
	vi32(tp.kind);
	vi32(tp.count);
	vf32(tp.radius);
	pv3(tp.points[0]);
	pv3(tp.points[tp.count - 1]);
}
