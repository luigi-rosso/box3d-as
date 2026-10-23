// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
// AnimaScript port of src/math_internal.h

import { FLT_MAX, FLT_MIN } from "./core";
import {
	b3AABB,
	b3AbsFloat,
	b3Add,
	b3AddMM,
	b3Conjugate,
	b3Cos,
	b3Cross,
	b3Dot,
	b3DotQuat,
	b3Length,
	b3LengthSquared,
	b3MakeMatrixFromQuat,
	b3MakeQuatFromAxisAngle,
	b3Matrix3,
	b3Max,
	b3MaxFloat,
	b3Min,
	b3MinFloat,
	b3MulMM,
	b3MulQuat,
	b3MulSV,
	b3NegateQuat,
	b3Normalize,
	b3NormalizeQuat,
	b3Plane,
	b3Quat,
	b3RotateVector,
	b3SegmentDistanceResult,
	b3Steiner,
	b3Sub,
	b3Transform,
	b3Transpose,
	b3Vec2,
	b3Vec3,
} from "./math_functions";
import { b3TriangleFeature } from "./collision";

export const B3_TWO_PI: f32 = 6.283185307;
export const B3_PI_OVER_TWO: f32 = 1.570796327;
export const B3_PI_OVER_FOUR: f32 = 0.785398163;
export const B3_SQRT3: f32 = 1.732050808;

// todo eliminate this
export const B3_BOUNDS3_EMPTY: b3AABB = b3AABB(b3Vec3(FLT_MAX, FLT_MAX, FLT_MAX), b3Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX));

export struct b3Matrix2 {
	cx: b3Vec2 = b3Vec2();
	cy: b3Vec2 = b3Vec2();

	constructor(cx: b3Vec2, cy: b3Vec2) {
		this.cx = cx;
		this.cy = cy;
	}
}

export struct b3Triangle {
	vertices: FixedArray<b3Vec3, 3>;
	i1: i32 = 0;
	i2: i32 = 0;
	i3: i32 = 0;
	flags: i32 = 0;
}

export struct b3TrianglePoint {
	point: b3Vec3 = b3Vec3();
	feature: b3TriangleFeature = 0;

	constructor(point: b3Vec3, feature: b3TriangleFeature) {
		this.point = point;
		this.feature = feature;
	}
}

export struct b3ShapeExtent {
	minExtent: f32 = 0;
	maxExtent: b3Vec3 = b3Vec3();
}

// b3ClosestPointOnTriangle, b3SphereInertia, b3CylinderInertia and b3BoxInertia live in
// math_functions.as like their C definitions. b3IntersectSegmentTriangle,
// b3IntersectSegmentSphere, b3ComputeMassProperties, b3IsValidMassData, b3GetProxySupport,
// b3GetPointSupport and b3IsSweepNormalized belong to the modules that define them in C.

// Align up to 8 byte alignment.
export function b3AlignUp8(x: i32): i32 {
	return (x + 7) & ~7;
}

// https://en.wikipedia.org/wiki/Floor_and_ceiling_functions
export function b3CeilingInt(numerator: i32, denominator: i32): i32 {
	return (numerator + denominator - 1) / denominator;
}

// Assumes denominator == 2^exponent
export function b3CeilingPow2(numerator: i32, denominator: i32, exponent: i32): i32 {
	return (numerator + denominator - 1) >> exponent;
}

export function b3Dot2(v1: b3Vec2, v2: b3Vec2): f32 {
	return v1.x * v2.x + v1.y * v2.y;
}

export function b3Length2(v: b3Vec2): f32 {
	return Mathf.sqrt(b3Dot2(v, v));
}

export function b3LengthSquared2(v: b3Vec2): f32 {
	return b3Dot2(v, v);
}

export function b3MinVec2(v1: b3Vec2, v2: b3Vec2): b3Vec2 {
	let v: b3Vec2;
	v.x = b3MinFloat(v1.x, v2.x);
	v.y = b3MinFloat(v1.y, v2.y);
	return v;
}

export function b3MaxVec2(v1: b3Vec2, v2: b3Vec2): b3Vec2 {
	let v: b3Vec2;
	v.x = b3MaxFloat(v1.x, v2.x);
	v.y = b3MaxFloat(v1.y, v2.y);
	return v;
}

export function b3Store(dst: StaticArray<f32>, offset: i32, src: b3Vec3): void {
	dst[offset + 0] = src.x;
	dst[offset + 1] = src.y;
	dst[offset + 2] = src.z;
}

export function b3ClampLength(v: b3Vec3, maxLength: f32): b3Vec3 {
	let lengthSq = b3LengthSquared(v);
	if (lengthSq <= maxLength * maxLength) {
		return v;
	}

	let length = Mathf.sqrt(lengthSq);
	return b3MulSV(maxLength / length, v);
}

// Assume v is a unit vector
export function b3ArbitraryPerp(v: b3Vec3): b3Vec3 {
	// Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
	// Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
	// of a unit vector must be greater or equal to 0.57735.
	let p: b3Vec3;
	if (v.x < -0.5 || 0.5 < v.x) {
		// x is non-zero and it should not go into the x component
		// dot([ay + bz, cx, dx], [x, y, z]) = ayx + bzx + cxy + dzx
		// for the dot product to be zero need: c = -a, d = -b
		let a: f32 = 0.67;
		let b: f32 = -0.42;
		p = b3Vec3(a * v.y + b * v.z, -a * v.x, -b * v.x);
	} else if (v.y < -0.5 || 0.5 < v.y) {
		// y is non-zero and it should not go into the y component
		// p = [ay, bx + cz, dy]
		// axy + bxy + cyz + dyz = 0
		// b = -a, d = -c
		let a: f32 = 0.67;
		let c: f32 = -0.42;
		p = b3Vec3(a * v.y, -a * v.x + c * v.z, -c * v.y);
	} else {
		// z is non-zero and it should not go into the z component
		// p = [az, bz, cx + dy]
		// axz + byz + cxz + dyz = 0
		// c = -a, d = -b
		let a: f32 = 0.67;
		let b: f32 = -0.42;
		p = b3Vec3(a * v.z, b * v.z, -a * v.x - b * v.y);
	}

	return b3Normalize(p);
}

export function b3QuatFromExponentialMap(v: b3Vec3): b3Quat {
	// Exponential map (Grassia)
	let threshold: f32 = 0.018581361;

	let angle = b3Length(v);
	if (angle < threshold) {
		// Taylor expansion
		let out: b3Quat;
		out.v = b3MulSV(0.5 + (angle * angle) / 48.0, v);
		out.s = b3Cos(0.5 * angle);

		return out;
	}

	return b3MakeQuatFromAxisAngle(b3MulSV(1.0 / angle, v), angle);
}

/// Integrate rotation from angular velocity
/// @param q1 initial rotation
/// @param deltaRotation the angular displacement vector in radians (angular velocity multiplied by the time step)
/// q2 = q1 + 0.5 * omega * q1
export function b3IntegrateRotation(q1: b3Quat, deltaRotation: b3Vec3): b3Quat {
	// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
	let qd = b3Quat(b3MulSV(0.5, deltaRotation), 0.0);
	qd = b3MulQuat(qd, q1);
	let q2 = b3Quat(b3Add(q1.v, qd.v), qd.s + q1.s);
	q2 = b3NormalizeQuat(q2);
	return q2;
}

// Pseudo angular velocity from a quaternion target
// w = 2 * (target - q) * conj(q)
export function b3DeltaQuatToRotation(q: b3Quat, target: b3Quat): b3Vec3 {
	let s = q;
	if (b3DotQuat(q, target) < 0.0) {
		// Correct polarity
		s = b3NegateQuat(q);
	}

	let diff = b3Quat(b3Sub(target.v, s.v), target.s - s.s);
	let product = b3MulQuat(diff, b3Conjugate(s));
	return b3MulSV(2.0, product.v);
}

export function b3ScalarTripleProduct(a: b3Vec3, b: b3Vec3, c: b3Vec3): f32 {
	let d: b3Vec3;
	d.x = b.y * c.z - b.z * c.y;
	d.y = b.z * c.x - b.x * c.z;
	d.z = b.x * c.y - b.y * c.x;
	return a.x * d.x + a.y * d.y + a.z * d.z;
}

// Get a value by index. Avoid undefined behavior of code like (&v.x)[2].
export function b3GetByIndex(v: b3Vec3, index: i32): f32 {
	return index == 0 ? v.x : index == 1 ? v.y : v.z;
}

export function b3MajorAxis(v: b3Vec3): i32 {
	return v.x < v.y ? (v.y < v.z ? 2 : 1) : v.x < v.z ? 2 : 0;
}

export function b3MinElement(v: b3Vec3): f32 {
	return b3MinFloat(v.x, b3MinFloat(v.y, v.z));
}

export function b3MaxElement(v: b3Vec3): f32 {
	return b3MaxFloat(v.x, b3MaxFloat(v.y, v.z));
}

export function b3MaxElementIndex(v: b3Vec3): i32 {
	return v.x < v.y ? (v.y < v.z ? 2 : 1) : v.x < v.z ? 2 : 0;
}

export function b3Add2(a: b3Vec2, b: b3Vec2): b3Vec2 {
	let c = b3Vec2(a.x + b.x, a.y + b.y);
	return c;
}

export function b3Sub2(a: b3Vec2, b: b3Vec2): b3Vec2 {
	let c = b3Vec2(a.x - b.x, a.y - b.y);
	return c;
}

export function b3Neg2(v: b3Vec2): b3Vec2 {
	let c = b3Vec2(-v.x, -v.y);
	return c;
}

export function b3MulSV2(s: f32, v: b3Vec2): b3Vec2 {
	let c = b3Vec2(s * v.x, s * v.y);
	return c;
}

// a + s * b
export function b3MulAdd2(a: b3Vec2, s: f32, b: b3Vec2): b3Vec2 {
	let c = b3Vec2(a.x + s * b.x, a.y + s * b.y);
	return c;
}

// a - s * b
export function b3MulSub2(a: b3Vec2, s: f32, b: b3Vec2): b3Vec2 {
	let c = b3Vec2(a.x - s * b.x, a.y - s * b.y);
	return c;
}

export function b3Cross2(a: b3Vec2, b: b3Vec2): f32 {
	return a.x * b.y - a.y * b.x;
}

export function b3DistanceSquared2(a: b3Vec2, b: b3Vec2): f32 {
	let dx = b.x - a.x;
	let dy = b.y - a.y;
	return dx * dx + dy * dy;
}

export function b3MulMV2(m: b3Matrix2, a: b3Vec2): b3Vec2 {
	let b = b3Vec2(m.cx.x * a.x + m.cy.x * a.y, m.cx.y * a.x + m.cy.y * a.y);
	return b;
}

export function b3MulMM2(m1: b3Matrix2, m2: b3Matrix2): b3Matrix2 {
	let out: b3Matrix2;
	out.cx = b3MulMV2(m1, m2.cx);
	out.cy = b3MulMV2(m1, m2.cy);
	return out;
}

export function b3Det2(m: b3Matrix2): f32 {
	return m.cx.x * m.cy.y - m.cx.y * m.cy.x;
}

export function b3Invert2(m: b3Matrix2): b3Matrix2 {
	let det = b3Det2(m);
	if (b3AbsFloat(det) > 1000.0 * FLT_MIN) {
		let invDet: f32 = 1.0 / det;
		return b3Matrix2(b3Vec2(invDet * m.cy.y, -invDet * m.cx.y), b3Vec2(-invDet * m.cy.x, invDet * m.cx.x));
	}

	return b3Matrix2(b3Vec2(0.0, 0.0), b3Vec2(0.0, 0.0));
}

// Assumes positive semi-definite
export function b3Solve2(m: b3Matrix2, b: b3Vec2): b3Vec2 {
	let det = b3Det2(m);
	if (det > 1000.0 * FLT_MIN) {
		let invDet: f32 = 1.0 / det;
		return b3Vec2(invDet * m.cy.y * b.x - invDet * m.cy.x * b.y, -invDet * m.cx.y * b.x + invDet * m.cx.x * b.y);
	}

	return b3Vec2(0.0, 0.0);
}

// Convenience function: s * a + t * b + u * c
export function b3Blend3(s: f32, a: b3Vec3, t: f32, b: b3Vec3, u: f32, c: b3Vec3): b3Vec3 {
	let d = b3Vec3(
		s * a.x + t * b.x + u * c.x,
		s * a.y + t * b.y + u * c.y,
		s * a.z + t * b.z + u * c.z,
	);
	return d;
}

export function b3ModifiedCross(a: b3Vec3, b: b3Vec3): b3Vec3 {
	let c: b3Vec3;
	c.x = a.y * b.z + a.z * b.y;
	c.y = a.z * b.x + a.x * b.z;
	c.z = a.x * b.y + a.y * b.x;
	return c;
}

export function b3MakeDiagonalMatrix(a: f32, b: f32, c: f32): b3Matrix3 {
	return b3Matrix3(b3Vec3(a, 0.0, 0.0), b3Vec3(0.0, b, 0.0), b3Vec3(0.0, 0.0, c));
}

export function b3NormalizePlane(plane: b3Plane): b3Plane {
	let invLength: f32 = 1.0 / b3Length(plane.normal);
	return b3Plane(b3MulSV(invLength, plane.normal), invLength * plane.offset);
}

export function b3MakePlaneFromNormalAndPoint(normal: b3Vec3, point: b3Vec3): b3Plane {
	return b3Plane(normal, b3Dot(normal, point));
}

export function b3MakePlaneFromPoints(point1: b3Vec3, point2: b3Vec3, point3: b3Vec3): b3Plane {
	let plane: b3Plane;
	plane.normal = b3Cross(b3Sub(point2, point1), b3Sub(point3, point1));
	plane.normal = b3Normalize(plane.normal);
	plane.offset = b3Dot(plane.normal, point1);
	return plane;
}

export function b3MakeNormalFromPoints(point1: b3Vec3, point2: b3Vec3, point3: b3Vec3): b3Vec3 {
	let normal = b3Cross(b3Sub(point2, point1), b3Sub(point3, point1));
	return b3Normalize(normal);
}

// normal2 = q * normal1
// offset2 = dot(normal2, p) + offset1
export function b3TransformPlane(transform: b3Transform, plane: b3Plane): b3Plane {
	let normal = b3RotateVector(transform.q, plane.normal);
	return b3Plane(normal, plane.offset + b3Dot(normal, transform.p));
}

/// Signed separation of a point from a plane
export function b3PlaneSeparation(plane: b3Plane, point: b3Vec3): f32 {
	return b3Dot(plane.normal, point) - plane.offset;
}

// Negative if p is below the triangle v1-v2-v3
export function b3SignedVolume(v1: b3Vec3, v2: b3Vec3, v3: b3Vec3, p: b3Vec3): f32 {
	let e1 = b3Sub(v2, v1);
	let e2 = b3Sub(v3, v1);
	let n = b3Cross(e1, e2);
	return b3Dot(n, b3Sub(p, v1));
}

// todo eliminate this
export function b3IsWithinSegments(result: b3SegmentDistanceResult): bool {
	return (
		0.0 <= result.fraction1 && result.fraction1 <= 1.0 && 0.0 <= result.fraction2 && result.fraction2 <= 1.0
	);
}

export function b3RotateInertia(q: b3Quat, centralInertia: b3Matrix3): b3Matrix3 {
	let rotationMatrix = b3MakeMatrixFromQuat(q);
	let inertia = b3MulMM(rotationMatrix, b3MulMM(centralInertia, b3Transpose(rotationMatrix)));
	return inertia;
}

export function b3TransformInertia(transform: b3Transform, centralInertia: b3Matrix3, mass: f32): b3Matrix3 {
	let inertia = b3RotateInertia(transform.q, centralInertia);
	inertia = b3AddMM(inertia, b3Steiner(mass, transform.p));
	return inertia;
}

// Add a point to an AABB.
export function b3AABB_AddPoint(a: b3AABB, point: b3Vec3): b3AABB {
	return b3AABB(b3Min(a.lowerBound, point), b3Max(a.upperBound, point));
}
